#!/usr/bin/env python3
"""
Magnetometer alignment check — does mag_to_body() (include/mag.h) match
how the MMC5603NJ is really mounted, and is its hard-iron calibration good?

Record a CSV with the GUI (Telemetry tab -> Record CSV) while slowly turning
the rocket through every orientation for about a minute — several full turns
about each of its three axes — away from steel desks and laptops. Then:

    python src/mag_check.py tlm_20261007_120000.csv
    python src/mag_check.py tlm_....csv --southern    # south of the equator

How it works: between two samples the gyro says exactly how the body turned.
The Earth's field doesn't move, so in body axes it must turn the opposite
way by the same amount. For each of the 48 ways the magnetometer's axes
could map onto the body (axis swaps and sign flips), this fits the leftover
hard-iron offset and measures how well that holds. The right mapping fits
far better than any other; a mapping that's wrong only shows up in some
orientations, which is why the rocket can look fine standing up.

Turning alone can't tell a mapping from the same mapping with all three
axes reversed (the field would just point the other way), so gravity
settles it: in the northern hemisphere the field dips down into the ground.

Needs only the standard library.
"""

import csv
import itertools
import math
import sys

WINDOW_S      = 0.25    # time between the two samples of each pair
MIN_TURN_DEG  = 5.0     # pairs that barely turned say nothing about the mapping
MAX_TURN_DEG  = 90.0    # ...and big turns between frames make the gyro integration less exact
MAX_GAP_S     = 0.1     # a longer gap between frames breaks the integration
MIN_COVER_DEG = 180.0   # turning wanted about each body axis
CONFIDENT     = 2.0     # runner-up must fit at least this many times worse

# The mapping in mag_to_body() today: body X = -mag Y, body Y = mag X, body Z = mag Z.
# lsm6dsox_to_body() is the same turn for the IMU, so IMU axes = FIRMWARE^T * body.
FIRMWARE = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
AXES = "XYZ"


# ---------------------------------------------------------------- small maths
def matvec(M, v):
    return tuple(sum(M[i][j] * v[j] for j in range(3)) for i in range(3))


def transpose(M):
    return tuple(tuple(M[j][i] for j in range(3)) for i in range(3))


def matmul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def det(M):
    return (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
            - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
            + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))


def solve3(A, b):
    """Solve a 3x3 system by Cramer's rule; None if singular."""
    d = det(A)
    if abs(d) < 1e-12:
        return None
    out = []
    for c in range(3):
        Ac = tuple(tuple(b[i] if j == c else A[i][j] for j in range(3)) for i in range(3))
        out.append(det(Ac) / d)
    return tuple(out)


def qmul(a, b):
    return (a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0])


def qrot(q):
    """Rotation matrix of a unit quaternion."""
    q0, q1, q2, q3 = q
    return ((1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)),
            (2*(q1*q2 + q0*q3), 1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1)),
            (2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1*q1 + q2*q2)))


def finite(*vals):
    return all(v == v and abs(v) != math.inf for v in vals)


# ---------------------------------------------------------------- data
def load(path):
    """(t, gyro [rad/s], mag [G], accel [g]) per frame — gyro and accel in
    body axes, mag in its own axes, as the GUI records them."""
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            try:
                t = float(r["t_ms"]) / 1000.0
                g = tuple(math.radians(float(r[k])) for k in ("gx", "gy", "gz"))
                m = tuple(float(r[k]) for k in ("mx", "my", "mz"))
                a = tuple(float(r[k]) for k in ("ax", "ay", "az"))
            except (KeyError, ValueError):
                continue
            if finite(t, *g, *m, *a):
                rows.append((t, g, m, a))
    return rows


def make_pairs(rows):
    """(m1, m2, R) with R the body's turn from sample 1 to 2, in body axes at
    sample 1 - so a fixed world vector reads R^T v1 at sample 2. Also the
    total turning about each body axis, to judge whether the test covered
    enough orientations."""
    pairs, cover = [], [0.0, 0.0, 0.0]
    for k in range(len(rows) - 1):
        dt = rows[k + 1][0] - rows[k][0]
        if 0 < dt <= MAX_GAP_S:
            w = [(a + b) / 2 for a, b in zip(rows[k][1], rows[k + 1][1])]
            for i in range(3):
                cover[i] += abs(math.degrees(w[i] * dt))

    rows = [r[:3] for r in rows]
    for i in range(0, len(rows) - 1, 3):
        q = (1.0, 0.0, 0.0, 0.0)
        j = i
        while j + 1 < len(rows) and rows[j + 1][0] - rows[i][0] <= WINDOW_S:
            dt = rows[j + 1][0] - rows[j][0]
            if not 0 < dt <= MAX_GAP_S:
                break
            w = [(a + b) / 2 for a, b in zip(rows[j][1], rows[j + 1][1])]
            ang = math.sqrt(sum(x * x for x in w)) * dt
            if ang > 0:
                s = math.sin(ang / 2) / (ang / dt)
                q = qmul(q, (math.cos(ang / 2), w[0] * s, w[1] * s, w[2] * s))
            j += 1
        if j == i or rows[j][0] - rows[i][0] < 0.6 * WINDOW_S:
            continue
        turn = math.degrees(2 * math.acos(min(1.0, abs(q[0]))))
        if MIN_TURN_DEG <= turn <= MAX_TURN_DEG:
            pairs.append((rows[i][2], rows[j][2], qrot(q)))
    return pairs, cover


# ---------------------------------------------------------------- fit
def candidates():
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            yield tuple(tuple(signs[i] if j == perm[i] else 0 for j in range(3)) for i in range(3))


def fit(M, pairs):
    """Least-squares hard-iron offset o (mag axes) for mapping M, and the RMS
    misfit relative to the field strength. Model per pair:
        M (m2 - o) = R^T M (m1 - o)
     => (I - R^T) M o = M m2 - R^T M m1"""
    AtA = [[0.0] * 3 for _ in range(3)]
    Atb = [0.0] * 3
    rows = []
    for m1, m2, R in pairs:
        Rt = transpose(R)
        A = tuple(tuple((1 if i == j else 0) - Rt[i][j] for j in range(3)) for i in range(3))
        A = matmul(A, M)
        b = tuple(x - y for x, y in zip(matvec(M, m2), matvec(Rt, matvec(M, m1))))
        rows.append((A, b))
        for i in range(3):
            Atb[i] += sum(A[k][i] * b[k] for k in range(3))
            for j in range(3):
                AtA[i][j] += sum(A[k][i] * A[k][j] for k in range(3))
    o = solve3(AtA, Atb) or (0.0, 0.0, 0.0)
    sq = sum(sum((sum(A[i][j] * o[j] for j in range(3)) - b[i]) ** 2 for i in range(3)) for A, b in rows)
    field = sum(math.sqrt(sum((x - y) ** 2 for x, y in zip(m1, o))) for m1, _, _ in pairs) / len(pairs)
    return o, math.sqrt(sq / len(pairs)) / field, field


def mean_dip(M, o, rows):
    """Mean angle [deg] the field dips below horizontal, using the accel for
    'down' on frames that aren't accelerating much. None without such frames."""
    dips = []
    for _, _, m, a in rows:
        an = math.sqrt(sum(x * x for x in a))
        if abs(an - 1.0) > 0.1:
            continue
        b = matvec(M, tuple(x - y for x, y in zip(m, o)))
        bn = math.sqrt(sum(x * x for x in b))
        if bn > 0:
            down = sum(-ai * bi for ai, bi in zip(a, b)) / (an * bn)   # accel reads up at rest
            dips.append(math.degrees(math.asin(max(-1.0, min(1.0, down)))))
    return sum(dips) / len(dips) if dips else None


# ---------------------------------------------------------------- report
def describe_vs_imu(M):
    """How each magnetometer axis runs relative to the IMU's axes."""
    N = matmul(transpose(FIRMWARE), M)          # mag axes -> IMU axes
    out = []
    for j in range(3):                           # mag axis j
        i = next(i for i in range(3) if N[i][j])
        out.append(f"mag {AXES[j]} = {'+' if N[i][j] > 0 else '-'}IMU {AXES[i]}")
    return ", ".join(out)


def c_code(M):
    names = ("x", "y", "z")
    lines = []
    for i, field in enumerate(("mag_x", "mag_y", "mag_z")):
        j = next(j for j in range(3) if M[i][j])
        lines.append(f"    m->{field} = {'-' if M[i][j] < 0 else ''}{names[j]};")
    return ("static inline void mag_to_body(mag_data *m) {\n"
            "    float x = m->mag_x, y = m->mag_y, z = m->mag_z;\n" + "\n".join(lines) + "\n}")

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    hemi = -1 if "--southern" in sys.argv else 1
    if len(args) != 1:
        print(__doc__)
        sys.exit(1)
    rows = load(args[0])
    pairs, cover = make_pairs(rows)
    print(f"{len(rows)} frames with magnetometer data, {len(pairs)} usable sample pairs.")
    print("Turning about each body axis: " + ", ".join(
        f"{AXES[i]} {cover[i]:.0f} deg" for i in range(3)) + "  (body X = long axis)")
    if len(pairs) < 50 or min(cover) < MIN_COVER_DEG:
        print()
        print(f"Not enough turning to decide: it needs at least {MIN_COVER_DEG:.0f} deg about every axis.")
        print("Record again, slowly turning the rocket end over end, side to side and about its")
        print("long axis - a few full turns each - for about a minute.")
        sys.exit(2)

    # Every mapping has a twin with all three axes reversed that fits the
    # turning exactly as well; keep the one whose field dips the right way.
    results = []
    for M in candidates():
        o, err, field = fit(M, pairs)
        results.append((err, M, o, field, mean_dip(M, o, rows)))
    results.sort(key=lambda r: r[0])
    usable = [r for r in results if r[4] is None or r[4] * hemi > 0]
    err, best, o, field, dip = usable[0]
    runner = usable[1][0]

    print()
    print("Best fits (misfit relative to the field - lower is better):")
    for e, M, _, _, _ in usable[:4]:
        tag = "   <- firmware today" if M == FIRMWARE else ""
        mirror = " (mirrored)" if det(M) < 0 else ""
        print(f"  {e * 100:5.1f}%   {describe_vs_imu(M)}{mirror}{tag}")
    if all(M != FIRMWARE for _, M, _, _, _ in usable[:4]):
        fw = next(r for r in results if r[1] == FIRMWARE)
        print(f"  {fw[0] * 100:5.1f}%   firmware today")

    print()
    if dip is not None:
        print(f"The field dips {abs(dip):.0f} deg below horizontal by this fit "
              "(about 50-75 deg across North America and Europe).")
    else:
        print("[WARN] No steady accelerometer frames - can't tell this mapping from the same one")
        print("       with all three axes reversed.")

    print()
    if runner < CONFIDENT * err:
        print("RESULT: no mapping fits clearly better than the rest - the field was probably")
        print("        disturbed (steel, laptop, motor) or the turning too fast. Record again.")
    elif best == FIRMWARE:
        print("RESULT: the firmware's mapping is right.")
    else:
        mirrored = " (a mirrored axis set)" if det(best) < 0 else ""
        print("RESULT: the firmware's mapping is WRONG. The magnetometer is mounted")
        print(f"        {describe_vs_imu(best)}{mirrored}.")
        print("        mag_to_body() in include/mag.h should be:")
        print()
        print(c_code(best))

    off = math.sqrt(sum(x * x for x in o))
    print()
    print(f"Leftover hard-iron offset on top of the saved calibration: "
          f"({o[0]:+.3f}, {o[1]:+.3f}, {o[2]:+.3f}) G, |{off:.3f}| G, field ~{field:.3f} G")
    if off > 0.1 * field:
        print("That's large next to the Earth's field: run the bench 'M' calibration with the board")
        print("in the airframe (battery and servos fitted), away from steel.")


if __name__ == "__main__":
    main()
