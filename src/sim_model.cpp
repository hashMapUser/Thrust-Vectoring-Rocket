// Simulated rocket for the bench simulation build — see include/sim_model.h.
// Compiled only with -D SIM_MODE, so no flight build carries any of it.
#ifdef SIM_MODE

#include <math.h>
#include <string.h>
#include "sim_model.h"

// --------------------------------------------------------
// Scenarios — the menu in the simulation build, in this order
// --------------------------------------------------------
const SimScenario SIM_SCENARIOS[] = {
    { "Nominal",       "F-15, 0.775 kg (as built): launch, burnout, apogee, main, landing",
      0.775f, 1.00f, 12.0f, SIM_FAULT_NONE,      0.0f },
    { "Heavy",         "0.90 kg (+125 g): same sequence, lower apogee",
      0.90f, 1.00f, 12.0f, SIM_FAULT_NONE,      0.0f },
    { "Weak motor",    "thrust -15%: same sequence, lower apogee",
      0.775f, 0.85f, 12.0f, SIM_FAULT_NONE,      0.0f },
    { "Calm descent",  "chute barely swings: landing must not be called mid-air",
      0.775f, 1.00f,  1.0f, SIM_FAULT_NONE,      0.0f },
    { "Pad bump",      "2.2 g knock for 150 ms, no ignition: no launch, no fire",
      0.775f, 1.00f, 12.0f, SIM_FAULT_PAD_BUMP,  0.0f },
    { "Baro fails",    "baro dead from T+1 s: apogee by velocity/timeout, landing by fallback",
      0.775f, 1.00f, 12.0f, SIM_FAULT_BARO_DEAD, 1.0f },
    { "IMU fails",     "IMU dead from T+2 s: TVC off, apogee by baro/timeout, landing by fallback",
      0.775f, 1.00f, 12.0f, SIM_FAULT_IMU_DEAD,  2.0f },
    { "Watchdog reset","firmware hangs at T+3 s: reboot, resume, chute still fires",
      0.775f, 1.00f, 12.0f, SIM_FAULT_WDT_RESET, 3.0f },
};
const int SIM_SCENARIO_COUNT = sizeof(SIM_SCENARIOS) / sizeof(SIM_SCENARIOS[0]);

// --------------------------------------------------------
// Constants
// --------------------------------------------------------
static const float G          = 9.80665f;
static const float PI_F       = 3.14159265f;
static const float DEG        = PI_F / 180.0f;
static const float K_BODY     = 0.5f * 1.225f * 0.45f * 0.0062f;   // rho Cd A / 2 — 3" airframe
static const float K_CHUTE    = 0.715f * 9.80665f / (4.5f * 4.5f);  // 4.5 m/s under the main at burnout mass (775 g - 60 g)
static const float INFLATE_S  = 0.6f;
static const float PROP_KG    = 0.060f;
static const float SITE_ELEV  = 150.0f;                             // m above sea level
static const float HEADING    = 40.0f * 0.0174532925f;              // the way the rocket leans
static const float MAG_NED[3] = { 0.20f, 0.00f, 0.45f };            // Earth's field [Gauss], 66 deg dip
static const double STEP_S    = 0.001;

// Certified Estes F-15 curve (thrustcurve.org) — same table as tvc_sim
static const float F15[][2] = {
    {0.000,  0.000}, {0.148,  7.638}, {0.228, 12.253}, {0.294, 16.391},
    {0.353, 20.210}, {0.382, 22.756}, {0.419, 25.260}, {0.477, 23.074},
    {0.520, 20.845}, {0.593, 19.093}, {0.688, 17.500}, {0.855, 16.225},
    {1.037, 15.427}, {1.205, 14.948}, {1.423, 14.627}, {1.452, 15.741},
    {1.503, 14.785}, {1.736, 14.623}, {1.955, 14.303}, {2.210, 14.141},
    {2.494, 13.819}, {2.763, 13.338}, {3.120, 13.334}, {3.382, 13.013},
    {3.404,  9.352}, {3.418,  4.895}, {3.450,  0.000},
};
static const int   F15_N       = sizeof(F15) / sizeof(F15[0]);
static const float F15_IMPULSE = 49.61f;

static float thrust_n(const SimModel *m, double t) {
    if (m->sc->fault == SIM_FAULT_PAD_BUMP || t <= 0.0 || t >= F15[F15_N - 1][0]) return 0.0f;
    for (int i = 1; i < F15_N; i++) {
        if (t <= F15[i][0]) {
            float f = (float)((t - F15[i - 1][0]) / (F15[i][0] - F15[i - 1][0]));
            return m->sc->thrust_scale * (F15[i - 1][1] + f * (F15[i][1] - F15[i - 1][1]));
        }
    }
    return 0.0f;
}

// --------------------------------------------------------
// Noise
// --------------------------------------------------------
static float urand(uint32_t *s) {
    *s ^= *s << 13;  *s ^= *s >> 17;  *s ^= *s << 5;
    return (float)(*s >> 8) * (1.0f / 16777216.0f);
}

static float gauss(uint32_t *s) {
    float u1 = urand(s);
    if (u1 < 1e-7f) u1 = 1e-7f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * urand(s));
}

// --------------------------------------------------------
// Attitude — the lean (tilt from vertical, as a north/east vector in
// degrees) and the spin about the airframe, as functions of time. Every
// piece joins the next one continuously, so the gyro never sees a jump.
// --------------------------------------------------------
static float smoothstep01(float x) {
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;
    return x * x * (3.0f - 2.0f * x);
}

static void lean_at(const SimModel *m, double t, float *ln, float *le) {
    const float hn = cosf(HEADING), he = sinf(HEADING);   // along the lean
    const float pn = -he,           pe = hn;              // across it

    if (m->t_touchdown >= 0.0 && t >= m->t_touchdown) {    // falls over onto its side
        float s   = smoothstep01((float)(t - m->t_touchdown) / 0.8f);
        float mag = hypotf(m->td_lean_n, m->td_lean_e);
        float un  = mag > 1.0f ? m->td_lean_n / mag : hn;
        float ue  = mag > 1.0f ? m->td_lean_e / mag : he;
        *ln = m->td_lean_n + (85.0f * un - m->td_lean_n) * s;
        *le = m->td_lean_e + (85.0f * ue - m->td_lean_e) * s;
        return;
    }
    if (m->t_deploy >= 0.0 && t >= m->t_deploy) {          // swinging under the chute
        float tc    = (float)(t - m->t_deploy);
        float decay = expf(-tc / 0.7f);
        float grow  = 1.0f - expf(-tc / 0.5f);
        float a     = m->sc->swing_deg * grow * sinf(2.0f * PI_F * 0.50f * tc);
        float c     = 0.5f * m->sc->swing_deg * grow * sinf(2.0f * PI_F * 0.43f * tc);
        *ln = m->dep_lean_n * decay + a * hn + c * pn;
        *le = m->dep_lean_e * decay + a * he + c * pe;
        return;
    }
    if (m->t_liftoff >= 0.0 && t >= m->t_liftoff) {        // launch wobble, then a gravity turn
        float tau = (float)(t - m->t_liftoff);
        float a   = 0.5f + 2.0f * (1.0f - expf(-tau / 0.4f))
                  + 3.0f * expf(-tau / 1.2f) * sinf(2.0f * PI_F * 1.1f * tau)
                  + 4.0f * fmaxf(0.0f, (float)t - SIM_BURN_S);
        float c   = 1.5f * expf(-tau / 1.0f) * sinf(2.0f * PI_F * 0.9f * tau);
        *ln = a * hn + c * pn;
        *le = a * he + c * pe;
        return;
    }
    *ln = 0.5f * hn;                                       // on the pad: rail cant
    *le = 0.5f * he;
}

static float spin_deg_at(const SimModel *m, double t) {
    if (m->t_liftoff < 0.0 || t < m->t_liftoff) return 0.0f;
    double end  = m->t_touchdown >= 0.0 ? m->t_touchdown : 1e9;
    double dep  = m->t_deploy    >= 0.0 ? m->t_deploy    : 1e9;
    double tt   = t < end ? t : end;
    if (tt <= dep) return (float)(20.0 * (tt - m->t_liftoff));
    return (float)(20.0 * (dep - m->t_liftoff) + 6.0 * (tt - dep));
}

// Quaternion (w, x, y, z) of the attitude filter's frame — X/Y horizontal,
// Z along the airframe toward the nozzle — relative to NED, as the
// firmware's filter would hold it: a swing toward the lean, then the spin.
static void quat_at(const SimModel *m, double t, float q[4]) {
    float ln, le;
    lean_at(m, t, &ln, &le);
    float th  = hypotf(ln, le) * DEG;
    float phi = atan2f(le, ln);
    float s   = sinf(th / 2.0f);
    float sw0 = cosf(th / 2.0f), sw1 = sinf(phi) * s, sw2 = -cosf(phi) * s;
    float psi = spin_deg_at(m, t) * DEG;
    float ct  = cosf(psi / 2.0f), st = sinf(psi / 2.0f);
    q[0] = sw0 * ct;
    q[1] = sw1 * ct + sw2 * st;
    q[2] = sw2 * ct - sw1 * st;
    q[3] = sw0 * st;
}

// v_sensor = R(q)^T v_ned
static void to_sensor(const float q[4], const float v[3], float out[3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y);
    float r10 = 2 * (x * y + w * z), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x);
    float r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y);
    out[0] = r00 * v[0] + r10 * v[1] + r20 * v[2];
    out[1] = r01 * v[0] + r11 * v[1] + r21 * v[2];
    out[2] = r02 * v[0] + r12 * v[1] + r22 * v[2];
}

// Filter frame -> chip axes. The firmware maps chip -> body
// (lsm6dsox_to_body / mag_to_body) and body -> filter (mahrs_tick); this
// is those two steps run backwards: chip = (-f.y, -f.z, f.x).
static void to_chip(const float f[3], float *cx, float *cy, float *cz) {
    *cx = -f[1];
    *cy = -f[2];
    *cz =  f[0];
}

// --------------------------------------------------------
// Dynamics
// --------------------------------------------------------
static float chute_k(const SimModel *m, double t) {
    if (m->t_deploy < 0.0) return 0.0f;
    float f = (float)((t - m->t_deploy) / INFLATE_S);
    return K_CHUTE * (f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f));
}

static void step(SimModel *m, double dt) {
    double t = m->t;
    float  T = thrust_n(m, t);
    if (t >= 0.0) {
        m->impulse += T * (float)dt;
        float burnt = m->impulse / (F15_IMPULSE * m->sc->thrust_scale);
        m->mass = m->sc->mass_kg - PROP_KG * (burnt < 1.0f ? burnt : 1.0f);
    }

    switch (m->phase) {
        case SIM_PAD:
            if (T > m->mass * G) {
                m->phase     = SIM_FLIGHT;
                m->t_liftoff = t;
            }
            break;

        case SIM_FLIGHT:
        case SIM_CHUTE: {
            float ln, le;
            lean_at(m, t, &ln, &le);
            float cos_tilt = cosf(hypotf(ln, le) * DEG);
            float k = K_BODY + chute_k(m, t);
            float a = (T * cos_tilt - k * m->v * fabsf(m->v)) / m->mass - G;
            m->v += a * (float)dt;
            m->h += m->v * (float)dt;
            if (m->t_apogee < 0.0 && m->v <= 0.0f && t > m->t_liftoff + 0.5) {
                m->t_apogee = t;
                m->apogee_m = m->h;
            }
            if (m->h <= 0.0f && t > m->t_liftoff + 0.5) {
                lean_at(m, t, &m->td_lean_n, &m->td_lean_e);
                m->h = 0.0f;
                m->v = 0.0f;
                m->phase       = SIM_GROUND;
                m->t_touchdown = t;
            }
            break;
        }

        case SIM_GROUND:
            break;
    }
    m->t = t + dt;
}

// --------------------------------------------------------
// Public API
// --------------------------------------------------------
void sim_model_init(SimModel *m, const SimScenario *sc, uint32_t seed) {
    memset(m, 0, sizeof(*m));
    m->sc          = sc;
    m->t           = -1000.0;
    m->mass        = sc->mass_kg;
    m->phase       = SIM_PAD;
    m->t_liftoff   = -1.0;
    m->t_apogee    = -1.0;
    m->t_deploy    = -1.0;
    m->t_touchdown = -1.0;
    m->rng         = seed ? seed : 1u;
}

void sim_model_advance(SimModel *m, double t) {
    if (m->phase == SIM_PAD && m->t < 0.0) m->t = t < 0.0 ? t : 0.0;   // nothing moves before ignition
    while (m->t < t - 1e-9) {
        double dt = t - m->t;
        step(m, dt < STEP_S ? dt : STEP_S);
    }
}

void sim_model_deploy(SimModel *m) {
    if (m->phase != SIM_FLIGHT) return;
    lean_at(m, m->t, &m->dep_lean_n, &m->dep_lean_e);
    m->t_deploy = m->t;
    m->phase    = SIM_CHUTE;
}

void sim_model_sensors(SimModel *m, SimSensors *out) {
    double t = m->t;
    float q[4], q_prev[4];
    quat_at(m, t, q);
    quat_at(m, t - 0.001, q_prev);

    // Specific force (what an accelerometer feels) in NED, m/s^2
    float f[3] = { 0.0f, 0.0f, -G };                    // resting on the pad or the ground
    if (m->phase == SIM_FLIGHT || m->phase == SIM_CHUTE) {
        float ln, le;
        lean_at(m, t, &ln, &le);
        float th = hypotf(ln, le) * DEG, phi = atan2f(le, ln);
        float T  = thrust_n(m, t) / m->mass;
        float k  = (K_BODY + chute_k(m, t)) * m->v * fabsf(m->v) / m->mass;
        f[0] = T * cosf(phi) * sinf(th);                // thrust along the nose
        f[1] = T * sinf(phi) * sinf(th);
        f[2] = -T * cosf(th) + k;                       // drag opposes the vertical motion
    } else if (m->phase == SIM_PAD && m->sc->fault == SIM_FAULT_PAD_BUMP && t >= 0.0 && t < 0.15) {
        f[2] = -2.2f * G;                               // the knock
    } else if (m->phase == SIM_GROUND && t - m->t_touchdown < 0.03) {
        f[2] = -4.0f * G;                               // impact
    }
    float fs[3];
    to_sensor(q, f, fs);
    for (int i = 0; i < 3; i++) fs[i] /= G;
    to_chip(fs, &out->ax, &out->ay, &out->az);
    out->ax += 0.008f * gauss(&m->rng);
    out->ay += 0.008f * gauss(&m->rng);
    out->az += 0.008f * gauss(&m->rng);

    // Body rates from the attitude change over the last millisecond
    float w0 = q_prev[0], x0 = -q_prev[1], y0 = -q_prev[2], z0 = -q_prev[3];   // conjugate
    float dw = w0 * q[0] - x0 * q[1] - y0 * q[2] - z0 * q[3];
    float dx = w0 * q[1] + x0 * q[0] + y0 * q[3] - z0 * q[2];
    float dy = w0 * q[2] - x0 * q[3] + y0 * q[0] + z0 * q[1];
    float dz = w0 * q[3] + x0 * q[2] - y0 * q[1] + z0 * q[0];
    float sgn = dw < 0.0f ? -1.0f : 1.0f;
    float rate[3] = { sgn * dx * 2.0f / 0.001f / DEG,
                      sgn * dy * 2.0f / 0.001f / DEG,
                      sgn * dz * 2.0f / 0.001f / DEG };
    to_chip(rate, &out->gx, &out->gy, &out->gz);
    out->gx += 0.12f * gauss(&m->rng);
    out->gy += 0.12f * gauss(&m->rng);
    out->gz += 0.12f * gauss(&m->rng);

    float ms[3];
    to_sensor(q, MAG_NED, ms);
    to_chip(ms, &out->mx, &out->my, &out->mz);
    out->mx += 0.002f * gauss(&m->rng);
    out->my += 0.002f * gauss(&m->rng);
    out->mz += 0.002f * gauss(&m->rng);

    float asl = SITE_ELEV + m->h;
    out->pressure_hpa  = 1013.25f * powf(1.0f - 2.25577e-5f * asl, 5.25588f) + 0.012f * gauss(&m->rng);
    out->temperature_c = 21.5f - 0.0065f * m->h + 0.01f * gauss(&m->rng);
}

#endif // SIM_MODE
