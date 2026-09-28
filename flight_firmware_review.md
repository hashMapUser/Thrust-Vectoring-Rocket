# TVC Flight Firmware Review: Issues and Recommended Fixes

**Repo:** `hashMapUser/Thrust-Vectoring-Rocket` @ `1f0c150`
**Scope:** `env:flight` (`main_control_loop.cpp`, `flight_sm.*`, `pyro.*`, `alt_estimator.*`, `lsm6dsox.*`), plus `tvc_sim` and the shared parts of `env:finned`
**Hardware context:** the board now has the SW401 arming switch. ARM SENSE reads about 0.32 × pack voltage when armed and 0 V when safe.

---

## Assumptions behind the numbers

The flight numbers below come from a 1-D vertical simulation using the values in `tvc_sim`:

| Parameter | Value |
|---|---|
| Liftoff mass | 0.90 kg |
| Motor | Estes F-15, full thrustcurve.org curve (≈50 N·s, ≈3.45 s burn), 60 g propellant |
| Drag | C_d = 0.45, 3" airframe (A = 0.0062 m²) |

**Rerun these with the real liftoff mass and motor before flight.** The failures below are threshold problems, so they shift with thrust-to-weight but do not go away.

| Case | Peak accel reading | Peak velocity | Apogee | Time to apogee |
|---|---|---|---|---|
| Nominal (0.90 kg) | 2.88 g | 24.0 m/s | 71 m | 5.7 s |
| Heavy (1.00 kg) | 2.59 g | 18.8 m/s | 52 m | 5.3 s |
| Weak motor (−15% thrust) | 2.45 g | 16.3 m/s | 43 m | 5.0 s |

"Accel reading" is what the IMU actually reports, which is specific force: (thrust − drag) / mass. It is **not** 1 g plus the acceleration.

---

## Summary

| # | Issue | Severity | Effect as written |
|---|---|---|---|
| 1 | Launch threshold (4.0 g) above what the vehicle produces | **Critical** | Launch never detected: no TVC, no apogee, no chute |
| 2 | Launch altitude confirmation window too short, and accel must stay high throughout | **Critical** | Real launches rejected as "false trigger" |
| 3 | TVC only enabled after launch is *confirmed* | **Critical** | No control for the first ~0.5–1 s of an unstable rocket |
| 4 | Apogee gates (25 m/s, edge-triggered crossing, timeout gated too) | **Critical** | Apogee never detected; chute never fires |
| 5 | 50 m minimum altitude for main deploy, never retried | **Critical** | Chute blocked on any flight under 50 m |
| 6 | Pyro "fired" flags in EEPROM are never cleared | **Critical** | After one fire, the chute is blocked on every later flight |
| 7 | Arming switch not integrated in firmware | **High** | Boot continuity check always reads OPEN; no pad status; stale design |
| 8 | IMU validity never checked; abort path disables recovery | **High** | An IMU failure goes undetected, and a detected one would disable the chute |
| 9 | `tvc_sim` doesn't match the firmware | **High** | PID gains tuned on a different plant and loop rate |
| 10 | Flight log exists only in RAM until landing | **Medium** | Power loss at landing loses the whole log; 32 s window is marginal |
| 11 | A mid-flight reset restarts the FSM on the pad | **Medium** | A brownout or watchdog reset in flight means no deployment |
| 12 | PID output units vs. sim nozzle units | **Medium (verify)** | Gains may be scaled by the linkage ratio |
| 13 | Stale config and docs | **Low** | Confusion and wrong assumptions later |

Issues 1, 2, 4, 5 and 6 each prevent chute deployment on their own, so all of them need fixing before flight.

---

## 1. Launch threshold is above what the vehicle produces (Critical)

**Where:** `flight_sm.h` → `LAUNCH_ACCEL_THRESHOLD_G 4.0f`

**Problem:** The header assumes "model motors deliver 5–15 g," but that's for light rockets. At 0.9 kg on an F-15, the accelerometer peaks at about **2.9 g** and sustains about **1.6 g**. It never reaches 4.0 g, so the FSM sits in IDLE for the whole flight.

**Fix:** Set the threshold between the pad-rest ceiling (1.10 g) and the worst-case peak (≈2.45 g):

```c
#define LAUNCH_ACCEL_THRESHOLD_G   1.8f   // F-15 @ 0.9 kg: peak ~2.9 g, sustain ~1.6 g
#define LAUNCH_ACCEL_MS            100    // accel must hold this long (bump rejection)
```

Nominally, the reading stays above 1.8 g for about 640 ms. The altitude confirmation in item 2 is what rejects bumps and knocks, so the accel hold time can be short.

---

## 2. Launch confirmation rejects real launches (Critical)

**Where:** `flight_sm.cpp` → `STATE_IDLE` case; `LAUNCH_ALT_DELTA_M 3.0f`, `LAUNCH_CONFIRM_MS 300`

**Problem:** There are two issues.

1. At this thrust-to-weight, the rocket is only about **0.9 m** up 300 ms after the accel trigger. It needs about 0.8 s to climb 3 m. The code declares a "FALSE LAUNCH TRIGGER" and clears the pad-rest latch, which can never re-latch while the rocket is flying.
2. The accel must stay above threshold for the *entire* window, because the `else` branch resets `launch_detect_ms`. Once thrust settles to the sustain level, a lower threshold would still drop out.

**Fix:** Require the accel hold only for `LAUNCH_ACCEL_MS`, then wait up to 1 s for the altitude gain:

```c
#define LAUNCH_ALT_DELTA_M   2.0f    // nominal: ~1.9 m at +0.5 s, ~6 m at +1.0 s
#define LAUNCH_CONFIRM_MS    1000    // weak-motor case still reaches ~4.8 m by +1.0 s

// Returns true when liftoff is confirmed. Call from IDLE and ARMED (see item 7).
static bool check_launch(FlightSM *fsm, float accel_up_g, float altitude_m, uint32_t now) {
    if (fsm->launch_detect_ms == 0) {
        if (fsm->pad_rest_satisfied && accel_up_g > LAUNCH_ACCEL_THRESHOLD_G) {
            fsm->launch_detect_ms = now;
            fsm->tvc_enabled      = true;      // item 3: steer from first motion
        }
        return false;
    }

    uint32_t t = now - fsm->launch_detect_ms;

    // Accel only has to hold for the short hold time.
    if (t < LAUNCH_ACCEL_MS && accel_up_g < LAUNCH_ACCEL_THRESHOLD_G) {
        fsm->launch_detect_ms = 0;
        fsm->tvc_enabled      = false;         // just a bump
        return false;
    }

    if ((altitude_m - fsm->pad_rest_baseline_alt_m) >= LAUNCH_ALT_DELTA_M)
        return true;                           // caller enters STATE_POWERED

    if (t >= LAUNCH_CONFIRM_MS) {              // no climb: false trigger
        fsm->launch_detect_ms   = 0;
        fsm->tvc_enabled        = false;
        fsm->pad_rest_satisfied = false;
    }
    return false;
}
```

---

## 3. TVC stays off during the slowest, least stable part of the flight (Critical)

**Where:** `flight_sm.cpp` sets `tvc_enabled = true` only on `LIFTOFF CONFIRMED`; `main_control_loop.cpp` runs the PID only when `state == STATE_POWERED`.

**Problem:** The static margin is −3.24 cal, so the airframe is unstable by design. Even with item 2 fixed, confirmation takes about 0.5–0.8 s. That's the lowest-velocity part of the flight, with the least aerodynamic damping, and it's when a tip-over is most likely to start. The rocket shouldn't fly it uncontrolled.

**Fix:** Enable TVC at the first accel trigger (already in the code for item 2), and gate the control loop on `tvc_enabled` instead of the state:

```c
// main_control_loop.cpp, section 5
static bool tvc_prev = false;
if (fsm.tvc_enabled && !tvc_prev) { pid_reset(&pid_pitch); pid_reset(&pid_yaw); }
tvc_prev = fsm.tvc_enabled;

if (fsm.tvc_enabled) {
    pitch_cmd = pid_update(&pid_pitch, 0.0f, attitude.tip_a, dt);
    yaw_cmd   = pid_update(&pid_yaw,   0.0f, attitude.tip_b, dt);
    servo_set_pitch(pitch_cmd);
    servo_set_yaw(yaw_cmd);
} else {
    servo_center();
}
```

If a bump causes a false trigger, TVC is active for at most `LAUNCH_ACCEL_MS` (100 ms). On a rocket sitting on the pad, that just twitches the gimbal.

---

## 4. Apogee is never detected (Critical)

**Where:** `flight_sm.cpp` → `STATE_COAST`; `MIN_FLIGHT_VELOCITY_MS 25.0f`, `COAST_APOGEE_MIN_MS 1500`, `APOGEE_TIMEOUT_MS 8000`

**Problems:**

1. **G3 can't be met.** Peak velocity is 16–24 m/s across the cases above, and the gate needs more than 25 m/s. Both the velocity crossing and the timeout are gated on G3, so nothing ever triggers.
2. **The crossing is edge-triggered.** `prev_velocity > 0 && velocity <= 0` only fires on the exact tick the sign flips. If that happens before the G4 time gates open, it's missed for good. In the weak-motor case, coast lasts only about 1.6 s, barely above `COAST_APOGEE_MIN_MS`.
3. **The timeout counts from coast entry, not from launch.** Burnout (~3.4 s) plus the 8 s timeout puts it at about 11.4 s after launch. A 43 m flight hits the ground before that.
4. **Velocity comes only from integrated acceleration.** `alt_update()` never corrects velocity with the barometer, so there is no independent apogee check.

**Fix:** Use lower gates, level-triggered checks, a baro backup, and a timeout measured from launch. Once the FSM is in COAST, launch has already been confirmed, so the timeout doesn't need any extra gates.

```c
#define MIN_FLIGHT_VELOCITY_MS          10.0f  // all cases reach 16+ m/s
#define COAST_APOGEE_MIN_MS             500
#define APOGEE_BARO_DROP_M              3.0f   // independent of integrated velocity
#define APOGEE_TIMEOUT_FROM_LAUNCH_MS   7000   // nominal apogee ~5.0-5.7 s after ignition

case STATE_COAST: {
    if (velocity_ms > fsm->peak_velocity_ms) fsm->peak_velocity_ms = velocity_ms;
    if (altitude_m  > fsm->peak_altitude_m)  fsm->peak_altitude_m  = altitude_m;   // new field

    uint32_t since_launch = now - fsm->powered_entry_ms;
    bool gate = (fsm->peak_velocity_ms > MIN_FLIGHT_VELOCITY_MS) &&
                (fsm_time_in_state(fsm) >= COAST_APOGEE_MIN_MS);

    bool vel_apogee  = gate && (velocity_ms <= 0.0f);                          // level, not edge
    bool baro_apogee = gate && (fsm->peak_altitude_m - altitude_m) >= APOGEE_BARO_DROP_M;
    bool timeout     = since_launch >= APOGEE_TIMEOUT_FROM_LAUNCH_MS;          // no extra gates

    if (vel_apogee || baro_apogee || timeout) enter_state(fsm, STATE_APOGEE);
    break;
}
```

Log which of the three conditions fired, so you can see after the flight whether the estimator or the backup caught apogee.

---

## 5. The 50 m deploy floor can silently cancel the chute (Critical)

**Where:** `pyro.h` → `PYRO_MAIN_MIN_ALT_M 50.0f`; `pyro_fire_main()`; also used by `simple_fsm.cpp` (finned build)

**Problem:** Apogee is 43–71 m across the cases above. Below 50 m, `pyro_fire_main()` prints "blocked" and returns. It's only called once, on entry to `STATE_MAIN`, so it's **never retried**. The floor is meant to keep a dual-deploy main from firing high. On a single-deploy rocket it only adds a way to lose the vehicle. The finned build shares this code, so it has the same exposure.

**Fix:** Remove the floor for single-deploy. The FSM only reaches apogee after a confirmed launch, so that already protects against firing on the ground.

```c
// pyro.h: single-deploy at apogee; ground protection comes from the FSM gates
#define PYRO_MAIN_MIN_ALT_M   0.0f
```

If you want a floor for a future dual-deploy setup, keep it for the *main* at low altitude, never for the apogee charge. And retry every loop until it fires, not just once.

---

## 6. Pyro "fired" flags are never cleared (Critical)

**Where:** `pyro.cpp` → `eeprom_save_fired()` / `eeprom_load_fired()`. No code path ever writes `false` back.

**Problem:** Persisting the flags across resets is a good idea, since it stops a mid-flight brownout from re-firing a spent channel. But nothing clears them. After a single fire, including a bench test with a bulb or a finned flight (which uses the same `pyro.cpp`), `pyro_fire_main()` returns "Main already fired" on **every later flight** with that Teensy. That may already be true of your board.

**Fix:** Clear the flags on the arming switch's **off → on edge**, observed after boot, while on the pad. Turning the switch on at the pad means a new flight is starting. A mid-flight reboot comes up with the switch *already on*, so there's no edge, and the flags stay set, which is what you want.

```c
// pyro.cpp
void pyro_clear_fired(PyroState *pyro) {
    pyro->drogue_fired = false;
    pyro->main_fired   = false;
    eeprom_save_fired(pyro);
    Serial.println("[PYRO] Fired flags cleared: new flight armed");
}
```

Also print the flags loudly at boot, with a distinct buzzer pattern if either is set. Add a manual clear command to the bench harness and GUI for testing.

---

## 7. Arming switch integration (High)

**Current code:** It was written before SW401 existed. `pyro_arm()` is called unconditionally at boot. `STATE_ARMED` is unused. The comments say "no arming mechanism on this board." The boot continuity check calls `read_pack_voltage()` through ARM SENSE, which reads 0 V when the switch is off. So at boot that check reports **OPEN** every time, and the pad has no real armed or continuity status.

**Recommended design:** The hardware switch is the real safety interlock, since it physically cuts PYRO PWR. The firmware should *report* the switch state, but never *block firing* because of it. A flaky ARM SENSE reading must not be able to stop a deployment.

| Condition | Firmware action |
|---|---|
| Boot, switch **off** | State IDLE, `BUZZ_IDLE`. Remember that "off" was seen since boot. |
| Boot, switch **on** (possible mid-flight reset) | Stay software-armed and **keep** the EEPROM fired flags. No continuity status until pad rest. |
| Switch **off → on**, in IDLE on the pad | `pyro_clear_fired()` (item 6), then a continuity check, then state ARMED. `BUZZ_ARMED` if continuity is OK; a distinct "continuity open" pattern if not. |
| While ARMED on the pad | Recheck continuity every ~1 s and update the buzzer and LEDs so the pad crew knows the state without a laptop. |
| Switch **on → off**, on the pad | Back to IDLE and `BUZZ_IDLE`. |
| Any time after launch | **Ignore** ARM SENSE for decisions; only log it. Vibration or contact bounce must not change behavior. |

Additional rules:

- **Launch detection runs in both IDLE and ARMED.** If someone forgets to arm, the motor still lights and the rocket still flies, so TVC must still steer. Log a "launched disarmed" warning.
- **Debounce ARM SENSE** (about 100 ms) and use a threshold well below the nominal reading. A 2S pack gives 1.9–2.7 V at ARM SENSE when armed, so about **1.2 V** is a good threshold.
- **Continuity is only meaningful when armed**, because the sense divider needs PYRO PWR. Don't check it at boot unless ARM SENSE already reads armed.
- Pack voltage for the continuity math comes from ARM SENSE while armed. That part of the existing code is correct; it's just called at the wrong time.
- Update the stale "no arming mechanism" comments in `main_control_loop.cpp`, `pyro.h` and `simple_fsm.h`, and give the finned build the same arming behavior.

```c
// Sketch of the pad-side logic in loop()
#define ARM_SENSE_ARMED_V   1.2f
#define ARM_DEBOUNCE_MS     100

bool arm_raw = (analogRead(PIN_ARM_SENSE) * 3.30f / 4095.0f) > ARM_SENSE_ARMED_V;
// ...debounce into arm_now...

bool on_pad = (fsm.state == STATE_IDLE || fsm.state == STATE_ARMED) && fsm.launch_detect_ms == 0;

if (!arm_now) seen_disarmed = true;

if (on_pad && arm_now && !arm_prev && seen_disarmed) {       // new flight armed
    pyro_clear_fired(&pyros);
    cont_ok = pyro_check_continuity(PIN_PYRO1_SENSE, read_pack_voltage());
    fsm_set_armed(&fsm, true);                                // IDLE -> ARMED
    buzzer_set(&buzz, cont_ok ? BUZZ_ARMED : BUZZ_CONT_OPEN); // new pattern
}
if (on_pad && !arm_now && arm_prev) {
    fsm_set_armed(&fsm, false);                               // ARMED -> IDLE
    buzzer_set(&buzz, BUZZ_IDLE);
}
arm_prev = arm_now;
```

Also move the pad-rest `BUZZ_ARMED` in section 1 of `loop()`. Right now it plays the "ordnance live" pattern based on pad rest, not the switch, so it gives false information.

---

## 8. IMU failures aren't detected, and the abort path would disable the chute (High)

**Where:** `lsm6dsox_read()` always sets `valid = true`; `fsm_update()` latches `imu_fault` on a single invalid sample, then `STATE_ABORT` calls `pyro_safe_all()`.

**Problems:**

1. A dead or disconnected IMU returns 0xFF or 0x00 on every byte, which decodes to values near zero, and the code accepts them as real data. Near-zero acceleration looks like free fall to the FSM.
2. If validity *were* checked, one glitched read would latch a permanent fault, and the abort would **disable the pyros**. The rocket would come down without a chute.

**Fix:**

- **In the driver:** flag a sample invalid if all 12 bytes are 0xFF or all are 0x00, or if the sample is bit-identical to the previous one several times in a row (a frozen sensor). Only report a fault after N consecutive bad samples (for example 10, which is 80 ms), and optionally re-check WHO_AM_I at that point.
- **In the FSM:** an in-flight IMU fault should mean **"stop steering,"** not "stop recovery." Center the servos and disable TVC, but keep apogee detection running on baro drop and the launch-referenced timeout from item 4, and still fire the chute. Only call `pyro_safe_all()` for an abort *on the pad*.

---

## 9. `tvc_sim` doesn't match the firmware (High)

**Problems:**

| Sim | Firmware / reality |
|---|---|
| F-15 curve ends at 1.74 s (≈24 N·s) | The real F-15 burns ≈3.45 s (≈50 N·s). Gains were tuned on only the first half of the burn. |
| `PIDDT = 0.005` (200 Hz) | Firmware loop is 8 ms (125 Hz). I and D terms and phase margin depend on the rate. |
| `absMaxTVCAngle = 7°` | `PID_OUT_MAX = 5` (see item 12 for units) |
| Constant mass and MOI | About 60 g of propellant burns off, and the CG moves forward. It's a small effect, but it changes the lever arm. |

**Fix:** Load the full F-15 curve from thrustcurve.org, set `PIDDT = 0.008`, match the output limit, add mass and CG change, then retune. Also add servo PWM quantization (200 Hz) to the sim.

---

## 10. The flight log exists only in RAM until landing (Medium)

**Where:** `logger.h`, a 4000-record RAM ring buffer (≈32 s at 125 Hz) that's written to SD only at `STATE_LANDED`.

**Problem:** If the battery disconnects on landing impact, or anything resets the board, the entire flight log is lost. The ring buffer also only holds the last 32 s. The nominal flight takes about 5.7 s to apogee, plus 12–15 s under the chute, plus the 5 s landing hold. That's close enough that a slower descent overwrites liftoff.

**Fix:** After the flash bodge (MISO/MOSI swap), stream the log to the GD25Q128 during flight and copy it to SD after landing.

- **Pre-erase the log region when the arming switch turns on** (the item 7 edge). A 64 KB block erase takes ~150 ms, so 1 MB (~65 s of log) takes a few seconds. After that, flight writes are only page programs of about 0.5 ms, which never stall the control loop.
- Buffer records in RAM and write one 256-byte page per loop at most.

---

## 11. A mid-flight reset restarts the FSM on the pad (Medium)

**Problem:** The 500 ms watchdog, or a brownout from servo current, reboots the Teensy into `STATE_IDLE`. In flight, pad rest never latches, so apogee is never detected and the chute never fires. The EEPROM fired flags protect against *re-firing*, but nothing protects against *never firing*.

**Fix:** At launch confirmation, save an "in flight" marker and the launch timestamp to EEPROM. Clear the marker at landing and on the arming edge. At boot, if the marker is set and the rocket isn't at rest, skip pad logic and go straight to a recovery-only mode: baro-drop apogee detection plus a timeout, then fire. Test this on the bench by forcing a reset during a simulated flight.

---

## 12. PID output units vs. sim nozzle units (Medium, verify)

**Where:** `servo_driver.cpp` maps angles at about 5.56 µs per degree of *servo horn* rotation (±40.5° → ±225 µs). The sim's output is *nozzle* deflection.

**Problem:** If the linkage between servo and gimbal isn't 1:1, the effective loop gain is scaled by that ratio, and the tuned gains don't carry over.

**Fix:** Command the servo to ±10° and measure the gimbal deflection. Add a `SERVO_PER_GIMBAL_RATIO` constant so the PID works in gimbal degrees, matching the sim. Then set `PID_OUT_MAX` to the real gimbal limit.

---

## 13. Stale config and docs (Low)

- `BUZZER_FREQ_HZ 2500` says "CMT-1203 nominal," but the board uses an SMT-0840-T. Set it to the frequency you measured with the sweep.
- `FC_V2 Pinout.txt` disagrees with `board_pins.h` (for example, the pyro pins). Delete it or regenerate it from `board_pins.h`.
- The flash comment in `logger.h` should mention the bodge fix once it's done.
- `STATE_ARMED` comments ("unused, never transitions into it") change with item 7.

---

## Recommended order of work

1. **Items 1, 2, 3, 4, 5, 6.** Each of these alone prevents deployment or control.
2. **Item 7** (arming switch). It builds on item 6 for clearing the flags.
3. **Item 8** (IMU validity and abort behavior).
4. **Item 9** (sim fidelity), then retune the gains.
5. **Items 10–13**, time permitting.

## Verification: `SIM_MODE` test matrix

Once the fixes are in, run each case through a simulated-sensor build and check the expected result:

| Test case | Expected result |
|---|---|
| Nominal F-15, 0.9 kg | Launch confirmed within ~0.6 s of ignition; TVC active from first motion; apogee via velocity at ~5.7 s; chute fires |
| Weak motor (−15%) and heavy (+10%) | Same sequence; chute fires below 50 m |
| Pad bump (1 g spike, no climb) | TVC twitches under 100 ms; false-trigger reset; pad rest re-latches |
| Launched with switch off | TVC flies normally; "launched disarmed" logged; no fire (no PYRO PWR) |
| Switch off → on on the pad | Fired flags cleared, continuity checked, ARMED buzzer |
| Board rebooted with switch already on | Fired flags **kept**, no clear |
| ARM SENSE bounces during boost | No behavior change; logged only |
| IMU dead mid-burn | TVC stops and centers; baro/timeout apogee; chute fires |
| Baro dead | Velocity apogee still fires; timeout backstop covers it |
| Reset at t = 3 s | Recovery-only mode; chute fires on baro drop or timeout |
| Bench fire with bulb, then re-arm | Second fire works (flags cleared by the arming edge) |
