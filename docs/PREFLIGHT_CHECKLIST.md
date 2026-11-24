# TVC Rocket — Preflight Checklist

Print this and tick it off. **A box you can't tick is a no-go.** Steps in *italics* need a laptop on USB.

---

## A. Before field day (at home, after any firmware or hardware change)

### Firmware and gains
- [ ] The commit you'll fly is tagged (e.g. `git tag flight-1`), so you know exactly which firmware and gains flew.
- [ ] `tvc_sim` matches the rocket as built:
  - [ ] `mass` within 50 g of the loaded rocket on a scale (currently 0.775 kg, motor included).
  - [ ] `MOI` from CAD **with the loaded motor in the model** (currently 0.01481 kg·m²).
  - [ ] Per-axis `lever` in `AXES` = gimbal pivot to CG for that axis (currently pitch 140.6 mm, yaw 182.6 mm). Confirm which CAD arm drives which servo: X arm → pin 5 (yaw), Z arm → pin 6 (pitch).
  - [ ] `distanceCPToCOM` = (nose tip → CG) − CP depth in the nose. No fins, so the CP is the nose cone's: parabolic 130 mm → 65 mm from the tip. Currently 300.47 − 65 = 235.5 mm. Redo it if the CG moves (battery, ballast).
  - [ ] `rocketCrossSectionalArea` matches the 82.55 mm OD (0.005352 m²).
  - [ ] `servoSecPer60` from the servo datasheet (0.12 s is an estimate). The pitch nozzle is the slow one, so this matters.
- [ ] PID gains in `include/pid.h` re-tuned in `tvc_sim` after the items above. Current gains: Kp 0.8 / Ki 0.215 / Kd 0.10, D filter 0.5, tuned with the CAD MOI, lever arms and CP (235 mm ahead of the CG). Still to confirm: servo speed from the datasheet. The old gains (0.34 / 0.215 / 0.05) tip the rocket over in simulation.
- [ ] `alpha` in `tvc_sim` equals `PID_D_FILTER_ALPHA` in `include/pid.h`.
- [ ] The tuned gains hold in the stress cases: MOI ±20%, sensor noise ×4, servo 2× slower, 3° nozzle slop, CP ±50%, both axes, several noise seeds.

### Bench tests — on the flight board (`pio run -e bench -t upload`)
- [ ] **1, 2, 3**: baro, IMU and magnetometer all PASS.
- [ ] **4, 9**: flash PASS (9 shows 20/20 at 10 MHz).
- [ ] **5, 6**: SD card PASS.
- [ ] **I**: on both axes the nozzle swings **away from** the lean. If it swings toward the lean, flip that axis's `SERVO_*_INVERT` in `include/servo_driver.h` and repeat.
- [ ] **M**: magnetometer calibrated on this board (redo it after any board or wiring change).
- [ ] **A**: the arming-switch reading is about 2.7 V closed and 0 V open.
- [ ] **C**: continuity OK with an e-match (or a bulb) connected.
- [ ] **F**: test fire into a bulb or a spare e-match — it fires.

### Bench simulation — the real flight code with simulated sensors (`pio run -e flight_sim -t upload`)
Pyro outputs are never driven in this build, but **disconnect the e-match anyway**. The servos move.
For each scenario:
1. Open a serial monitor and wait for `[FSM] PAD REST SATISFIED`.
2. Send the scenario number, then `L`.
3. Wait for the `SIMULATION REPORT`.
4. Send `Z` to reboot for the next scenario.

- [ ] 1 Nominal — PASS *(run this one with SW401 closed too)*
- [ ] 2 Heavy — PASS
- [ ] 3 Weak motor — PASS
- [ ] 4 Calm descent — PASS (no touchdown called mid-air)
- [ ] 5 Pad bump — PASS (no launch, no fire)
- [ ] 6 Baro fails — PASS (the report takes up to ~2 min)
- [ ] 7 IMU fails — PASS (up to ~2 min)
- [ ] 8 Watchdog reset — PASS (the USB port drops once mid-run)
- [ ] Each run's `FLIGHT_XXX.CSV` plots sensibly in `tools/flight_plot.m`.
- [ ] Flash logging, checked in the serial output during the runs:
  - [ ] Every boot shows `[FLASH] GD25Q128 found`, **not** `JEDEC mismatch`.
  - [ ] Scenario 1 shows `Flash log recording` at liftoff, then `Wrote N records from flash to FLIGHT_XXX.CSV` at landing.
  - [ ] Scenario 8 shows `Flash log continuing after the reset`.
  - [ ] The next boot with an SD card copies that flight from flash, and its CSV covers both sides of the reset.

### Power
- [ ] Servos run from the flight battery's supply, not USB. They hold centre at idle with no buzz or twitch.
- [ ] Arming-switch voltage reads about 2.7 V with a charged pack (the arming message shows `pack=` about 8.4 V, not about 4 V).

### Flight firmware back on, ready to pack
- [ ] **Flight firmware uploaded**: `pio run -e flight -t upload`. *Not* `bench` or `flight_sim`.
- [ ] *The boot messages show `FLIGHT COMPUTER READY` and no `[SIM]` lines.*
- [ ] *Gyro calibrated: on a still bench, in IDLE, send `G` and see `[CAL] Gyro bias saved`.*
- [ ] *No `[LOGGER] Flash holds a flight that ...` message. If there is one, let it copy to the SD card first.*
- [ ] SD card: test and simulation `FLIGHT_XXX` files deleted, card inserted.
- [ ] Flight battery fully charged. It's a 2S pack: full is 8.4 V; **don't fly below 8.0 V**.

## B. Pack list
- [ ] Rocket, nose cone, gimbal and servos.
- [ ] Main chute, shock cord, chute protector.
- [ ] Motors: Estes F-15, in the variant your recovery plan calls for — plugged if the flight computer deploys the chute, or with a delay if motor ejection is your backup.
- [ ] Igniters.
- [ ] E-matches, and charge material measured to your ground test.
- [ ] Charged flight battery, plus a spare.
- [ ] Spare SD card.
- [ ] Laptop and USB cable.
- [ ] Tools, tape, wadding.
- [ ] This checklist.

## C. At the field — prep table (SW401 OPEN throughout)
- [ ] Airframe inspected: fins/gimbal mount secure, no cracks.
- [ ] Gimbal moves freely through full travel; linkages tight; no binding.
- [ ] Chute packed, shock cord attached at both ends, protector in place.
- [ ] Ejection charge loaded and the e-match connected, **with SW401 OPEN**.
- [ ] Motor installed and retained. **No igniter yet.**
- [ ] Battery connected, power on:
  - [ ] **White LED blinking** — the flight firmware is running.
  - [ ] **No fast red blink.** A fast red blink means an IMU fault: no-go.
  - [ ] Servos centred.
- [ ] *Serial (optional):*
  - [ ] *`SD ready — this flight will write FLIGHT_0NN.CSV` and `[FLASH] GD25Q128 found`.*
  - [ ] *No `[WARN]` or `[FAULT]` lines.*
- [ ] RSO / safety check passed.

## D. At the pad
- [ ] Rocket on the rail, vertical and **still** (pad rest needs 2 s upright and motionless).
- [ ] **Close SW401 (arm).** Listen:
  - [ ] **Rapid double chirp every second**: armed, e-match continuity OK. Go.
  - **Rapid triple chirp**: e-match OPEN. Open SW401, fix the connection, re-arm.
  - **Silent**: no pyro power. Check the battery and the switch.
- [ ] Stand clear and keep the rocket still for ~3 s after arming. In that time:
  - the launch ground reference is captured;
  - the flash log is pre-erased.
- [ ] Install the igniter last, per range procedure.
- [ ] Avoid power-cycling the flight computer after arming. If it does reboot with SW401 closed, it re-arms itself about 0.1 s after boot: continuity check, fired flags cleared, new ground reference. Keep the rocket still for 3 s afterwards and listen for the double chirp.
- [ ] Clear the pad. Launch on the RSO's countdown.

## E. Recovery
- [ ] **1 Hz continuous beep (locator)**: touchdown detected and the log written to SD. If it's still the armed double chirp, the computer hasn't called touchdown yet. That's fine: the flight is in flash.
- [ ] **Open SW401 before handling the rocket.** Treat any unfired charge as live.
- [ ] Power off. Flash keeps the flight even if it never reached the SD card.

## F. After the flight
- [ ] Copy `FLIGHT_0NN.CSV` and `FLIGHT_0NN.LOG` off the SD card.
  - If the CSV is missing, power the board with a card inserted: the flight is copied from flash at boot.
  - Or send `U` over USB to dump it.
- [ ] Plot it: `flight_plot('FLIGHT_0NN.CSV')` in MATLAB. Check that the summary's "PID rebuild vs logged output" line has no warning.
- [ ] Note any `[FSM]`, `[PYRO]` or `[WARN]` lines from the `.LOG` file in your flight notes.

---

### Quick reference
| Signal | Meaning |
|---|---|
| White LED blinking | Flight firmware running |
| Green flash | Pad rest latched (ready for launch detection) |
| Fast red blink at boot | IMU failed to start — no-go |
| Rapid double chirp / 1 s | Armed, e-match continuity OK |
| Rapid triple chirp / 1 s | Armed, e-match **OPEN** |
| 1 Hz continuous beep | Landed, log written |
