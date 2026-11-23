// ============================================================
//  servo_limit_finder.cpp
//
//  Finds the real mechanical travel of the TVC nozzle so the
//  SERVO_*_MIN_US / MAX_US stops in include/servo_driver.h can be
//  re-measured. Self-contained: flash it, open any serial monitor
//  at 115200 and type keys, plus Enter if your monitor sends whole
//  lines ("aaaa" + Enter is four jogs). 'h' prints the keys.
//
//    pio run -e servo_cal -t upload
//    pio device monitor                 (or the VS Code serial monitor)
//    python src/find_servo_limits.py    (optional: arrow keys, no Enter)
//
//  Unlike servo_driver.cpp, this drives the servos with raw pulse
//  widths and does NOT clamp to the MIN_US/MAX_US in servo_driver.h.
//  The point is to find out how far past them the gimbal can go.
//  The only clamp is the absolute ABS_MIN_US..ABS_MAX_US window, and
//  every move is slew-limited, so a typed value can't slam the
//  nozzle into a stop at full servo speed.
//
//  Safety: the servos drive only while a host has the serial port
//  open. On connect both go to SERVO_*_CENTER_US. When the port
//  closes, or on 'l', both go limp. Pyro fire pins are held LOW.
//
//  Keys are lowercase on purpose: the flight firmware's serial
//  commands are uppercase ('X' = abort), so these keys do nothing
//  if typed into the flight build by mistake.
// ============================================================

#include <Arduino.h>
#include <Servo.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

#include "board_pins.h"
#include "servo_driver.h"   // centre trims, plus the current MIN/MAX stops for comparison (not enforced)

#define ABS_MIN_US      500      // never command outside this window
#define ABS_MAX_US      2500
#define SLEW_US_PER_S   400.0f   // about 40 deg/s of servo horn on a typical servo
#define SLEW_TICK_US    2000     // slew update period
#define MARGIN_US       10       // suggested limits back off this far from each hard stop

static const int STEPS[] = { 1, 2, 5, 10, 25, 50 };   // jog sizes [us]
static const int N_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);
static int step_i = 4;                                 // start at 25 us: big enough to see

static Servo pitch_servo;
static Servo yaw_servo;

struct Axis {
    Servo      *servo;
    const char *label;       // as in the #defines
    int         pin;
    int         center_us;
    int         code_min;    // current servo_driver.h stops
    int         code_max;
    float       cur_us;      // what's on the wire now
    int         target_us;   // where the slew is heading
    bool        driving;
    int         low;         // marked hard stops [us], 0 = not marked yet
    int         high;
};

static Axis axes[2] = {
    { &pitch_servo, "PITCH", SERVO_PITCH_PIN, SERVO_PITCH_CENTER_US,
      SERVO_PITCH_MIN_US, SERVO_PITCH_MAX_US,
      SERVO_PITCH_CENTER_US, SERVO_PITCH_CENTER_US, false, 0, 0 },
    { &yaw_servo,   "YAW",   SERVO_YAW_PIN,   SERVO_YAW_CENTER_US,
      SERVO_YAW_MIN_US,   SERVO_YAW_MAX_US,
      SERVO_YAW_CENTER_US,   SERVO_YAW_CENTER_US,   false, 0, 0 },
};
static int sel = 0;   // axis being jogged

// ------------------------------------------------------------
//  Servo output
// ------------------------------------------------------------

static void engage(Axis &a) {
    if (a.driving) return;
    // Attach with the absolute window so the library doesn't clamp to
    // servo_driver.h's stops. attach() starts the channel at 1500 us,
    // so put the real position on the wire straight away.
    a.servo->attach(a.pin, ABS_MIN_US, ABS_MAX_US);
    a.servo->writeMicroseconds((int)lroundf(a.cur_us));
    a.driving = true;
}

static void go_limp() {
    for (Axis &a : axes) {
        if (a.driving) {
            a.servo->detach();
            digitalWrite(a.pin, LOW);   // don't leave the line high if detach landed mid-pulse
        }
        a.driving   = false;
        a.target_us = (int)lroundf(a.cur_us);   // re-engaging holds position, no lurch
    }
}

static void slew_tick() {
    static uint32_t last = micros();
    const uint32_t now = micros();
    if (now - last < SLEW_TICK_US) return;
    const float max_step = SLEW_US_PER_S * (float)(now - last) * 1e-6f;
    last = now;

    for (Axis &a : axes) {
        if (!a.driving) continue;
        const float err = (float)a.target_us - a.cur_us;
        if (err == 0.0f) continue;
        a.cur_us = (fabsf(err) <= max_step) ? (float)a.target_us
                                            : a.cur_us + copysignf(max_step, err);
        a.servo->writeMicroseconds((int)lroundf(a.cur_us));
    }
}

// ------------------------------------------------------------
//  Console
// ------------------------------------------------------------

static void print_help() {
    Serial.printf("SERVOCAL READY X_CENTER=%d Y_CENTER=%d MIN=%d MAX=%d\n",
                  SERVO_PITCH_CENTER_US, SERVO_YAW_CENTER_US, ABS_MIN_US, ABS_MAX_US);
    Serial.println(F("Nozzle travel-limit finder. Type a key, plus Enter if your monitor sends whole lines."));
    Serial.println(F("  a / d     jog down / up one step   (\"aaaa\" = four steps)"));
    Serial.println(F("  w / s     bigger / smaller step    (1 2 5 10 25 50 us, starts at 25)"));
    Serial.printf("  x         switch axis: PITCH (pin %d) <-> YAW (pin %d)\n", SERVO_PITCH_PIN, SERVO_YAW_PIN);
    Serial.println(F("  [ / ]     mark here as the LOW / HIGH stop  (they become MIN_US / MAX_US)"));
    Serial.println(F("  1350g     go straight to 1350 us  (Enter works instead of g)"));
    Serial.println(F("  c         both servos to centre"));
    Serial.println(F("  l         limp: stop driving both servos; any jog re-engages"));
    Serial.println(F("  r         results + #defines to paste into servo_driver.h"));
    Serial.println(F("  h or ?    this help"));
    Serial.println(F("Finding a stop: get close at 25 us, then press s to drop to 1-5 us steps and creep in."));
    Serial.println(F("The stop is the last step that still moved it. If the servo hums, it's"));
    Serial.println(F("pushing on the stop: step back right away or press l."));
    Serial.println(F("Closing the serial monitor makes both servos go limp."));
    Serial.println();
}

static void print_status() {
    const Axis &a = axes[sel];
    const int pos = a.target_us;
    Serial.printf("%-5s %4d us (%+d)   step %d us   | code %d..%d   | marked ",
                  a.label, pos, pos - a.center_us, STEPS[step_i], a.code_min, a.code_max);
    if (a.low)  Serial.print(a.low);  else Serial.print('-');
    Serial.print("..");
    if (a.high) Serial.print(a.high); else Serial.print('-');
    if (pos < a.code_min)      Serial.printf("   << %d past code MIN", a.code_min - pos);
    else if (pos > a.code_max) Serial.printf("   >> %d past code MAX", pos - a.code_max);
    if (!a.driving) Serial.print("   LIMP");
    Serial.println();
}

static void move_to(int us) {
    Axis &a = axes[sel];
    if (us < ABS_MIN_US || us > ABS_MAX_US) {
        Serial.printf("%d us is outside the %d..%d us window - not moved\n", us, ABS_MIN_US, ABS_MAX_US);
        return;
    }
    a.target_us = us;
    engage(a);
    print_status();
}

static void mark(bool high) {
    Axis &a = axes[sel];
    const int pos = a.target_us;
    if (!high && pos >= a.center_us) {
        Serial.printf("The LOW stop has to be below centre (%d us). CENTER_US is fixed, so jog below it first.\n",
                      a.center_us);
        return;
    }
    if (high && pos <= a.center_us) {
        Serial.printf("The HIGH stop has to be above centre (%d us). CENTER_US is fixed, so jog above it first.\n",
                      a.center_us);
        return;
    }
    (high ? a.high : a.low) = pos;
    const int extra = high ? pos - a.code_max : a.code_min - pos;
    Serial.printf("%s %s stop marked at %d us (%+d from centre): ",
                  a.label, high ? "HIGH" : "LOW", pos, pos - a.center_us);
    if (extra > 0)      Serial.printf("%d us more travel than the code allows\n", extra);
    else if (extra < 0) Serial.printf("%d us INSIDE the code limit - the code drives past this stop!\n", -extra);
    else                Serial.println("same as the code limit");
}

// Suggested symmetric limits: back off MARGIN_US from each stop, then the
// closer side sets the span both ways. Same rule servo_driver.h already
// follows: CENTER_US is fixed and the driver scales both sides alike.
// Returns the span, or 0 if both stops aren't marked (lo/hi untouched).
static int suggest(const Axis &a, int &lo, int &hi) {
    if (!a.low || !a.high) return 0;
    const int span = min(a.center_us - (a.low + MARGIN_US), (a.high - MARGIN_US) - a.center_us);
    if (span <= 0) return 0;
    lo = a.center_us - span;
    hi = a.center_us + span;
    return span;
}

static void print_define(const char *label, const char *what, int value, const char *why) {
    char name[32];
    snprintf(name, sizeof(name), "SERVO_%s_%s", label, what);
    Serial.printf("#define %-24s%-7d// %s\n", name, value, why);
}

static void print_results() {
    Serial.println();
    for (const Axis &a : axes) {
        const int code_lo = a.center_us - a.code_min;
        const int code_hi = a.code_max - a.center_us;
        Serial.printf("%s (pin %d), centre %d us (fixed)\n", a.label, a.pin, a.center_us);
        Serial.printf("  code now     %d .. %d us   (-%d / +%d)\n", a.code_min, a.code_max, code_lo, code_hi);
        if (!a.low || !a.high) {
            Serial.printf("  %s stop not marked - keeping the current values\n\n",
                          (!a.low && !a.high) ? "LOW and HIGH" : !a.low ? "LOW" : "HIGH");
            continue;
        }
        Serial.printf("  hard stops   %d .. %d us   (-%d / +%d)\n",
                      a.low, a.high, a.center_us - a.low, a.high - a.center_us);
        int lo, hi;
        const int span = suggest(a, lo, hi);
        if (!span) {
            Serial.printf("  no travel left after the %d us margin - re-check the stops\n\n", MARGIN_US);
            continue;
        }
        Serial.printf("  suggested    %d .. %d us   (+/-%d: %d us back from each stop, closer side sets both)\n",
                      lo, hi, span, MARGIN_US);
        const int gain = span - min(code_lo, code_hi);
        Serial.printf("  vs code      %+d us each way\n", gain);
        if (gain < 0)
            Serial.println("  WARNING: the current code drives the servo past the stop you found - "
                           "update servo_driver.h before the next run.");
        Serial.println();
    }

    Serial.println("// ---- paste into include/servo_driver.h ----");
    for (const Axis &a : axes) {
        int lo = a.code_min, hi = a.code_max;
        char why_lo[64], why_hi[64];
        if (suggest(a, lo, hi)) {
            snprintf(why_lo, sizeof(why_lo), "stop found at %d; %d us margin, symmetric", a.low, MARGIN_US);
            snprintf(why_hi, sizeof(why_hi), "stop found at %d; %d us margin, symmetric", a.high, MARGIN_US);
        } else {
            snprintf(why_lo, sizeof(why_lo), "unchanged - not measured");
            snprintf(why_hi, sizeof(why_hi), "unchanged - not measured");
        }
        print_define(a.label, "MIN_US", lo, why_lo);
        print_define(a.label, "MAX_US", hi, why_hi);
        print_define(a.label, "CENTER_US", a.center_us, "fixed - do not change");
        Serial.println();
    }
}

static void handle_char(char c) {
    static char   digits[6];   // a pulse width being typed, e.g. "1350"
    static size_t n = 0;

    if (isdigit((unsigned char)c)) {
        if (n < sizeof(digits) - 1) digits[n++] = c;
        return;
    }
    if ((c == '\b' || c == 0x7f) && n) {   // backspace within a number
        n--;
        return;
    }
    if (n && (c == '\r' || c == '\n' || c == 'g' || c == 'G')) {
        digits[n] = '\0';
        n = 0;
        move_to(atoi(digits));
        return;
    }
    n = 0;   // any other key drops a half-typed number

    switch (tolower((unsigned char)c)) {
        case 'a': move_to(axes[sel].target_us - STEPS[step_i]); break;
        case 'd': move_to(axes[sel].target_us + STEPS[step_i]); break;
        case 'w': if (step_i < N_STEPS - 1) step_i++; print_status(); break;
        case 's': if (step_i > 0) step_i--;           print_status(); break;
        case 'x': sel ^= 1; print_status(); break;
        case '[': mark(false); break;
        case ']': mark(true);  break;
        case 'c':
            for (Axis &a : axes) {
                a.target_us = a.center_us;
                engage(a);
            }
            Serial.println("Both servos -> centre");
            print_status();
            break;
        case 'l':
            go_limp();
            Serial.println("Both servos limp. Any jog or c re-engages (back to the last position first).");
            print_status();
            break;
        case 'g': Serial.println("Type a pulse width first, e.g. 1350g"); break;
        case 'r': print_results(); break;
        case 'h': case '?': print_help(); print_status(); break;
        case ' ': case '\t': case '\r': case '\n': case '\b': case 0x7f: break;
        default:
            Serial.printf("Unknown key '%c' - h for help\n", c);
            break;
    }
}

// ------------------------------------------------------------
//  Arduino entry points
// ------------------------------------------------------------

void setup() {
    // Pyro outputs LOW before enabling the driver (see pyro_init). This
    // build never fires, but don't leave the gates floating either.
    digitalWrite(PIN_PYRO1_FIRE, LOW);
    digitalWrite(PIN_PYRO2_FIRE, LOW);
    pinMode(PIN_PYRO1_FIRE, OUTPUT);
    pinMode(PIN_PYRO2_FIRE, OUTPUT);

    // No pulses until a host connects: servos stay limp.
    digitalWrite(PIN_SERVO_X, LOW);
    digitalWrite(PIN_SERVO_Y, LOW);
    pinMode(PIN_SERVO_X, OUTPUT);
    pinMode(PIN_SERVO_Y, OUTPUT);

    Serial.begin(115200);
}

void loop() {
    static bool host = false;
    const bool connected = (bool)Serial;   // true while the host holds DTR (port open)

    if (connected && !host) {
        for (Axis &a : axes) {
            a.cur_us    = (float)a.center_us;
            a.target_us = a.center_us;
            engage(a);
        }
        print_help();
        print_status();
    } else if (!connected && host) {
        go_limp();
    }
    host = connected;

    if (host) {
        while (Serial.available()) handle_char((char)Serial.read());
    }
    slew_tick();
}
