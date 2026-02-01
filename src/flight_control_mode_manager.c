/* flight_control_mode_manager.c
 * Mini-projet "avionique-like" (version C)
 *
 * Compile:
 *   gcc -std=c11 -O2 -Wall -Wextra -Wpedantic -Werror flight_control_mode_manager.c -o sim
 * Run:
 *   ./sim
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* ----------------------------- Types -------------------------------- */

typedef enum {
    MODE_OFF = 0,
    MODE_NORMAL,
    MODE_DEGRADED,
    MODE_FAILSAFE
} Mode;

typedef struct {
    bool sensor_invalid;
    bool overspeed;
} FaultFlags;

typedef struct {
    double airspeed;        /* knots */
    double altitude;        /* feet */
    bool pilot_engage;
    bool pilot_disengage;
} Inputs;

typedef struct {
    Mode mode;
    double cmd;             /* commande normalisée */
    FaultFlags flags;
} Outputs;

static const char* mode_to_string(Mode m) {
    switch (m) {
        case MODE_OFF:      return "OFF";
        case MODE_NORMAL:   return "NORMAL";
        case MODE_DEGRADED: return "DEGRADED";
        case MODE_FAILSAFE: return "FAILSAFE";
        default:            return "UNKNOWN";
    }
}

/* -------------------------- Utils ----------------------------------- */

static double clamp(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ------------------------ Parameters --------------------------------- */

/* Sensor ranges (mini-projet) */
static const double AIRSPEED_MIN = 0.0;
static const double AIRSPEED_MAX = 250.0;
static const double ALTITUDE_MIN = -100.0;
static const double ALTITUDE_MAX = 50000.0;

/* Overspeed hysteresis */
static const double OVERSPEED_ON  = 200.0;
static const double OVERSPEED_OFF = 195.0;

/* Cmd saturations */
static const double CMD_SAT_NORMAL    = 1.0;
static const double CMD_SAT_OVERSPEED = 0.2;

/* LLR-03.3 persistence cycles before FAILSAFE (50 cycles @ 20ms = 1s) */
static const uint32_t INVALID_PERSIST_CYCLES_TO_FAILSAFE = 50;

/* ----------------------- "Modules" ---------------------------------- */

static bool validate_sensors(const Inputs* in) {
    const bool air_ok = (in->airspeed >= AIRSPEED_MIN) && (in->airspeed <= AIRSPEED_MAX);
    const bool alt_ok = (in->altitude >= ALTITUDE_MIN) && (in->altitude <= ALTITUDE_MAX);
    return (air_ok && alt_ok);
}

static void update_overspeed_flag(const Inputs* in, FaultFlags* flags) {
    if (!flags->overspeed) {
        if (in->airspeed >= OVERSPEED_ON) {
            flags->overspeed = true;
        }
    } else {
        if (in->airspeed < OVERSPEED_OFF) {
            flags->overspeed = false;
        }
    }
}

/* Simple demo command computation + protections (replaceable) */
static double compute_command(const Inputs* in, Mode mode, const FaultFlags* flags) {
    /* FAILSAFE forces cmd = 0 (LLR-05.1) */
    if (mode == MODE_FAILSAFE) {
        return 0.0;
    }

    /* rawCmd: simple stable demo (altitude mapped to [-1,1]) */
    double rawCmd = clamp(in->altitude / 50000.0, -1.0, 1.0);

    /* In DEGRADED, reduce effect */
    if (mode == MODE_DEGRADED) {
        rawCmd *= 0.5;
    }

    /* Overspeed saturation (LLR-04.2) */
    const double sat = flags->overspeed ? CMD_SAT_OVERSPEED : CMD_SAT_NORMAL;
    return clamp(rawCmd, -sat, sat);
}

/* ------------------------ Mode Manager ------------------------------- */

typedef struct {
    Mode mode;
    FaultFlags flags;
    uint32_t invalid_persist_count;
} ModeManager;

static void mm_init(ModeManager* mm) {
    mm->mode = MODE_OFF;
    mm->flags.sensor_invalid = false;
    mm->flags.overspeed = false;
    mm->invalid_persist_count = 0;
}

static void mm_reset_to_off(ModeManager* mm) {
    mm->mode = MODE_OFF;
    mm->flags.sensor_invalid = false;
    mm->flags.overspeed = false;
    mm->invalid_persist_count = 0;
}

static void mm_apply_state_transitions(ModeManager* mm, const Inputs* in) {
    switch (mm->mode) {
        case MODE_OFF:
            mm->invalid_persist_count = 0;

            if (in->pilot_engage) {
                if (!mm->flags.sensor_invalid) {
                    /* LLR-01.1: OFF -> NORMAL */
                    mm->mode = MODE_NORMAL;
                } else {
                    /* LLR-01.3: stay OFF, flag already set */
                    mm->mode = MODE_OFF;
                }
            }
            break;

        case MODE_NORMAL:
            if (mm->flags.sensor_invalid) {
                /* LLR-03.2: NORMAL -> DEGRADED */
                mm->mode = MODE_DEGRADED;
                mm->invalid_persist_count = 1;
            } else {
                mm->invalid_persist_count = 0;
            }
            break;

        case MODE_DEGRADED:
            if (mm->flags.sensor_invalid) {
                /* LLR-03.3: count persistence */
                if (mm->invalid_persist_count < INVALID_PERSIST_CYCLES_TO_FAILSAFE) {
                    mm->invalid_persist_count++;
                }

                if (mm->invalid_persist_count >= INVALID_PERSIST_CYCLES_TO_FAILSAFE) {
                    mm->mode = MODE_FAILSAFE;
                }
            } else {
                /* For mini-project: recover to NORMAL when fault clears */
                mm->mode = MODE_NORMAL;
                mm->invalid_persist_count = 0;
            }
            break;

        case MODE_FAILSAFE:
            /* LLR-05.2: only exit through pilot_disengage (handled earlier) */
            mm->mode = MODE_FAILSAFE;
            break;

        default:
            mm_reset_to_off(mm);
            break;
    }
}

static Outputs mm_step(ModeManager* mm, const Inputs* in) {
    Outputs out;
    out.mode = mm->mode;
    out.cmd = 0.0;
    out.flags = mm->flags;

    /* LLR-01.2: pilot_disengage => OFF at next cycle regardless of current mode */
    if (in->pilot_disengage) {
        mm_reset_to_off(mm);
        out.mode = mm->mode;
        out.flags = mm->flags;
        out.cmd = compute_command(in, out.mode, &out.flags);
        return out;
    }

    /* Sensor validation (LLR-02.x) */
    mm->flags.sensor_invalid = !validate_sensors(in); /* LLR-02.3 (same cycle) */

    /* Overspeed flags (LLR-04.x) */
    update_overspeed_flag(in, &mm->flags);

    /* State transitions */
    mm_apply_state_transitions(mm, in);

    /* Compute command with protections */
    out.mode = mm->mode;
    out.flags = mm->flags;
    out.cmd = compute_command(in, out.mode, &out.flags);
    return out;
}

/* ------------------------------ Tests -------------------------------- */

static void expect(bool cond, const char* msg) {
    if (!cond) {
        fprintf(stderr, "TEST FAILED: %s\n", msg);
        exit(1);
    }
}

static void run_unit_tests(void) {
    ModeManager mm;
    mm_init(&mm);

    /* T-01 (LLR-01.1): engage + valid -> NORMAL */
    {
        Inputs in = { .airspeed = 100.0, .altitude = 1000.0, .pilot_engage = true, .pilot_disengage = false };
        Outputs out = mm_step(&mm, &in);
        expect(out.mode == MODE_NORMAL, "T-01: OFF->NORMAL on engage with valid sensors");
    }

    /* T-02 (LLR-01.2): disengage -> OFF */
    {
        Inputs in = { .airspeed = 100.0, .altitude = 1000.0, .pilot_engage = false, .pilot_disengage = true };
        Outputs out = mm_step(&mm, &in);
        expect(out.mode == MODE_OFF, "T-02: ANY->OFF on pilot_disengage");
    }

    /* T-03 (LLR-01.3): engage + invalid -> stay OFF + SENSOR_INVALID */
    {
        Inputs in = { .airspeed = -1.0, .altitude = 1000.0, .pilot_engage = true, .pilot_disengage = false };
        Outputs out = mm_step(&mm, &in);
        expect(out.mode == MODE_OFF, "T-03: remain OFF if engage with invalid sensors");
        expect(out.flags.sensor_invalid, "T-03: SENSOR_INVALID flag set when invalid detected");
    }

    /* Setup: back to NORMAL */
    {
        Inputs in = { .airspeed = 120.0, .altitude = 2000.0, .pilot_engage = true, .pilot_disengage = false };
        Outputs out = mm_step(&mm, &in);
        expect(out.mode == MODE_NORMAL, "Setup: back to NORMAL");
    }

    /* T-10 (LLR-03.2): invalid while engaged -> DEGRADED (next cycle) */
    {
        Inputs in = { .airspeed = 999.0, .altitude = 2000.0, .pilot_engage = false, .pilot_disengage = false };
        Outputs out = mm_step(&mm, &in);
        expect(out.mode == MODE_DEGRADED, "T-10: NORMAL->DEGRADED on sensor invalid");
    }

    /* T-11 (LLR-03.3): persist invalid N cycles -> FAILSAFE */
    {
        Inputs in = { .airspeed = 999.0, .altitude = 2000.0, .pilot_engage = false, .pilot_disengage = false };
        Outputs out = {0};
        for (uint32_t i = 0; i < INVALID_PERSIST_CYCLES_TO_FAILSAFE; i++) {
            out = mm_step(&mm, &in);
        }
        expect(out.mode == MODE_FAILSAFE, "T-11: DEGRADED->FAILSAFE after N invalid cycles");
        expect(out.cmd == 0.0, "T-16: FAILSAFE forces cmd=0");
    }

    /* T-18 (LLR-05.2): FAILSAFE + disengage -> OFF */
    {
        Inputs in = { .airspeed = 100.0, .altitude = 1000.0, .pilot_engage = false, .pilot_disengage = true };
        Outputs out = mm_step(&mm, &in);
        expect(out.mode == MODE_OFF, "T-18: FAILSAFE->OFF on pilot_disengage");
    }

    /* T-13/14/15 (LLR-04.x): overspeed flag + cmd saturation + hysteresis */
    {
        /* Engage to NORMAL */
        Inputs eng = { .airspeed = 100.0, .altitude = 50000.0, .pilot_engage = true, .pilot_disengage = false };
        Outputs out = mm_step(&mm, &eng);
        expect(out.mode == MODE_NORMAL, "Overspeed tests setup: NORMAL");

        /* Overspeed ON at 200 */
        Inputs in1 = { .airspeed = 200.0, .altitude = 50000.0, .pilot_engage = false, .pilot_disengage = false };
        out = mm_step(&mm, &in1);
        expect(out.flags.overspeed, "T-13: OVERSPEED set at >=200");
        expect(out.cmd <= CMD_SAT_OVERSPEED + 1e-12, "T-14: cmd saturated to overspeed limit");

        /* Still overspeed at 195 */
        Inputs in2 = { .airspeed = 195.0, .altitude = 50000.0, .pilot_engage = false, .pilot_disengage = false };
        out = mm_step(&mm, &in2);
        expect(out.flags.overspeed, "T-15: overspeed stays true at 195");

        /* Overspeed clears below 195 */
        Inputs in3 = { .airspeed = 194.0, .altitude = 50000.0, .pilot_engage = false, .pilot_disengage = false };
        out = mm_step(&mm, &in3);
        expect(!out.flags.overspeed, "T-15: overspeed clears below 195");
    }

    printf("All unit tests passed.\n");
}

/* ------------------------------ Demo --------------------------------- */

int main(void) {
    run_unit_tests();

    ModeManager mm;
    mm_init(&mm);

    printf("\n--- Demo simulation (20 cycles) ---\n");

    for (int t = 0; t < 20; t++) {
        Inputs in;
        in.pilot_disengage = false;
        in.pilot_engage = (t == 1);

        /* make a scenario: valid then invalid then overspeed */
        if (t < 8) {
            in.airspeed = 120.0;
            in.altitude = 1000.0 + 200.0 * t;
        } else if (t < 14) {
            in.airspeed = 999.0;   /* invalid => DEGRADED / maybe FAILSAFE if long enough */
            in.altitude = 2500.0;
        } else {
            in.airspeed = 205.0;   /* overspeed */
            in.altitude = 50000.0; /* command wants to saturate */
        }

        Outputs out = mm_step(&mm, &in);
        printf("t=%02d  air=%.1f alt=%.1f  mode=%-8s  cmd=%.3f  flags{sens=%d, over=%d}\n",
               t, in.airspeed, in.altitude, mode_to_string(out.mode), out.cmd,
               out.flags.sensor_invalid ? 1 : 0,
               out.flags.overspeed ? 1 : 0);
    }

    return 0;
}
