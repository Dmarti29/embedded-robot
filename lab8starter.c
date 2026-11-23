/*
 * Arty A7-100 Robot Lab -- Lab 8 line follower (completed starter)
 * Vivado / Vitis 2025.2, MicroBlaze, standalone BSP
 *
 * Hardware (from the TA's XSA / xparameters.h):
 *   JA top row: Pmod MAXSONAR  -> AXI UART Lite, 9600 baud ("R###\r", inches)
 *   JC:         Pmod DHB1      -> AXI GPIO, 8 bits (EN/DIR out, encoders in)
 *   JD top row: Pmod LS1       -> AXI GPIO, 4 inputs (S1 and S4 used)
 *   USB:        AXI UART Lite, 115200 baud (serial console)
 *
 * There is no PWM IP for the motors, so speed is set with software PWM:
 * the main loop runs every TICK_US and toggles the EN bits.
 *
 * ---------------------------------------------------------------------
 * HOW TO USE  (change RUN_MODE below, rebuild, run)
 *   1. MODE_SENSOR_TEST - motors never move. Open the serial terminal
 *      (115200). Slide tape under each light sensor and hold your hand
 *      in front of the sonar. Set LS1_ON_TAPE_LEVEL / LEFT/RIGHT sensor
 *      so the printout matches reality.
 *   2. MODE_MOTOR_TEST - WHEELS OFF THE GROUND. Prints each step:
 *      "LEFT wheel forward" etc. Fix LEFT_IS_MOTOR1 and M1/M2_FWD_DIR
 *      until every step matches what the wheels do.
 *   3. MODE_RUN - put it on the track. Tune SPEED_* values.
 * SAFETY: test with the wheels raised until steps 1 and 2 are correct.
 * ---------------------------------------------------------------------
 */

#include "xparameters.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xuartlite_l.h"
#include "sleep.h"

/* =====================================================================
 * MODE
 * ===================================================================== */
#define MODE_SENSOR_TEST 0
#define MODE_MOTOR_TEST  1
#define MODE_RUN         2
#define RUN_MODE         MODE_RUN   /* <-- change as you go */

/* =====================================================================
 * Addresses - taken from xparameters.h so they always match the XSA.
 * (The starter's LS1 example address 0x41200000 was wrong; the XSA
 *  puts axi_gpio_ls1 at 0x40010000.)
 * ===================================================================== */
#define USB_UART_BASE    ((UINTPTR)XPAR_AXI_UARTLITE_USB_BASEADDR)   /* 0x40600000 */
#define SONAR_UART_BASE  ((UINTPTR)XPAR_AXI_UARTLITE_SONAR_BASEADDR) /* 0x40610000 */
#define LS1_GPIO_BASE    ((UINTPTR)XPAR_AXI_GPIO_LS1_BASEADDR)       /* 0x40010000 */
#define MOTOR_GPIO_BASE  ((UINTPTR)XPAR_AXI_GPIO_MOTORS_BASEADDR)    /* 0x40000000 */

/* AXI GPIO channel 1 register offsets */
#define GPIO_DATA_OFFSET 0x00U
#define GPIO_TRI_OFFSET  0x04U

/* =====================================================================
 * Light sensors (PmodLS1 on JD: S1 -> GPIO[0], S4 -> GPIO[3])
 * ===================================================================== */
#define LS1_S1_BIT 0U
#define LS1_S4_BIT 1U   /* S4 channel on our LS1 is stuck high -> sensor moved to S2 (bit 1) */

/* Value a sensor bit reads when it is over the silver tape.
 * Check in MODE_SENSOR_TEST (and adjust the pots on the LS1). */
#define LS1_ON_TAPE_LEVEL 1U

/* Which sensor is on the car's LEFT side: 1 = S1 is left, 0 = S4 is left */
#define S1_IS_LEFT 1

/* How the two IR sensors sit relative to the tape:
 *   SENSORS_STRADDLE: tape runs BETWEEN them, both normally see floor.
 *   SENSORS_ON_LINE:  both normally sit ON the tape. */
#define SENSORS_STRADDLE 0
#define SENSORS_ON_LINE  1
#define SENSOR_LAYOUT    SENSORS_STRADDLE

#define LS1_DEBOUNCE_TICKS 3U   /* reading must be stable this many ticks */

/* =====================================================================
 * Motors (Pmod DHB1 on JC). Bit layout = Pmod pin order:
 *   pin1 EN1, pin2 DIR1, pin3 SA1, pin4 SB1, pin7 EN2, pin8 DIR2, pin9 SA2, pin10 SB2
 * ===================================================================== */
#define MOTOR1_EN_BIT  0U
#define MOTOR1_DIR_BIT 1U
#define MOTOR2_EN_BIT  4U
#define MOTOR2_DIR_BIT 5U
#define MOTOR_GPIO_INPUT_MASK 0xCCU /* encoder signals are inputs */

/* Fix these with MODE_MOTOR_TEST */
#define LEFT_IS_MOTOR1 1    /* 1: motor 1 (J5?) is the left wheel      */
#define M1_FWD_DIR     0U   /* DIR level that drives motor 1 forward   */
#define M2_FWD_DIR     1U   /* motors are mirrored, so usually opposite */

#define DIR_CHANGE_US  200U /* dead time: EN low before/after DIR flips */

/* =====================================================================
 * Timing / software PWM
 * ===================================================================== */
#define TICK_US        100U  /* main loop period                      */
#define PWM_STEPS      20U   /* 20 ticks = ~2 ms PWM period, 5% steps */
#define MS_TO_TICKS(ms) ((u32)(ms) * 1000U / TICK_US)

/* =====================================================================
 * Driving behaviour - tune on the track (percent, negative = reverse)
 * ===================================================================== */
#define SPEED_STRAIGHT     40
#define SPEED_TURN_OUTER   50
#define SPEED_TURN_INNER   0     /* inside wheel stops during a turn   */
#define SPEED_PIVOT_INNER  0     /* (no reversing / spinning in place) */
#define PIVOT_AFTER_MS     200U
#define HOLD_TURN_MS       150U  /* keep turning after the sensor clears */
#define START_DELAY_MS     3000U /* time to set the car down           */
#define KICK_SPEED         70    /* short push so the motors get going */
#define KICK_MS            250U

/* =====================================================================
 * Sonar / obstacle
 * ===================================================================== */
#define STOP_DISTANCE_IN   10U   /* stop when obstacle this close       */
#define SONAR_CONFIRM      2U    /* consecutive close readings (~50 ms each) */
#define SONAR_STALE_MS     500U  /* no reading this long -> ignore sonar */
#define LATCH_STOP         1     /* 1: stay stopped once obstacle seen  */

/* ===================================================================== */

typedef struct {
    u8 s1;
    u8 s4;
} LightSensors;

typedef struct {
    u32 distance_inches;
    u8 valid;
} SonarReading;

/* ---------------------------------------------------------------------
 * Serial console (provided by starter, plus number printing)
 * ------------------------------------------------------------------- */
static void serial_char(char c)
{
    if (c == '\n') {
        XUartLite_SendByte(USB_UART_BASE, (u8)'\r');
    }
    XUartLite_SendByte(USB_UART_BASE, (u8)c);
}

static void serial_text(const char *message)
{
    while (*message != '\0') {
        serial_char(*message++);
    }
}

static void serial_uint(u32 value)
{
    char buf[11];
    int i = 0;
    do {
        buf[i++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && i < 10);
    while (i > 0) {
        serial_char(buf[--i]);
    }
}

static void serial_hex8(u32 value)
{
    const char *hex = "0123456789ABCDEF";
    serial_text("0x");
    serial_char(hex[(value >> 4) & 0xFU]);
    serial_char(hex[value & 0xFU]);
}

/* ---------------------------------------------------------------------
 * TODO 1: light sensors
 * ------------------------------------------------------------------- */
static LightSensors read_light_sensors(void)
{
    LightSensors result;
    u32 data = Xil_In32(LS1_GPIO_BASE + GPIO_DATA_OFFSET);

    result.s1 = (u8)((data >> LS1_S1_BIT) & 1U);
    result.s4 = (u8)((data >> LS1_S4_BIT) & 1U);
    return result;
}

static u8 on_tape(u8 bit)
{
    return (u8)(bit == LS1_ON_TAPE_LEVEL);
}

/* ---------------------------------------------------------------------
 * TODO 2: nonblocking MAXSONAR receiver
 * Message format: 'R', three ASCII digits (inches), '\r'.
 * Drains everything in the UART FIFO; returns the newest complete
 * reading with valid=1, or valid=0 if no new complete message.
 * ------------------------------------------------------------------- */
static u32 sonar_bytes_rx = 0U;   /* diagnostics for sensor test */
static u32 sonar_errors   = 0U;

static SonarReading poll_sonar(void)
{
    static u8  state = 0U;    /* 0 = wait for 'R', 1..3 = digits, 4 = wait '\r' */
    static u32 value = 0U;
    SonarReading result = {0U, 0U};

    while (1) {
        u32 status = XUartLite_ReadReg(SONAR_UART_BASE, XUL_STATUS_REG_OFFSET);
        if ((status & XUL_SR_RX_FIFO_VALID_DATA) == 0U) {
            break;
        }
        u8 c = (u8)XUartLite_ReadReg(SONAR_UART_BASE, XUL_RX_FIFO_OFFSET);
        sonar_bytes_rx++;

        if ((status & (XUL_SR_FRAMING_ERROR | XUL_SR_OVERRUN_ERROR |
                       XUL_SR_PARITY_ERROR)) != 0U) {
            sonar_errors++;
            state = 0U;          /* resynchronise on the next 'R' */
        }

        if (c == 'R') {          /* 'R' always starts a new message */
            state = 1U;
            value = 0U;
        } else if (state >= 1U && state <= 3U) {
            if (c >= '0' && c <= '9') {
                value = value * 10U + (u32)(c - '0');
                state++;
            } else {
                state = 0U;      /* malformed */
            }
        } else if (state == 4U) {
            if (c == '\r') {
                result.distance_inches = value;
                result.valid = 1U;
            }
            state = 0U;
        }
    }
    return result;
}

/* ---------------------------------------------------------------------
 * TODO 3 / 4: motor control
 * motor_set() writes enables and directions. If a direction changes it
 * first turns both enables off, waits, flips DIR, waits, then enables.
 * Encoder bits are inputs, so writing 0 there has no effect.
 * ------------------------------------------------------------------- */
static u8 cur_dir1 = M1_FWD_DIR;
static u8 cur_dir2 = M2_FWD_DIR;

static void motor_write(u8 en1, u8 dir1, u8 en2, u8 dir2)
{
    u32 word = ((u32)(en1 & 1U) << MOTOR1_EN_BIT)  |
               ((u32)(dir1 & 1U) << MOTOR1_DIR_BIT) |
               ((u32)(en2 & 1U) << MOTOR2_EN_BIT)  |
               ((u32)(dir2 & 1U) << MOTOR2_DIR_BIT);
    Xil_Out32(MOTOR_GPIO_BASE + GPIO_DATA_OFFSET, word);
}

static void motor_set(u8 motor1_enable, u8 motor1_direction,
                      u8 motor2_enable, u8 motor2_direction)
{
    if (motor1_direction != cur_dir1 || motor2_direction != cur_dir2) {
        motor_write(0U, cur_dir1, 0U, cur_dir2);            /* stop first */
        usleep(DIR_CHANGE_US);
        motor_write(0U, motor1_direction, 0U, motor2_direction);
        usleep(DIR_CHANGE_US);
        cur_dir1 = motor1_direction;
        cur_dir2 = motor2_direction;
    }
    motor_write(motor1_enable, motor1_direction,
                motor2_enable, motor2_direction);
}

static void motor_stop(void)
{
    motor_write(0U, cur_dir1, 0U, cur_dir2);
}

/* ---------------------------------------------------------------------
 * Software PWM: drive(left, right) sets speeds (-100..100);
 * pwm_tick() must be called every TICK_US to generate the EN signal.
 * ------------------------------------------------------------------- */
static int speed_left  = 0;
static int speed_right = 0;
static u32 pwm_phase   = 0U;

static void drive(int left, int right)
{
    if (left > 100)   left = 100;
    if (left < -100)  left = -100;
    if (right > 100)  right = 100;
    if (right < -100) right = -100;
    speed_left  = left;
    speed_right = right;
}

static u8 pwm_on(int speed)
{
    u32 mag = (u32)((speed < 0) ? -speed : speed);
    u32 on_steps = (mag * PWM_STEPS + 50U) / 100U;
    return (u8)(pwm_phase < on_steps);
}

static void pwm_tick(void)
{
    int m1, m2;
    u8 d1, d2;

#if LEFT_IS_MOTOR1
    m1 = speed_left;  m2 = speed_right;
#else
    m1 = speed_right; m2 = speed_left;
#endif
    d1 = (m1 >= 0) ? M1_FWD_DIR : (u8)(M1_FWD_DIR ^ 1U);
    d2 = (m2 >= 0) ? M2_FWD_DIR : (u8)(M2_FWD_DIR ^ 1U);

    motor_set(pwm_on(m1), d1, pwm_on(m2), d2);

    pwm_phase++;
    if (pwm_phase >= PWM_STEPS) {
        pwm_phase = 0U;
    }
}

/* ---------------------------------------------------------------------
 * TODO 5: decision-making logic. Called once per tick.
 * ------------------------------------------------------------------- */
typedef enum { STEER_STRAIGHT, STEER_LEFT, STEER_RIGHT } Steer;

static void line_follow_step(LightSensors light, SonarReading sonar)
{
    /* --- state kept across calls --- */
    static u8    stopped      = 0U;
    static u32   close_count  = 0U;
    static u32   sonar_age    = 0xFFFFFFFFU;  /* ticks since last reading */
    static Steer steer        = STEER_STRAIGHT;
    static Steer last_turn    = STEER_STRAIGHT;
    static u32   turn_ticks   = 0U;
    static u32   hold_ticks   = 0U;
    static u8    stable_l = 0U, stable_r = 0U, raw_prev = 0xFFU;
    static u32   stable_count = 0U;

    /* ---------- obstacle detection ---------- */
    if (sonar.valid != 0U) {
        sonar_age = 0U;
        if (sonar.distance_inches <= STOP_DISTANCE_IN) {
            close_count++;
        } else {
            close_count = 0U;
        }
    } else if (sonar_age != 0xFFFFFFFFU) {
        sonar_age++;
    }
    if (sonar_age > MS_TO_TICKS(SONAR_STALE_MS)) {
        close_count = 0U;              /* no recent data: don't trust it */
    }

    if (stopped == 0U && close_count >= SONAR_CONFIRM) {
        stopped = 1U;
        drive(0, 0);
        motor_stop();
        serial_text("OBSTACLE at ");
        serial_uint(sonar.distance_inches);
        serial_text(" in - stopping\n");
    } else if (stopped != 0U && !LATCH_STOP && close_count == 0U) {
        stopped = 0U;
        serial_text("path clear - resuming\n");
    }
    if (stopped != 0U) {
        drive(0, 0);
        return;
    }

    /* ---------- light sensors (debounced) ---------- */
    u8 left_raw, right_raw;
#if S1_IS_LEFT
    left_raw  = on_tape(light.s1);
    right_raw = on_tape(light.s4);
#else
    left_raw  = on_tape(light.s4);
    right_raw = on_tape(light.s1);
#endif
    u8 packed = (u8)((left_raw << 1) | right_raw);
    if (packed == raw_prev) {
        if (stable_count < LS1_DEBOUNCE_TICKS) {
            stable_count++;
        }
    } else {
        raw_prev = packed;
        stable_count = 0U;
    }
    if (stable_count >= LS1_DEBOUNCE_TICKS) {
        stable_l = left_raw;
        stable_r = right_raw;
    }
    u8 L = stable_l, R = stable_r;

    /* ---------- desired steering ---------- */
    Steer want;
#if SENSOR_LAYOUT == SENSORS_STRADDLE
    /* Tape between sensors. A sensor touching tape means the car drifted
     * the other way, so turn TOWARD that sensor. */
    if (L && !R)      want = STEER_LEFT;
    else if (R && !L) want = STEER_RIGHT;
    else if (L && R)  want = STEER_STRAIGHT;     /* both see tape: go straight */
    else              want = STEER_STRAIGHT;
#else
    /* Both sensors on the tape. Losing one means the tape curved the
     * other way, so turn toward the sensor that still sees tape. */
    if (L && R)       want = STEER_STRAIGHT;
    else if (L)       want = STEER_LEFT;
    else if (R)       want = STEER_RIGHT;
    else              want = last_turn;          /* lost line: search */
#endif

    /* Keep turning briefly after the sensor clears so the car swings
     * back over the line instead of wobbling along the edge. */
    if (want != STEER_STRAIGHT) {
        if (want != steer) {
            turn_ticks = 0U;
        }
        steer = want;
        last_turn = want;
        hold_ticks = MS_TO_TICKS(HOLD_TURN_MS);
    } else if (hold_ticks > 0U) {
        hold_ticks--;
    } else {
        steer = STEER_STRAIGHT;
        turn_ticks = 0U;
    }

    /* ---------- drive ---------- */
    if (steer == STEER_STRAIGHT) {
        drive(SPEED_STRAIGHT, SPEED_STRAIGHT);
    } else {
        int inner = (turn_ticks >= MS_TO_TICKS(PIVOT_AFTER_MS))
                        ? SPEED_PIVOT_INNER : SPEED_TURN_INNER;
        turn_ticks++;
        if (steer == STEER_LEFT) {
            drive(inner, SPEED_TURN_OUTER);
        } else {
            drive(SPEED_TURN_OUTER, inner);
        }
    }
}

/* =====================================================================
 * Test modes
 * ===================================================================== */

/* Run the PWM for a while (also keeps the sonar FIFO drained). */
static void run_ticks(u32 ticks)
{
    while (ticks-- > 0U) {
        (void)poll_sonar();
        pwm_tick();
        usleep(TICK_US);
    }
}

static void sensor_test(void)
{
    u32 last_dist = 0U, age_ticks = 0U, report = 0U;
    u8 have = 0U;

    serial_text("SENSOR TEST - motors stay off\n");
    serial_text("Slide tape under each sensor; put a hand in front of the sonar.\n");

    while (1) {
        SonarReading s = poll_sonar();
        if (s.valid != 0U) {
            last_dist = s.distance_inches;
            age_ticks = 0U;
            have = 1U;
        } else {
            age_ticks++;
        }

        if (++report >= MS_TO_TICKS(250U)) {
            LightSensors l = read_light_sensors();
            u32 raw = Xil_In32(LS1_GPIO_BASE + GPIO_DATA_OFFSET) & 0xFU;
            report = 0U;

            serial_text("LS1 raw=");  serial_hex8(raw);
            serial_text("  S1=");     serial_uint(l.s1);
            serial_text(on_tape(l.s1) ? "(TAPE)" : "(floor)");
            serial_text("  S4=");     serial_uint(l.s4);
            serial_text(on_tape(l.s4) ? "(TAPE)" : "(floor)");
            serial_text("  | sonar ");
            if (have != 0U && age_ticks < MS_TO_TICKS(SONAR_STALE_MS)) {
                serial_uint(last_dist);
                serial_text(" in");
            } else {
                serial_text("NO DATA");
            }
            serial_text("  bytes=");  serial_uint(sonar_bytes_rx);
            serial_text(" err=");     serial_uint(sonar_errors);
            serial_text("\n");
        }
        usleep(TICK_US);
    }
}

static void motor_step(const char *what, int left, int right, u32 ms)
{
    serial_text(what);
    serial_text("\n");
    drive(left, right);
    run_ticks(MS_TO_TICKS(ms));
}

static void motor_test(void)
{
    serial_text("MOTOR TEST - wheels must be off the ground!\n");
    run_ticks(MS_TO_TICKS(2000U));
    while (1) {
        motor_step("LEFT wheel forward only",       60,   0, 2000U);
        motor_step("stop",                           0,   0, 1000U);
        motor_step("RIGHT wheel forward only",       0,  60, 2000U);
        motor_step("stop",                           0,   0, 1000U);
        motor_step("both forward, slow (40%)",      40,  40, 2000U);
        motor_step("both forward, fast (100%)",    100, 100, 2000U);
        motor_step("stop",                           0,   0, 1000U);
        motor_step("pivot LEFT (L back, R fwd)",   -50,  50, 2000U);
        motor_step("pivot RIGHT (L fwd, R back)",   50, -50, 2000U);
        motor_step("stop - repeating in 3 s",        0,   0, 3000U);
    }
}

/* ===================================================================== */
int main(void)
{
    SonarReading latest_sonar = {0U, 0U};
    LightSensors light;

    /* Motor safety: outputs low before anything else. */
    Xil_Out32(MOTOR_GPIO_BASE + GPIO_DATA_OFFSET, 0U);
    Xil_Out32(MOTOR_GPIO_BASE + GPIO_TRI_OFFSET, MOTOR_GPIO_INPUT_MASK);
    Xil_Out32(LS1_GPIO_BASE + GPIO_TRI_OFFSET, 0xFU);      /* all inputs */
    motor_write(0U, cur_dir1, 0U, cur_dir2);

    /* Throw away anything the sonar sent during configuration. */
    XUartLite_WriteReg(SONAR_UART_BASE, XUL_CONTROL_REG_OFFSET,
                       XUL_CR_FIFO_RX_RESET);

    serial_text("\n=== Arty Robot Lab 8 ===\n");

    if (RUN_MODE == MODE_SENSOR_TEST) {
        sensor_test();
    } else if (RUN_MODE == MODE_MOTOR_TEST) {
        motor_test();
    }

    /* MODE_RUN */
    serial_text("Line follower starting in ");
    serial_uint(START_DELAY_MS);
    serial_text(" ms\n");
    run_ticks(MS_TO_TICKS(START_DELAY_MS));
    serial_text("GO\n");
    drive(KICK_SPEED, KICK_SPEED);
    run_ticks(MS_TO_TICKS(KICK_MS));

    while (1) {
        SonarReading new_sonar = poll_sonar();
        latest_sonar.valid = 0U;
        if (new_sonar.valid != 0U) {
            latest_sonar = new_sonar;   /* valid=1 only on the tick it arrives */
        }

        light = read_light_sensors();
        line_follow_step(light, latest_sonar);
        pwm_tick();

        usleep(TICK_US);
    }
}
