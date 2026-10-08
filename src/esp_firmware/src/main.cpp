/*
 * ESP Controller - merge of MainCode.ino + motor_encoder_test_v2.ino
 * Board : ESP32-S3 (MainPCB, custom) / ESP32 Arduino Core 3.x
 *
 * Core split:
 *   Core 1 (loop)             : Command handling - Jetson protocol, USB debug,
 *                               button, servo/steering, battery measurement.
 *                               Allowed to block.
 *   Core 0 (motorControlTask) : Drive - VNH5019, encoder, PID position controller.
 *                               The only place that touches the motor pins.
 *   Core 0 (pixelTask)        : RGBW status LED (SK6812) - animations at 50 Hz.
 *                               Only reads its setpoint, never blocks the drive.
 *
 * Hand-over between the cores: exclusively via the setXxx/getXxx
 * functions, protected by a spinlock, plus a queue for the results of
 * completed position moves. Never write directly
 * to the setpoints - otherwise the motor task sees a new direction
 * with the old duty.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <ESP32Encoder.h>
#include <esp_timer.h>
#include "SCServo.h"

// ==========================================
// 0. CONSOLE - USB-CDC and optionally UART0
// ==========================================
//
// The ESP32-S3 can output the console in two ways:
//   - native USB-Serial-JTAG (GPIO19/20). With ARDUINO_USB_CDC_ON_BOOT=1 and
//     ARDUINO_USB_MODE=1, "Serial" points exactly there (see platformio.ini).
//   - UART0 (GPIO43/44), where some boards have a USB-UART converter attached.
//
// Pitfall during bring-up: HWCDC::write() discards the data completely
// as long as isCDC_Connected() returns false - i.e. as long as the host has
// not established the CDC connection. A terminal that does not set DTR
// therefore never sees anything, even though the COM port exists and the ESP is running.
//
// -DCONSOLE_MIRROR_UART0=1 (env "esp32-s3-uart0") additionally routes the console
// to UART0. Then a USB-TTL adapter on GPIO43 (ESP-TX) / GPIO44
// (ESP-RX) is enough as an emergency exit, independent of the USB state. Only enable
// this if these two pins are free on the board.
#ifndef CONSOLE_MIRROR_UART0
#define CONSOLE_MIRROR_UART0 0
#endif

#if CONSOLE_MIRROR_UART0
class ConsoleIO : public Print {
public:
    void begin(unsigned long baud) {
        HWCDCSerial.begin();
        Serial0.begin(baud);          // UART0 on its default pins
    }
    // Always on UART0, on CDC only when connected: otherwise every call costs
    // up to tx_timeout_ms (100 ms) on the TX semaphore, and that inside loop().
    size_t write(uint8_t c) override {
        Serial0.write(c);
        if (HWCDCSerial) HWCDCSerial.write(c);
        return 1;
    }
    size_t write(const uint8_t* b, size_t n) override {
        Serial0.write(b, n);
        if (HWCDCSerial) HWCDCSerial.write(b, n);
        return n;
    }
    void flush() override {
        Serial0.flush();
        if (HWCDCSerial) HWCDCSerial.flush();
    }
    int available() { return HWCDCSerial.available() + Serial0.available(); }
    int read() {
        if (HWCDCSerial.available()) return HWCDCSerial.read();
        if (Serial0.available())     return Serial0.read();
        return -1;
    }
};
static ConsoleIO Console;

// The core itself defines "Serial" as a macro for HWCDCSerial. Redirect it instead
// of touching every call site - "Serial1"/"Serial2" are separate tokens and
// remain unaffected.
#undef Serial
#define Serial Console
#endif  // CONSOLE_MIRROR_UART0

// ==========================================
// 1. PIN AND PROTOCOL DEFINITIONS
// ==========================================

// --- Drive: VNH5019 + quadrature encoder ---
constexpr int PIN_MOTOR_PWM = 41;   // VNH5019 PWM
constexpr int PIN_MOTOR_INA = 42;   // VNH5019 INA
constexpr int PIN_MOTOR_INB = 38;   // VNH5019 INB
// Current sense: on the new board on GPIO8 = ADC1_CH7, so readable as analog
// (the old GPIO39 was not). For scaling see CS_MV_PER_A_NOMINAL.
constexpr int PIN_MOTOR_CS  = 8;    // VNH5019 CS, ADC1_CH7
constexpr int PIN_ENC_A     = 15;   // Encoder A
constexpr int PIN_ENC_B     = 16;   // Encoder B

// Driving direction. Which rotation direction is "forward" is decided by the
// wiring, not by the protocol: which VNH5019 terminals the motor leads are
// connected to and which way round the encoder is mounted.
// Set to true if a POSITIVE motor value makes the vehicle drive
// backwards.
//
// The switch flips motor output AND encoder together - this is intentional.
// If the two had different signs, the position controller would never
// find its target: it would then pull away from the setpoint position
// instead of towards it, until the timeout hits or something breaks.
constexpr bool DRIVE_INVERT = true;

// --- Peripherals ---
// Servo bus via the half-duplex buffers of the new board, pins from the ESP's view:
//   ESP-TX (GPIO17) -> A input of the SN74LVC1G126 (drives the bus line)
//   ESP-RX (GPIO18) <- Y output of the SN74LVC1G125 (attached to the bus line)
// Direction switching is done by the hardware via the two OE inputs;
// the firmware has no direction pin. Swappable at runtime via "svpin<rx>,<tx>"
// in case the assignment was populated the other way round after all.
#define PIN_SERVO_RX    18
#define PIN_SERVO_TX    17
#define PIN_JETSON_RX   10
#define PIN_JETSON_TX   11
#define PIN_LED         13
#define PIN_PIXEL       40          // SK6812 RGBW data in (addressable LED)
#define PIN_BUTTON      9
#define PIN_BATTERY     1           // ADC1_CH0, divider 100k / 22k to GND

// --- PWM ---
// Keep 10 bits so that CMD_MOTOR (16-bit speed 0..1023) stays unchanged.
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES  = 10;
constexpr uint16_t DUTY_MAX = (1 << PWM_RES) - 1;   // 1023

// --- Protocol: Jetson -> ESP ---
#define START_BYTE      0xA5

// Second start byte for packets with a send timestamp:
//
//   A6 <CMD> <uint32 t_tx_us> <PAYLOAD as with A5>
//
// The stamp is the lower 32 bits of the ESP clock (esp_timer, microseconds
// since boot) and denotes the moment the *last byte* of the packet leaves
// the line - not the moment of the call. The time for shifting out
// (10 bits per byte at 115200 baud) is included so that
// both sides share the same reference point and a long packet does not
// appear to start earlier than a short one.
//
// The conversion from ESP clock to Jetson clock uses the offset from the ping-pong of
// CMD_TIME_SYNC / CMD_TIME_RSP (see docs/JETSON_BRIDGE.md).
//
// Stamping is OFF by default and is enabled with CMD_STAMP_MODE (or "ts1" on the
// USB console) - a bridge that does not know 0xA6 would otherwise see
// nothing but garbage. The ESP's receive path always understands 0xA6, so the bridge
// may stamp its commands at any time.
#define START_BYTE_TS   0xA6
#define CMD_MOTOR       0x10   // 3B: dir, speedHi, speedLo
#define CMD_SERVO       0x20   // 3B: id, pctHi, pctLo
#define CMD_LED         0x30   // 1B: on/off
#define CMD_PIXEL       0x31   // 9B: mode, R, G, B, W, brightness, uint16 period ms, count (all LEDs)
#define CMD_PIXEL_ONE   0x32   // 10B: LED index + the 9 bytes of CMD_PIXEL
#define CMD_CALIBRATE   0x40   // 0B: starts the manual calibration
#define CMD_CAL         0x41   // 2B: action, argument (manual calibration)
#define CMD_TORQUE      0x50   // 0B
#define CMD_TRIM        0x60   // 1B: 0=left 1=right 2=save
#define CMD_PID_SET     0x80   // 5B: paramId, int32 value (x1000)
#define CMD_PID_GET     0x81   // 0B
#define CMD_PID_SAVE    0x83   // 0B: write current parameters to NVS
#define CMD_MOVE        0x90   // 5B: moveId, int32 target in 1/10 degree (absolute)
#define CMD_MOVE_ABORT  0x91   // 0B
#define CMD_PROGRESS    0x92   // 0B
#define CMD_BATTERY     0xA0   // 0B
#define CMD_TIME_SYNC   0xB0   // 1B: seq  -> reply CMD_TIME_RSP
#define CMD_STAMP_MODE  0xB2   // 1B: 0=off 1=on -> reply CMD_STAMP_RSP
#define CMD_TELEM_RATE  0xC0   // 2B: uint16 interval in ms, 0 = off
#define CMD_EMERGENCY   0xFF   // 0B

// Actions in CMD_CAL
#define CAL_ACT_START   0x00   // start calibration mode
#define CAL_ACT_MINUS   0x01   // one step towards position 0
#define CAL_ACT_PLUS    0x02   // one step towards position 1023
#define CAL_ACT_CENTER  0x03   // current position = center
#define CAL_ACT_LEFT    0x04   // current position = left end stop
#define CAL_ACT_RIGHT   0x05   // current position = right end stop
#define CAL_ACT_SAVE    0x06   // validate, write to NVS, exit mode
#define CAL_ACT_ABORT   0x07   // abort, stored values are kept
#define CAL_ACT_FREE    0x08   // torque off - move steering by hand
#define CAL_ACT_HOLD    0x09   // torque on - hold current position
#define CAL_ACT_GOTO_C  0x0A   // move to the stored center
#define CAL_ACT_STEP    0x0B   // arg = new step size in ticks
#define CAL_ACT_STATUS  0x0C   // only query state

// Status codes in CMD_CAL_RSP
#define CAL_ST_OK       0x00   // action executed
#define CAL_ST_SAVED    0x01   // calibration saved, mode exited
#define CAL_ST_REJECTED 0x02   // incomplete or implausible - not saved
#define CAL_ST_NOSERVO  0x03   // servo does not respond
#define CAL_ST_INACTIVE 0x04   // action requires an active calibration mode
#define CAL_ST_LIMIT    0x05   // end of range 0 / 1023 reached

// --- Protocol: ESP -> Jetson ---
#define CMD_CAL_RSP     0x42   // 11B: active, flags, status, int16 pos/center/left/right
#define CMD_BUTTON      0x70   // 1B: 0x01 = pressed
#define CMD_PID_RSP     0x82   // 12B: int32 Kp, Ki, Kd (each x1000)
#define CMD_PID_SAVED   0x84   // 1B: 0x00 = saved, 0x01 = error
#define CMD_MOVE_DONE   0x93   // 6B: moveId, status, int32 actual pos (1/10 degree)
#define CMD_PROGRESS_RSP 0x94  // 11B: moveId, active, percent, int32 actual, int32 target
#define CMD_BATTERY_RSP 0xA1   // 6B: int32 pack mV, int16 cell mV
#define CMD_BATTERY_WARN 0xA2  // 6B: like CMD_BATTERY_RSP, unsolicited on undervoltage
#define CMD_TIME_RSP    0xB1   // 17B: seq, int64 t_rx_us, int64 t_tx_us
#define CMD_STAMP_RSP   0xB3   // 1B: current stamp mode
#define CMD_TELEMETRY   0xC1   // 12B: int32 pos, int32 speed, int16 duty, int16 mA

// Status codes in CMD_MOVE_DONE
#define MOVE_OK         0x00
#define MOVE_TIMEOUT    0x01
#define MOVE_ABORTED    0x02

#define JETSON_TIMEOUT  5000

// ==========================================
// 2. SHARED STATE BETWEEN THE CORES
// ==========================================

enum MotorMode : uint8_t {
    MOTOR_COAST    = 0,   // H-bridge high-impedance, motor coasts to a stop
    MOTOR_DRIVE    = 1,   // drives open-loop with duty in direction reverse
    MOTOR_BRAKE    = 2,   // short-circuit brake with duty as braking force
    MOTOR_POSITION = 3    // PID controls to target position
};

struct MotorCommand {
    MotorMode mode    = MOTOR_COAST;
    bool      reverse = false;
    uint16_t  duty    = 0;          // 0..DUTY_MAX
};

// PID parameters. Initial values are estimates for the Pololu 25D with 408
// counts/rev - must be tuned on the real setup (command "pid").
struct PidParams {
    float    kp        = 4.0f;    // duty per count of control error
    float    ki        = 0.5f;    // duty per (count * second)
    float    kd        = 0.10f;   // duty per (count / second)
    float    iTermLim  = 200.0f;  // limit of the I term in duty (anti-windup)
    uint16_t maxDuty   = 700;     // actuator output limit
    // Breakaway duty against static friction. Just before the target the control
    // error is so small that kp*err drops below the breakaway threshold - the motor
    // stalls and the move runs into the timeout. As long as the target has not
    // been reached, at least this value is applied. 0 = off.
    uint16_t minDuty   = 0;
    int32_t  tolDeg10  = 50;      // target window, 1/10 degree (50 = 5.0 degrees)
    uint32_t settleMs  = 200;     // this long inside the window -> done
    uint32_t timeoutMs = 10000;   // abort if target not reached
};

struct MoveState {
    uint8_t  id         = 0;
    bool     active     = false;
    uint8_t  progress   = 0;      // 0..100 %
    long     startCnt   = 0;
    long     targetCnt  = 0;
};

struct MoveResult {
    uint8_t id;
    uint8_t status;
    int32_t finalDeg10;
};

static MotorCommand   g_motorCmd;
static PidParams      g_pid;
static MoveState      g_move;
static unsigned long  g_lastCmdTime = 0;   // watchdog timestamp
static portMUX_TYPE   g_motorMux = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t  g_moveResultQueue = nullptr;

// Telemetry: motor task writes, core 1 only reads for output.
static volatile float   g_rpm        = 0.0f;
static volatile long    g_encCount   = 0;
static volatile int16_t g_dutySigned = 0;   // + forward, - backward

ESP32Encoder encoder;
// Enter COUNTS_PER_REV of the ACTIVE motor:
// ServoCity DE3 (open collector): 3 PPR x 4 x 42.875 gearbox = ~514.5
// Pololu 25D #4841 (push-pull)  : 48 CPR x 4.4 gearbox = 211.2 (output shaft)
constexpr float COUNTS_PER_REV = 408.0f;

// --- Speed measurement ---
// Sampling runs at the motor task's rate (TASK_PERIOD, 10 ms). The measurement
// is taken over a SLIDING window of the last g_speedWindow samples:
//
//   speed = (count[now] - count[now - n]) / (t[now] - t[now - n])
//
// This yields a new value at every sample - i.e. at 100 Hz - while the
// window can still be longer than 10 ms. The two quantities are
// decoupled, and that is exactly the point of the exercise: previously the
// output rate was tied to the window length, one value every 100 ms.
//
// The difference of the two edge values IS the mean over the window -
// all intermediate values cancel out. Hence no mean of the
// individual measurements and no EMA: the delay is exactly half a
// window and can be written down, instead of staying hidden in the fog
// as a time constant.
//
// What the window length costs is resolution. A single pulse in a
// window of n x 10 ms corresponds to:
//
//   n = 1  (10 ms)   14.7 rpm   88 deg/s   <- as coarse as full deflection
//   n = 5  (50 ms)    2.9 rpm   17.6 deg/s
//   n = 10 (100 ms)   1.5 rpm    8.8 deg/s
//
// at 408 pulses per revolution and roughly 30 rpm top speed. The window
// trades noise against delay, adjustable at runtime via
// "sw<n>" on the console.
constexpr uint8_t SPEED_WINDOW_MAX = 32;
static volatile uint8_t g_speedWindow = 1;

// --- Output shaft conversion: 1/10 degree <-> encoder counts ---
static inline long deg10ToCounts(int32_t deg10) {
    return lroundf(deg10 * COUNTS_PER_REV / 3600.0f);
}
static inline int32_t countsToDeg10(long counts) {
    return (int32_t)lroundf(counts * 3600.0f / COUNTS_PER_REV);
}

// ==========================================
// 3. ACCESS TO THE SHARED STATE
// ==========================================

// Open-loop control command. Feeds the watchdog - every valid command keeps
// the drive alive, whether from the Jetson or USB. Aborts a running
// position move so that two controllers do not fight over the motor.
void setMotorCommand(MotorMode mode, bool reverse, uint16_t duty) {
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    portENTER_CRITICAL(&g_motorMux);
    bool wasMoving = g_move.active;
    uint8_t movingId = g_move.id;
    long curCnt = g_encCount;
    g_motorCmd.mode    = mode;
    g_motorCmd.reverse = reverse;
    g_motorCmd.duty    = duty;
    g_move.active      = false;
    g_lastCmdTime      = millis();
    portEXIT_CRITICAL(&g_motorMux);

    if (wasMoving && g_moveResultQueue) {
        MoveResult r = { movingId, MOVE_ABORTED, countsToDeg10(curCnt) };
        xQueueSend(g_moveResultQueue, &r, 0);
    }
}

// Start a position move. deltaDeg10 is RELATIVE to the current position, in
// 1/10 degree of the output shaft. 900 = "turn 90 degrees further", -450 = "45 degrees
// back". This makes a move reset-proof: it does not need a zero point.
// Only one move runs at a time. If a new one is started while another is
// still active, the old ID gets a MOVE_ABORTED - otherwise the Jetson waits
// forever for an acknowledgement that never arrives.
void startMove(uint8_t id, int32_t deltaDeg10) {
    portENTER_CRITICAL(&g_motorMux);
    bool    superseded = g_move.active;
    uint8_t oldId      = g_move.id;
    long    curCnt     = g_encCount;
    g_motorCmd.mode   = MOTOR_POSITION;
    g_motorCmd.duty   = 0;
    g_move.id         = id;
    g_move.active     = true;
    g_move.progress   = 0;
    g_move.startCnt   = curCnt;
    g_move.targetCnt  = curCnt + deg10ToCounts(deltaDeg10);
    g_lastCmdTime     = millis();
    portEXIT_CRITICAL(&g_motorMux);

    if (superseded && g_moveResultQueue) {
        MoveResult r = { oldId, MOVE_ABORTED, countsToDeg10(curCnt) };
        xQueueSend(g_moveResultQueue, &r, 0);
    }
}

void abortMove() {
    setMotorCommand(MOTOR_COAST, false, 0);
}

MotorCommand getMotorCommand(unsigned long* lastCmdTimeOut) {
    MotorCommand c;
    portENTER_CRITICAL(&g_motorMux);
    c = g_motorCmd;
    if (lastCmdTimeOut) *lastCmdTimeOut = g_lastCmdTime;
    portEXIT_CRITICAL(&g_motorMux);
    return c;
}

MoveState getMoveState() {
    MoveState m;
    portENTER_CRITICAL(&g_motorMux);
    m = g_move;
    portEXIT_CRITICAL(&g_motorMux);
    return m;
}

PidParams getPid() {
    PidParams p;
    portENTER_CRITICAL(&g_motorMux);
    p = g_pid;
    portEXIT_CRITICAL(&g_motorMux);
    return p;
}

void setPid(const PidParams& p) {
    portENTER_CRITICAL(&g_motorMux);
    g_pid = p;
    portEXIT_CRITICAL(&g_motorMux);
}

// ==========================================
// 4. SERVO / STEERING - state
// ==========================================

Preferences prefs;
int trimOffset = 0;

const int SERVO_ID = 1;

// SC series (SCSCL, big-endian): position range 0..1023 (10 bits), center 512.
// The SC09 is an SCS servo, NOT an STS - it replies big-endian and with half
// the resolution. Hence SCSCL instead of SMS_STS.
constexpr int SERVO_POS_MAX    = 1023;
constexpr int SERVO_CENTER_DEF = 512;
// SCSCL WritePosEx takes speed (acc/time internally 0). On the SC09, speed 0 is NOT
// "maximum speed" but "no speed" - the target lands in the
// register, but the servo does not move. Hence > 0 for steering as well.
constexpr uint16_t SERVO_SPEED_FAST  = 1500;   // steering: brisk, but with speed
constexpr uint16_t SERVO_SPEED_CALIB = 500;    // calibration: slowed down
constexpr uint8_t  SERVO_ACC         = 50;      // ignored by SCSCL, kept for compatibility
// SC09: 0..1023 ticks over ~300 degrees full range. For display only.
constexpr float    SERVO_DEG_PER_TICK = 300.0f / (SERVO_POS_MAX + 1);

// Convention from the calibration: leftLimit is the end stop with the LARGER
// raw position, rightLimit the one with the smaller, centerLimit the manually
// set straight-ahead position. softwareCenterPos = centerLimit + trimOffset.
// The mapping in CMD_SERVO is computed symmetrically, so a mirror-mounted
// steering also works with swapped values - only the sign
// direction of the steering command is then flipped.
int softwareCenterPos = SERVO_CENTER_DEF;
int centerLimit = SERVO_CENTER_DEF;
int leftLimit  = SERVO_POS_MAX;
int rightLimit = 0;

int servoManualPos = SERVO_CENTER_DEF;   // position of the manual console control (j/l/m)

// Serial2 pins for the servo bus, changeable at runtime ("svpin<rx>,<tx>").
// For narrowing down swapped wiring without having to solder.
int g_servoRx = PIN_SERVO_RX;
int g_servoTx = PIN_SERVO_TX;
uint32_t g_servoBaud   = 1000000;   // can be lowered via "svbaud<n>" for scope measurements
bool     g_servoTxTest = false;      // continuously sends 0x55 for oscilloscope viewing

TaskHandle_t MotorControlTaskHandle;

volatile bool buttonTriggered = false;

void IRAM_ATTR buttonISR() {
    buttonTriggered = true;
}

// ==========================================
// 5. MOTOR DRIVER (VNH5019) - call only from the motor task (core 0)!
// ==========================================

class MotorDriver {
private:
    uint8_t pwmPin, inaPin, inbPin;

public:
    MotorDriver(uint8_t pwm, uint8_t ina, uint8_t inb)
        : pwmPin(pwm), inaPin(ina), inbPin(inb) {}

    void begin() {
        pinMode(inaPin, OUTPUT);
        pinMode(inbPin, OUTPUT);
        ledcAttach(pwmPin, PWM_FREQ, PWM_RES);
        coast();
    }

    void coast() {
        ledcWrite(pwmPin, 0);
        digitalWrite(inaPin, LOW);
        digitalWrite(inbPin, LOW);
    }

    void drive(bool reverse, uint16_t duty) {
        // != acts like XOR on bool: DRIVE_INVERT flips the direction.
        const bool rev = (reverse != DRIVE_INVERT);
        digitalWrite(inaPin, rev ? LOW  : HIGH);
        digitalWrite(inbPin, rev ? HIGH : LOW);
        ledcWrite(pwmPin, duty);
    }

    // INA=INB=LOW with duty > 0: short-circuit brake to GND.
    void brake(uint16_t duty) {
        digitalWrite(inaPin, LOW);
        digitalWrite(inbPin, LOW);
        ledcWrite(pwmPin, duty);
    }
};

// ==========================================
// 6. MOTOR CONTROL TASK (core 0)
// ==========================================

// Ramp: max. duty change per task cycle.
// 25 -> full scale in ~410 ms. Higher = snappier, but larger current spike.
constexpr uint16_t RAMP_STEP    = 25;
constexpr uint32_t TASK_PERIOD  = 10;    // ms - also the sampling interval
                                         // of the speed measurement

void motorControlTask(void* pvParameters) {
    MotorDriver* motor = (MotorDriver*)pvParameters;

    uint16_t  appliedDuty    = 0;       // what is actually applied right now
    bool      appliedReverse = false;
    MotorMode appliedMode    = MOTOR_COAST;

    // Ring buffer of the speed measurement: encoder count and timestamp per
    // sample. Timestamp in microseconds, because 10 ms in milliseconds is only
    // 10 steps - the quantization error of the time would otherwise be as
    // large as the window itself.
    long     speedCnt[SPEED_WINDOW_MAX] = {0};
    uint32_t speedUs[SPEED_WINDOW_MAX]  = {0};
    uint8_t  speedHead = 0;    // next write slot
    uint8_t  speedFill = 0;    // how many slots are already valid

    // PID state
    float integral   = 0.0f;
    long  lastPosCnt = 0;
    bool  pidPrimed  = false;
    unsigned long moveStartMs = 0;
    unsigned long inWindowSince = 0;
    bool    wasMoving  = false;
    uint8_t lastMoveId = 0;

    const float dt = TASK_PERIOD / 1000.0f;

    for (;;) {
        unsigned long now = millis();
        long posCnt = (long)encoder.getCount();
        g_encCount = posCnt;

        unsigned long lastCmdTime;
        MotorCommand cmd = getMotorCommand(&lastCmdTime);
        MoveState    mv  = getMoveState();
        PidParams    pid = getPid();

        // --- Watchdog ---
        // Only applies in open-loop control mode. A position move may take longer
        // than JETSON_TIMEOUT without a follow-up command - there
        // pid.timeoutMs is the safety net.
        if (cmd.mode == MOTOR_DRIVE && (now - lastCmdTime > JETSON_TIMEOUT)) {
            cmd.mode = MOTOR_COAST;
            cmd.duty = 0;
        }

        uint16_t wantDuty    = 0;
        bool     wantReverse = cmd.reverse;
        MotorMode wantMode   = cmd.mode;

        // ------------------------------------------------
        // PID position controller
        // ------------------------------------------------
        if (cmd.mode == MOTOR_POSITION && mv.active) {
            // New move = none active before OR a different ID. Without the
            // ID comparison, a superseding move would inherit the integral and start time
            // of the old one and run into the timeout prematurely.
            if (!wasMoving || mv.id != lastMoveId) {
                integral      = 0.0f;
                lastPosCnt    = posCnt;
                pidPrimed     = false;
                moveStartMs   = now;
                inWindowSince = 0;
                wasMoving     = true;
                lastMoveId    = mv.id;
            }

            long  errCnt = mv.targetCnt - posCnt;
            float err    = (float)errCnt;

            long tolCnt = deg10ToCounts(pid.tolDeg10);
            if (tolCnt < 1) tolCnt = 1;

            // D term on the measured value instead of the error: no
            // derivative kick when a new target is set.
            float dMeas = pidPrimed ? ((float)(posCnt - lastPosCnt) / dt) : 0.0f;
            lastPosCnt  = posCnt;
            pidPrimed   = true;

            integral += err * dt;
            float iTerm = pid.ki * integral;
            // Anti-windup: limit I term in duty and back-calculate the integral
            if (iTerm >  pid.iTermLim) { iTerm =  pid.iTermLim; integral = (pid.ki != 0.0f) ?  pid.iTermLim / pid.ki : 0.0f; }
            if (iTerm < -pid.iTermLim) { iTerm = -pid.iTermLim; integral = (pid.ki != 0.0f) ? -pid.iTermLim / pid.ki : 0.0f; }

            float out = pid.kp * err + iTerm - pid.kd * dMeas;

            if (labs(errCnt) <= tolCnt) {
                // Dead band: no further correction inside the target window. Otherwise
                // a residual duty from the I term is applied during the settle time and
                // the drive pushes against friction without achieving anything.
                out = 0.0f;
                integral = 0.0f;
            } else if (pid.minDuty > 0 && fabsf(out) < (float)pid.minDuty) {
                // Overcome static friction: outside the target window never
                // apply less than minDuty. Direction comes from the sign
                // of the control error, not from out - out can briefly have the
                // wrong sign due to the D term.
                out = (errCnt < 0) ? -(float)pid.minDuty : (float)pid.minDuty;
            }

            uint16_t lim = min<uint16_t>(pid.maxDuty, DUTY_MAX);
            if (out >  (float)lim) out =  (float)lim;
            if (out < -(float)lim) out = -(float)lim;

            wantReverse = (out < 0.0f);
            wantDuty    = (uint16_t)fabsf(out);
            wantMode    = MOTOR_POSITION;

            // --- Progress ---
            long span = labs(mv.targetCnt - mv.startCnt);
            uint8_t pct = 100;
            if (span > 0) {
                long doneCnt = labs(posCnt - mv.startCnt);
                if (doneCnt > span) doneCnt = span;
                pct = (uint8_t)((doneCnt * 100) / span);
            }
            portENTER_CRITICAL(&g_motorMux);
            g_move.progress = pct;
            portEXIT_CRITICAL(&g_motorMux);

            // --- Target reached? ---
            bool finished = false;
            uint8_t status = MOVE_OK;

            if (labs(errCnt) <= tolCnt) {
                if (inWindowSince == 0) inWindowSince = now;
                if (now - inWindowSince >= pid.settleMs) { finished = true; status = MOVE_OK; }
            } else {
                inWindowSince = 0;
            }

            if (!finished && (now - moveStartMs > pid.timeoutMs)) {
                finished = true;
                status = MOVE_TIMEOUT;
            }

            if (finished) {
                portENTER_CRITICAL(&g_motorMux);
                g_move.active      = false;
                g_move.progress    = (status == MOVE_OK) ? 100 : g_move.progress;
                g_motorCmd.mode    = MOTOR_COAST;
                g_motorCmd.duty    = 0;
                portEXIT_CRITICAL(&g_motorMux);

                MoveResult r = { mv.id, status, countsToDeg10(posCnt) };
                xQueueSend(g_moveResultQueue, &r, 0);

                wantMode  = MOTOR_COAST;
                wantDuty  = 0;
                wasMoving = false;
            }
        } else {
            wasMoving = false;
            if (cmd.mode == MOTOR_POSITION) {   // move finished, not yet switched over
                wantMode = MOTOR_COAST;
                wantDuty = 0;
            } else {
                wantDuty = (cmd.mode == MOTOR_COAST) ? 0 : cmd.duty;
            }
        }

        // --- Every mode or direction change goes through duty 0 ---
        // Otherwise INA/INB switch under load (current spike), or - worse -
        // an emergency stop leaves appliedMode at DRIVE and the braking force
        // acts as a drive command.
        bool changing = (wantMode != appliedMode) ||
                        ((wantMode == MOTOR_DRIVE || wantMode == MOTOR_POSITION) &&
                         wantReverse != appliedReverse);
        if (changing && appliedDuty > 0) {
            wantDuty = 0;
        }

        // Take over direction/mode as soon as the bridge is de-energized.
        // MUST come before the ramp: otherwise appliedDuty is never 0 again after the first
        // ramp step and appliedMode stays stuck at COAST forever
        // - the motor then never even starts.
        if (appliedDuty == 0) {
            appliedReverse = wantReverse;
            appliedMode    = wantMode;
        }

        // --- Slew rate limiter ---
        // Only limit when accelerating. Reducing is electrically
        // uncritical (equivalent to the normal PWM duty cycle) and must
        // take effect immediately, otherwise the emergency stop takes half a second.
        // Braking is also applied at full strength immediately.
        if (appliedDuty < wantDuty) {
            uint16_t step = (appliedMode == MOTOR_BRAKE) ? DUTY_MAX : RAMP_STEP;
            appliedDuty = min<uint16_t>(wantDuty, appliedDuty + step);
        } else {
            appliedDuty = wantDuty;
        }

        switch (appliedMode) {
            case MOTOR_DRIVE:
            case MOTOR_POSITION: motor->drive(appliedReverse, appliedDuty); break;
            case MOTOR_BRAKE:    motor->brake(appliedDuty);                 break;
            case MOTOR_COAST:
            default:             motor->coast();                            break;
        }

        // What is actually applied at the bridge, for plotter and telemetry.
        g_dutySigned = (appliedMode == MOTOR_DRIVE || appliedMode == MOTOR_POSITION)
                       ? (int16_t)(appliedReverse ? -(int)appliedDuty : (int)appliedDuty)
                       : 0;

        // --- Speed: sliding window ---
        speedCnt[speedHead] = posCnt;
        speedUs[speedHead]  = micros();
        speedHead = (uint8_t)((speedHead + 1) % SPEED_WINDOW_MAX);
        if (speedFill < SPEED_WINDOW_MAX) speedFill++;

        {
            uint8_t want = g_speedWindow;
            if (want < 1) want = 1;
            if (want > SPEED_WINDOW_MAX - 1) want = SPEED_WINDOW_MAX - 1;

            // As long as the ring is not yet full, look back as far as
            // possible - otherwise the "oldest" value would be a zero from the
            // initialization and the speed would shoot to absurd values at startup.
            uint8_t back = (want < speedFill) ? want : (uint8_t)(speedFill - 1);

            if (back >= 1) {
                uint8_t newest = (uint8_t)((speedHead + SPEED_WINDOW_MAX - 1)
                                           % SPEED_WINDOW_MAX);
                uint8_t oldest = (uint8_t)((newest + SPEED_WINDOW_MAX - back)
                                           % SPEED_WINDOW_MAX);
                // Real elapsed time instead of nominal period: the task can
                // get its turn late, and an assumed dt that is too short
                // inflates the speed. The subtraction is also correct across
                // the overflow of micros() (71.6 min).
                uint32_t dtUs = speedUs[newest] - speedUs[oldest];
                if (dtUs > 0) {
                    long delta = speedCnt[newest] - speedCnt[oldest];
                    g_rpm = ((float)delta / COUNTS_PER_REV)
                            * (60000000.0f / (float)dtUs);
                }
            } else {
                g_rpm = 0.0f;
            }
        }

        vTaskDelay(TASK_PERIOD / portTICK_PERIOD_MS);
    }
}

// ==========================================
// 7. BATTERY (Core 1)
// ==========================================

// Voltage divider 100k high side / 22k to GND -> factor (100+22)/22.
// Adjustable via Serial ("vc<factor>") and stored in NVS, because resistor
// tolerances make the calculated value miss by several percent.
constexpr float BATT_DIVIDER_NOMINAL = (100.0f + 22.0f) / 22.0f;   // 5.545
float battDivider = BATT_DIVIDER_NOMINAL;

// From here on the ADC is at its limit: 12 dB reaches up to ~3.1 V at the pin,
// above that it stubbornly returns its maximum code. Every higher voltage then
// looks the same - the "reading" is no longer a measurement but the ceiling itself.
// 4S full (16.8 V) ends up at ~3.03 V through the divider and stays below it.
constexpr float BATT_ADC_CLIP_MV = 3100.0f;

constexpr int   BATT_CELLS        = 4;        // 4S
constexpr float BATT_WARN_CELL    = 3.80f;    // warning threshold per cell
constexpr float BATT_RECOVER_CELL = 3.85f;    // hysteresis: only clear the warning above this
constexpr uint32_t BATT_INTERVAL  = 15000;    // measure every 15 s
constexpr uint32_t BATT_WARN_REPEAT = 60000;  // warning at most once per minute

// --- Motor current (VNH5019 CS on GPIO8) ---
// The VNH5019 mirrors a fraction of the motor current onto CS; the sense
// resistor on the board turns that into a voltage. The Pololu carrier board
// gives ~0.14 V/A - if the resistor on our own board is sized differently,
// the value is wrong. Hence adjustable via "ic<mV per A>" and stored in NVS.
// Cross-check with a current clamp or the lab power supply.
constexpr float CS_MV_PER_A_NOMINAL = 140.0f;
float csMvPerA  = CS_MV_PER_A_NOMINAL;
int   csZeroMv  = 0;      // zero point with the motor stopped ("iz")

// Raw voltage at the CS pin in mV. No zero-point subtraction - the caller does that.
float readMotorCsMv() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(PIN_MOTOR_CS);
    return sum / 16.0f;
}

float readMotorCurrentA() {
    if (csMvPerA <= 0.0f) return 0.0f;
    float a = (readMotorCsMv() - csZeroMv) / csMvPerA;
    return (a < 0.0f) ? 0.0f : a;   // CS can only report current in one direction
}

float    battPackV   = 0.0f;
float    battCellV   = 0.0f;
bool     battLow     = false;
uint32_t battLastRead = 0;
uint32_t battLastWarn = 0;

// Live output of the tap ("vm"). The 15 s interval is useless for troubleshooting:
// a flaky solder joint can only be found by wiggling it while watching.
// 0 = off. Deliberately runs as a flag instead of a blocking loop,
// so that the drive and the Jetson link keep running meanwhile.
uint32_t g_battMonMs   = 0;
uint32_t g_battMonLast = 0;

// Voltage at the divider tap in mV. analogReadMilliVolts uses the factory
// calibration of the ADC - considerably more accurate than analogRead/4095*3.3.
// A separate function because without the raw value a clipped reading cannot
// be told apart from a wrongly calibrated divider factor.
float readBatteryMv() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(PIN_BATTERY);
    return sum / 16.0f;
}

// Returns the pack voltage in volts.
float readBatteryVolts() {
    return (readBatteryMv() / 1000.0f) * battDivider;
}

// Measures the tap three times: floating, then against the internal pull
// resistors of the ESP (~45 kOhm). How much the voltage gives way reveals the
// source impedance of the node - and thus whether the intended divider is
// actually still connected to the pin. Without battery these are the expected values:
//   divider intact:   free ~0 mV, pullup ~1080 mV (3.3 V across 45k/22k)
//   22k open, 100k on a live rail: pulldown pulls to ~1.5 V
//   short to 3V3: stays high even with pulldown
void batteryPinDiagnose() {
    Serial.printf("Battery pin diagnosis on GPIO%d (battery should be disconnected):\n",
                  PIN_BATTERY);

    float mvFree = readBatteryMv();

    pinMode(PIN_BATTERY, INPUT_PULLDOWN);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    float mvDown = readBatteryMv();

    pinMode(PIN_BATTERY, INPUT_PULLUP);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    float mvUp = readBatteryMv();

    // Back to a plain analog input. The order is the same as in
    // setup(): read first (attaches the pin to the ADC channel), then attenuate.
    pinMode(PIN_BATTERY, INPUT);
    (void)analogRead(PIN_BATTERY);
    analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    Serial.printf("  free     %7.1f mV\n", mvFree);
    Serial.printf("  Pulldown %7.1f mV\n", mvDown);
    Serial.printf("  Pullup   %7.1f mV\n", mvUp);
    Serial.print("  -> ");

    if (mvFree < 300.0f) {
        Serial.println("Tap is at ground - this is the expected picture without a battery.");
        if (mvUp > 700.0f && mvUp < 1500.0f)
            Serial.println("     Pullup value matches the 22k to GND: divider is intact.");
        else
            Serial.println("     But pullup value does not match 22k to GND - check the tap.");
    } else if (mvDown < 300.0f) {
        Serial.println("Node is high-impedance: it floats, nothing pulls it to ground.");
        Serial.println("     The 22k leg to GND is open (cold solder joint/break).");
    } else if (mvDown < 2500.0f) {
        Serial.println("Something feeds in via ~100k while the 22k to GND is missing.");
        Serial.println("     Looks like the upper divider leg on a live rail.");
    } else {
        Serial.println("Driven hard at low impedance - the pulldown cannot overcome it.");
        Serial.println("     Solder bridge or wrongly fitted part to 3V3 most likely.");
    }
}

// ==========================================
// 8. SERVO HELPER FUNCTIONS (calibration & torque) - Core 1
// ==========================================

void printTorque(SCSCL* servo) {
    int rawLoad = servo->ReadLoad(SERVO_ID);
    if (rawLoad != -1) {
        int actualLoad = rawLoad & 0x3FF;   // Bits 0-9
        Serial.print("ESP: Current servo torque: ");
        Serial.println(actualLoad);
    } else {
        Serial.println("ESP: Error reading the torque!");
    }
}

// Half-duplex self-echo: every other read request chokes on our own transmitted
// packet being mirrored back and returns -1. Retrying sits that out - up to
// 6 attempts, a valid result usually arrives on the 2nd. Only affects reads;
// WritePosEx (steering) does not need it.
template <typename F>
int servoReadRetry(F fn, int tries = 6) {
    for (int i = 0; i < tries; i++) {
        int v = fn();
        if (v != -1) return v;
    }
    return -1;
}

// Writes a raw servo position 0..1023 bypassing the steering limits and
// reports the return value.
void servoWriteRaw(SCSCL* servo, int pos) {
    pos = constrain(pos, 0, SERVO_POS_MAX);
    servoManualPos = pos;
    int ret = servo->WritePosEx(SERVO_ID, pos, SERVO_SPEED_CALIB, SERVO_ACC);
    Serial.printf("Servo -> Pos %d (WritePosEx ret=%d%s)\n", pos, ret,
                  ret == -1 ? " NO RESPONSE" : "");
}

// Tests both protocols of the SCServo lib on the same bus (Serial2) and
// reports which one responds. SC series (SCSCL) and STS/SMS series (SMS_STS) are
// mutually incompatible - an STS servo does not respond to SCSCL packets.
// Restart Serial2 with new pins for the servo bus. RX/TX are from the ESP's
// point of view: rx = where the ESP receives (to U1RXD), tx = where it transmits (to U1TXD).
void servoBusRestart(SCSCL* servo) {
    Serial2.end();
    Serial2.begin(g_servoBaud, SERIAL_8N1, g_servoRx, g_servoTx);
    servo->pSerial = &Serial2;
}

void servoSetPins(SCSCL* servo, int rx, int tx) {
    g_servoRx = rx;
    g_servoTx = tx;
    servoBusRestart(servo);
    Serial.printf("-> Servo bus restarted: RX=IO%d  TX=IO%d @ %lu baud. Now test 'svs'.\n",
                  g_servoRx, g_servoTx, (unsigned long)g_servoBaud);
}

void servoScanProtocols(SCSCL* scs) {
    Serial.println("--- Protocol scan on Serial2 (ID 1) ---");

    // Cross-check with the STS driver (little-endian, 0..4095) on the same bus.
    SMS_STS sts;
    sts.pSerial = &Serial2;
    int stsPos = servoReadRetry([&]{ return sts.ReadPos(SERVO_ID); });
    Serial.printf("  SMS_STS (STS series, 0..4095): %s\n",
                  stsPos == -1 ? "no response" : String("Pos " + String(stsPos)).c_str());

    int scsPos = servoReadRetry([&]{ return scs->ReadPos(SERVO_ID); });
    Serial.printf("  SCSCL  (SC series, 0..1023): %s\n",
                  scsPos == -1 ? "no response" : String("Pos " + String(scsPos)).c_str());

    if (scsPos != -1 || stsPos != -1) {
        Serial.println("  => Servo responds - firmware uses SCSCL (SC series), correct.");
    } else {
        Serial.println("  => Both silent: check wiring/power/baud rate.");
    }
}

// Complete servo diagnosis. The most important command when nothing moves:
// it separates bus, power, torque and mapping problems from each other.
void servoDiagnose(SCSCL* servo) {
    int pos  = servoReadRetry([&]{ return servo->ReadPos(SERVO_ID); });
    int volt = servoReadRetry([&]{ return servo->ReadVoltage(SERVO_ID); });
    int load = servoReadRetry([&]{ return servo->ReadLoad(SERVO_ID); });
    int mode = servoReadRetry([&]{ return servo->ReadMode(SERVO_ID); });

    Serial.println("--- Servo diagnosis (ID 1) ---");
    if (pos == -1 && volt == -1) {
        Serial.println("  NO response from the servo bus.");
        Serial.printf("  -> Check wiring of Serial2 (RX IO%d / TX IO%d), 1 Mbit,\n",
                      g_servoRx, g_servoTx);
        Serial.println("     servo power supply and common ground.");
        return;
    }
    Serial.printf("  Position : %d\n", pos);
    Serial.printf("  Voltage  : %.1f V%s\n", volt / 10.0f,
                  (volt != -1 && volt < 40) ? "  (< 4 V - servo undersupplied!)" : "");
    Serial.printf("  Load     : %d\n", load);
    Serial.printf("  Mode     : %d %s\n", mode,
                  mode == 0 ? "(Position)" : mode == 1 ? "(PWM/wheel - does not turn to a position!)" : "");

    // Force torque on - most common cause of "does not hold/move".
    int te = servo->EnableTorque(SERVO_ID, 1);
    Serial.printf("  Torque enabled (EnableTorque ret=%d)\n", te);
    if (pos != -1) {
        servo->WritePosEx(SERVO_ID, pos, SERVO_SPEED_FAST, SERVO_ACC);   // hold position
        servoManualPos = pos;
    }
    Serial.println("  Test: 'j'/'l' move, 'm' center, 'sv512' center pos.");
}

// Forces the SC servo into position mode. Unlike STS, the SC series has
// NO mode register (33). Position vs. continuous rotation (wheel) depends solely on the
// angle limits: MIN==MAX==0 => wheel, otherwise position servo. So set MIN=0,
// MAX=1023. Lives in the EPROM: unlock -> write -> lock. All of these are
// write commands, hence not affected by the half-duplex echo.
void servoForcePositionMode(SCSCL* servo) {
    servo->unLockEprom(SERVO_ID);
    servo->writeWord(SERVO_ID, SCSCL_MIN_ANGLE_LIMIT_L, 0);
    servo->writeWord(SERVO_ID, SCSCL_MAX_ANGLE_LIMIT_L, SERVO_POS_MAX);
    servo->LockEprom(SERVO_ID);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    servo->EnableTorque(SERVO_ID, 1);
    Serial.println("-> Position mode set (limits 0..1023, torque on).");
    Serial.println("   Now test 'm' / 'j' / 'l'.");
}

// Register dump + write/read-back test. Answers the core question: do writes
// reach the servo at all? Writes a goal and reads the goal register back.
void servoRegisterDump(SCSCL* servo) {
    // SC series: mode is derived from the angle limits (ReadMode: 0=position,
    // 3=wheel). No dedicated mode register as on STS.
    int mode   = servoReadRetry([&]{ return servo->ReadMode(SERVO_ID); });
    int torque = servoReadRetry([&]{ return servo->readByte(SERVO_ID, SCSCL_TORQUE_ENABLE); });
    int minA   = servoReadRetry([&]{ return servo->readWord(SERVO_ID, SCSCL_MIN_ANGLE_LIMIT_L); });
    int maxA   = servoReadRetry([&]{ return servo->readWord(SERVO_ID, SCSCL_MAX_ANGLE_LIMIT_L); });
    int pos    = servoReadRetry([&]{ return servo->ReadPos(SERVO_ID); });
    Serial.println("--- Servo registers ---");
    Serial.printf("  Mode=%d  Torque(40)=%d  MinAng(9)=%d  MaxAng(11)=%d  Pos(56)=%d\n",
                  mode, torque, minA, maxA, pos);

    // Write test: set the torque register directly and read it back.
    servo->writeByte(SERVO_ID, SCSCL_TORQUE_ENABLE, 1);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    int tqBack = servoReadRetry([&]{ return servo->readByte(SERVO_ID, SCSCL_TORQUE_ENABLE); });
    Serial.printf("  Write test Torque=1 -> read back %d  %s\n", tqBack,
                  tqBack == 1 ? "WRITE ARRIVES" : "WRITE DOES NOT ARRIVE!");

    // Write the goal and read back the goal register.
    servo->WritePosEx(SERVO_ID, SERVO_CENTER_DEF, SERVO_SPEED_FAST, SERVO_ACC);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    int goal = servoReadRetry([&]{ return servo->readWord(SERVO_ID, SCSCL_GOAL_POSITION_L); });
    Serial.printf("  WritePosEx(%d) -> GoalPos(42)=%d  %s\n", SERVO_CENTER_DEF, goal,
                  goal == SERVO_CENTER_DEF ? "ARRIVED (not turning = torque/mechanics)"
                               : "NOT arrived (write problem)");
}

// ==========================================
// 8b. MANUAL STEERING CALIBRATION (Core 1)
// ==========================================
//
// The SC09 has no torque limiting. If it drives against an end stop on its
// own, it keeps pushing with full torque until the linkage or gearbox gives
// way - automatic probing therefore cannot be operated safely.
// That is why calibration is done by hand: the operator moves in small steps
// (default 10 ticks ~ 3 degrees) and sets the center and both end stops himself.
//
// The stored limits remain unchanged until 'calsave'. An abort
// (or a reset midway) therefore leaves the old calibration intact.

constexpr int CAL_STEP_DEF = 10;    // ticks per step (~2.9 degrees)
constexpr int CAL_STEP_MIN = 1;
constexpr int CAL_STEP_MAX = 200;
// End stops must be at least this far apart (~15 degrees), otherwise something
// obviously went wrong while setting them.
constexpr int CAL_MIN_SPAN = 50;

struct CalState {
    bool active     = false;
    bool freeMode   = false;                 // torque off, steering by hand
    int  pos        = SERVO_CENTER_DEF;      // last commanded raw position
    int  step       = CAL_STEP_DEF;
    bool haveCenter = false;
    bool haveLeft   = false;
    bool haveRight  = false;
    int  center     = SERVO_CENTER_DEF;
    int  left       = SERVO_POS_MAX;
    int  right      = 0;
};
static CalState g_cal;

static int calReadPos(SCSCL* servo) {
    return servoReadRetry([&]{ return servo->ReadPos(SERVO_ID); });
}

static int calReadLoad(SCSCL* servo) {
    int raw = servoReadRetry([&]{ return servo->ReadLoad(SERVO_ID); });
    return (raw == -1) ? -1 : (raw & 0x3FF);   // bits 0-9, bit 10 is the direction
}

static void calPrintMark(const char* name, bool have, int value) {
    if (have) Serial.printf("  %-8s: %4d\n", name, value);
    else      Serial.printf("  %-8s:    - (not set yet)\n", name);
}

void calPrintStatus(SCSCL* servo) {
    int ist  = calReadPos(servo);
    int load = calReadLoad(servo);

    Serial.printf("--- Calibration: %s ---\n",
                  !g_cal.active ? "inactive ('cal' starts it)"
                                : (g_cal.freeMode ? "ACTIVE, torque free" : "ACTIVE"));
    Serial.printf("  Position: target=%d  actual=%s  load=%s\n", g_cal.pos,
                  ist  == -1 ? "?" : String(ist).c_str(),
                  load == -1 ? "?" : String(load).c_str());
    Serial.printf("  Step    : %d ticks (~%.1f deg)\n",
                  g_cal.step, g_cal.step * SERVO_DEG_PER_TICK);
    calPrintMark("Center", g_cal.haveCenter, g_cal.center);
    calPrintMark("Left",   g_cal.haveLeft,   g_cal.left);
    calPrintMark("Right",  g_cal.haveRight,  g_cal.right);
    Serial.printf("  stored: Center=%d  Left=%d  Right=%d  Trim=%d\n",
                  centerLimit, leftLimit, rightLimit, trimOffset);
    if (g_cal.active) {
        Serial.println("  + / -      one step (also '+50' for a one-off 50 ticks)");
        Serial.println("  caln<t>    step size      calm/call/calr  store center/left/right");
        Serial.println("  calfree    torque off (turn by hand)      calhold  hold again");
        Serial.println("  calgo      go to center                   calsave  save");
        Serial.println("  calq       abort (stored values are kept)");
    }
}

void calStart(SCSCL* servo) {
    // Stop the drive - nothing should roll away during calibration.
    setMotorCommand(MOTOR_COAST, false, 0);

    g_cal.active     = true;
    g_cal.freeMode   = false;
    g_cal.haveCenter = g_cal.haveLeft = g_cal.haveRight = false;
    g_cal.step       = CAL_STEP_DEF;

    servo->EnableTorque(SERVO_ID, 1);
    int p = calReadPos(servo);
    if (p != -1) {
        // First adopt the actual position and write exactly that, otherwise
        // the servo jumps to an old goal when torque is switched on.
        g_cal.pos = p;
        servoManualPos = p;
        servo->WritePosEx(SERVO_ID, p, SERVO_SPEED_CALIB, SERVO_ACC);
    } else {
        // Without a read-back actual position deliberately write NOTHING: a guessed
        // goal would make the steering swing across the whole range.
        g_cal.pos = servoManualPos;
        Serial.println("!! Servo not readable - position is not being set.");
        Serial.println("   Run 'sv' (diagnosis) first, then restart calibration.");
    }

    Serial.println("=== Manual steering calibration ===");
    Serial.println("  1) use + / - to set straight ahead      -> 'calm'");
    Serial.println("  2) slowly to the LEFT end stop          -> 'call'");
    Serial.println("  3) slowly to the RIGHT end stop         -> 'calr'");
    Serial.println("  4) 'calsave' saves, 'calq' aborts");
    Serial.println("  Stop just BEFORE the hard end stop: the SC09 has no");
    Serial.println("  torque limiting and will otherwise keep pushing against it.");
    Serial.println("  Alternative: 'calfree', move steering to the end stop by hand,");
    Serial.println("  then 'call'/'calr' - no servo force acts at all.");
    calPrintStatus(servo);
}

// One step of delta ticks. Reports back actual position and load, so that at
// the end stop it becomes visible that the servo no longer follows the goal.
uint8_t calMove(SCSCL* servo, int delta) {
    if (!g_cal.active) {
        Serial.println("-> Calibration not running ('cal' starts it)");
        return CAL_ST_INACTIVE;
    }
    if (g_cal.freeMode) {
        Serial.println("-> Torque is free ('calhold' switches it back on)");
        return CAL_ST_INACTIVE;
    }

    int target = constrain(g_cal.pos + delta, 0, SERVO_POS_MAX);
    if (target == g_cal.pos) {
        Serial.printf("-> End of range %d reached - cannot go further\n", target);
        return CAL_ST_LIMIT;
    }

    g_cal.pos = target;
    servoManualPos = target;
    servo->WritePosEx(SERVO_ID, target, SERVO_SPEED_CALIB, SERVO_ACC);
    // Let the step finish before measuring. SERVO_SPEED_CALIB is
    // the travel speed in ticks/s, plus a bit of start-up time.
    vTaskDelay((60 + abs(delta) * 1000 / (int)SERVO_SPEED_CALIB) / portTICK_PERIOD_MS);

    int ist  = calReadPos(servo);
    int load = calReadLoad(servo);
    Serial.printf("Servo -> %d (%+d)  actual=%s  load=%s\n", target, delta,
                  ist  == -1 ? "?" : String(ist).c_str(),
                  load == -1 ? "?" : String(load).c_str());

    if (ist == -1) return CAL_ST_NOSERVO;

    // If the actual position lags more than half a step behind the goal,
    // it is jammed. This is exactly where the operator should stop.
    if (abs(ist - target) > max(4, abs(delta) / 2)) {
        Serial.printf("   !! not following the goal (%d ticks deviation) - end stop?\n",
                      abs(ist - target));
        Serial.println("      Set the end stop here with 'call'/'calr' and back off.");
    }
    return CAL_ST_OK;
}

// Store the current position as center / left / right. What counts is the
// read-back actual position, not the command - in free mode there is no command
// at all, and at the end stop the two deliberately differ.
uint8_t calSetMark(SCSCL* servo, uint8_t what) {
    if (!g_cal.active) {
        Serial.println("-> Calibration not running ('cal' starts it)");
        return CAL_ST_INACTIVE;
    }

    uint8_t st = CAL_ST_OK;
    int p = calReadPos(servo);
    if (p == -1) {
        p  = g_cal.pos;
        st = CAL_ST_NOSERVO;
        Serial.println("   (servo not readable - commanded position used)");
    }
    g_cal.pos = p;   // track in free mode so that steps continue from here

    switch (what) {
        case CAL_ACT_CENTER: g_cal.center = p; g_cal.haveCenter = true;
                             Serial.printf("-> Center = %d\n", p);  break;
        case CAL_ACT_LEFT:   g_cal.left   = p; g_cal.haveLeft   = true;
                             Serial.printf("-> Left   = %d\n", p);  break;
        case CAL_ACT_RIGHT:  g_cal.right  = p; g_cal.haveRight  = true;
                             Serial.printf("-> Right  = %d\n", p);  break;
        default: return CAL_ST_REJECTED;
    }

    if (g_cal.haveCenter && g_cal.haveLeft && g_cal.haveRight) {
        Serial.println("   All three marks set - 'calsave' stores them.");
    }
    return st;
}

void calSetStep(int ticks) {
    g_cal.step = constrain(ticks, CAL_STEP_MIN, CAL_STEP_MAX);
    Serial.printf("-> Step size = %d ticks (~%.1f deg)\n",
                  g_cal.step, g_cal.step * SERVO_DEG_PER_TICK);
}

// Torque off: the steering can be moved by hand up to the end stop,
// without any servo force. When switching back on, the actual position is set
// as the goal, otherwise the servo pulls back to its old goal.
uint8_t calSetFree(SCSCL* servo, bool freeIt) {
    if (!g_cal.active) {
        Serial.println("-> Calibration not running ('cal' starts it)");
        return CAL_ST_INACTIVE;
    }
    g_cal.freeMode = freeIt;
    servo->EnableTorque(SERVO_ID, freeIt ? 0 : 1);

    if (freeIt) {
        Serial.println("-> Torque OFF: set steering by hand, then calm/call/calr.");
        return CAL_ST_OK;
    }

    int p = calReadPos(servo);
    if (p == -1) {
        Serial.println("-> Torque ON, but position not readable.");
        return CAL_ST_NOSERVO;
    }
    g_cal.pos = p;
    servoManualPos = p;
    servo->WritePosEx(SERVO_ID, p, SERVO_SPEED_CALIB, SERVO_ACC);
    Serial.printf("-> Torque ON, holding at %d.\n", p);
    return CAL_ST_OK;
}

// Move to the stored center mark (or to the saved one as long as none is set).
uint8_t calGoCenter(SCSCL* servo) {
    if (!g_cal.active) {
        Serial.println("-> Calibration not running ('cal' starts it)");
        return CAL_ST_INACTIVE;
    }
    if (g_cal.freeMode) calSetFree(servo, false);

    int target = g_cal.haveCenter ? g_cal.center : centerLimit;
    g_cal.pos = target;
    servoManualPos = target;
    servo->WritePosEx(SERVO_ID, target, SERVO_SPEED_CALIB, SERVO_ACC);
    Serial.printf("-> moving to center %d (%s)\n", target,
                  g_cal.haveCenter ? "newly set" : "saved");
    return CAL_ST_OK;
}

// Checks the three marks for plausibility and writes them to NVS. Only here
// do the active limits change.
bool calSave(SCSCL* servo) {
    if (!g_cal.active) {
        Serial.println("-> Calibration not running ('cal' starts it)");
        return false;
    }
    if (!g_cal.haveCenter || !g_cal.haveLeft || !g_cal.haveRight) {
        Serial.printf("-> NOT saved, missing:%s%s%s\n",
                      g_cal.haveCenter ? "" : " center (calm)",
                      g_cal.haveLeft   ? "" : " left (call)",
                      g_cal.haveRight  ? "" : " right (calr)");
        return false;
    }

    int lo = min(g_cal.left, g_cal.right);
    int hi = max(g_cal.left, g_cal.right);
    if (hi - lo < CAL_MIN_SPAN) {
        Serial.printf("-> NOT saved: end stops only %d ticks apart "
                      "(min. %d).\n", hi - lo, CAL_MIN_SPAN);
        return false;
    }
    if (g_cal.center <= lo || g_cal.center >= hi) {
        Serial.printf("-> NOT saved: center %d is not between the "
                      "end stops %d..%d.\n", g_cal.center, lo, hi);
        return false;
    }

    leftLimit   = g_cal.left;
    rightLimit  = g_cal.right;
    centerLimit = g_cal.center;
    // The old trim referred to the old center and is therefore obsolete.
    trimOffset  = 0;
    softwareCenterPos = centerLimit;

    bool ok = true;
    ok &= prefs.putInt("lLim10", leftLimit)   > 0;
    ok &= prefs.putInt("rLim10", rightLimit)  > 0;
    ok &= prefs.putInt("cLim10", centerLimit) > 0;
    prefs.putInt("offset10", trimOffset);   // 0 may not rewrite NVS at all

    g_cal.active   = false;
    g_cal.freeMode = false;

    servo->EnableTorque(SERVO_ID, 1);
    servo->WritePosEx(SERVO_ID, softwareCenterPos, SERVO_SPEED_CALIB, SERVO_ACC);
    servoManualPos = softwareCenterPos;

    Serial.printf("-> %s: Center=%d  Left=%d  Right=%d  (trim reset to 0)\n",
                  ok ? "saved" : "ERROR while saving (NVS)",
                  centerLimit, leftLimit, rightLimit);
    Serial.printf("   Travel: left %+d ticks (~%.0f deg), right %+d ticks (~%.0f deg),\n",
                  leftLimit - centerLimit, (leftLimit - centerLimit) * SERVO_DEG_PER_TICK,
                  rightLimit - centerLimit, (rightLimit - centerLimit) * SERVO_DEG_PER_TICK);
    Serial.println("   of which the steering uses 80% per side (+-100%).");
    return ok;
}

void calAbort(SCSCL* servo) {
    g_cal.active   = false;
    g_cal.freeMode = false;
    servo->EnableTorque(SERVO_ID, 1);
    servo->WritePosEx(SERVO_ID, softwareCenterPos, SERVO_SPEED_CALIB, SERVO_ACC);
    servoManualPos = softwareCenterPos;
    Serial.printf("-> Calibration aborted. Saved values are kept "
                  "(Center=%d L=%d R=%d), moving to center %d.\n",
                  centerLimit, leftLimit, rightLimit, softwareCenterPos);
}

// Execute an action. Common entry point for the USB console and the Jetson link,
// so that both paths do exactly the same thing. arg is the step size (0 = current)
// or, for CAL_ACT_STEP, the new step size.
uint8_t calHandleAction(SCSCL* servo, uint8_t action, uint8_t arg) {
    switch (action) {
        case CAL_ACT_START:  calStart(servo);                       return CAL_ST_OK;
        case CAL_ACT_MINUS:  return calMove(servo, -(arg ? (int)arg : g_cal.step));
        case CAL_ACT_PLUS:   return calMove(servo,  (arg ? (int)arg : g_cal.step));
        case CAL_ACT_CENTER:
        case CAL_ACT_LEFT:
        case CAL_ACT_RIGHT:  return calSetMark(servo, action);
        case CAL_ACT_SAVE:   return calSave(servo) ? CAL_ST_SAVED : CAL_ST_REJECTED;
        case CAL_ACT_ABORT:  calAbort(servo);                       return CAL_ST_OK;
        case CAL_ACT_FREE:   return calSetFree(servo, true);
        case CAL_ACT_HOLD:   return calSetFree(servo, false);
        case CAL_ACT_GOTO_C: return calGoCenter(servo);
        case CAL_ACT_STEP:   calSetStep(arg);                       return CAL_ST_OK;
        case CAL_ACT_STATUS: calPrintStatus(servo);                 return CAL_ST_OK;
        default:
            Serial.printf("-> unknown calibration action 0x%02X\n", action);
            return CAL_ST_REJECTED;
    }
}

// ==========================================
// 8c. RGBW STATUS LEDS (SK6812 chain on GPIO40) - task on Core 0
// ==========================================
//
// Why core 0: loop() on core 1 carries the live control from the Jetson and
// is allowed to block (calibration steps, servo reads with retries, console
// output). An animation driven from there would stutter with every such
// pause. The motor task on core 0 needs only a few microseconds per 10 ms
// cycle, so core 0 has plenty of headroom. One frame here costs a few
// microseconds of maths plus ~40 us of RMT transmission per LED, during
// which the task blocks on the driver instead of spinning - the motor task
// does not notice it.
//
// Hand-over like the drive: core 1 only writes the setpoints via setPixel()
// under a spinlock, the task copies them once per frame.
//
// Every LED in the chain has its own state, in two layers:
//   base     the persistent state (count = 0). Stays until replaced.
//   one-shot a triggered animation (count > 0). Runs count periods on top of
//            the base, then the base comes back by itself. Event signals
//            ("flash red 3x") therefore need no follow-up command.
// A new base does NOT cancel a running one-shot - a bridge that refreshes its
// status colour every second would otherwise cut off every event flash.

// 9 LEDs round the LiDAR since 08.10.2026: 0-2 left side, 3-5 front, 6-8
// right side, counted from the left. The sweep/scan modes work in groups of
// PIXEL_GROUP neighbouring LEDs (= one side), the position in the group comes
// from the index -- no protocol change, each LED of a side gets the same
// command.
constexpr int      PIXEL_COUNT      = 9;     // LEDs in the chain (max. 32)
constexpr int      PIXEL_GROUP      = 3;     // LEDs per side (sweep / scan)
static_assert(PIXEL_COUNT >= 1 && PIXEL_COUNT <= 32, "g_pxShotMask holds 32 LEDs");
constexpr uint32_t PIXEL_FRAME_MS   = 20;    // 50 Hz
constexpr uint32_t PIXEL_REFRESH_MS = 1000;  // resend even if unchanged (glitch recovery)

enum PixelMode : uint8_t {
    PX_OFF       = 0,
    PX_SOLID     = 1,
    PX_BLINK     = 2,   // 50 % on, 50 % off
    PX_BREATHE   = 3,   // smooth fade in and out
    PX_RAINBOW   = 4,   // hue cycle, shifted along the chain; colour bytes ignored
    PX_STROBE    = 5,   // short flash at the start of each period
    PX_HEARTBEAT = 6,   // double pulse
    PX_SWEEP     = 7,   // sequential turn signal: the LEDs of a group light up one
                        // after the other (ascending index), stay on, all off
    PX_SWEEP_REV = 8,   // the same, descending index
    PX_SCAN      = 9,   // running light: one dot with a fading tail moves back and
                        // forth over the group (KITT)
    PX_SCAN_ALL  = 10,  // the same over the whole chain
    PX_MODE_COUNT
};

static const char* const PX_MODE_NAMES[PX_MODE_COUNT] = {
    "off", "solid", "blink", "breathe", "rainbow", "strobe", "heart",
    "sweep", "sweepr", "scan", "scanall"
};

struct PixelAnim {
    uint8_t  mode       = PX_OFF;
    uint8_t  r = 0, g = 0, b = 0, w = 0;
    uint8_t  brightness = 64;     // 0..255, scales everything (incl. W)
    uint16_t periodMs   = 0;      // 0 = default of the mode
    uint8_t  count      = 0;      // 0 = persistent, n = one-shot over n periods
};

static PixelAnim     g_pxBase[PIXEL_COUNT];
static PixelAnim     g_pxShot[PIXEL_COUNT];
static uint32_t      g_pxBaseSeq[PIXEL_COUNT] = {0};   // ++ -> phase restarts
static uint32_t      g_pxShotSeq[PIXEL_COUNT] = {0};   // ++ -> one-shot starts over
static portMUX_TYPE  g_pxMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t g_pxShotMask = 0;   // bit i = one-shot running on LED i (task writes)
static volatile bool g_pxReady = false;      // RMT channel is up

TaskHandle_t PixelTaskHandle;

static uint16_t pixelDefaultPeriod(uint8_t mode) {
    switch (mode) {
        case PX_BLINK:     return 1000;
        case PX_BREATHE:   return 3000;
        case PX_RAINBOW:   return 5000;
        case PX_STROBE:    return 1000;
        case PX_HEARTBEAT: return 1200;
        case PX_SWEEP:
        case PX_SWEEP_REV: return 700;    // ~1.4 Hz like a car indicator
        case PX_SCAN:      return 900;
        case PX_SCAN_ALL:  return 1800;
        default:           return 1000;   // off/solid: time base for count only
    }
}

static inline uint16_t pixelPeriod(const PixelAnim& a) {
    return a.periodMs ? a.periodMs : pixelDefaultPeriod(a.mode);
}

// Set one LED (idx 0..PIXEL_COUNT-1) or all of them (idx < 0).
// count = 0 replaces the base, count > 0 triggers a one-shot.
//
// The base phase is only restarted when mode or period change: a bridge that
// resends the same blink every 100 ms would otherwise freeze it in its first
// half period. If it does change on any of the addressed LEDs, ALL of them
// restart together - otherwise "all blink" would leave the LEDs that were
// already blinking out of step with the rest.
void setPixel(int idx, const PixelAnim& a) {
    if (idx >= PIXEL_COUNT) return;
    const int lo = (idx < 0) ? 0 : idx;
    const int hi = (idx < 0) ? PIXEL_COUNT - 1 : idx;

    portENTER_CRITICAL(&g_pxMux);
    if (a.count) {
        for (int i = lo; i <= hi; i++) { g_pxShot[i] = a; g_pxShotSeq[i]++; }
    } else {
        bool restart = false;
        for (int i = lo; i <= hi; i++) {
            if (a.mode != g_pxBase[i].mode || a.periodMs != g_pxBase[i].periodMs) restart = true;
        }
        for (int i = lo; i <= hi; i++) {
            if (restart) g_pxBaseSeq[i]++;
            g_pxBase[i] = a;
        }
    }
    portEXIT_CRITICAL(&g_pxMux);
}

// End running one-shots early (idx < 0 = all), the base takes over immediately.
void stopPixelShot(int idx) {
    if (idx >= PIXEL_COUNT) return;
    const int lo = (idx < 0) ? 0 : idx;
    const int hi = (idx < 0) ? PIXEL_COUNT - 1 : idx;
    portENTER_CRITICAL(&g_pxMux);
    for (int i = lo; i <= hi; i++) { g_pxShot[i].count = 0; g_pxShotSeq[i]++; }
    portEXIT_CRITICAL(&g_pxMux);
}

PixelAnim getPixelBase(int idx) {
    PixelAnim a;
    if (idx < 0 || idx >= PIXEL_COUNT) idx = 0;
    portENTER_CRITICAL(&g_pxMux);
    a = g_pxBase[idx];
    portEXIT_CRITICAL(&g_pxMux);
    return a;
}

// Colour of LED idx, t milliseconds after its pattern started, already scaled
// by the brightness. out = R, G, B, W.
static void pixelRender(const PixelAnim& a, int idx, uint32_t t, uint8_t out[4]) {
    const uint32_t per = pixelPeriod(a);
    const uint32_t ph  = t % per;             // position within the period
    uint8_t  c[4]  = {a.r, a.g, a.b, a.w};
    uint32_t level = 255;                     // envelope 0..255

    switch (a.mode) {
        case PX_SOLID:
            break;
        case PX_BLINK:
            level = (ph < per / 2) ? 255 : 0;
            break;
        case PX_BREATHE: {
            // Raised cosine 0 -> 1 -> 0, squared: the eye perceives brightness
            // roughly logarithmically, linear fading would look like it hangs
            // at "bright" and drops off at the end.
            float x = 0.5f - 0.5f * cosf(2.0f * PI * (float)ph / (float)per);
            level = (uint32_t)lroundf(x * x * 255.0f);
            break;
        }
        case PX_RAINBOW: {
            // Hue wheel in six 256-step segments, white channel stays off.
            // Each LED is shifted by 1/PIXEL_COUNT of the wheel, so the
            // colours run along the chain instead of all LEDs in lockstep.
            uint32_t h = (ph * 1536u / per + (uint32_t)idx * 1536u / PIXEL_COUNT) % 1536u;
            uint8_t  x = (uint8_t)(h & 0xFF);
            switch (h >> 8) {
                case 0:  c[0] = 255;     c[1] = x;       c[2] = 0;       break;
                case 1:  c[0] = 255 - x; c[1] = 255;     c[2] = 0;       break;
                case 2:  c[0] = 0;       c[1] = 255;     c[2] = x;       break;
                case 3:  c[0] = 0;       c[1] = 255 - x; c[2] = 255;     break;
                case 4:  c[0] = x;       c[1] = 0;       c[2] = 255;     break;
                default: c[0] = 255;     c[1] = 0;       c[2] = 255 - x; break;
            }
            c[3] = 0;
            break;
        }
        case PX_STROBE: {
            uint32_t on = min<uint32_t>(50, per / 2);
            level = (ph < on) ? 255 : 0;
            break;
        }
        case PX_HEARTBEAT: {
            // Two beats at 0..10 % and 20..30 % of the period, then a pause.
            uint32_t p10 = ph * 10 / per;
            level = (p10 == 0 || p10 == 2) ? 255 : 0;
            break;
        }
        case PX_SWEEP:
        case PX_SWEEP_REV: {
            // 0..50 %: the LEDs come on one after the other, until 75 % all
            // on, then dark -- the classic sequential indicator.
            int slot = idx % PIXEL_GROUP;
            if (a.mode == PX_SWEEP_REV) slot = PIXEL_GROUP - 1 - slot;
            uint32_t on_at  = per / 2 * (uint32_t)slot / PIXEL_GROUP;
            uint32_t off_at = per * 3 / 4;
            level = (ph >= on_at && ph < off_at) ? 255 : 0;
            break;
        }
        case PX_SCAN:
        case PX_SCAN_ALL: {
            // Dot position as a triangle wave over the group (0 .. n-1 .. 0),
            // brightness falls off with the distance to it: the LED it just
            // left still glows -- the KITT tail.
            const int n    = (a.mode == PX_SCAN_ALL) ? PIXEL_COUNT : PIXEL_GROUP;
            const int slot = (a.mode == PX_SCAN_ALL) ? idx : idx % PIXEL_GROUP;
            float u   = (float)ph / (float)per;                    // 0..1
            float pos = (u < 0.5f ? 2.0f * u : 2.0f - 2.0f * u) * (float)(n - 1);
            float dist = fabsf((float)slot - pos);
            float x = 1.0f - dist / 1.6f;                          // ~1.6 LEDs wide
            if (x < 0.0f) x = 0.0f;
            level = (uint32_t)lroundf(x * x * 255.0f);
            break;
        }
        case PX_OFF:
        default:
            level = 0;
            break;
    }

    for (int i = 0; i < 4; i++) {
        out[i] = (uint8_t)((uint32_t)c[i] * level * a.brightness / (255u * 255u));
    }
}

// Send the whole chain via RMT. The first 32 bits go to the LED nearest to
// the ESP, it passes the rest on. SK6812 RGBW expects the order G, R, B, W,
// MSB first. Timing at 10 MHz (100 ns per tick) from the SK6812 datasheet:
// 0 = 0.3 us high / 0.9 us low, 1 = 0.6 / 0.6. The core's rgbLedWrite() uses
// 0.4/0.8 and 0.8/0.4 (WS2812) - 0.8 us is already outside the SK6812
// tolerance for T1H, and it cannot drive the fourth channel anyway.
// The >80 us reset pause comes for free from the frame interval.
static void pixelWrite(const uint8_t rgbw[][4]) {
    static rmt_data_t sym[PIXEL_COUNT * 32];
    int k = 0;
    for (int led = 0; led < PIXEL_COUNT; led++) {
        const uint8_t grbw[4] = {rgbw[led][1], rgbw[led][0], rgbw[led][2], rgbw[led][3]};
        for (int ch = 0; ch < 4; ch++) {
            for (int bit = 7; bit >= 0; bit--) {
                bool one = grbw[ch] & (1 << bit);
                sym[k].level0    = 1;
                sym[k].duration0 = one ? 6 : 3;
                sym[k].level1    = 0;
                sym[k].duration1 = one ? 6 : 9;
                k++;
            }
        }
    }
    rmtWrite(PIN_PIXEL, sym, k, RMT_WAIT_FOR_EVER);
}

void pixelTask(void* pvParameters) {
    // Set up the RMT channel HERE and not in setup(): the driver allocates its
    // interrupt on the core that creates the channel - so it stays on core 0
    // as well, away from the live control.
    //
    // Memory: one block holds 48 symbols = 1.5 LEDs. Reserve enough blocks for
    // the whole frame (4 LEDs = 128 symbols -> 3 blocks), then it goes out in
    // one piece. Otherwise the driver refills by interrupt mid-transmission,
    // and a late refill stretches a bit beyond the 80 us reset - the rest of
    // the chain would then latch garbage. Only longer chains than 6 LEDs fall
    // back to refilling (the S3 has 4 TX blocks, nothing else uses RMT).
    constexpr int PX_RMT_BLOCKS = min(4, (PIXEL_COUNT * 32 + 47) / 48);
    if (!rmtInit(PIN_PIXEL, RMT_TX_MODE, (rmt_reserve_memsize_t)PX_RMT_BLOCKS, 10000000)) {
        Serial.printf("ESP: RGBW LED - RMT init on IO%d failed, LED disabled.\n", PIN_PIXEL);
        vTaskDelete(nullptr);
        return;
    }
    g_pxReady = true;

    uint32_t baseSeq[PIXEL_COUNT], shotSeq[PIXEL_COUNT] = {0};
    uint32_t baseT0[PIXEL_COUNT] = {0}, shotT0[PIXEL_COUNT] = {0};
    bool     shotRunning[PIXEL_COUNT] = {false};
    for (int i = 0; i < PIXEL_COUNT; i++) baseSeq[i] = UINT32_MAX;

    uint8_t  last[PIXEL_COUNT][4] = {{0}};
    uint32_t lastWrite = 0;
    bool     first = true;

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        PixelAnim base[PIXEL_COUNT], shot[PIXEL_COUNT];
        uint32_t  bs[PIXEL_COUNT], ss[PIXEL_COUNT];
        portENTER_CRITICAL(&g_pxMux);
        for (int i = 0; i < PIXEL_COUNT; i++) {
            base[i] = g_pxBase[i];  bs[i] = g_pxBaseSeq[i];
            shot[i] = g_pxShot[i];  ss[i] = g_pxShotSeq[i];
        }
        portEXIT_CRITICAL(&g_pxMux);

        // One timestamp for the whole frame: LEDs started together stay in step.
        uint32_t now  = millis();
        uint32_t mask = 0;
        uint8_t  px[PIXEL_COUNT][4];

        for (int i = 0; i < PIXEL_COUNT; i++) {
            if (bs[i] != baseSeq[i]) { baseSeq[i] = bs[i]; baseT0[i] = now; }
            if (ss[i] != shotSeq[i]) {
                shotSeq[i] = ss[i]; shotT0[i] = now; shotRunning[i] = (shot[i].count > 0);
            }
            // One-shot over? Its duration is count x period, i.e. it always
            // ends after a complete pattern, never in the middle of a flash.
            if (shotRunning[i] &&
                now - shotT0[i] >= (uint32_t)pixelPeriod(shot[i]) * shot[i].count) {
                shotRunning[i] = false;
            }
            if (shotRunning[i]) {
                mask |= (1u << i);
                pixelRender(shot[i], i, now - shotT0[i], px[i]);
            } else {
                pixelRender(base[i], i, now - baseT0[i], px[i]);
            }
        }
        g_pxShotMask = mask;

        // Only send on change - plus a periodic refresh, in case a motor
        // spike has corrupted a bit and an LED shows a wrong colour.
        if (first || memcmp(px, last, sizeof(px)) != 0 || now - lastWrite >= PIXEL_REFRESH_MS) {
            pixelWrite(px);
            memcpy(last, px, sizeof(px));
            lastWrite = now;
            first = false;
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PIXEL_FRAME_MS));
    }
}

// Power-on default in NVS: the base of every LED (a one-shot is an event and
// is not stored). One blob of 8 bytes per LED: mode, R, G, B, W, brightness,
// period hi/lo. If the chain length changes, the LEDs that are in both the
// old and the new chain keep their stored state.
constexpr size_t PX_NVS_REC = 8;

bool savePixelDefault() {
    uint8_t blob[PIXEL_COUNT * PX_NVS_REC];
    for (int i = 0; i < PIXEL_COUNT; i++) {
        PixelAnim a = getPixelBase(i);
        uint8_t* p = &blob[i * PX_NVS_REC];
        p[0] = a.mode; p[1] = a.r; p[2] = a.g; p[3] = a.b; p[4] = a.w;
        p[5] = a.brightness;
        p[6] = (uint8_t)(a.periodMs >> 8); p[7] = (uint8_t)a.periodMs;
    }
    return prefs.putBytes("pxdef", blob, sizeof(blob)) == sizeof(blob);
}

void loadPixelDefault() {
    if (!prefs.isKey("pxdef")) return;   // never saved -> LEDs stay off
    uint8_t blob[32 * PX_NVS_REC];
    size_t len = prefs.getBytes("pxdef", blob, sizeof(blob));
    int n = min((int)(len / PX_NVS_REC), PIXEL_COUNT);
    for (int i = 0; i < n; i++) {
        const uint8_t* p = &blob[i * PX_NVS_REC];
        if (p[0] >= PX_MODE_COUNT) continue;
        PixelAnim a;
        a.mode = p[0]; a.r = p[1]; a.g = p[2]; a.b = p[3]; a.w = p[4];
        a.brightness = p[5];
        a.periodMs   = (uint16_t)((p[6] << 8) | p[7]);
        setPixel(i, a);
    }
}

void printPixelState() {
    Serial.printf("RGBW LEDs IO%d (%d x SK6812, LED 0 = nearest to the ESP)%s\n",
                  PIN_PIXEL, PIXEL_COUNT, g_pxReady ? "" : "  !! RMT not running");
    uint32_t mask = g_pxShotMask;
    for (int i = 0; i < PIXEL_COUNT; i++) {
        PixelAnim a = getPixelBase(i);
        Serial.printf("  [%d] %-7s rgbw=%u,%u,%u,%u  bri=%u  period=%u ms%s%s\n", i,
                      PX_MODE_NAMES[a.mode], a.r, a.g, a.b, a.w, a.brightness,
                      pixelPeriod(a), a.periodMs ? "" : " (default)",
                      (mask & (1u << i)) ? "  + one-shot running" : "");
    }
}

// ==========================================
// 9. COMMUNICATION (Jetson Bridge) - Core 1
// ==========================================

// All multi-byte values in the protocol are big-endian, as in the existing
// CMD_MOTOR/CMD_SERVO. Floats are transmitted as int32 x1000.
static inline int32_t readI32BE(const uint8_t* b) {
    return ((int32_t)b[0] << 24) | ((int32_t)b[1] << 16) |
           ((int32_t)b[2] << 8)  |  (int32_t)b[3];
}
static inline void writeI32BE(uint8_t* b, int32_t v) {
    b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16);
    b[2] = (uint8_t)(v >> 8);  b[3] = (uint8_t)v;
}
static inline uint32_t readU32BE(const uint8_t* b) {
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
}
static inline void writeU32BE(uint8_t* b, uint32_t v) {
    b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16);
    b[2] = (uint8_t)(v >> 8);  b[3] = (uint8_t)v;
}
static inline void writeI64BE(uint8_t* b, int64_t v) {
    for (int i = 0; i < 8; i++) b[i] = (uint8_t)(v >> (56 - 8 * i));
}

// --- Time base of the link ---
// esp_timer_get_time() counts microseconds since boot as int64 and - unlike
// micros() - does not overflow every 71 minutes. On the wire, the frame stamp
// carries only the lower 32 bits (saves 4 bytes per packet); CMD_TIME_RSP
// regularly delivers the full value, which the bridge can use to unwrap the
// overflow again.
static inline int64_t nowUs() { return esp_timer_get_time(); }

// Attach a send timestamp to all outgoing packets (frame 0xA6 instead of
// 0xA5). Off by default, see START_BYTE_TS.
bool g_stampTx = false;

// --- Drive telemetry ---
// Interval between two CMD_TELEMETRY packets in milliseconds, 0 = off. The
// Jetson sets it with CMD_TELEM_RATE, the USB console with "tel<ms>".
//
// Lower bound TELEMETRY_MS_MIN (10 ms) = the cycle time of the motor task.
// Sending faster would be pointless: position and speed are both produced at
// this rate, and both are fresh in every packet. How finely the speed is
// resolved does not depend on the send rate but on the measurement window -
// see g_speedWindow.
//
// Deliberately not stored in NVS: a bridge that needs the rate sets it itself
// when connecting, and an ESP without a peer should not transmit into the
// void.
uint16_t g_telemetryMs = 0;
constexpr uint16_t TELEMETRY_MS_MIN = 10;

bool savePidParams();   // Defined further below, used by CMD_PID_SAVE

// --- Logging of the Jetson link on the USB console ---
// 0 = off, 1 = decoded packets, 2 = additionally every raw byte.
// Level 1 is the default. Caution: a motor heartbeat at 100 ms intervals produces
// 10 lines/s - switch it off with "dbg0" during continuous driving.
uint8_t g_linkDebug = 1;

// Plotter mode: periodically prints plain "name:value" lines, as expected by
// the Serial Plotter of the Arduino IDE. While it is running, the link log
// stays silent - otherwise every foreign line breaks up the curve.
bool g_plotMode = false;
// Set by the command "gp<deg>": the plotter runs only for the duration of one move
// and then switches itself off. The Serial Plotter of the Arduino IDE has no
// input field - this way a move can be triggered in the monitor and the curve
// viewed afterwards in the plotter, without having to type anything in between.
bool g_plotAuto = false;

static const char* cmdName(uint8_t cmd) {
    switch (cmd) {
        case CMD_MOTOR:         return "MOTOR";
        case CMD_SERVO:         return "SERVO";
        case CMD_LED:           return "LED";
        case CMD_PIXEL:         return "PIXEL";
        case CMD_PIXEL_ONE:     return "PIXEL_ONE";
        case CMD_CALIBRATE:     return "CALIBRATE";
        case CMD_CAL:           return "CAL";
        case CMD_CAL_RSP:       return "CAL_RSP";
        case CMD_TORQUE:        return "TORQUE";
        case CMD_TRIM:          return "TRIM";
        case CMD_PID_SET:       return "PID_SET";
        case CMD_PID_GET:       return "PID_GET";
        case CMD_PID_SAVE:      return "PID_SAVE";
        case CMD_MOVE:          return "MOVE";
        case CMD_MOVE_ABORT:    return "MOVE_ABORT";
        case CMD_PROGRESS:      return "PROGRESS";
        case CMD_BATTERY:       return "BATTERY";
        case CMD_EMERGENCY:     return "EMERGENCY";
        case CMD_BUTTON:        return "BUTTON";
        case CMD_PID_RSP:       return "PID_RSP";
        case CMD_PID_SAVED:     return "PID_SAVED";
        case CMD_MOVE_DONE:     return "MOVE_DONE";
        case CMD_PROGRESS_RSP:  return "PROGRESS_RSP";
        case CMD_BATTERY_RSP:   return "BATTERY_RSP";
        case CMD_BATTERY_WARN:  return "BATTERY_WARN";
        case CMD_TIME_SYNC:     return "TIME_SYNC";
        case CMD_TIME_RSP:      return "TIME_RSP";
        case CMD_STAMP_MODE:    return "STAMP_MODE";
        case CMD_STAMP_RSP:     return "STAMP_RSP";
        case CMD_TELEM_RATE:    return "TELEM_RATE";
        case CMD_TELEMETRY:     return "TELEMETRY";
        default:                return "???";
    }
}

static const char* calActionName(uint8_t act) {
    switch (act) {
        case CAL_ACT_START:  return "start";
        case CAL_ACT_MINUS:  return "step-";
        case CAL_ACT_PLUS:   return "step+";
        case CAL_ACT_CENTER: return "set center";
        case CAL_ACT_LEFT:   return "set left";
        case CAL_ACT_RIGHT:  return "set right";
        case CAL_ACT_SAVE:   return "save";
        case CAL_ACT_ABORT:  return "abort";
        case CAL_ACT_FREE:   return "torque free";
        case CAL_ACT_HOLD:   return "torque hold";
        case CAL_ACT_GOTO_C: return "go to center";
        case CAL_ACT_STEP:   return "step size";
        case CAL_ACT_STATUS: return "status";
        default:             return "???";
    }
}

class JetsonComms {
private:
    HardwareSerial* serialPort;
    SCSCL* servo;

    unsigned long stateTime;
    uint8_t buffer[24];
    int bufIndex = 0;
    unsigned long linkBaud = 115200;

    enum State { WAITING_START, WAITING_CMD, READING_STAMP, READING_DATA };
    State currentState = WAITING_START;
    uint8_t currentCmd = 0;
    uint8_t dataLength = 0;

    // --- Timestamps of the reception currently in progress ---
    bool     frameStamped = false;   // Frame came in as 0xA6
    uint8_t  stampBuf[4];
    int      stampIndex = 0;
    uint32_t frameStampUs = 0;       // Send stamp of the peer (0xA6 only)
    int64_t  frameRxUs    = 0;       // ESP clock at the last byte of the frame

    // Link statistics for troubleshooting
    uint32_t rxPackets  = 0;   // completely received and executed
    uint32_t rxUnknown  = 0;   // unknown CMD byte discarded
    uint32_t rxTimeouts = 0;   // packet remained incomplete
    uint32_t rxStray    = 0;   // bytes outside a packet (e.g. ASCII)
    uint32_t txPackets  = 0;
    unsigned long lastRxMs = 0;
    uint32_t syncRequests = 0;    // answered CMD_TIME_SYNC requests
    int64_t  lastSyncUs   = 0;    // ESP clock of the last response

    // Suppression of identical repetitions in the log. A heartbeat at 100 ms
    // intervals would otherwise produce 10 useless lines/s and hide exactly
    // the packets you are waiting for.
    uint8_t  lastLogCmd = 0xFF;
    uint8_t  lastLogBuf[24];
    uint8_t  lastLogLen = 0;
    uint32_t repeatCount = 0;
    unsigned long repeatSince = 0;

public:
    JetsonComms(HardwareSerial* s, SCSCL* sv) : serialPort(s), servo(sv) {}

    void begin(unsigned long baud = 115200) {
        serialPort->begin(baud, SERIAL_8N1, PIN_JETSON_RX, PIN_JETSON_TX);
        linkBaud  = baud;
        stateTime = millis();
    }

    void sendButtonEvent() {
        uint8_t p[3] = {START_BYTE, CMD_BUTTON, 0x01};   // 0x01 = Pressed
        sendPacket(p, sizeof(p));
    }

    void sendMoveDone(const MoveResult& r) {
        uint8_t p[8] = {START_BYTE, CMD_MOVE_DONE, r.id, r.status};
        writeI32BE(&p[4], r.finalDeg10);
        sendPacket(p, sizeof(p));
    }

    void sendProgress() {
        MoveState m = getMoveState();
        uint8_t p[13] = {START_BYTE, CMD_PROGRESS_RSP, m.id,
                         (uint8_t)(m.active ? 1 : 0), m.progress};
        writeI32BE(&p[5], countsToDeg10(g_encCount));
        writeI32BE(&p[9], countsToDeg10(m.targetCnt));
        sendPacket(p, sizeof(p));
    }

    void sendBattery(uint8_t cmd) {
        uint8_t p[8] = {START_BYTE, cmd};
        writeI32BE(&p[2], (int32_t)lroundf(battPackV * 1000.0f));
        int16_t cellmV = (int16_t)lroundf(battCellV * 1000.0f);
        p[6] = (uint8_t)(cellmV >> 8);
        p[7] = (uint8_t)cellmV;
        sendPacket(p, sizeof(p));
    }

    void sendPidParams() {
        PidParams pid = getPid();
        uint8_t p[14] = {START_BYTE, CMD_PID_RSP};
        writeI32BE(&p[2],  (int32_t)lroundf(pid.kp * 1000.0f));
        writeI32BE(&p[6],  (int32_t)lroundf(pid.ki * 1000.0f));
        writeI32BE(&p[10], (int32_t)lroundf(pid.kd * 1000.0f));
        sendPacket(p, sizeof(p));
    }

    // Complete state of the manual calibration. Sent after every CMD_CAL
    // action so the bridge can build a display without having to keep count
    // itself.
    void sendCalState(uint8_t status) {
        uint8_t flags = (g_cal.haveCenter ? 0x01 : 0) |
                        (g_cal.haveLeft   ? 0x02 : 0) |
                        (g_cal.haveRight  ? 0x04 : 0) |
                        (g_cal.freeMode   ? 0x08 : 0);
        uint8_t p[13] = {START_BYTE, CMD_CAL_RSP,
                         (uint8_t)(g_cal.active ? 1 : 0), flags, status};
        auto put16 = [&](int idx, int v) {
            p[idx]     = (uint8_t)((uint16_t)(int16_t)v >> 8);
            p[idx + 1] = (uint8_t)((uint16_t)(int16_t)v);
        };
        put16(5,  g_cal.pos);
        put16(7,  g_cal.haveCenter ? g_cal.center : centerLimit);
        put16(9,  g_cal.haveLeft   ? g_cal.left   : leftLimit);
        put16(11, g_cal.haveRight  ? g_cal.right  : rightLimit);
        sendPacket(p, sizeof(p));
    }

    void sendPidSaved(bool ok) {
        uint8_t p[3] = {START_BYTE, CMD_PID_SAVED, (uint8_t)(ok ? 0x00 : 0x01)};
        sendPacket(p, sizeof(p));
    }

    // Response to CMD_TIME_SYNC. Carries both points in time on the ESP side
    // as full int64 microseconds:
    //   t_rx = last byte of the request received
    //   t_tx = last byte of this response on the wire
    // Together with t1/t4 from the Jetson side (each also the last byte), the
    // clock offset is ((t2-t1) + (t3-t4)) / 2 and the round-trip time is
    // (t4-t1) - (t3-t2). See docs/JETSON_BRIDGE.md.
    //
    // This packet is deliberately NEVER sent as 0xA6: its t_tx is already in
    // the payload at full width, an additional 32-bit stamp would just be
    // redundant - and the bridge needs the time sync before it can even
    // decide whether it wants stamps at all.
    void sendTimeSync(uint8_t seq, int64_t rxUs) {
        uint8_t p[19] = {START_BYTE, CMD_TIME_RSP, seq};
        writeI64BE(&p[3], rxUs);
        // p[11..18] (t_tx) is filled in by sendPacket itself, as late as possible.
        sendPacket(p, sizeof(p), 11);
        syncRequests++;
        lastSyncUs = nowUs();
    }

    // Drive state: where the output shaft is, how fast it is turning, what is
    // applied to the bridge and how much current is flowing.
    //
    // Speed in 1/10 degree per second, same unit as the distances in
    // CMD_MOVE. The motor task internally keeps RPM - factor 60 (one
    // revolution is 3600 tenths of a degree, one minute is 60 seconds).
    void sendTelemetry() {
        uint8_t p[14] = {START_BYTE, CMD_TELEMETRY};

        // Position and speed are signed: the encoder delta in the motor task
        // is a signed long, so reversing yields negative values. writeI32BE
        // shifts arithmetically, so the two's complement reaches the wire
        // unchanged.
        writeI32BE(&p[2], countsToDeg10(g_encCount));
        writeI32BE(&p[6], (int32_t)lroundf(g_rpm * 60.0f));

        // Likewise duty: negative means reverse.
        int16_t duty = g_dutySigned;
        p[10] = (uint8_t)((uint16_t)duty >> 8);
        p[11] = (uint8_t)duty;

        // The current, on the other hand, is always positive - the VNH5019 only
        // reports the magnitude on CS, not the direction. Anyone who needs it reads it from the duty.
        int32_t mA = (int32_t)lroundf(readMotorCurrentA() * 1000.0f);
        mA = constrain(mA, 0L, 32767L);
        p[12] = (uint8_t)((uint16_t)(int16_t)mA >> 8);
        p[13] = (uint8_t)(int16_t)mA;

        sendPacket(p, sizeof(p));
    }

    void sendStampMode() {
        uint8_t p[3] = {START_BYTE, CMD_STAMP_RSP, (uint8_t)(g_stampTx ? 1 : 0)};
        sendPacket(p, sizeof(p));
    }

    void printLinkStats() {
        Serial.printf("Link IO%d(RX)/IO%d(TX) @115200 | debug=%u\n",
                      PIN_JETSON_RX, PIN_JETSON_TX, g_linkDebug);
        Serial.printf("  RX: %lu packets, %lu unknown, %lu incomplete, %lu stray bytes\n",
                      (unsigned long)rxPackets, (unsigned long)rxUnknown,
                      (unsigned long)rxTimeouts, (unsigned long)rxStray);
        Serial.printf("  TX: %lu packets%s\n", (unsigned long)txPackets,
                      g_stampTx ? " (with timestamp, frame 0xA6)" : "");
        Serial.printf("  Time: %lld us since boot | %lu syncs",
                      (long long)nowUs(), (unsigned long)syncRequests);
        if (lastSyncUs) Serial.printf(" | last sync %lld ms ago",
                                      (long long)((nowUs() - lastSyncUs) / 1000));
        Serial.println();
        if (rxPackets == 0 && rxStray == 0) {
            Serial.println("  !! NOTHING received yet - check wiring/baud rate/GND");
        } else if (lastRxMs) {
            Serial.printf("  last packet %lu ms ago\n", millis() - lastRxMs);
        }
    }

    void process() {
        while (serialPort->available()) {
            uint8_t byte = serialPort->read();

            if (currentState != WAITING_START && (millis() - stateTime > 100)) {
                rxTimeouts++;
                if (g_linkDebug && !g_plotMode) {
                    Serial.printf("[RX] ABORT cmd=0x%02X %s after %d/%u bytes (>100 ms gap) - resync\n",
                                  currentCmd, cmdName(currentCmd), bufIndex, dataLength);
                }
                currentState = WAITING_START;
            }
            stateTime = millis();

            switch (currentState) {
                case WAITING_START:
                    if (byte == START_BYTE || byte == START_BYTE_TS) {
                        frameStamped = (byte == START_BYTE_TS);
                        currentState = WAITING_CMD;
                        bufIndex = 0;
                        stampIndex = 0;
                        frameStampUs = 0;
                    } else {
                        // No start byte: either ASCII plain text or garbage after
                        // a loss of sync. Only reported individually in full mode.
                        rxStray++;
                        if (g_linkDebug >= 2 && !g_plotMode) {
                            Serial.printf("[RX] sync search: 0x%02X%s\n", byte,
                                          (byte >= 32 && byte < 127) ? " (ASCII)" : "");
                        }
                    }
                    break;

                case WAITING_CMD: {
                    currentCmd = byte;
                    int len = payloadLength(currentCmd);
                    if (len < 0) {                  // unknown command
                        rxUnknown++;
                        if (g_linkDebug && !g_plotMode) {
                            Serial.printf("[RX] UNKNOWN cmd=0x%02X - discarded\n", currentCmd);
                        }
                        currentState = WAITING_START;
                        break;
                    }
                    dataLength = (uint8_t)len;
                    // With 0xA6, four bytes of send stamp come first, then the
                    // payload.
                    currentState = frameStamped ? READING_STAMP : READING_DATA;
                    if (!frameStamped && dataLength == 0) finishFrame();
                    break;
                }

                case READING_STAMP:
                    stampBuf[stampIndex++] = byte;
                    if (stampIndex >= 4) {
                        frameStampUs = readU32BE(stampBuf);
                        currentState = READING_DATA;
                        if (dataLength == 0) finishFrame();
                    }
                    break;

                case READING_DATA:
                    buffer[bufIndex++] = byte;
                    if (bufIndex >= dataLength) finishFrame();
                    break;
            }
        }
    }

private:
    // Frame is complete: record the receive time, log it, execute it, and
    // go back to searching for sync.
    //
    // frameRxUs refers to the last byte of the frame - the same reference
    // point the bridge uses for its t1. The only inaccuracy is that process()
    // reads from within loop(): the byte had already been sitting in the
    // driver buffer for a while. This delay is contained in the measured
    // round-trip time, so the bridge sees it and can discard outliers.
    void finishFrame() {
        frameRxUs = nowUs();
        logRxPacket();
        executeCommand();
        currentState = WAITING_START;
    }

    // Point in time at which the last byte of an n-byte packet has left the
    // wire. 8N1 = 10 bits per byte. The transmit buffer must be empty before
    // the call, otherwise the rest of the previous packet gets in front of it.
    int64_t txDoneUs(size_t n) const {
        return nowUs() + (int64_t)n * 10 * 1000000 / (int64_t)linkBaud;
    }

    // Send out a packet, count it and optionally log it.
    //
    // p always contains the unstamped frame (START_BYTE, CMD, payload).
    // If g_stampTx is on, sendPacket builds the 0xA6 frame from it and inserts
    // four bytes of timestamp between CMD and payload.
    //
    // txStampOffset >= 0: at this position in the *unstamped* frame there is an
    // int64 field that receives the packet's own send time (CMD_TIME_RSP only).
    // Such packets never additionally get a frame stamp.
    void sendPacket(const uint8_t* p, size_t n, int txStampOffset = -1) {
        const bool stamped = g_stampTx && txStampOffset < 0;
        const bool needsClock = stamped || txStampOffset >= 0;

        uint8_t out[48];
        size_t  k = 0;

        if (stamped) {
            out[k++] = START_BYTE_TS;
            out[k++] = p[1];
            k += 4;                              // Placeholder for the stamp
            memcpy(&out[k], p + 2, n - 2);
            k += n - 2;
        } else {
            memcpy(out, p, n);
            k = n;
        }

        // Let the buffer drain first, then stamp: only then does the
        // calculation "now + transmission time" hold. If an ASCII line is still
        // in the buffer (CMD_EMERGENCY, CMD_TRIM), the stamp would otherwise be too early.
        if (needsClock) {
            serialPort->flush();
            int64_t done = txDoneUs(k);
            if (stamped)              writeU32BE(&out[2], (uint32_t)done);
            if (txStampOffset >= 0)   writeI64BE(&out[txStampOffset], done);
        }

        serialPort->write(out, k);
        txPackets++;
        if (g_linkDebug && !g_plotMode) {
            const uint8_t* payload = &out[k - (n - 2)];   // Payload at the end
            Serial.printf("[TX] %s", cmdName(p[1]));
            if (stamped) Serial.printf(" t=%lu", (unsigned long)readU32BE(&out[2]));
            for (size_t i = 0; i < n - 2; i++) Serial.printf(" %02X", payload[i]);
            Serial.println();
        }
    }

    // Log a received packet on the USB console.
    void logRxPacket() {
        rxPackets++;
        lastRxMs = millis();
        if (!g_linkDebug || g_plotMode) return;

        // Byte-identical repetition? Then only count it and print a summary
        // line every 5 s. In dbg2 (raw bytes) everything stays unfiltered.
        bool same = (currentCmd == lastLogCmd && dataLength == lastLogLen &&
                     memcmp(buffer, lastLogBuf, dataLength) == 0);
        // Never merge time sync packets - when measuring the link you want to
        // see every single round, even if the seq happens to be the same.
        if (currentCmd == CMD_TIME_SYNC) same = false;

        if (same && g_linkDebug < 2) {
            repeatCount++;
            if (millis() - repeatSince >= 5000) {
                Serial.printf("[RX] %-12s  %lux unchanged in %lu s\n",
                              cmdName(currentCmd), (unsigned long)repeatCount,
                              (millis() - repeatSince) / 1000);
                repeatCount = 0;
                repeatSince = millis();
            }
            return;
        }

        if (repeatCount) {
            Serial.printf("[RX] %-12s  %lux unchanged\n",
                          cmdName(lastLogCmd), (unsigned long)repeatCount);
            repeatCount = 0;
        }
        lastLogCmd = currentCmd;
        lastLogLen = dataLength;
        memcpy(lastLogBuf, buffer, dataLength);
        repeatSince = millis();

        Serial.printf("[RX] %-12s", cmdName(currentCmd));
        if (frameStamped) Serial.printf(" t=%lu", (unsigned long)frameStampUs);
        if (g_linkDebug >= 2) {
            Serial.print(" raw:");
            for (int i = 0; i < dataLength; i++) Serial.printf(" %02X", buffer[i]);
        }

        switch (currentCmd) {
            case CMD_MOTOR:
                Serial.printf("  dir=%u speed=%u", buffer[0], (buffer[1] << 8) | buffer[2]);
                break;
            case CMD_SERVO:
                Serial.printf("  id=%u steer=%d%%", buffer[0],
                              (int16_t)((buffer[1] << 8) | buffer[2]));
                break;
            case CMD_LED:
                Serial.printf("  %s", buffer[0] ? "on" : "off");
                break;
            case CMD_PIXEL:
            case CMD_PIXEL_ONE: {
                const uint8_t* b = buffer;
                if (currentCmd == CMD_PIXEL_ONE) Serial.printf("  [%u]", *b++);
                else                             Serial.print("  [all]");
                Serial.printf(" %s rgbw=%u,%u,%u,%u bri=%u %u ms",
                              b[0] < PX_MODE_COUNT ? PX_MODE_NAMES[b[0]] : "???",
                              b[1], b[2], b[3], b[4], b[5], (b[6] << 8) | b[7]);
                if (b[8]) Serial.printf(" x%u (one-shot)", b[8]);
                break;
            }
            case CMD_TRIM:
                Serial.printf("  %s", buffer[0] == 0 ? "left" :
                                      buffer[0] == 1 ? "right" : "save");
                break;
            case CMD_PID_SET: {
                int32_t raw = readI32BE(&buffer[1]);
                Serial.printf("  param=%u value=%.3f", buffer[0], raw / 1000.0f);
                break;
            }
            case CMD_MOVE: {
                int32_t d = readI32BE(&buffer[1]);
                Serial.printf("  id=%u by %+.1f deg", buffer[0], d / 10.0f);
                break;
            }
            case CMD_CAL:
                Serial.printf("  %s arg=%u", calActionName(buffer[0]), buffer[1]);
                break;
            case CMD_TIME_SYNC:
                Serial.printf("  seq=%u", buffer[0]);
                break;
            case CMD_STAMP_MODE:
                Serial.printf("  %s", buffer[0] ? "on" : "off");
                break;
            case CMD_TELEM_RATE: {
                uint16_t ms = (buffer[0] << 8) | buffer[1];
                if (ms) Serial.printf("  every %u ms (%.1f Hz)", ms, 1000.0f / ms);
                else    Serial.print("  off");
                break;
            }
            default:
                break;   // Commands without payload
        }
        Serial.println();
    }

    // Payload length per command. -1 = unknown.
    static int payloadLength(uint8_t cmd) {
        switch (cmd) {
            case CMD_MOTOR:      return 3;
            case CMD_SERVO:      return 3;
            case CMD_LED:        return 1;
            case CMD_PIXEL:      return 9;
            case CMD_PIXEL_ONE:  return 10;
            case CMD_TRIM:       return 1;
            case CMD_PID_SET:    return 5;
            case CMD_MOVE:       return 5;
            case CMD_CAL:        return 2;
            case CMD_TIME_SYNC:  return 1;
            case CMD_STAMP_MODE: return 1;
            case CMD_TELEM_RATE: return 2;
            case CMD_CALIBRATE:
            case CMD_TORQUE:
            case CMD_PID_GET:
            case CMD_PID_SAVE:
            case CMD_MOVE_ABORT:
            case CMD_PROGRESS:
            case CMD_BATTERY:
            case CMD_EMERGENCY:  return 0;
            default:             return -1;
        }
    }

    void executeCommand() {
        switch (currentCmd) {

        case CMD_EMERGENCY:
            // Brake actively instead of just coasting; also aborts a move.
            setMotorCommand(MOTOR_BRAKE, false, DUTY_MAX);
            serialPort->println("ESP: EMERGENCY STOP EXECUTED!");
            break;

        case CMD_MOTOR: {
            bool     reverse = buffer[0];
            uint16_t speed   = (buffer[1] << 8) | buffer[2];   // 0..1023
            // Speed 0 = coast. Braking is done via CMD_EMERGENCY.
            setMotorCommand(speed ? MOTOR_DRIVE : MOTOR_COAST, reverse, speed);
            break;
        }

        case CMD_MOVE: {
            uint8_t id = buffer[0];
            int32_t targetDeg10 = readI32BE(&buffer[1]);
            startMove(id, targetDeg10);
            break;
        }

        case CMD_MOVE_ABORT:
            abortMove();
            break;

        case CMD_PROGRESS:
            sendProgress();
            break;

        case CMD_BATTERY:
            sendBattery(CMD_BATTERY_RSP);
            break;

        // Time synchronization. Must be answered without detour: every millisecond
        // that passes between reception and response enters the clock offset as
        // uncertainty. That is why it is handled directly here in the parser
        // context and not via the queue or the next loop().
        case CMD_TIME_SYNC:
            sendTimeSync(buffer[0], frameRxUs);
            break;

        case CMD_STAMP_MODE:
            g_stampTx = (buffer[0] != 0);
            sendStampMode();
            break;

        // Set the drive telemetry rate. The immediate response also serves as
        // an acknowledgement - it tells the bridge that the rate has arrived,
        // without having to wait for the first regular packet.
        case CMD_TELEM_RATE: {
            uint16_t ms = (buffer[0] << 8) | buffer[1];
            g_telemetryMs = ms ? (ms < TELEMETRY_MS_MIN ? TELEMETRY_MS_MIN : ms) : 0;
            sendTelemetry();
            break;
        }

        case CMD_PID_GET:
            sendPidParams();
            break;

        case CMD_PID_SAVE:
            sendPidSaved(savePidParams());
            break;

        case CMD_PID_SET: {
            uint8_t param = buffer[0];
            int32_t raw   = readI32BE(&buffer[1]);
            float   val   = raw / 1000.0f;
            PidParams p = getPid();
            switch (param) {
                case 0: p.kp        = val;                        break;
                case 1: p.ki        = val;                        break;
                case 2: p.kd        = val;                        break;
                case 3: p.iTermLim  = val;                        break;
                case 4: p.maxDuty   = (uint16_t)constrain(raw / 1000, 0L, (long)DUTY_MAX); break;
                case 5: p.tolDeg10  = raw / 1000;                 break;
                case 6: p.settleMs  = (uint32_t)(raw / 1000);     break;
                case 7: p.timeoutMs = (uint32_t)(raw / 1000);     break;
                case 8: p.minDuty   = (uint16_t)constrain(raw / 1000, 0L, (long)DUTY_MAX); break;
                default: return;   // unknown parameter, silently discard
            }
            setPid(p);
            break;
        }

        case CMD_SERVO: {
            uint8_t id = buffer[0];
            int16_t steerPct = (buffer[1] << 8) | buffer[2];

            // During calibration no steering command may interfere -
            // it would immediately discard the manually approached position.
            if (g_cal.active) {
                static uint32_t lastHint = 0;
                if (millis() - lastHint > 2000) {
                    lastHint = millis();
                    Serial.println("ESP: Steering command ignored - calibration running.");
                }
                break;
            }

            // --- Dynamic travel limit to 80% ---
            const float MAX_THROW_FACTOR = 1.0f;

            int maxDistRight = softwareCenterPos - rightLimit;
            int maxDistLeft  = leftLimit - softwareCenterPos;

            int safeRight = softwareCenterPos - (maxDistRight * MAX_THROW_FACTOR);
            int safeLeft  = softwareCenterPos + (maxDistLeft  * MAX_THROW_FACTOR);

            int physicalPos = softwareCenterPos;

            if (steerPct < -100) steerPct = -100;
            if (steerPct >  100) steerPct =  100;

            if (steerPct > 0) {
                physicalPos = map(steerPct, 0, 100, softwareCenterPos, safeLeft);
            } else if (steerPct < 0) {
                physicalPos = map(steerPct, -100, 0, safeRight, softwareCenterPos);
            }

            servo->WritePosEx(id, physicalPos, SERVO_SPEED_FAST, SERVO_ACC);
            break;
        }

        case CMD_LED:
            digitalWrite(PIN_LED, buffer[0] ? HIGH : LOW);
            break;

        // RGBW status LEDs: 0x31 for all, 0x32 for one (index in front).
        // Only hands the setpoint over to the pixel task on core 0 - nothing
        // here waits for the LEDs.
        case CMD_PIXEL:
        case CMD_PIXEL_ONE: {
            int idx = -1;
            const uint8_t* b = buffer;
            if (currentCmd == CMD_PIXEL_ONE) {
                idx = *b++;
                if (idx >= PIXEL_COUNT) break;        // LED does not exist, discard
            }
            if (b[0] >= PX_MODE_COUNT) break;        // unknown mode, discard
            PixelAnim a;
            a.mode       = b[0];
            a.r          = b[1];
            a.g          = b[2];
            a.b          = b[3];
            a.w          = b[4];
            a.brightness = b[5];
            a.periodMs   = (b[6] << 8) | b[7];
            a.count      = b[8];
            setPixel(idx, a);
            break;
        }

        // Starts the manual calibration. Automatic probing no longer exists -
        // the SC09 cannot limit its torque.
        // The actual steps are handled via CMD_CAL.
        case CMD_CALIBRATE:
            calStart(servo);
            sendCalState(CAL_ST_OK);
            break;

        case CMD_CAL:
            sendCalState(calHandleAction(servo, buffer[0], buffer[1]));
            break;

        case CMD_TORQUE:
            printTorque(servo);
            break;

        case CMD_TRIM: {
            uint8_t action = buffer[0];
            if (action == 0x00) {          // Trim -8 ticks
                trimOffset        -= 8;
                softwareCenterPos  = centerLimit + trimOffset;
                servo->WritePosEx(SERVO_ID, softwareCenterPos, SERVO_SPEED_FAST, SERVO_ACC);
                serialPort->printf("ESP: Trim L | Offset: %d | Pos: %d\n", trimOffset, softwareCenterPos);
            } else if (action == 0x01) {   // Trim +8 ticks
                trimOffset        += 8;
                softwareCenterPos  = centerLimit + trimOffset;
                servo->WritePosEx(SERVO_ID, softwareCenterPos, SERVO_SPEED_FAST, SERVO_ACC);
                serialPort->printf("ESP: Trim R | Offset: %d | Pos: %d\n", trimOffset, softwareCenterPos);
            } else if (action == 0x02) {   // Save offset permanently
                prefs.putInt("offset10", trimOffset);
                serialPort->printf("ESP: Trim offset (%d) saved to flash!\n", trimOffset);
            }
            break;
        }
        }
    }
};

// ==========================================
// 10. USB DEBUG CONSOLE (Core 1)
// ==========================================

MotorDriver planetaryMotor(PIN_MOTOR_PWM, PIN_MOTOR_INA, PIN_MOTOR_INB);
SCSCL sc09Servo;
JetsonComms jetson(&Serial1, &sc09Servo);

String cmdBuf;
uint8_t nextLocalMoveId = 1;   // IDs for moves started from the USB console

void printPidParams() {
    PidParams p = getPid();
    Serial.printf("PID  Kp=%.3f  Ki=%.3f  Kd=%.3f\n", p.kp, p.ki, p.kd);
    Serial.printf("     iLim=%.0f  maxDuty=%u  minDuty=%u  tol=%.1f deg (%ld counts)\n",
                  p.iTermLim, p.maxDuty, p.minDuty, p.tolDeg10 / 10.0f,
                  (long)deg10ToCounts(p.tolDeg10));
    Serial.printf("     settle=%lu ms  timeout=%lu ms\n",
                  (unsigned long)p.settleMs, (unsigned long)p.timeoutMs);
}

// One line in Arduino Serial Plotter format: "name:value" separated by
// spaces.
//
// Both angles are RELATIVE to the start of the move, not absolute. Absolute
// values would be around 1900 degrees, and the plotter would scale the Y axis to
// that - the actual 90-degree movement would then be 5 % of the axis height and
// look like a flat line. Relative, actual runs from 0 to target, and duty in
// percent is in the same order of magnitude.
void plotTick() {
    static uint32_t lastPlot = 0;
    if (!g_plotMode) return;
    uint32_t now = millis();
    if (now - lastPlot < 20) return;   // 50 Hz
    lastPlot = now;

    MoveState m = getMoveState();
    Serial.printf("target:%.1f actual:%.1f duty:%.1f\n",
                  countsToDeg10(m.targetCnt - m.startCnt) / 10.0f,
                  countsToDeg10(g_encCount   - m.startCnt) / 10.0f,
                  (g_dutySigned * 100.0f) / DUTY_MAX);
}

// Writes the complete parameter set to NVS. Returns false as soon as a key
// could not be written (full or corrupted partition).
// NVS itself discards writes with an unchanged value, so saving identical
// values repeatedly costs no flash cycles.
bool savePidParams() {
    PidParams p = getPid();
    bool ok = true;
    ok &= prefs.putFloat("kp",     p.kp)        > 0;
    ok &= prefs.putFloat("ki",     p.ki)        > 0;
    ok &= prefs.putFloat("kd",     p.kd)        > 0;
    ok &= prefs.putFloat("ilim",   p.iTermLim)  > 0;
    ok &= prefs.putUShort("mduty", p.maxDuty)   > 0;
    ok &= prefs.putUShort("mind",  p.minDuty)   > 0;
    ok &= prefs.putInt("tol",      p.tolDeg10)  > 0;
    ok &= prefs.putUInt("settle",  p.settleMs)  > 0;
    ok &= prefs.putUInt("tmo",     p.timeoutMs) > 0;
    return ok;
}

// Missing keys are caught via isKey(). Without that, ESP-IDF logs an [E] line
// ("nvs_get_blob") for every value that has never been saved, which looks like
// a real error while debugging - even though the default takes effect.
void loadPidParams() {
    PidParams p;   // defaults from the struct
    if (prefs.isKey("kp"))     p.kp        = prefs.getFloat("kp",     p.kp);
    if (prefs.isKey("ki"))     p.ki        = prefs.getFloat("ki",     p.ki);
    if (prefs.isKey("kd"))     p.kd        = prefs.getFloat("kd",     p.kd);
    if (prefs.isKey("ilim"))   p.iTermLim  = prefs.getFloat("ilim",   p.iTermLim);
    if (prefs.isKey("mduty"))  p.maxDuty   = prefs.getUShort("mduty", p.maxDuty);
    if (prefs.isKey("mind"))   p.minDuty   = prefs.getUShort("mind",  p.minDuty);
    if (prefs.isKey("tol"))    p.tolDeg10  = prefs.getInt("tol",      p.tolDeg10);
    if (prefs.isKey("settle")) p.settleMs  = prefs.getUInt("settle",  p.settleMs);
    if (prefs.isKey("tmo"))    p.timeoutMs = prefs.getUInt("tmo",     p.timeoutMs);
    setPid(p);
}

void printHelp() {
    Serial.println("--- Motor (open loop) ---");
    Serial.println("  f<0-255> r<0-255> b<0-255>  fwd/rev/brake");
    Serial.println("  c        coast            z  encoder to 0");
    Serial.println("  e        telemetry (counts, deg, RPM, duty, current)");
    Serial.println("  i        motor current  ic<mV/A> scale  iz  zero point");
    Serial.println("--- Position ---");
    Serial.println("  g<deg>   rotate X degrees further (relative, e.g. g90.0 / g-45)");
    Serial.println("  gp<deg>  same, plots automatically until the move ends");
    Serial.println("  q        abort the running move");
    Serial.println("  w        progress of the move");
    Serial.println("--- PID ---");
    Serial.println("  kp<f> ki<f> kd<f>          controller gains");
    Serial.println("  kl<f> anti-windup limit    km<0-1023> max duty");
    Serial.println("  ka<0-1023> start-up duty   kt<deg> target window");
    Serial.println("  kn<ms> settle time         kx<ms> timeout");
    Serial.println("  pid   show                 pids  save to flash");
    Serial.println("  p     plotter on/off (Arduino Serial Plotter, 50 Hz)");
    Serial.println("--- Jetson link (IO10 RX / IO11 TX) ---");
    Serial.println("  dbg      statistics   dbg0 off  dbg1 packets  dbg2 +raw bytes");
    Serial.println("  ts       clock + sync status ts0/ts1  TX timestamps off/on");
    Serial.println("  tel      drive telemetry  tel<ms> set interval (tel0 = off)");
    Serial.println("  sw       speed window     sw<n> in multiples of 10 ms");
    Serial.println("  o        toggle LED (button reports as [TX] BUTTON)");
    Serial.println("--- RGBW LEDs (SK6812 chain, IO40) ---");
    Serial.println("  px       status   px 255 0 0 [w] / px #ff0000 / px red  colour");
    Serial.println("  px blink|breathe|rainbow|strobe|heart|solid|off [ms] [count]");
    Serial.println("           count > 0 = one-shot, then back   px bri<0-255>");
    Serial.println("  px stop  end one-shot   px save  power-on default");
    Serial.println("  px<n> .. only LED n, e.g. px2 red / px0 blink 300 3 (no space!)");
    Serial.println("--- Battery ---");
    Serial.println("  v        measure now       vc<factor>  calibrate divider");
    Serial.println("  vd       pin diagnosis (battery off): checks the divider at the ADC input");
    Serial.println("  vm       live output for wiggle test  vm<ms> interval  vm0 off");
    Serial.println("--- Steering: manual calibration ---");
    Serial.println("  cal      start / status (x = same)");
    Serial.println("  + / -    one step (also '+50' = one-off 50 ticks)");
    Serial.println("  caln<t>  step size in ticks (default 10, ~2.9 deg)");
    Serial.println("  calm     center  call  left end stop  calr  right end stop");
    Serial.println("  calfree  torque off (set by hand)        calhold  hold again");
    Serial.println("  calgo    go to center  calsave  save  calq  abort");
    Serial.println("--- Steering: operation ---");
    Serial.println("  t torque  a/d trim Pos-/Pos+  s save trim");
    Serial.println("  j/l Pos-/Pos+  m center  sv diagnostics  sv<0-1023> pos  sve0/1 torque");
    Serial.println("  svs protocol scan  svpos force position mode (against wheel mode)");
    Serial.println("  svpin<rx>,<tx> swap Serial2 pins (e.g. svpin17,18)");
    Serial.println("  svbaud<n> baud (scope)  svtx continuous 0x55 signal on TX (scope test)");
    Serial.println("  h        this help");
}

// Named colours for the console, RGBW. Pure white uses the W channel - it is
// brighter and cleaner than R+G+B mixed.
struct PixelColorName { const char* name; uint8_t r, g, b, w; };
static const PixelColorName PX_COLORS[] = {
    {"red",     255,   0,   0,   0}, {"green",  0, 255,   0,   0},
    {"blue",      0,   0, 255,   0}, {"white",  0,   0,   0, 255},
    {"yellow",  255, 160,   0,   0}, {"orange", 255, 60,  0,   0},
    {"cyan",      0, 255, 255,   0}, {"magenta", 255, 0, 255,   0},
    {"purple",  120,   0, 255,   0}, {"warm",  255, 80,   0, 180},
};

// "px ..." - RGBW LEDs from the console. Arguments separated by spaces or commas:
//   px                         status
//   px <r> <g> <b> [w]         colour 0..255     px #RRGGBB[WW]  same in hex
//   px red|green|...           named colour
//   px <mode> [ms] [count]     animation with the current colour; count > 0 =
//                              one-shot, afterwards the previous state returns
//   px bri<0-255>  px stop  px save
// Without a number everything applies to all LEDs. A number written directly
// after "px" selects one LED: "px2 red", "px0 breathe 2000". It has to be
// glued on - "px 2 ..." would be indistinguishable from a colour "px 255 0 0".
// A colour keeps the running animation (breathe stays breathe, only bluer) -
// only from off or rainbow, where the colour would be invisible, it switches
// to solid. For all LEDs, LED 0 is the reference for what is kept.
void handlePixelCommand(String arg) {
    int idx = -1;
    if (arg.length() > 0 && isDigit(arg.charAt(0))) {
        int end = 0;
        while (end < (int)arg.length() && isDigit(arg.charAt(end))) end++;
        idx = arg.substring(0, end).toInt();
        arg = arg.substring(end);
        if (idx >= PIXEL_COUNT) {
            Serial.printf("?? LED %d does not exist (0..%d)\n", idx, PIXEL_COUNT - 1);
            return;
        }
    }
    arg.replace(",", " ");
    arg.trim();

    String tok[5];
    int n = 0;
    while (arg.length() > 0 && n < 5) {
        int sp = arg.indexOf(' ');
        tok[n++] = (sp < 0) ? arg : arg.substring(0, sp);
        arg = (sp < 0) ? String() : arg.substring(sp + 1);
        arg.trim();
    }

    if (n == 0) {
        printPixelState();
        Serial.println("  px <r> <g> <b> [w] | px #RRGGBB[WW] | px red/green/blue/white/...");
        Serial.println("  px <off|solid|blink|breathe|rainbow|strobe|heart> [ms] [count]");
        Serial.println("  px bri<0-255> | px stop (end one-shot) | px save (power-on default)");
        Serial.printf ("  one LED: number right after px, e.g. px2 red, px0 blink 300 3 (0..%d)\n",
                       PIXEL_COUNT - 1);
        return;
    }

    PixelAnim a = getPixelBase(idx);
    a.count = 0;
    auto colourSet = [&]() {
        if (a.mode == PX_OFF || a.mode == PX_RAINBOW) a.mode = PX_SOLID;
    };

    if (tok[0] == "save") {
        Serial.println(savePixelDefault() ? "-> LED states saved as power-on default"
                                          : "-> ERROR while saving (NVS)");
        return;
    }
    if (tok[0] == "stop") {
        stopPixelShot(idx);
        Serial.println("-> one-shot ended");
        return;
    }
    if (tok[0].startsWith("bri")) {
        String v = tok[0].substring(3);
        if (v.length() == 0 && n > 1) v = tok[1];
        a.brightness = (uint8_t)constrain(v.toInt(), 0L, 255L);
    }
    else if (tok[0].charAt(0) == '#') {
        String hex = tok[0].substring(1);
        if (hex.length() != 6 && hex.length() != 8) {
            Serial.println("?? format #RRGGBB or #RRGGBBWW");
            return;
        }
        uint32_t v = strtoul(hex.c_str(), nullptr, 16);
        if (hex.length() == 6) v <<= 8;
        a.r = (uint8_t)(v >> 24); a.g = (uint8_t)(v >> 16);
        a.b = (uint8_t)(v >> 8);  a.w = (uint8_t)v;
        colourSet();
    }
    else if (isDigit(tok[0].charAt(0))) {
        if (n < 3) { Serial.println("?? px <r> <g> <b> [w]"); return; }
        a.r = (uint8_t)constrain(tok[0].toInt(), 0L, 255L);
        a.g = (uint8_t)constrain(tok[1].toInt(), 0L, 255L);
        a.b = (uint8_t)constrain(tok[2].toInt(), 0L, 255L);
        a.w = (n > 3) ? (uint8_t)constrain(tok[3].toInt(), 0L, 255L) : 0;
        colourSet();
    }
    else {
        bool found = false;
        for (const auto& c : PX_COLORS) {
            if (tok[0] == c.name) {
                a.r = c.r; a.g = c.g; a.b = c.b; a.w = c.w;
                colourSet();
                found = true;
                break;
            }
        }
        if (!found) {
            for (uint8_t m = 0; m < PX_MODE_COUNT; m++) {
                if (tok[0] == PX_MODE_NAMES[m]) {
                    a.mode     = m;
                    a.periodMs = (n > 1) ? (uint16_t)constrain(tok[1].toInt(), 0L, 65535L) : 0;
                    a.count    = (n > 2) ? (uint8_t)constrain(tok[2].toInt(), 0L, 255L)   : 0;
                    found = true;
                    break;
                }
            }
        }
        if (!found) {
            Serial.println("?? unknown colour/mode - 'px' shows the options");
            return;
        }
    }

    setPixel(idx, a);
    if (idx < 0) Serial.print("-> all LEDs ");
    else         Serial.printf("-> LED %d ", idx);
    Serial.printf("%s rgbw=%u,%u,%u,%u bri=%u period=%u ms",
                  PX_MODE_NAMES[a.mode], a.r, a.g, a.b, a.w, a.brightness, pixelPeriod(a));
    if (a.count) Serial.printf(" x%u (one-shot, %lu ms)", a.count,
                               (unsigned long)pixelPeriod(a) * a.count);
    Serial.println();
}

void handleDebugCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    // --- Multi-letter commands first ---
    if (cmd.startsWith("pids")) {
        Serial.println(savePidParams() ? "-> PID saved to flash"
                                       : "-> ERROR while saving (NVS)");
        return;
    }
    if (cmd.startsWith("pid"))  { printPidParams(); return; }

    // Start a move and plot it. Must come before the single-letter 'g'.
    if (cmd.startsWith("gp")) {
        float grad = cmd.substring(2).toFloat();
        uint8_t id = nextLocalMoveId++;
        if (nextLocalMoveId == 0) nextLocalMoveId = 1;
        Serial.printf("-> move id=%u by %.1f deg | plotter until move ends\n", id, grad);
        g_plotMode = true;
        g_plotAuto = true;
        startMove(id, (int32_t)lroundf(grad * 10.0f));
        return;
    }

    // --- Manual steering calibration ---
    // Must come before the single-letter commands ('c' is coast).
    if (cmd.startsWith("cal")) {
        String arg = cmd.substring(3);
        arg.trim();
        if (arg.length() == 0) {
            if (g_cal.active) calPrintStatus(&sc09Servo);
            else              calStart(&sc09Servo);
        }
        else if (arg == "?")     calPrintStatus(&sc09Servo);
        else if (arg == "m")     calSetMark(&sc09Servo, CAL_ACT_CENTER);
        else if (arg == "l")     calSetMark(&sc09Servo, CAL_ACT_LEFT);
        else if (arg == "r")     calSetMark(&sc09Servo, CAL_ACT_RIGHT);
        else if (arg == "save")  calSave(&sc09Servo);
        else if (arg == "q")     calAbort(&sc09Servo);
        else if (arg == "free")  calSetFree(&sc09Servo, true);
        else if (arg == "hold")  calSetFree(&sc09Servo, false);
        else if (arg == "go")    calGoCenter(&sc09Servo);
        else if (arg.charAt(0) == 'n') calSetStep(arg.substring(1).toInt());
        else if (arg.charAt(0) == '+' || arg.charAt(0) == '-') {
            int n = arg.substring(1).toInt();
            if (n <= 0) n = g_cal.step;
            calMove(&sc09Servo, arg.charAt(0) == '+' ? n : -n);
        }
        else {
            Serial.println("?? cal, cal+/cal-, caln<ticks>, calm/call/calr,");
            Serial.println("   calfree/calhold/calgo, calsave, calq");
        }
        return;
    }

    // --- Speed measurement window ---
    // Must come before the single-letter commands ('s' = stop).
    if (cmd.startsWith("sw")) {
        String arg = cmd.substring(2);
        arg.trim();
        if (arg.length() > 0) {
            long n = arg.toInt();
            if (n < 1)                       n = 1;
            if (n > SPEED_WINDOW_MAX - 1)    n = SPEED_WINDOW_MAX - 1;
            g_speedWindow = (uint8_t)n;
        }
        uint32_t fensterMs = (uint32_t)g_speedWindow * TASK_PERIOD;
        float schrittRpm = 60000.0f / (COUNTS_PER_REV * (float)fensterMs);
        Serial.printf("-> speed window %u x %lu ms = %lu ms\n",
                      g_speedWindow, (unsigned long)TASK_PERIOD,
                      (unsigned long)fensterMs);
        Serial.printf("   one pulse = %.1f RPM = %.1f deg/s, "
                      "delay %.0f ms\n",
                      schrittRpm, schrittRpm * 6.0f, fensterMs / 2.0f);
        Serial.println("   Output rate is not affected by this (see 'tel').");
        return;
    }

    // --- Drive telemetry ---
    // Must come before the single-letter commands ('t' = torque).
    if (cmd.startsWith("tel")) {
        String arg = cmd.substring(3);
        arg.trim();
        if (arg.length() > 0) {
            long ms = arg.toInt();
            if (ms <= 0)                      g_telemetryMs = 0;
            else if (ms < TELEMETRY_MS_MIN)   g_telemetryMs = TELEMETRY_MS_MIN;
            else if (ms > 60000)              g_telemetryMs = 60000;
            else                              g_telemetryMs = (uint16_t)ms;
        }
        if (g_telemetryMs) {
            Serial.printf("-> telemetry every %u ms (%.1f Hz)\n",
                          g_telemetryMs, 1000.0f / g_telemetryMs);
            Serial.printf("   speed from a window of %u ms ('sw'), "
                          "but recomputed for every packet.\n",
                          (unsigned)g_speedWindow * (unsigned)TASK_PERIOD);
        } else {
            Serial.println("-> telemetry off ('tel100' = 10 Hz)");
        }
        Serial.printf("   now: %.1f deg  %.1f deg/s  duty=%d  I=%.2f A\n",
                      countsToDeg10(g_encCount) / 10.0f, g_rpm * 6.0f,
                      g_dutySigned, readMotorCurrentA());
        return;
    }

    // --- Time synchronization / TX timestamps ---
    // Must come before the single-letter commands ('t' = torque).
    if (cmd.startsWith("ts")) {
        String arg = cmd.substring(2);
        arg.trim();
        if (arg.length() > 0) {
            g_stampTx = (arg.toInt() != 0);
            jetson.sendStampMode();      // inform the bridge about the change
        }
        Serial.printf("-> TX timestamps %s (frame 0x%02X)\n",
                      g_stampTx ? "ON" : "OFF", g_stampTx ? START_BYTE_TS : START_BYTE);
        Serial.printf("   ESP clock: %lld us since boot (%.3f s)\n",
                      (long long)nowUs(), nowUs() / 1000000.0);
        Serial.println("   The Jetson does the alignment with CMD_TIME_SYNC (0xB0).");
        return;
    }

    if (cmd.startsWith("dbg")) {
        String arg = cmd.substring(3);
        if (arg.length() > 0) {
            g_linkDebug = (uint8_t)constrain(arg.toInt(), 0L, 2L);
            Serial.printf("-> link debug = %u (%s)\n", g_linkDebug,
                          g_linkDebug == 0 ? "off" :
                          g_linkDebug == 1 ? "packets" : "packets + raw bytes");
        }
        jetson.printLinkStats();
        return;
    }

    // --- RGBW LED --- must come before the single-letter 'p' (plotter).
    if (cmd.startsWith("px")) {
        handlePixelCommand(cmd.substring(2));
        return;
    }

    // --- Calibrate motor current ---
    if (cmd.startsWith("ic")) {
        float f = cmd.substring(2).toFloat();
        if (f > 1.0f && f < 5000.0f) {
            csMvPerA = f;
            prefs.putFloat("csmv", csMvPerA);
            Serial.printf("-> current sense = %.1f mV/A (saved)\n", csMvPerA);
        } else {
            Serial.printf("-> currently %.1f mV/A (nominal %.1f). Format: ic140\n",
                          csMvPerA, CS_MV_PER_A_NOMINAL);
        }
        return;
    }
    // Zero point with the motor at rest. Must be measured in coast, otherwise
    // the quiescent current ends up in the offset and every later reading is too low.
    if (cmd == "iz") {
        setMotorCommand(MOTOR_COAST, false, 0);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        csZeroMv = (int)lroundf(readMotorCsMv());
        prefs.putInt("csoff", csZeroMv);
        Serial.printf("-> zero point = %d mV (saved)\n", csZeroMv);
        return;
    }

    if (cmd == "vd") {
        batteryPinDiagnose();
        return;
    }

    if (cmd.startsWith("vm")) {
        long ms = cmd.substring(2).toInt();
        if (cmd.length() == 2) ms = g_battMonMs ? 0 : 250;   // "vm" toggles
        g_battMonMs = (ms > 0) ? (uint32_t)max(ms, 50L) : 0;
        g_battMonLast = 0;
        if (g_battMonMs) Serial.printf("-> battery live output every %lu ms (vm0 = off)\n",
                                       (unsigned long)g_battMonMs);
        else             Serial.println("-> battery live output off");
        return;
    }

    if (cmd.startsWith("vc")) {
        float f = cmd.substring(2).toFloat();
        if (f > 1.0f && f < 50.0f) {
            battDivider = f;
            prefs.putFloat("bdiv", battDivider);
            Serial.printf("-> divider factor = %.4f (saved)\n", battDivider);
        } else {
            Serial.printf("-> current divider factor = %.4f (nominal %.4f)\n",
                          battDivider, BATT_DIVIDER_NOMINAL);
        }
        return;
    }

    // --- Direct servo/steering (must come before the single-letter commands) ---
    if (cmd.startsWith("sv")) {
        String arg = cmd.substring(2);
        if (arg.length() == 0) {
            servoDiagnose(&sc09Servo);                 // "sv" = diagnostics
        } else if (arg == "s") {
            servoScanProtocols(&sc09Servo);            // "svs" = protocol scan
        } else if (arg.startsWith("baud")) {           // "svbaud<n>" for the scope
            long b = arg.substring(4).toInt();
            if (b >= 1200 && b <= 1000000) {
                g_servoBaud = (uint32_t)b;
                servoBusRestart(&sc09Servo);
                Serial.printf("-> servo baud = %lu (for real operation set svbaud1000000 again!)\n",
                              (unsigned long)g_servoBaud);
            } else {
                Serial.printf("-> currently %lu baud. Range 1200..1000000.\n",
                              (unsigned long)g_servoBaud);
            }
        } else if (arg == "tx") {                       // "svtx" continuous 0x55 for the scope
            g_servoTxTest = !g_servoTxTest;
            Serial.printf("-> TX test pattern 0x55 %s (measure at IO%d)%s\n",
                          g_servoTxTest ? "ON" : "OFF", g_servoTx,
                          g_servoTxTest ? "" : "");
            if (g_servoTxTest && g_servoBaud > 115200)
                Serial.println("   Note: for slow scopes run 'svbaud9600' first.");
        } else if (arg.startsWith("pin")) {            // "svpin<rx>,<tx>"
            String rest = arg.substring(3);
            rest.replace(",", " ");
            int sp = rest.indexOf(' ');
            if (sp > 0) {
                int rx = rest.substring(0, sp).toInt();
                int tx = rest.substring(sp + 1).toInt();
                servoSetPins(&sc09Servo, rx, tx);
            } else {
                Serial.printf("-> currently RX=IO%d TX=IO%d. Format: svpin18,17\n",
                              g_servoRx, g_servoTx);
            }
        } else if (arg == "pos") {
            servoForcePositionMode(&sc09Servo);            // "svpos" = force position mode
        } else if (arg == "r") {
            servoRegisterDump(&sc09Servo);                 // "svr" = registers + write test
        } else if (arg == "c") {
            servoWriteRaw(&sc09Servo, SERVO_CENTER_DEF);   // "svc" = center (raw)
        } else if (arg.charAt(0) == 'e') {
            int on = arg.substring(1).toInt();
            int ret = sc09Servo.EnableTorque(SERVO_ID, on ? 1 : 0);
            Serial.printf("-> torque %s (ret=%d)\n", on ? "ON" : "OFF", ret);
        } else {
            servoWriteRaw(&sc09Servo, arg.toInt());    // "sv700" = fixed position
        }
        return;
    }

    if (cmd.length() >= 2 && cmd.charAt(0) == 'k') {
        PidParams p = getPid();
        float f = cmd.substring(2).toFloat();
        switch (cmd.charAt(1)) {
            case 'p': p.kp = f;                                           break;
            case 'i': p.ki = f;                                           break;
            case 'd': p.kd = f;                                           break;
            case 'l': p.iTermLim = f;                                     break;
            case 'm': p.maxDuty = (uint16_t)constrain((long)f, 0L, (long)DUTY_MAX); break;
            case 'a': p.minDuty = (uint16_t)constrain((long)f, 0L, (long)DUTY_MAX); break;
            case 't': p.tolDeg10 = (int32_t)lroundf(f * 10.0f);           break;
            case 'n': p.settleMs = (uint32_t)f;                           break;
            case 'x': p.timeoutMs = (uint32_t)f;                          break;
            default: Serial.println("?? unknown PID parameter");      return;
        }
        setPid(p);
        printPidParams();
        return;
    }

    // --- Single-letter commands ---
    char c = cmd.charAt(0);

    // f/r/b still take 0-255 (as in the test script) and are scaled to 10 bits.
    int val255 = constrain(cmd.substring(1).toInt(), 0, 255);
    uint16_t duty = (uint16_t)((val255 * DUTY_MAX) / 255);

    switch (c) {
        case 'f': setMotorCommand(MOTOR_DRIVE, false, duty); Serial.printf("-> forward %d (duty %u)\n", val255, duty); break;
        case 'r': setMotorCommand(MOTOR_DRIVE, true,  duty); Serial.printf("-> reverse %d (duty %u)\n", val255, duty); break;
        case 'b': setMotorCommand(MOTOR_BRAKE, false, duty); Serial.printf("-> brake %d\n", val255);                   break;
        case 'c': setMotorCommand(MOTOR_COAST, false, 0);    Serial.println("-> coast");                               break;

        case 'z':
            encoder.clearCount();
            Serial.println("-> encoder = 0");
            break;

        case 'e':
            Serial.printf("enc=%ld (%.1f deg)  rpm=%.1f  duty=%d  I=%.2f A (%.0f mV)\n",
                          g_encCount, countsToDeg10(g_encCount) / 10.0f,
                          g_rpm, g_dutySigned, readMotorCurrentA(), readMotorCsMv());
            break;

        // --- Motor current ---
        case 'i':
            Serial.printf("Motor current: %.2f A  (CS %.0f mV, zero %d mV, %.1f mV/A)\n",
                          readMotorCurrentA(), readMotorCsMv(), csZeroMv, csMvPerA);
            break;

        // --- Position move ---
        case 'g': {
            float grad = cmd.substring(1).toFloat();
            uint8_t id = nextLocalMoveId++;
            if (nextLocalMoveId == 0) nextLocalMoveId = 1;
            startMove(id, (int32_t)lroundf(grad * 10.0f));
            Serial.printf("-> move id=%u by %.1f deg (from %.1f)\n",
                          id, grad, countsToDeg10(g_encCount) / 10.0f);
            break;
        }

        case 'q':
            abortMove();
            Serial.println("-> move aborted");
            break;

        case 'w': {
            MoveState m = getMoveState();
            Serial.printf("move id=%u %s  %u%%  actual=%.1f  target=%.1f deg\n",
                          m.id, m.active ? "ACTIVE" : "idle", m.progress,
                          countsToDeg10(g_encCount) / 10.0f,
                          countsToDeg10(m.targetCnt) / 10.0f);
            break;
        }

        // --- Battery ---
        case 'v': {
            float pinMv = readBatteryMv();
            battPackV = (pinMv / 1000.0f) * battDivider;
            battCellV = battPackV / BATT_CELLS;
            Serial.printf("Battery: %.2f V pack | %.3f V/cell (%dS)%s\n",
                          battPackV, battCellV, BATT_CELLS,
                          battCellV < BATT_WARN_CELL ? "  *** LOW ***" : "");
            Serial.printf("      GPIO%d: %.1f mV at divider tap, raw %d/4095, factor %.4f\n",
                          PIN_BATTERY, pinMv, analogRead(PIN_BATTERY), battDivider);
            if (pinMv >= BATT_ADC_CLIP_MV) {
                Serial.println("      *** ADC CLIPPED - pack voltage not measurable ***");
                Serial.println("      Hold multimeter on GPIO1 against GND:");
                Serial.println("        ~3.0 V (pack/5.5) -> divider ok, pack too high for measuring range");
                Serial.println("        clearly higher    -> 22k leg to GND open (cold solder joint?)");
            }
            break;
        }

        // --- Steering ---
        case 'x':   // 'c' is coast -> calibration is on 'x'
            calStart(&sc09Servo);
            break;

        case 't':
            printTorque(&sc09Servo);
            break;

        // --- Move the servo step by step ---
        // Without a number the calibration step size applies (or 100 ticks outside
        // of calibration), with a number the given value: "-40", "j25".
        case '+':
        case '-':
        case 'j':   // position lower
        case 'l': { // position higher
            bool plus = (c == '+' || c == 'l');
            int n = cmd.substring(1).toInt();
            if (n <= 0) n = g_cal.active ? g_cal.step : 100;
            int delta = plus ? n : -n;
            if (g_cal.active) calMove(&sc09Servo, delta);
            else              servoWriteRaw(&sc09Servo, servoManualPos + delta);
            break;
        }

        case 'm':   // go to center
            if (g_cal.active) calGoCenter(&sc09Servo);
            else              servoWriteRaw(&sc09Servo, softwareCenterPos);
            break;

        // Trim shifts the center in raw position ticks. Which steering direction
        // that is depends on the mounting - hence deliberately Pos-/Pos+ here
        // instead of left/right. Sign as before, so muscle memory and
        // CMD_TRIM stay the same.
        case 'a':   // trim position lower
            trimOffset        -= 20;
            softwareCenterPos  = centerLimit + trimOffset;
            sc09Servo.WritePosEx(SERVO_ID, softwareCenterPos, SERVO_SPEED_FAST, SERVO_ACC);
            servoManualPos = softwareCenterPos;
            Serial.printf("Trim: %d | Current pos: %d\n", trimOffset, softwareCenterPos);
            break;

        case 'd':   // trim position higher
            trimOffset        += 20;
            softwareCenterPos  = centerLimit + trimOffset;
            sc09Servo.WritePosEx(SERVO_ID, softwareCenterPos, SERVO_SPEED_FAST, SERVO_ACC);
            servoManualPos = softwareCenterPos;
            Serial.printf("Trim: %d | Current pos: %d\n", trimOffset, softwareCenterPos);
            break;

        case 's':   // save trim
            prefs.putInt("offset10", trimOffset);
            Serial.println("ESP: Trim offset saved permanently!");
            break;

        // Toggle the LED. Otherwise only reachable via CMD_LED from the Jetson -
        // when bringing up a new board you want to be able to check it without a Jetson.
        case 'o': {
            static bool ledOn = false;
            ledOn = !ledOn;
            digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
            Serial.printf("-> LED (IO%d) %s\n", PIN_LED, ledOn ? "ON" : "OFF");
            break;
        }

        case 'p':
            g_plotMode = !g_plotMode;
            Serial.printf("-> Plotter %s%s\n", g_plotMode ? "ON" : "OFF",
                          g_plotMode ? " (target/actual in deg, duty in %, 50 Hz)" : "");
            break;

        case 'h':
            printHelp();
            break;

        default:
            Serial.println("?? unknown command ('h' for help)");
    }
}

// ==========================================
// 11. BATTERY MONITORING (Core 1)
// ==========================================

void batteryTick() {
    uint32_t now = millis();

    if (g_battMonMs && now - g_battMonLast >= g_battMonMs) {
        g_battMonLast = now;
        float mv = readBatteryMv();
        Serial.printf("vm: GPIO%d %7.1f mV  raw %4d/4095  -> %6.2f V%s\n",
                      PIN_BATTERY, mv, analogRead(PIN_BATTERY),
                      (mv / 1000.0f) * battDivider,
                      mv >= BATT_ADC_CLIP_MV ? "  CLIPPED" : "");
    }

    if (now - battLastRead < BATT_INTERVAL) return;
    battLastRead = now;

    float pinMv = readBatteryMv();
    battPackV = (pinMv / 1000.0f) * battDivider;
    battCellV = battPackV / BATT_CELLS;

    // A clipped value always sits at the top - so the undervoltage warning could
    // never trigger, no matter how empty the battery really is. Therefore report
    // it loudly instead of faking a measurement. Same interval as the battery warning.
    if (pinMv >= BATT_ADC_CLIP_MV &&
        (battLastWarn == 0 || now - battLastWarn >= BATT_WARN_REPEAT)) {
        battLastWarn = now;
        Serial.printf("ESP: WARNING battery ADC clipped (%.0f mV at GPIO%d)! %.2f V is "
                      "only the upper limit, not a measurement - undervoltage protection is "
                      "blind. 'v' shows details.\n",
                      pinMv, PIN_BATTERY, battPackV);
    }

    if (!battLow && battCellV < BATT_WARN_CELL) {
        battLow = true;
        battLastWarn = 0;              // warn immediately
    } else if (battLow && battCellV > BATT_RECOVER_CELL) {
        battLow = false;               // hysteresis: only clear the warning above 3.85 V
        Serial.printf("ESP: Battery ok again: %.2f V (%.3f V/cell)\n", battPackV, battCellV);
    }

    if (battLow && (battLastWarn == 0 || now - battLastWarn >= BATT_WARN_REPEAT)) {
        battLastWarn = now;
        Serial.printf("ESP: WARNING battery low! %.2f V pack | %.3f V/cell\n",
                      battPackV, battCellV);
        jetson.sendBattery(CMD_BATTERY_WARN);
    }
}

// Drive telemetry at the configured interval. Runs on Core 1 from loop() -
// so the interval is not hard, but depends on the loop duration.
// That is exactly why the packet carries its TX timestamp: the Jetson does not
// have to derive the time from the interval, it simply reads it.
void telemetryTick() {
    if (g_telemetryMs == 0) return;

    static uint32_t lastSend = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - lastSend) < g_telemetryMs) return;

    // Advance the schedule instead of setting it to "now": otherwise the
    // remainder of each loop() pass creeps into the interval and 100 Hz becomes 90.
    lastSend += g_telemetryMs;
    // After a longer pause (position move, long console output) do not
    // fire off the missed packets, but restart the schedule instead.
    if ((uint32_t)(now - lastSend) > g_telemetryMs) lastSend = now;

    jetson.sendTelemetry();
}

// ==========================================
// 12. SETUP & LOOP (Core 1)
// ==========================================

void setup() {
    Serial.begin(115200);
    delay(300);

    // --- Drive + encoder ---
    planetaryMotor.begin();

    ESP32Encoder::useInternalWeakPullResistors = puType::up;  // Hall open-drain
    // Attach the channels swapped when the drive direction is inverted - otherwise
    // the encoder counts backwards when driving forward.
    encoder.attachFullQuad(DRIVE_INVERT ? PIN_ENC_B : PIN_ENC_A,
                           DRIVE_INVERT ? PIN_ENC_A : PIN_ENC_B);   // 4x in HW
    // Glitch filter in APB clock cycles (80 MHz). 250 = ~3.1 us: kills brush noise,
    // lets real edges through (~333 us apart at full speed).
    encoder.setFilter(250);
    encoder.clearCount();

    // --- Battery ADC ---
    // 12 dB attenuation: measuring range up to ~3.1 V at the pin. 4S full (16.8 V)
    // ends up at ~3.03 V through the divider - i.e. just below the clipping limit.
    // The first analogRead attaches the pin to an ADC channel. Without it,
    // analogSetPinAttenuation fails in Core 3.x ("__analogChannelConfig(): Pin
    // is not ADC pin") and the attenuation stays at the default.
    (void)analogRead(PIN_BATTERY);
    analogSetPinAttenuation(PIN_BATTERY, ADC_11db);

    // --- Motor current ADC (VNH5019 CS) ---
    // Same order as for the battery: read first, then set attenuation.
    (void)analogRead(PIN_MOTOR_CS);
    analogSetPinAttenuation(PIN_MOTOR_CS, ADC_11db);

    // --- Peripherals ---
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), buttonISR, FALLING);

    jetson.begin(115200);

    Serial2.begin(1000000, SERIAL_8N1, PIN_SERVO_RX, PIN_SERVO_TX);
    sc09Servo.pSerial = &Serial2;
    delay(500);

    // --- NVS: steering, PID, battery calibration ---
    prefs.begin("steering", false);
    // 10-bit keys (SC servo 0..1023). Old *12 keys from the mistaken STS assumption
    // are deliberately ignored - they are in the 0..4095 range.
    leftLimit   = prefs.getInt("lLim10", SERVO_POS_MAX);
    rightLimit  = prefs.getInt("rLim10", 0);
    trimOffset  = prefs.getInt("offset10", 0);
    if (prefs.isKey("bdiv"))  battDivider = prefs.getFloat("bdiv", BATT_DIVIDER_NOMINAL);
    if (prefs.isKey("csmv"))  csMvPerA    = prefs.getFloat("csmv", CS_MV_PER_A_NOMINAL);
    if (prefs.isKey("csoff")) csZeroMv    = prefs.getInt("csoff", 0);
    // Since the manual calibration, the center is set independently.
    // If the key is missing (calibration from earlier), the computed center
    // between the end stops applies as before.
    centerLimit = prefs.getInt("cLim10", (leftLimit + rightLimit) / 2);
    softwareCenterPos = centerLimit + trimOffset;
    loadPidParams();

    // --- Motor task on Core 0 ---
    // Queue and pins must be set up before the task.
    g_moveResultQueue = xQueueCreate(8, sizeof(MoveResult));
    setMotorCommand(MOTOR_COAST, false, 0);
    xTaskCreatePinnedToCore(
        motorControlTask,
        "Motor_Task",
        4096,
        (void*)&planetaryMotor,
        1,
        &MotorControlTaskHandle,
        0                        // Core 0
    );

    // --- RGBW LED task, also on Core 0 ---
    // Core 1 carries the live control and may block; there an animation would
    // stutter. Same priority as the motor task: the LED task blocks almost
    // the whole time (vTaskDelayUntil, RMT transmission), so the two never
    // compete for more than a few microseconds.
    loadPixelDefault();
    xTaskCreatePinnedToCore(
        pixelTask,
        "Pixel_Task",
        4096,
        nullptr,
        1,
        &PixelTaskHandle,
        0                        // Core 0
    );

    // Enable torque explicitly. Without this the SC09 accepts position commands
    // but does not hold them - the steering only responds after
    // 'cal' or 'tq1' has set the torque.
    sc09Servo.EnableTorque(SERVO_ID, 1);

    // Hold the current servo position to avoid startup strain
    int startPos = sc09Servo.ReadPos(SERVO_ID);
    if (startPos != -1) {
        sc09Servo.WritePosEx(SERVO_ID, startPos, SERVO_SPEED_FAST, SERVO_ACC);
    }

    // First battery measurement immediately, then every 15 s
    battPackV = readBatteryVolts();
    battCellV = battPackV / BATT_CELLS;
    battLastRead = millis();

    // Print the pin assignment at startup - when bringing up a new board, the
    // first check whether the firmware is driving the right pins at all.
    Serial.printf("Pins: Motor PWM%d INA%d INB%d CS%d | Enc %d/%d | Servo TX%d RX%d @%lu"
                  " | Jetson RX%d TX%d | Batt %d | LED %d | RGBW %d | Button %d\n",
                  PIN_MOTOR_PWM, PIN_MOTOR_INA, PIN_MOTOR_INB, PIN_MOTOR_CS,
                  DRIVE_INVERT ? PIN_ENC_B : PIN_ENC_A,
                  DRIVE_INVERT ? PIN_ENC_A : PIN_ENC_B, g_servoTx, g_servoRx,
                  (unsigned long)g_servoBaud, PIN_JETSON_RX, PIN_JETSON_TX,
                  PIN_BATTERY, PIN_LED, PIN_PIXEL, PIN_BUTTON);
    if (DRIVE_INVERT) {
        Serial.println("Drive direction inverted (DRIVE_INVERT) - motor and encoder.");
    }
    Serial.printf("System Ready. Steering: center:%d L:%d R:%d | Trim: %d | Battery: %.2f V (%.3f V/cell)\n",
                  centerLimit, leftLimit, rightLimit, trimOffset, battPackV, battCellV);
    if (!prefs.isKey("cLim10")) {
        Serial.println("Note: steering never calibrated manually - run 'cal'.");
    }
    printHelp();
}

void loop() {
    // --- Button ---
    if (buttonTriggered) {
        buttonTriggered = false;
        static unsigned long lastPressTime = 0;
        if (millis() - lastPressTime > 200) {   // 200 ms debounce time
            lastPressTime = millis();
            jetson.sendButtonEvent();
        }
    }

    // --- Report results of completed position moves to the Jetson ---
    MoveResult r;
    while (xQueueReceive(g_moveResultQueue, &r, 0) == pdTRUE) {
        // Auto plotter ends with the move - switch it off first, then report,
        // otherwise the message still ends up in the curve stream.
        if (g_plotAuto) { g_plotMode = false; g_plotAuto = false; }
        jetson.sendMoveDone(r);
        const char* txt = (r.status == MOVE_OK)      ? "DONE"
                        : (r.status == MOVE_TIMEOUT) ? "TIMEOUT"
                                                     : "ABORTED";
        Serial.printf("-> move id=%u %s at %.1f deg\n", r.id, txt, r.finalDeg10 / 10.0f);
    }

    // --- USB debug console ---
    // Input is echoed back. Without an echo there is no way to tell
    // whether the monitor sends nothing at all or whether only the line ending
    // is missing and the command is therefore never executed.
    static uint32_t lastCharMs = 0;
    static bool sawLineEnding = false;   // does the monitor append a \n or \r?

    while (Serial.available()) {
        char ch = Serial.read();
        lastCharMs = millis();
        if (ch == '\n' || ch == '\r') {
            sawLineEnding = true;
            if (!g_plotMode) Serial.println();
            handleDebugCommand(cmdBuf);
            cmdBuf = "";
        } else {
            cmdBuf += ch;
            if (!g_plotMode) Serial.print(ch);
            if (cmdBuf.length() > 40) cmdBuf = "";   // discard on overflow
        }
    }

    // Emergency exit for monitors that send no line ending (e.g. the
    // VS Code Serial Monitor extension with "Line ending: None"): after a
    // pause in transmission, execute the buffer anyway.
    // Only as long as NO line ending has EVER arrived - otherwise, for someone
    // typing character by character into a terminal, every pause to think would
    // trigger half a command. As soon as the monitor has once identified itself
    // as line-based, this path is off for good.
    if (!sawLineEnding && cmdBuf.length() > 0 && millis() - lastCharMs > 400) {
        if (!g_plotMode) Serial.println("   (received without line ending)");
        handleDebugCommand(cmdBuf);
        cmdBuf = "";
    }

    // --- Jetson protocol ---
    jetson.process();

    // --- Battery every 15 s ---
    batteryTick();

    // --- Drive telemetry (position, speed, duty, current) ---
    telemetryTick();

    // --- Plotter output (only when enabled with "p") ---
    plotTick();

    // --- Servo TX test pattern for the oscilloscope (svtx) ---
    // Fills the TX buffer with 0x55 so that a continuous square wave appears on IO_TX.
    // Limited per loop, otherwise write() blocks at low baud rates.
    if (g_servoTxTest) {
        for (int i = 0; i < 8 && Serial2.availableForWrite() > 0; i++) {
            Serial2.write(0x55);
        }
    }
}
