// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * ============================================================
 * CALIBRATION - ESP32 + 4 MECANUM + 2 L298N + 4 ENCODERS
 * ============================================================
 *
 * Board: ESP32 NodeMCU-32S
 * Serial: 115200 baud
 * IMU: not used
 *
 * Motor order:
 *
 *                   FRONT
 *          M1                    M2
 *      FRONT LEFT           FRONT RIGHT
 *
 *          M3                    M4
 *       REAR LEFT            REAR RIGHT
 *                    BACK
 *
 * Wiring used in this program
 * ------------------------------------------------------------
 * Encoder M1: A=GPIO34, B=GPIO35
 * Encoder M2: A=GPIO32, B=GPIO33
 * Encoder M3: A=GPIO25, B=GPIO26
 * Encoder M4: A=GPIO27, B=GPIO14
 *
 * Motor M1: PWM=GPIO15, INA=GPIO23, INB=GPIO22
 * Motor M2: PWM=GPIO2,  INA=GPIO21, INB=GPIO19
 * Motor M3: PWM=GPIO0,  INA=GPIO18, INB=GPIO5
 * Motor M4: PWM=GPIO4,  INA=GPIO17, INB=GPIO16
 *
 * Robot parameters
 * ------------------------------------------------------------
 * Motor rated speed: 282 RPM at 12 V
 * Measured supply voltage: 11.4 V
 * Wheel diameter: 0.08 m
 * Initial maximum RPM ratio for the final firmware: 0.30
 *
 * Serial commands
 * ------------------------------------------------------------
 * help       : show commands
 * spin       : run M1 -> M4 for 2 seconds at 35% PWM
 * sample     : run M1 -> M4 for 10 seconds at 100% PWM and
 *              estimate COUNTS_PER_REV
 * cpr        : reset all encoder counts for a manual test
 * readcpr    : print manual CPR results after 1 wheel turn
 * cpr1..cpr4 : reset only one encoder for a manual test
 * readcpr1..readcpr4 : print CPR for one wheel
 * status     : print current raw encoder counts
 * inv1..inv4 : toggle motor direction in RAM for testing
 * stop       : stop all motors
 *
 * IMPORTANT
 * ------------------------------------------------------------
 * 1. Raise the robot so all four wheels are off the ground.
 * 2. Remove the ENA/ENB jumpers on L298N when PWM is connected
 *    to the ESP32.
 * 3. ESP32, both L298N boards, encoders and battery must share GND.
 * 4. GPIO34 and GPIO35 have no internal pull-up resistors.
 *    If the encoder outputs are open-collector, add external
 *    pull-ups to 3.3 V.
 * 5. ESP32 GPIO pins are not 5 V tolerant. Encoder signals entering
 *    the ESP32 must be 3.3 V compatible.
 */

#include <Arduino.h>
#include "driver/gpio.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

/* ============================================================
 * USER PARAMETERS
 * ============================================================ */

static constexpr float MOTOR_MAX_RPM = 282.0f;
static constexpr float MOTOR_OPERATING_VOLTAGE = 12.0f;
static constexpr float MOTOR_POWER_MEASURED_VOLTAGE = 11.4f;
static constexpr float MAX_RPM_RATIO = 0.30f;

static constexpr float WHEEL_DIAMETER_M = 0.068f;
// Not required for CPR calibration. Replace 0.30 later with the
// measured left-to-right wheel-center distance in esp32_config.h.
static constexpr float LR_WHEELS_DISTANCE_M = 0.30f;

static constexpr uint8_t MOTOR_COUNT = 4;
static constexpr uint8_t PWM_BITS = 10;
static constexpr uint32_t PWM_FREQUENCY = 20000;
static constexpr uint16_t PWM_MAX_VALUE = (1U << PWM_BITS) - 1U;

static constexpr uint8_t SPIN_PWM_PERCENT = 70;
static constexpr float ALL_PWM_PERCENT[MOTOR_COUNT] = {
    70.00f,
    69.47f,
    69.77f,
    69.90f
};
static constexpr uint8_t SAMPLE_PWM_PERCENT = 100;

static constexpr uint8_t SPIN_TIME_SECONDS = 2;
static constexpr uint8_t ALL_TIME_SECONDS = 5;
static constexpr uint8_t SAMPLE_TIME_SECONDS = 10;
static constexpr uint8_t CPR_TEST_TURNS = 1;

static constexpr unsigned long SPIN_TIME_MS =
    static_cast<unsigned long>(SPIN_TIME_SECONDS) * 1000UL;

static constexpr unsigned long ALL_TIME_MS =
    static_cast<unsigned long>(ALL_TIME_SECONDS) * 1000UL;

static constexpr unsigned long SAMPLE_TIME_MS =
    static_cast<unsigned long>(SAMPLE_TIME_SECONDS) * 1000UL;

/* ============================================================
 * PIN CONFIGURATION
 * ============================================================ */

DRAM_ATTR static const uint8_t ENCODER_A_PINS[MOTOR_COUNT] = {
    34, 32, 36, 27
};

DRAM_ATTR static const uint8_t ENCODER_B_PINS[MOTOR_COUNT] = {
    35, 33, 39, 14
};

static const uint8_t MOTOR_PWM_PINS[MOTOR_COUNT] = {
    15, 2, 0, 4
};

static const uint8_t MOTOR_IN_A_PINS[MOTOR_COUNT] = {
    23, 21, 18, 17
};

static const uint8_t MOTOR_IN_B_PINS[MOTOR_COUNT] = {
    22, 19, 5, 16
};

#if ESP_ARDUINO_VERSION_MAJOR < 3
static const uint8_t PWM_CHANNELS[MOTOR_COUNT] = {
    0, 1, 2, 3
};
#endif

static const char *MOTOR_LABELS[MOTOR_COUNT] = {
    "FRONT LEFT  - M1",
    "FRONT RIGHT - M2",
    "REAR LEFT   - M3",
    "REAR RIGHT  - M4"
};

/*
 * Keep all directions false for the first test.
 * Use inv1, inv2, inv3 or inv4 in Serial Monitor to toggle a
 * motor direction without recompiling.
 *
 * After finding the correct directions, copy them to:
 * MOTOR1_INV, MOTOR2_INV, MOTOR3_INV, MOTOR4_INV
 * in esp32_config.h.
 */
static bool motor_inverted[MOTOR_COUNT] = {
    false, true, false, false
};

/* ============================================================
 * QUADRATURE ENCODER
 * ============================================================ */

static volatile int32_t encoder_counts[MOTOR_COUNT] = {
    0, 0, 0, 0
};

static volatile uint8_t encoder_states[MOTOR_COUNT] = {
    0, 0, 0, 0
};

static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

/*
 * Quadrature transition table.
 * Counts all valid A/B edges (x4 decoding).
 */
DRAM_ATTR static const int8_t QUADRATURE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static inline uint8_t IRAM_ATTR readEncoderState(uint8_t motor_index)
{
    const uint8_t a =
        static_cast<uint8_t>(
            gpio_get_level(
                static_cast<gpio_num_t>(ENCODER_A_PINS[motor_index])
            )
        );

    const uint8_t b =
        static_cast<uint8_t>(
            gpio_get_level(
                static_cast<gpio_num_t>(ENCODER_B_PINS[motor_index])
            )
        );

    return static_cast<uint8_t>((a << 1U) | b);
}

static inline void IRAM_ATTR updateEncoder(uint8_t motor_index)
{
    const uint8_t new_state = readEncoderState(motor_index);

    portENTER_CRITICAL_ISR(&encoder_mux);

    const uint8_t transition =
        static_cast<uint8_t>(
            (encoder_states[motor_index] << 2U) | new_state
        );

    encoder_counts[motor_index] += QUADRATURE_TABLE[transition];
    encoder_states[motor_index] = new_state;

    portEXIT_CRITICAL_ISR(&encoder_mux);
}

void IRAM_ATTR encoder1ISR()
{
    updateEncoder(0);
}

void IRAM_ATTR encoder2ISR()
{
    updateEncoder(1);
}

void IRAM_ATTR encoder3ISR()
{
    updateEncoder(2);
}

void IRAM_ATTR encoder4ISR()
{
    updateEncoder(3);
}

static int32_t readEncoderCount(uint8_t motor_index)
{
    int32_t value;

    portENTER_CRITICAL(&encoder_mux);
    value = encoder_counts[motor_index];
    portEXIT_CRITICAL(&encoder_mux);

    return value;
}

static void resetEncoderCount(uint8_t motor_index)
{
    const uint8_t current_state = readEncoderState(motor_index);

    portENTER_CRITICAL(&encoder_mux);
    encoder_counts[motor_index] = 0;
    encoder_states[motor_index] = current_state;
    portEXIT_CRITICAL(&encoder_mux);
}

static void resetAllEncoderCounts()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        resetEncoderCount(i);
    }
}

static int32_t absoluteCount(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* ============================================================
 * PWM COMPATIBILITY FOR ESP32 ARDUINO CORE 2.x AND 3.x
 * ============================================================ */

static bool initializePwmPin(uint8_t motor_index)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    return ledcAttach(
        MOTOR_PWM_PINS[motor_index],
        PWM_FREQUENCY,
        PWM_BITS
    );
#else
    const double configured_frequency = ledcSetup(
        PWM_CHANNELS[motor_index],
        PWM_FREQUENCY,
        PWM_BITS
    );

    ledcAttachPin(
        MOTOR_PWM_PINS[motor_index],
        PWM_CHANNELS[motor_index]
    );

    return configured_frequency > 0.0;
#endif
}

static void writePwm(uint8_t motor_index, uint16_t duty)
{
    if (duty > PWM_MAX_VALUE)
    {
        duty = PWM_MAX_VALUE;
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(MOTOR_PWM_PINS[motor_index], duty);
#else
    ledcWrite(PWM_CHANNELS[motor_index], duty);
#endif
}

static uint16_t percentToDuty(float percent)
{
    if (percent < 0.0f)
    {
        percent = 0.0f;
    }

    if (percent > 100.0f)
    {
        percent = 100.0f;
    }

    return static_cast<uint16_t>(
        (static_cast<float>(PWM_MAX_VALUE) * percent / 100.0f) + 0.5f
    );
}
/* ============================================================
 * MOTOR CONTROL
 * ============================================================ */

static void stopMotor(uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT)
    {
        return;
    }

    writePwm(motor_index, 0);
    digitalWrite(MOTOR_IN_A_PINS[motor_index], LOW);
    digitalWrite(MOTOR_IN_B_PINS[motor_index], LOW);
}

static void stopAllMotors()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        stopMotor(i);
    }
}

static void runMotorForward(uint8_t motor_index, float pwm_percent)
{
    if (motor_index >= MOTOR_COUNT)
    {
        return;
    }

    writePwm(motor_index, 0);

    if (!motor_inverted[motor_index])
    {
        digitalWrite(MOTOR_IN_A_PINS[motor_index], HIGH);
        digitalWrite(MOTOR_IN_B_PINS[motor_index], LOW);
    }
    else
    {
        digitalWrite(MOTOR_IN_A_PINS[motor_index], LOW);
        digitalWrite(MOTOR_IN_B_PINS[motor_index], HIGH);
    }

    delay(2);
    writePwm(motor_index, percentToDuty(pwm_percent));
}

static void initializeMotors()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        pinMode(MOTOR_IN_A_PINS[i], OUTPUT);
        pinMode(MOTOR_IN_B_PINS[i], OUTPUT);

        if (!initializePwmPin(i))
        {
            Serial.print("WARNING: PWM setup failed for ");
            Serial.println(MOTOR_LABELS[i]);
        }
    }

    stopAllMotors();
}

/* ============================================================
 * ENCODER INITIALIZATION
 * ============================================================ */

static void initializeEncoders()
{
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        /*
         * INPUT is intentionally used.
         * GPIO34 and GPIO35 do not support internal pull-ups.
         */
        pinMode(ENCODER_A_PINS[i], INPUT);
        pinMode(ENCODER_B_PINS[i], INPUT);

        encoder_states[i] = readEncoderState(i);
    }

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_PINS[0]),
        encoder1ISR,
        CHANGE
    );
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_B_PINS[0]),
        encoder1ISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_PINS[1]),
        encoder2ISR,
        CHANGE
    );
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_B_PINS[1]),
        encoder2ISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_PINS[2]),
        encoder3ISR,
        CHANGE
    );
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_B_PINS[2]),
        encoder3ISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_PINS[3]),
        encoder4ISR,
        CHANGE
    );
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_B_PINS[3]),
        encoder4ISR,
        CHANGE
    );

    resetAllEncoderCounts();
}

/* ============================================================
 * PRINT FUNCTIONS
 * ============================================================ */

static void printDivider()
{
    Serial.println(
        "============================================================"
    );
}

static void printCurrentDirections()
{
    Serial.println("Motor inversion currently used:");

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print("_INV = ");
        Serial.println(motor_inverted[i] ? "true" : "false");
    }
}

static void printEncoderStatus()
{
    stopAllMotors();

    Serial.println();
    Serial.println("================ ENCODER STATUS ========================");

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        Serial.print(MOTOR_LABELS[i]);
        Serial.print(": ");
        Serial.println(readEncoderCount(i));
    }

    printDivider();
    Serial.println();
}

static void printHelp()
{
    Serial.println();
    Serial.println("================ COMMANDS ==============================");
    Serial.println("help       : show commands");
    Serial.println("spin       : test M1 -> M4 at 70% PWM for 2 seconds");
    Serial.println("all        : run all motors with individual compensated PWM");    
    Serial.println("sample     : estimate CPR at 100% PWM for 10 seconds");
    Serial.println("cpr        : reset all encoders for manual CPR test");
    Serial.println("readcpr    : show CPR for all wheels after 10 turns");
    Serial.println("cpr1..cpr4 : reset one encoder");
    Serial.println("readcpr1..readcpr4 : show CPR for one encoder");
    Serial.println("status     : show current raw encoder counts");
    Serial.println("inv1..inv4 : toggle one motor direction");
    Serial.println("stop       : stop all motors");
    printDivider();
    Serial.println();
}

/* ============================================================
 * MANUAL CPR
 * ============================================================ */

static void resetManualCprAll()
{
    stopAllMotors();
    resetAllEncoderCounts();

    Serial.println();
    Serial.println("================ MANUAL CPR TEST =======================");
    Serial.println("All four encoder counts were reset.");
    Serial.print("Rotate ONE wheel exactly ");
    Serial.print(CPR_TEST_TURNS);
    Serial.println(" turns while the other wheels remain still.");
    Serial.println("Then enter readcpr.");
    Serial.println("Repeat cpr before measuring the next wheel.");
    printDivider();
    Serial.println();
}

static void resetManualCprOne(uint8_t motor_index)
{
    stopAllMotors();
    resetEncoderCount(motor_index);

    Serial.println();
    Serial.print("Reset ");
    Serial.print(MOTOR_LABELS[motor_index]);
    Serial.println(" encoder to zero.");
    Serial.print("Rotate that wheel exactly ");
    Serial.print(CPR_TEST_TURNS);
    Serial.println(" turns, then enter readcpr1..readcpr4.");
    Serial.println();
}

static void printManualCprOne(uint8_t motor_index)
{
    stopAllMotors();

    const int32_t count = readEncoderCount(motor_index);
    const float cpr =
        absoluteCount(count) / static_cast<float>(CPR_TEST_TURNS);

    Serial.println();
    Serial.print(MOTOR_LABELS[motor_index]);
    Serial.print(": raw count = ");
    Serial.print(count);
    Serial.print(" | CPR x4 = ");
    Serial.println(cpr, 2);

    if (count < 0)
    {
        Serial.print("Suggestion: MOTOR");
        Serial.print(motor_index + 1);
        Serial.println("_ENCODER_INV = true");
    }
    else if (count == 0)
    {
        Serial.println(
            "No encoder pulses detected. Check encoder wiring and power."
        );
    }

    Serial.println();
}

static void printManualCprAll()
{
    stopAllMotors();

    Serial.println();
    Serial.println("================ MANUAL CPR RESULT =====================");

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const int32_t count = readEncoderCount(i);
        const float cpr =
            absoluteCount(count) / static_cast<float>(CPR_TEST_TURNS);

        Serial.print(MOTOR_LABELS[i]);
        Serial.print(": raw count = ");
        Serial.print(count);
        Serial.print(" | CPR x4 = ");
        Serial.println(cpr, 2);
    }

    Serial.println();
    Serial.println("Copy only the result of the wheel you rotated.");
    Serial.println(
        "A negative raw count means MOTORx_ENCODER_INV should be true."
    );
    printDivider();
    Serial.println();
}

/* ============================================================
 * TIMED MOTOR TEST
 * ============================================================ */

static int32_t runTimedMotor(
    uint8_t motor_index,
    uint8_t pwm_percent,
    unsigned long duration_ms
)
{
    stopAllMotors();
    delay(1000);

    resetEncoderCount(motor_index);

    Serial.print("SPINNING ");
    Serial.print(MOTOR_LABELS[motor_index]);
    Serial.print(" at ");
    Serial.print(pwm_percent);
    Serial.print("% PWM: ");

    runMotorForward(motor_index, pwm_percent);

    const unsigned long start_time = millis();
    unsigned long last_dot_time = start_time;

    while ((millis() - start_time) < duration_ms)
    {
        if ((millis() - last_dot_time) >= 1000UL)
        {
            last_dot_time = millis();
            Serial.print(".");
        }

        delay(1);
    }

    stopMotor(motor_index);
    delay(300);

    const int32_t count = readEncoderCount(motor_index);

    Serial.print(" count = ");
    Serial.println(count);

    return count;
}

static void spinMotors()
{
    Serial.println();
    Serial.println("================ MOTOR DIRECTION TEST ==================");
    Serial.println("Expected order: M1 front-left, M2 front-right,");
    Serial.println("M3 rear-left, M4 rear-right.");
    Serial.println("Each wheel must rotate in the robot-forward direction.");
    Serial.println();

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        runTimedMotor(
            i,
            SPIN_PWM_PERCENT,
            SPIN_TIME_MS
        );
    }

    stopAllMotors();

    Serial.println();
    Serial.println("Use inv1, inv2, inv3 or inv4 for a reversed motor.");
    printCurrentDirections();
    printDivider();
    Serial.println();
}
static void runAllMotorsTogether()
{
    stopAllMotors();
    delay(1000);

    resetAllEncoderCounts();

    Serial.println();
    Serial.println("================ ALL 4 MOTORS TEST =====================");
    Serial.println("Running all four motors with individual PWM:");

for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
{
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print(" = ");
    Serial.print(ALL_PWM_PERCENT[i]);
    Serial.println("%");
}

Serial.print("Running time: ");
Serial.print(ALL_TIME_SECONDS);
Serial.println(" seconds.");
    Serial.println("Use inv1..inv4 first if any wheel rotates backward.");
    Serial.println();

    /*
     * Cho cả bốn motor chạy gần như đồng thời.
     * Mỗi lần gọi hàm chỉ lệch nhau vài mili giây.
     */
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        runMotorForward(i, ALL_PWM_PERCENT[i]);
    }

    const unsigned long start_time = millis();
    unsigned long last_print_time = start_time;
    uint8_t elapsed_seconds = 0;

    while ((millis() - start_time) < ALL_TIME_MS)
    {
        if ((millis() - last_print_time) >= 1000UL)
        {
            last_print_time = millis();
            ++elapsed_seconds;

            Serial.print("t=");
            Serial.print(elapsed_seconds);
            Serial.print("s");

            for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
            {
                Serial.print(" | M");
                Serial.print(i + 1);
                Serial.print("=");
                Serial.print(readEncoderCount(i));
            }

            Serial.println();
        }

        delay(1);
    }

    stopAllMotors();
    delay(300);

    Serial.println();
    Serial.println("Final encoder counts after the simultaneous run:");

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        Serial.print(MOTOR_LABELS[i]);
        Serial.print(": ");
        Serial.println(readEncoderCount(i));
    }

    Serial.println();
    Serial.println("Equal PWM does not guarantee equal RPM.");
    Serial.println(
        "True speed synchronization requires all 4 encoders and closed-loop PID."
    );

    printDivider();
    Serial.println();
}

/* ============================================================
 * AUTOMATIC CPR ESTIMATE
 * ============================================================ */

static void sampleMotors()
{
    const float voltage_ratio =
        MOTOR_POWER_MEASURED_VOLTAGE / MOTOR_OPERATING_VOLTAGE;

    const float estimated_rpm =
        MOTOR_MAX_RPM * voltage_ratio;

    const float estimated_revolutions =
        estimated_rpm *
        (static_cast<float>(SAMPLE_TIME_SECONDS) / 60.0f);

    int32_t raw_counts[MOTOR_COUNT] = {0, 0, 0, 0};
    float estimated_cpr[MOTOR_COUNT] = {0, 0, 0, 0};

    Serial.println();
    Serial.println("================ AUTO CPR ESTIMATE =====================");
    Serial.print("Rated motor RPM: ");
    Serial.println(MOTOR_MAX_RPM, 2);
    Serial.print("Rated voltage: ");
    Serial.print(MOTOR_OPERATING_VOLTAGE, 2);
    Serial.println(" V");
    Serial.print("Configured measured voltage: ");
    Serial.print(MOTOR_POWER_MEASURED_VOLTAGE, 2);
    Serial.println(" V");
    Serial.print("Estimated RPM: ");
    Serial.println(estimated_rpm, 2);
    Serial.print("Estimated turns in ");
    Serial.print(SAMPLE_TIME_SECONDS);
    Serial.print(" seconds: ");
    Serial.println(estimated_revolutions, 2);
    Serial.println();

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        raw_counts[i] = runTimedMotor(
            i,
            SAMPLE_PWM_PERCENT,
            SAMPLE_TIME_MS
        );

        if (estimated_revolutions > 0.0f)
        {
            estimated_cpr[i] =
                absoluteCount(raw_counts[i]) / estimated_revolutions;
        }
    }

    stopAllMotors();

    Serial.println();
    Serial.println("================ RESULTS ===============================");

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        Serial.print(MOTOR_LABELS[i]);
        Serial.print(": raw = ");
        Serial.print(raw_counts[i]);
        Serial.print(" | estimated CPR x4 = ");
        Serial.println(estimated_cpr[i], 2);
    }

    Serial.println();
    Serial.println("Copy format for esp32_config.h:");

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        Serial.print("#define COUNTS_PER_REV");
        Serial.print(i + 1);
        Serial.print(" ");
        Serial.println(
            static_cast<long>(
                estimated_cpr[i] + 0.5f
            )
        );
    }

    Serial.println();

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (raw_counts[i] < 0)
        {
            Serial.print("#define MOTOR");
            Serial.print(i + 1);
            Serial.println("_ENCODER_INV true");
        }
        else
        {
            Serial.print("#define MOTOR");
            Serial.print(i + 1);
            Serial.println("_ENCODER_INV false");
        }
    }

    Serial.println();
    Serial.println(
        "WARNING: L298N voltage drop and real motor speed make this"
    );
    Serial.println(
        "an estimate. The manual 10-turn CPR test is more reliable."
    );

    const float allowed_rpm = MOTOR_MAX_RPM * MAX_RPM_RATIO;
    const float linear_speed =
        (allowed_rpm / 60.0f) *
        PI *
        WHEEL_DIAMETER_M;

    Serial.println();
    Serial.print("Final firmware MAX_RPM_RATIO: ");
    Serial.println(MAX_RPM_RATIO, 2);
    Serial.print("Allowed wheel RPM: ");
    Serial.println(allowed_rpm, 2);
    Serial.print("Approx. allowed linear speed: ");
    Serial.print(linear_speed, 3);
    Serial.println(" m/s");
    Serial.print("Configured LR wheel distance: ");
    Serial.print(LR_WHEELS_DISTANCE_M, 3);
    Serial.println(" m (measure this before final firmware)");

    printDivider();
    Serial.println();
}

/* ============================================================
 * COMMAND HANDLER
 * ============================================================ */

static int commandMotorIndex(
    const String &command,
    const String &prefix
)
{
    if (!command.startsWith(prefix))
    {
        return -1;
    }

    const String suffix = command.substring(prefix.length());

    if (suffix.length() != 1)
    {
        return -1;
    }

    const int motor_number = suffix.toInt();

    if (motor_number < 1 || motor_number > MOTOR_COUNT)
    {
        return -1;
    }

    return motor_number - 1;
}

static void processCommand(String command)
{
    command.trim();
    command.toLowerCase();

    if (command.length() == 0)
    {
        return;
    }

    if (command == "help")
    {
        printHelp();
        return;
    }

    if (command == "spin")
    {
        spinMotors();
        return;
    }
    if (command == "all")
{
    runAllMotorsTogether();
    return;
}

    if (command == "sample")
    {
        sampleMotors();
        return;
    }

    if (command == "cpr")
    {
        resetManualCprAll();
        return;
    }

    if (command == "readcpr")
    {
        printManualCprAll();
        return;
    }

    if (command == "status")
    {
        printEncoderStatus();
        return;
    }

    if (command == "stop")
    {
        stopAllMotors();
        Serial.println("All motors stopped.");
        return;
    }

    int motor_index = commandMotorIndex(command, "cpr");

    if (motor_index >= 0)
    {
        resetManualCprOne(static_cast<uint8_t>(motor_index));
        return;
    }

    motor_index = commandMotorIndex(command, "readcpr");

    if (motor_index >= 0)
    {
        printManualCprOne(static_cast<uint8_t>(motor_index));
        return;
    }

    motor_index = commandMotorIndex(command, "inv");

    if (motor_index >= 0)
    {
        stopAllMotors();

        motor_inverted[motor_index] =
            !motor_inverted[motor_index];

        Serial.print("M");
        Serial.print(motor_index + 1);
        Serial.print("_INV is now ");
        Serial.println(
            motor_inverted[motor_index] ? "true" : "false"
        );

        return;
    }

    Serial.print("Unknown command: ");
    Serial.println(command);
    Serial.println("Enter help to show commands.");
}

/* ============================================================
 * SETUP AND LOOP
 * ============================================================ */

void setup()
{
    Serial.begin(115200);
    delay(1000);

    initializeMotors();
    initializeEncoders();
    stopAllMotors();

    Serial.println();
    printDivider();
    Serial.println("ESP32 MECANUM CALIBRATION READY");
    Serial.println("4 motors + 2 L298N + 4 quadrature encoders");
    Serial.println("Serial baud: 115200");
    printDivider();

    Serial.println();
    Serial.println("SAFETY:");
    Serial.println("- Raise all four wheels off the ground.");
    Serial.println("- Remove L298N ENA/ENB jumpers.");
    Serial.println("- Connect all grounds together.");
    Serial.println("- sample runs each motor at 100% PWM for 10 seconds.");
    Serial.println();

    printHelp();
}

void loop()
{
    static String command;

    while (Serial.available() > 0)
    {
        const char character =
            static_cast<char>(Serial.read());

        if (character == '\r' || character == '\n')
        {
            if (command.length() > 0)
            {
                Serial.print("> ");
                Serial.println(command);

                processCommand(command);
                command = "";
            }
        }
        else
        {
            /*
             * Prevent an accidentally huge command from consuming RAM.
             */
            if (command.length() < 64)
            {
                command += character;
            }
        }
    }

    delay(1);
}
