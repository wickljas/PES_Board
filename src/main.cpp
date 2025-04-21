#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DCMotor.h"
#include "DebounceIn.h"

// —— CONFIGURATION —————————————————————————————————————————————————
static constexpr int   MAIN_PERIOD_MS           = 20;
static constexpr float VOLTAGE_MAX              = 12.0f;
static constexpr float GEAR_RATIO               = 156.0f;
static constexpr float KN_RPM_PER_V             = 89.0f/12.0f;
static constexpr float MAX_VEL                  = 0.5f;
static constexpr float WHEEL_CIRCUMFERENCE_MM   = 210.0f;   // adjust to your wheel
static constexpr float ROTATION_ERROR_TOL_DEG   = 0.5f;     // acceptable tolerance

// —— HARDWARE PINS —————————————————————————————————————————————————
DigitalOut    user_led(LED1);
DebounceIn    button(BUTTON1);

// UART from ESP32 on USART1: TX=PA_9, RX=PA_10
static BufferedSerial esp_serial(PA_9, PA_10, 115200);

// Two DC Motors: M1 = drive, M2 = rotation
DCMotor       motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                       GEAR_RATIO, KN_RPM_PER_V, VOLTAGE_MAX);
DCMotor       motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2,
                       GEAR_RATIO, KN_RPM_PER_V, VOLTAGE_MAX);
DigitalOut    enable_motors(PB_ENABLE_DCMOTORS);

// Conversion helpers
inline float deg2rot(float deg) { return deg / 360.0f; }
inline float rot2deg(float rot) { return rot * 360.0f; }

int main() {
    // Console via USB-STLink
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("[Nucleo] USART1 PA_9/PA_10 at 115200\n");

    // Enable motion planner for both motors
    enable_motors = 1;
    motor_M1.enableMotionPlanner();
    motor_M2.enableMotionPlanner();
    motor_M1.setMaxVelocity(MAX_VEL);
    motor_M2.setMaxVelocity(MAX_VEL);

    // Track M1 initial rotation for relative moves
    float m1_start_rot = motor_M1.getRotation();

    // Receive buffer
    static constexpr size_t BUF_SIZE = 256;
    char buffer[BUF_SIZE];
    size_t idx = 0;

    while (true) {
        if (esp_serial.readable()) {
            char c;
            if (esp_serial.read(&c, 1) == 1) {
                user_led = !user_led;  // blink on each byte
                buffer[idx++] = c;
                if (c == '\n' || idx >= BUF_SIZE-1) {
                    buffer[idx] = '\0';
                    printf("[Nucleo] CMD recv: %s", buffer);

                    // Parse JSON fields
                    bool isRotate  = strstr(buffer, "\"ROTATE\"")  != nullptr;
                    bool isForward = strstr(buffer, "\"FORWARD\"") != nullptr;
                    int value = 0;
                    if (char* p = strstr(buffer, "\"value\"")) {
                        value = atoi(p + 8);
                    }
                    printf("[Nucleo] Parsed: rotate=%d, forward=%d, value=%d\n",
                           isRotate, isForward, value);

                    // Execute rotate
                    if (isRotate) {
                        float target_rot = deg2rot((float)value);
                        motor_M2.setRotation(target_rot);
                        // Wait until within tolerance
                        while (fabsf(motor_M2.getRotation() - target_rot) > ROTATION_ERROR_TOL_DEG / 360.0f) {
                            thread_sleep_for(1);
                        }
                        int ackDeg = (int)rot2deg(motor_M2.getRotation());
                        char out[64];
                        int n = snprintf(out, sizeof(out), "{\"ack\":\"OK\",\"pos\":%d}\n", ackDeg);
                        esp_serial.write(out, n);
                        printf("[Nucleo] ACK sent: %s", out);
                    }
                    // Execute forward
                    else if (isForward) {
                        // Compute degrees for wheel rotation
                        float wheel_rotations = (float)value / WHEEL_CIRCUMFERENCE_MM;
                        float wheel_degrees   = rot2deg(wheel_rotations);
                        motor_M1.setRotation(m1_start_rot + deg2rot(wheel_degrees));
                        // Wait until within tolerance
                        while (fabsf(motor_M1.getRotation() - (m1_start_rot + deg2rot(wheel_degrees))) > ROTATION_ERROR_TOL_DEG / 360.0f) {
                            thread_sleep_for(1);
                        }
                        m1_start_rot = motor_M1.getRotation();  // update base
                        char out[64];
                        int n = snprintf(out, sizeof(out), "{\"ack\":\"OK\",\"pos\":%d}\n", value);
                        esp_serial.write(out, n);
                        printf("[Nucleo] ACK sent: %s", out);
                    }
                    idx = 0;
                }
            }
        }
        thread_sleep_for(MAIN_PERIOD_MS);
    }
}
