#include <mbed.h>

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"
#include "pm2_drivers/IMU.h"
#include "pm2_drivers/Servo.h"
#include "pm2_drivers/FastPWM/FastPWM.h"
#include "pm2_drivers/DCMotor.h"
#include <complex>

#define BUTTONPIN PC_11

//Speed Control
const float acceleration = 5.0f;

//Motor Values
const float max_voltage = 12.0f;               // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
const float counts_per_turn = 64.0f; // define counts per turn at gearbox end: counts/turn * gearratio
const float kn = 33.0f / 12.0f;               // define motor constant in RPM/V
const float max_speed_rps = 67.0f/60.0f;
const float calibration_speed = 0.2f; // Speed for calibration
const float load_factor_threshold = 0.1f;

volatile float motor_angle = 0.0f;

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once


enum MotorState { Idle, Moving, Calibration };
const char* motorStateNames[] = { "Idle", "Moving", "Calibration" };
MotorState motor_state = Idle;

InterruptIn mech_button(BUTTONPIN);
const auto debounce_delay = 50ms; // Adjust debounce delay (50 ms is typical)
Timer debounce_timer;
Timer button_timer;
Timer calibration_timer;  // Timer for Calibration delay
volatile bool button_pressed = false;

// objects for user button (blue button) handling on nucleo board
//DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button

void handleButtonEvent();
bool detectResistance();
// Variables and function calls
DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, 150.0f, kn, max_voltage, counts_per_turn);

// main runs as an own thread
int main()
{
    printf("Setup\n");
    const int main_task_period_ms = 20;
    Timer main_task_timer;

    DigitalOut board_led(LED1);
    board_led = 0;

    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    enable_motors = 1;

    // Add Mechanical Button Interrupt
    mech_button.mode(PullUp);
    mech_button.fall(&handleButtonEvent); // Button press
    mech_button.rise(&handleButtonEvent); // Button release

    
    const float kn2 = (33.0f / 12.0f) / 60.0f;
               
    motor_M1.enableMotionPlanner(true);
    motor_M1.setVelocityCntrl(2.5f, 0.3f, 0.01f);
    motor_M1.setVelocityCntrlIntegratorLimitsPercent(5.0f);
    motor_M1.setMaxAcceleration(acceleration);
  
    button_timer.start();
    debounce_timer.start();

    printf("Setup Complete");  
    main_task_timer.start();

    while(true) {
        //printf("Current State: %s\n", motorStateNames[motor_state]);
        switch (motor_state) {
            case Idle:
                //printf("Idle\n");
                motor_M1.setVelocity(0.0f);  // Stop the motor
                board_led = 0;               // LED off in Idle
                break;

            case Moving:
                //printf("Moving\n");
                motor_M1.setVelocity(1.0f);  // Move with specified velocity

                board_led = 1;               // LED on to indicate Moving mode
                break;      

            case Calibration:
                //printf("Calibrating\n");
                motor_M1.setVelocity(calibration_speed);  // Move slowly
                board_led = !board_led;                   // Blink LED to indicate Calibration mode

                if (!calibration_timer.elapsed_time().count()) {
                    calibration_timer.start();
                }               


                // Check for resistance and stop if detected
                // Wait for 0.5 seconds before starting to detect resistance
                if (calibration_timer.elapsed_time() >= 500ms) {
                    // Check for resistance and stop if detected
                    if (detectResistance()) {
                        motor_state = Idle;  // Return to Idle after calibration
                        calibration_timer.stop();  // Reset the timer
                        calibration_timer.reset();
                        printf("Calibration stopped due to detected resistance\n");
                    }
                }
                break;                    
        }

        detectResistance();
        //printf("Velocity: %f, Motor Pos: %f, Voltage: %f\n", motor_M1.getVelocity(), motor_M1.getRotation(), motor_M1.getVoltage());
        

        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms);
    }

}

// Single handler function
void handleButtonEvent() {
    // Debouncing
    if (debounce_timer.elapsed_time() < debounce_delay) {
        return; // Ignore events within the debounce delay
    }
    debounce_timer.reset();

    if (mech_button.read() == 0) { // Button pressed (low)
        button_pressed = true;
        button_timer.reset();
        button_timer.start();
    } else { // Button released (high)
        if (!button_pressed) return; // Ignore if not pressed before release

        auto press_duration = button_timer.elapsed_time();
        button_timer.stop();
        if (press_duration >= 2s) {
            motor_state = Calibration; // Long press: Calibration mode
            //printf("Long Press: Entering Calibration mode\n");
        } else {
            motor_state = (motor_state == Moving) ? Idle : Moving; // Short press: Toggle Moving/Idle
            //printf("Short Press: Toggled to %s\n", motorStateNames[motor_state]);
        }
        button_pressed = false; // Reset button state
    }
}
bool detectResistance()
{
    // Calculate expected velocity for no-load condition
    float appliedVoltage = motor_M1.getVoltage();
    float expectedVelocity = kn * appliedVoltage / 60.0f;

    // Check if expectedVelocity is too low to calculate load factor
    if (expectedVelocity < 0.01f) {
        return false;  // Avoid load factor calculation for very low expected velocities
    }

    // Calculate the current load factor
    float actualVelocity = motor_M1.getVelocity();
    float loadFactor = (expectedVelocity - actualVelocity) / expectedVelocity;

    // Debug output to monitor load factor
    printf("Expected Velocity: %f, Actual Velocity: %f, Load Factor: %f\n", expectedVelocity, actualVelocity, loadFactor);

    // Return true if load factor exceeds the threshold, indicating resistance
    return (loadFactor > load_factor_threshold);
}
