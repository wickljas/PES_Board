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

//Speed Control
const float acceleration = 5.0f;

//Motor Values
const float max_voltage = 12.0f;               // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
const float counts_per_turn = 64.0f; // define counts per turn at gearbox end: counts/turn * gearratio
const float kn = 33.0f / 12.0f;               // define motor constant in RPM/V
const float max_speed_rps = 67.0f/60.0f;

volatile float motor_angle = 0.0f;

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button

void user_button_pressed();
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

    
    const float kn2 = (33.0f / 12.0f) / 60.0f;
               
    motor_M1.enableMotionPlanner(true);
    motor_M1.setVelocityCntrl(2.5f, 0.3f, 0.01f);
    motor_M1.setVelocityCntrlIntegratorLimitsPercent(5.0f);
    motor_M1.setMaxAcceleration(acceleration);
  
    
    
    main_task_timer.start();
    while(true) {
        main_task_timer.reset();
        float appliedVoltage = motor_M1.getVoltage();
        float actualVelocity = motor_M1.getVelocity();

        float expectedVelocity = kn2 * appliedVoltage;
        //printf("expected velocity: %f ", expectedVelocity);
        float loadFactor = (expectedVelocity - actualVelocity) / expectedVelocity;
        printf("Load Factor: %f ", loadFactor);

        if (user_button.read() == 0) {
            board_led.write(1);
            motor_M1.setVelocity(motor_M1.getMaxVelocity());
           
        } else {
            board_led.write(0);
            motor_M1.setVelocity(0.000000f);
        }
        printf("Motor velocity: %f ", motor_M1.getVelocity());
        //printf("Motor Voltage: %f ", motor_M1.getVoltage());
        printf("Motor Rotation: %ld \n", motor_M1.getEncoderCount());

        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }

}

