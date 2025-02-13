#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "IMU.h"
#include "IRSensor.h"
#include "Servo.h"
#include "UltrasonicSensor.h"

// unused, i just want to see if it compiles
#include "AvgFilter.h"
#include "Chirp.h"
#include <Eigen/Dense>
#include "EncoderCounter.h"
#include "FastPWM.h"
#include "GPA.h"
#include "IIRFilter.h"
#include "LinearCharacteristics3.h"
#include "LineFollower.h"
#include "LSM9DS1.h"
#include "Mahony.h"
#include "MedianFilter3.h"
#include "Motion.h"
#include "PIDCntrl.h"
#include "SDLogger.h"
#include "SDWriter.h"
#include "SensorBar.h"
#include "serial_pipe.h"
#include "SerialStream.h"
#include "Stepper.h"
#include "ThreadFlag.h"

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);     // create DebounceIn to evaluate the user button
                                     // falling and rising edge
void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                     // button gets pressed, definition below

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object, button has a pull-up resistor
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an aditional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via a resistor
    DigitalOut led1(PB_9);

    // ir distance sensor with average filter and implicit calibration
    float ir_distance_avg = 0.0f;
    IRSensor ir_sensor(PC_2);                      // before the calibration the read function will return the averaged mV value
    ir_sensor.setCalibration(2.574e+04f, -29.37f); // after the calibration the read function will return the calibrated value

    // create servo objects to command servos
    Servo servo_D0(PB_D0);
    Servo servo_D1(PB_D1);
    Servo servo_D2(PB_D2);

    // calibration values
    // - Futuba S3001
    float servo_D0_ang_min = 0.0150f;
    float servo_D0_ang_max = 0.1150f;
    // - Modelcraft RS2 MG/BB
    float servo_D1_ang_min = 0.0325f;
    float servo_D1_ang_max = 0.1250f;
    // - REELY S0090
    float servo_D2_ang_min = 0.0325f;
    float servo_D2_ang_max = 0.1175f;

    // setNormalisedPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
    servo_D2.calibratePulseMinMax(servo_D2_ang_min, servo_D2_ang_max);
    // setNormalisedPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)

    // default acceleration for the motion profile would be 1.0e6f
    servo_D0.setMaxAcceleration(3.0f);
    servo_D1.setMaxAcceleration(3.0f);
    servo_D2.setMaxAcceleration(3.0f);

    servo_D0.enable();
    servo_D1.enable();
    servo_D2.enable();

    // ultra sonic sensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;

    const float voltage_max = 12.0f;     // maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack

    // https://www.pololu.com/product/3475/specs
    const float gear_ratio_M1 = 31.25f;                        // 31.25:1 gear box
    const float kn_M1 = 450.0f / 12.0f;                        // motor constant in RPM / V
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    motor_M1.setMaxVelocity(motor_M1.getMaxVelocity() * 0.5f);
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.5f);
    // const float velocity_max_M1 = kn_M1 / 60.0f * voltage_max; // maximum velocity in rotations per second
    // motor_M1.setMaxVelocity(velocity_max_M1 * 0.5f);           // set maximum velocity to 50% of maximum velocity
    
    // https://www.pololu.com/product/3485/specs
    const float gear_ratio_M2 = 488.28125f;                    // 488.28125:1 gear box
    const float kn_M2 = 28.0f / 12.0f;                         // motor constant in RPM / V
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    motor_M2.setMaxVelocity(motor_M2.getMaxVelocity() * 0.5f);
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);
    // const float velocity_max_M2 = kn_M2 / 60.0f * voltage_max; // maximum velocity in rotations per second
    // motor_M2.setMaxVelocity(velocity_max_M2 * 0.5f);           // set maximum velocity to 50% of maximum velocity

    // https://www.pololu.com/product/3477/specs
    const float gear_ratio_M3 = 78.125f;                       // 78.125:1 gear box
    const float kn_M3 = 180.0f / 12.0f;                        // motor constant in RPM / V
    DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max);
    motor_M3.setMaxVelocity(motor_M3.getMaxVelocity() * 0.5f);
    motor_M3.setMaxAcceleration(motor_M3.getMaxAcceleration() * 0.5f);
    // const float velocity_max_M3 = kn_M3 / 60.0f * voltage_max; // maximum velocity in rotations per second
    // motor_M3.setMaxVelocity(velocity_max_M3 * 0.5f);           // set maximum velocity to 50% of maximum velocity

    const float motor_setpoint_M1 = 300.0f / gear_ratio_M1;
    const float motor_setpoint_M2 = 300.0f / gear_ratio_M2;
    const float motor_setpoint_M3 = 300.0f / gear_ratio_M3;

    // imu
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;

            // read analog input
            ir_distance_avg = ir_sensor.read();  // after the calibration you actually only need this function call

            // commanding the servos
            servo_D0.setNormalisedPulseWidth(1.0f);
            servo_D1.setNormalisedPulseWidth(1.0f);
            servo_D2.setNormalisedPulseWidth(1.0f);

            // read us sensor distance, only valid measurements will update us_distance_cm
            const float us_distance_cm_candidate = us_sensor.read();
            if (us_distance_cm_candidate > 0.0f) {
                us_distance_cm = us_distance_cm_candidate;
            }

            // read imu data
            imu_data = imu.getImuData();

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
                ir_distance_avg = 0.0f;
                servo_D0.setNormalisedPulseWidth(0.0f);
                servo_D1.setNormalisedPulseWidth(0.0f);
                servo_D2.setNormalisedPulseWidth(0.0f);
                us_distance_cm = 0.0f;
                // imu_data.init();
            }
        }

        // toggling the user led
        user_led = !user_led;

        // // print to the serial terminal
        // printf("IR cm: %.2f, US cm: %.2f \n",
        //     ir_distance_avg,
        //     us_distance_cm);

        // print to the serial terminal
        printf("IR cm: %6.2f, US cm: %6.2f, R deg: %6.2f, P deg: %6.2f, Y deg: %6.2f \n",
            ir_distance_avg,
            us_distance_cm,
            imu_data.rpy(0) * (180.0f / M_PIf),
            imu_data.rpy(1) * (180.0f / M_PIf),
            imu_data.rpy(2) * (180.0f / M_PIf));

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
