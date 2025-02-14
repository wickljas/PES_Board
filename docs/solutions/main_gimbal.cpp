#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "IMU.h"
#include "Servo.h"

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

    // servo
    Servo servo_roll(PB_D0);
    Servo servo_pitch(PB_D1);

    // imu
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);    
    Eigen::Vector2f rp;

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // modelcraft RS2 MG/BB
    float servo_ang_min = 0.0325f;
    float servo_ang_max = 0.1250f;

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
    servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);

    // angle limits of the servos
    const float angle_range_min = -M_PI/2.0f;
    const float angle_range_max = M_PI/2.0f;    

    // angle to pulse width coefficients
    const float normalised_angle_gain = 1.0f / M_PI;
    const float normalised_angle_offset = 0.5f;

    // pulse width
    static float roll_servo_width = 0.5f;
    static float pitch_servo_width = 0.5f;

    servo_roll.setPulseWidth(roll_servo_width);
    servo_pitch.setPulseWidth(pitch_servo_width);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // enable the servos
            if (!servo_roll.isEnabled())
                servo_roll.enable();
            if (!servo_pitch.isEnabled())
                servo_pitch.enable();

            // read imu data
            imu_data = imu.getImuData();

            // roll angle
            rp(0) = atan2f(imu_data.quat.x() + imu_data.quat.z(), imu_data.quat.w() - imu_data.quat.y()) - atan2f(imu_data.quat.z() - imu_data.quat.x(), imu_data.quat.y() + imu_data.quat.w());
            // pitch angle
            rp(1) = acosf((imu_data.quat.w() - imu_data.quat.y()) * (imu_data.quat.w() - imu_data.quat.y()) + (imu_data.quat.x() + imu_data.quat.z()) * (imu_data.quat.x() + imu_data.quat.z()) - 1.0f) - M_PI / 2.0f;

            // map to servo commands
            roll_servo_width = -normalised_angle_gain * rp(0) + normalised_angle_offset;
            pitch_servo_width = normalised_angle_gain * rp(1) + normalised_angle_offset;
            if (rp(0) < angle_range_max && rp(0) > angle_range_min)
                servo_roll.setPulseWidth(roll_servo_width);
            if (rp(0) < angle_range_max && rp(0) > angle_range_min)
                servo_pitch.setPulseWidth(pitch_servo_width);

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;
                roll_servo_width = 0.5f;
                pitch_servo_width = 0.5f;
                servo_roll.setPulseWidth(roll_servo_width);
                servo_pitch.setPulseWidth(pitch_servo_width);
            }
        }

        // toggling the user led
        user_led = !user_led;

        // print to the serial terminal
        printf("%f, %f \n", roll_servo_width, pitch_servo_width);

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
