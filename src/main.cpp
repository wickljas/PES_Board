#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"
#include "DCMotor.h"

// drivers
#include "DebounceIn.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DigitalIn button(BUTTON1);


// main runs as an own thread
int main()
{
    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // start timer
    main_task_timer.start();

    int buttonNow = button.read();
    int buttonBefore = buttonNow;

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    const float gear_ratio_M2 = 156.0f; // gear ratio
    const float kn_M2 = 89.0f / 12.0f;  // motor constant [rpm/V]
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M2, kn_M2, voltage_max);

    

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        motor_M1.setVelocity(0.5f);
        buttonNow = button.read();
        if (buttonNow && !buttonBefore) {
            user_led = !user_led;
            
        }

        buttonBefore = buttonNow;
        printf("Motor velocity: %f \n", motor_M1.getMaxVelocity());

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

