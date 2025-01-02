// rotc:  SDFileSystem sd(PB_5, PB_4, PB_3, PB_10, "sd"); //mosi, miso, sclk, cs
// - this might be the old version of the pes board

// borla: SDFileSystem sd(PC_12, PC_11, PC_10, PD_2, "sd"); // mosi miso clk cs
// - PC_12 SPI3_MOSI mosi
// - PC_11 SPI3_MISO miso
// - PC_10 SPI3_SCK  clk
// - PD_2  PD2       cs


#include "mbed.h"

// pes board pin map
#include "pesboard-lib/PESBoardPinMap.h"

// drivers
#include "pesboard-lib/DebounceIn.h"
#include "pesboard-lib/Stepper.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button
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
    DigitalOut user_led(USER_LED);

    // stepper motors
    Stepper stepper_M1(PB_9, PB_8);
    Stepper stepper_M2(PB_4, PA_7);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // stepper_M1.setVelocity(0.1f);
            // stepper_M2.setVelocity(-0.1f);
            stepper_M1.setRotation(-1.0f, 1.0f);
            stepper_M2.setRotation(+2.0f, 1.0f);

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                stepper_M1.setRotationRelative(-1.0f, 1.0f);
                stepper_M2.setRotationRelative(+2.0f, 1.0f);
                
            }
        }

        printf("Stepper M1: %d, %d, %0.3f, ", stepper_M1.getStepsSetpoint(), stepper_M1.getSteps(), stepper_M1.getRotation());
        printf("M2: %d, %d, %0.3f \n", stepper_M2.getStepsSetpoint(), stepper_M2.getSteps(), stepper_M2.getRotation());

        // toggling the user led
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
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