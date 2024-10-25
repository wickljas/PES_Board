#include <mbed.h>

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"
#include "pm2_drivers/IMU.h"
#include "pm2_drivers/Servo.h"
#include <complex>

//Motor Values
const float max_voltage = 12.0f;               // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
const float counts_per_turn = 20.0f * 78.125f; // define counts per turn at gearbox end: counts/turn * gearratio
const float kn = 180.0f / 12.0f;               // define motor constant in RPM/V
const float k_gear = 100.0f / 78.125f;         // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
const float kp = 0.2f;                         // define custom kp, this is the default speed controller gain for gear box 78.125:1
const float max_speed_rps = 1.0f;

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once
// while loop gets executed every main_task_period_ms milliseconds, this is a
// simple approach to repeatedly execute main
const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                    // the main task will run 50 times per second

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button

void user_button_pressed();
// Variables and function calls
DigitalOut board_led(LED1);

// main runs as an own thread
int main()
{
    printf("Setup\n");
    //attach user_button_pressed to the user button, so that it executes when pressed on falling edge.
    user_button.fall(&user_button_pressed);

}

void user_button_pressed() {
    do_execute_main_task = true;

    if (do_execute_main_task) {
        board_led = 1;
    } else {
        board_led = 0;
    }
}