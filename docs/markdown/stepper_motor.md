<!-- link list, last updated 03.01.2024 -->
[0]: https://www.pololu.com/product/1209
[1]: https://www.pololu.com/product/2267
[2]: https://www.pololu.com/product/1182
[3]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/

# Stepper motor

A stepper motor is an electromechanical device that converts electrical pulses into precise mechanical movements. Unlike traditional motors, stepper motors move in discrete steps, allowing for accurate control of position and speed without the need for feedback systems. This characteristic makes them ideal for applications requiring precise positioning, such as in robotics, CNC machines, and 3D printers. By energizing the motor windings in a specific sequence, the stepper motor's rotor aligns with the magnetic field, resulting in controlled and incremental rotation. This precise control over movement is why stepper motors are a popular choice in automation and control systems.

<p align="center">
    <img src="../images/stepper_motor.png" alt="Stepper motor" width="400"/> </br>
    <i>Stepper motor</i>
</p>

## Specifications
Below are the specifications of the stepper motor and the external driver that operates it:

| Parameter           | NEMA 14  |NEMA 17    |
| ------------------- | ---------|-----------|
| Polarity            | Bipolar  | Bipolar   |
| Size                | 35x36 mm | 42.3x38 mm|
| Current rating      | 1000 mA  | 1680 mA   |
| Voltage rating      | 2.7 V    | 2.8 V     |
| Steps per revolution| 200      | 200       |    
| Resistance          | 2.7 Ohm  | 1.65 Ohm  |
| Number of leads     | 4        | 4         |

A4988 Stepper Motor Driver Carrier

| Parameter                   | Value                     |
| --------------------------- | ------------------------- |
| Minimum operating voltage   | 8 V                       |
| Maximum operating voltage   | 35 V                      |
| Continous current per phase | 1 A                       |
| Maximum current per phase   | 2 A                       |
| Minimum logic voltage       | 3 V                       |
| Maximum logic voltage       | 5.5 V                     |
| Microstep resolutions       | full, 1/2, 1/4, 1/8, 1/16 |

## Links

[NEMA 14][0] </br>
[NEMA 17][1] </br>
[A4988 Stepper Motor Driver Carrier][2]

## Datasheets

[A4988 Stepper Motor Driver Carrier](../datasheets/A4988.pdf)

## Practical Tips
The A4988 stepper motor driver allows for adjustable current limiting, which helps prevent motor damage and optimize performance. The current limit is set using a trimmer potentiometer on the board. The method involves calculating the reference voltage (VREF) using the formula:

\[
I_{limit} = \frac{V_{REF}}{8 \times R_{sense}}
\]

where \( R_{sense} \) is the sense resistor value, typically 0.05 ohms for this board. By adjusting the VREF, you can set the desired current limit to match your stepper motor's requirements. It's important to measure VREF directly from the potentiometer with the driver powered on but the motor disconnected.

This setup helps ensure your motor operates within safe limits, especially during high-demand scenarios.

For more information and video instruction look [HERE][2]

## Stepper Motor Example

The `Stepper` class driver controls a stepper motor. It initializes with specified step and direction pins and step-per-revolution settings. The class provides methods to retrieve the current step count, rotation, and velocity of the motor. It also includes methods to set the motor's velocity, absolute and relative rotation, and the number of steps. Additionally, the driver manages motor stepping, threading, and timing through its internal methods and variables.

To start working with the motors, it is necessary to plug it correclty and create an object in the ***main.cpp*** file and assign proper pins, in following example there will be two motors in use.

### Connection to the Nucleo Board
The connection of the board to the motors is made via a driver. The motors are directly connected to the driver according to the diagram below. 

<p align="center">
    <img src="../images/pololu_driver_connection.png" alt="Pololu driver connection" width="400"/> </br>
    <i>Pololu driver connections</i>
</p>


The mentioned driver is connected to the board and this is how the motors are controlled. Connection uses two pins with digital output, one to indicate direction and the other to indicate steps. The voltage, supplying the motors (8 - 35V), should also be connected to the driver (a 330nF condenser should also be added) and the voltage supplying the driver logic with a value. In addition, some of the pins should also be bridged as shown in the attached diagram.

**Motor 1**
```
step PB_9
direction PB_8
```
**Motor 2**
```
step PB_4
direction PA_7
```

[Mbed ST-Nucleo-F446RE][0]

Below is a link to a diagram with information on how to connect the two motors together with the drivers.

The electronics connection is included in the file, which can be found [HERE](../dev/dev_steppermotor/stepper_motor_connection.pdf) <br>

### Create Stepper Object
Initially, it's essential to add the suitable drivers to the ***main.cpp*** file and then create an `` Stepper`` object inside ``main()`` function with the pin's names passed as an argument.

```
#include "pm2_drivers/Stepper.h"
```

```
// stepper motors
Stepper stepper_M1(PB_9, PB_8);
Stepper stepper_M2(PB_4, PA_7);
```

### Stepper Motor Driver Overview

The driver is designed to work with stepper motors that require direction and step control signals. It supports various operations like setting the motor's rotation, controlling its velocity, and handling relative rotations.

**Key Functionalities**

1. **Setting Rotation (`setRotation`)**:
   - **Purpose**: Moves the motor to a specified rotation count. You can set the rotation with or without specifying the velocity.
   - **Usage**:
     ```cpp
     myStepper.setRotation(2.0); // Moves to 2 full rotations at default velocity
     myStepper.setRotation(3.5, 2.0); // Moves to 3.5 rotations at 2 rotations per second
     ```

2. **Relative Rotation (`setRotationRelative`)**:
   - **Purpose**: Rotates the motor by a specified relative amount from its current position. Like `setRotation`, you can specify velocity or use the current velocity.
   - **Usage**:
     ```cpp
     myStepper.setRotationRelative(1.0); // Moves 1 full rotation forward from the current position
     myStepper.setRotationRelative(-0.5, 1.5); // Moves 0.5 rotations backward at 1.5 rotations per second
     ```

3. **Setting Velocity (`setVelocity`)**:
   - **Purpose**: Sets the motor's velocity directly, either moving it at a constant speed or stopping it if the velocity is zero.
   - **Usage**:
     ```cpp
     myStepper.setVelocity(2.0); // Moves continuously at 2 rotations per second
     myStepper.setVelocity(0.0); // Stops the motor
     ```

4. **Steps Control (`setSteps`)**:
   - **Purpose**: Moves the motor to a specific step count at a given velocity. It handles the direction automatically.
   - **Usage**:
     ```cpp
     myStepper.setSteps(1600, 2.0); // Moves to step 1600 at 2 rotations per second
     ```

The example below integrates the driver with a main loop that runs continuously, allowing the stepper motor to be controlled dynamically based on a button press.

```cpp
#include "mbed.h"

// Include necessary headers
#include "pm2_drivers/PESBoardPinMap.h"
#include "pm2_drivers/DebounceIn.h"
#include "pm2_drivers/Stepper.h"

bool do_execute_main_task = false; // Flag to control the main task execution
bool do_reset_all_once = false;    // Flag to reset variables and states once

// Debounced user button handling
DebounceIn user_button(USER_BUTTON); 

void toggle_do_execute_main_fcn(); // Function to toggle task execution

int main()
{
    // Attach the button fall function
    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20; // Define the main task period in ms
    Timer main_task_timer;              // Timer for controlling the loop execution time

    // LED on Nucleo board
    DigitalOut user_led(USER_LED);

    // Initialize stepper motors
    Stepper stepper_M1(PB_9, PB_8);
    Stepper stepper_M2(PB_4, PA_7);

    // Start the timer
    main_task_timer.start();

    // Main loop
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            // If the task is active, set the desired rotations and velocity
            stepper_M1.setRotation(-1.0f, 1.0f);
            stepper_M2.setRotation(+2.0f, 1.0f);

        } else {
            // Execute this block only once when resetting
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // Set relative rotations with specified velocity
                stepper_M1.setRotationRelative(-1.0f, 1.0f);
                stepper_M2.setRotationRelative(+2.0f, 1.0f);
            }
        }

        // Output the current state of the steppers
        printf("Stepper M1: %d, %d, %0.3f, ", stepper_M1.getStepsSetpoint(), stepper_M1.getSteps(), stepper_M1.getRotation());
        printf("M2: %d, %d, %0.3f \n", stepper_M2.getStepsSetpoint(), stepper_M2.getSteps(), stepper_M2.getRotation());

        // Toggle the user LED
        user_led = !user_led;

        // Calculate the elapsed time and sleep for the remaining period
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

// Function to toggle the execution of the main task when the user button is pressed
void toggle_do_execute_main_fcn()
{
    do_execute_main_task = !do_execute_main_task; // Toggle the execution flag
    if (do_execute_main_task)
        do_reset_all_once = true; // Set the reset flag when starting the task
}
```

### Explanation of the Example

- **Non-Blocking Execution**: The main loop runs continuously, and the timing is managed using a timer, allowing the loop to execute periodically without blocking.
- **Stepper Motor Control**: The `setRotation` and `setRotationRelative` functions are used to control the stepper motors based on the current state of the task. These functions move the motor to a specified rotation with a given velocity.
- **Button Handling**: The user button toggles the main task execution. When pressed, it triggers `toggle_do_execute_main_fcn`, which sets the `do_execute_main_task` flag and optionally resets the state with `do_reset_all_once`.
- **LED Toggling**: The LED on the board is toggled with each loop iteration to provide a visual indication of the loop's operation.

### Usage Scenarios

- **Continuous Rotation**: If you want the motors to rotate continuously at a set speed, you can use the `setVelocity` function instead of `setRotation`.
- **Precise Positioning**: For applications requiring precise control over the motor's position, `setRotation` and `setRotationRelative` provide accurate movement control.

This approach allows for effective motor control in applications where real-time responsiveness and non-blocking execution are critical.