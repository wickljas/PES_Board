#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DCMotor.h"
#include "SDLogger.h"
#include "bme68x.h"
#include "bme680_mbed.h"

// Pins & objects
DigitalIn  start_btn(BUTTON1);
DigitalOut user_led(LED1);
I2C        i2c(PB_9, PB_8);
SDLogger   sd_logger(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);
bme68x_dev bme; 
bme68x_conf conf; 
bme68x_heatr_conf heatr_conf;

const float gear_ratio  = 156.0f;
const float kn          = 89.0f/12.0f;
const float voltage_max = 12.0f;

DCMotor motor1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
DCMotor motor2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

// State machine
enum State { WAIT_FOR_START, LOG_AND_SCAN, ROTATE_SENSOR, ADVANCE_ROBOT };
State state = WAIT_FOR_START;

// Timers & counters
Timer scan_timer;            // for BME680 heater cycle
float  current_angle = 0.0f; // degrees
const float step_deg = 5.0f;

// Initialize BME680
void init_bme680() {
    bme.intf       = BME68X_I2C_INTF;
    bme.read       = user_i2c_read;
    bme.write      = user_i2c_write;
    bme.delay_us   = user_delay_us;
    bme.intf_ptr   = nullptr;
    if (bme68x_init(&bme) != BME68X_OK) {
        printf("BME init failed\n");
        while(1);
    }
    conf.os_hum   = BME68X_OS_2X;
    conf.os_temp  = BME68X_OS_4X;
    conf.os_pres  = BME68X_OS_8X;
    conf.filter   = BME68X_FILTER_OFF;
    bme68x_set_conf(&conf, &bme);
    heatr_conf.enable     = BME68X_ENABLE;
    heatr_conf.heatr_temp = 320;
    heatr_conf.heatr_dur  = 150;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
}

int main() {
    enable_motors = 1;
    motor1.setMaxVelocity(0.5f);
    motor2.setMaxVelocity(0.5f);
    motor1.enableMotionPlanner();
    motor2.enableMotionPlanner();
    init_bme680();

    scan_timer.start();

    while (true) {
        switch (state) {

        case WAIT_FOR_START:
            user_led = 0;
            if (start_btn.read()) {
                // open CSV header
                sd_logger.write(float(0)); // placeholder for angle
                sd_logger.write(float(0)); // placeholder for distance
                sd_logger.send();
                state = LOG_AND_SCAN;
            }
            break;

        case LOG_AND_SCAN: {
            user_led = 1;
            // 1) Trigger BME680 & wait heater
            bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
            bme.delay_us(heatr_conf.heatr_dur * 1000, nullptr);
            bme68x_data d; uint8_t n;
            float temp=0, hum=0, pres=0, gas=0;
            if (bme68x_get_data(BME68X_FORCED_MODE, &d, &n, &bme)==BME68X_OK && n) {
                temp = d.temperature;
                hum  = d.humidity;
                pres = d.pressure/100.0f;
                gas  = d.gas_resistance;
            }
            // 2) Ultrasonic
            //float dist = measureDistance();

            // 3) Log: angle, dist, T, H, P, G
            sd_logger.write(current_angle);
            //sd_logger.write(dist);
            sd_logger.write(temp);
            sd_logger.write(hum);
            sd_logger.write(pres);
            sd_logger.write(gas);
            sd_logger.send();

            state = ROTATE_SENSOR;
            break;
        }

        case ROTATE_SENSOR: {
            // compute how many full motor-rotations = 5° step
            float rotations = step_deg / 360.0f;  

            // command the motor to rotate by that amount (closed-loop)
            motor2.setRotation(rotations);    

            // wait (non-blocking would be better, but this illustrates the idea)
            while (motor2.getRotation() <= rotations) {
                thread_sleep_for(5);
            }

            // update our scan angle
            current_angle += step_deg;
            if (current_angle >= 360.0f) {
                current_angle = 0.0f;
                state = ADVANCE_ROBOT;
            } else {
                state = LOG_AND_SCAN;
            }
            break;
        }

        case ADVANCE_ROBOT: {
            // move robot forward half-wheel
            float start_pos = motor1.getRotation();
            float half_rot  = 360.0f / 2.0f;  // adjust per wheel/encoder resolution
            motor1.setVelocity(0.5f);
            while (motor1.getRotation() < start_pos + half_rot) { /* drive */ }
            motor1.setVelocity(0.0f);

            state = WAIT_FOR_START;
            break;
        }
        }

        thread_sleep_for(10); // small delay for debouncing/RTOS yield
    }
}
