#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DCMotor.h"
#include "SDLogger.h"
#include "bme68x.h"
#include "bme680_mbed.h"
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"
#include <chrono>

using namespace std::chrono_literals;
using namespace std::chrono;

// I2C bus for sensors
I2C i2c(PB_9, PB_8);


// HC-SR04 ultrasonic sensor pins
DigitalOut us_trig(D3);
DigitalIn  us_echo(D2, PullDown);
#define MAX_WAIT_US 30000

// BME680 globals
bme68x_dev        bme;
bme68x_conf       conf;
bme68x_heatr_conf heatr_conf;

// MLX90640 globals
#define MLX_ADDR 0x33
static uint16_t eeMLX[832];
static paramsMLX90640 mlxParams;
static uint16_t rawMLX[834];
static float    frameMLX[768];

// Motor enable and objects (simulated)
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
DCMotor motor1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, 156.0f, 89.0f/12.0f, 12.0f);
DCMotor motor2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, 156.0f, 89.0f/12.0f, 12.0f);

// Button and LED
DigitalIn  start_btn(BUTTON1, PullUp);
DigitalOut status_led(LED1);

// Sequence parameters
const float step_deg       = 5.0f;
const float turns_per_step = step_deg / 360.0f;
const float travel_turns   = 2.0f;
const float total_rotation = 360.0f;
const int   totalSegments  = 2;
const auto  sensorTimeout  = 5000ms;

// State machine
enum State { WAIT_FOR_START, LOG_AND_SCAN, ROTATE_SENSOR, ADVANCE_ROBOT, RETURN_HOME };
State state = WAIT_FOR_START;
float current_angle = 0.0f;
int   segmentCount  = 0;

// Logging timer
Timer logging_timer;
static microseconds time_previous_us{0};

// Sensor read functions
float measureDistanceCm() {
    us_trig = 0; wait_us(2);
    us_trig = 1; wait_us(10);
    us_trig = 0;
    Timer t, to; to.start();
    while (!us_echo) if (to.elapsed_time().count() > MAX_WAIT_US) return NAN;
    t.start();
    while (us_echo) if (to.elapsed_time().count() > 2*MAX_WAIT_US) { t.stop(); return NAN; }
    t.stop();
    return (t.elapsed_time().count() * 0.0343f) / 2.0f;
}

bool init_bme680() {
    bme.intf     = BME68X_I2C_INTF;
    bme.read     = user_i2c_read;
    bme.write    = user_i2c_write;
    bme.delay_us = user_delay_us;
    bme.intf_ptr = &i2c;
    if (bme68x_init(&bme) != BME68X_OK) return false;
    conf.os_hum   = BME68X_OS_2X;
    conf.os_temp  = BME68X_OS_4X;
    conf.os_pres  = BME68X_OS_8X;
    conf.filter   = BME68X_FILTER_OFF;
    bme68x_set_conf(&conf, &bme);
    heatr_conf.enable     = BME68X_ENABLE;
    heatr_conf.heatr_temp = 320;
    heatr_conf.heatr_dur  = 150;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    return true;
}

bool read_bme680(float &T, float &H, float &P, float &G) {
    bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    bme.delay_us(heatr_conf.heatr_dur * 1000, nullptr);
    bme68x_data data; uint8_t n = 0;
    if (bme68x_get_data(BME68X_FORCED_MODE, &data, &n, &bme) == BME68X_OK && n) {
        T = data.temperature;
        H = data.humidity;
        P = data.pressure / 100.0f;
        G = data.gas_resistance;
        return true;
    }
    return false;
}

bool init_mlx90640() {
    MLX90640_I2CInit();
    MLX90640_I2CFreqSet(1000);
    if (MLX90640_DumpEE(MLX_ADDR, eeMLX) != 0) return false;
    if (MLX90640_ExtractParameters(eeMLX, &mlxParams) != 0) return false;
    MLX90640_SetRefreshRate(MLX_ADDR, 0x05);
    MLX90640_SetChessMode(MLX_ADDR);
    return true;
}

bool read_mlx90640_frame(float &amb, float &center) {
    if (MLX90640_GetFrameData(MLX_ADDR, rawMLX) < 0) return false;
    amb = MLX90640_GetTa(rawMLX, &mlxParams);
    MLX90640_CalculateTo(rawMLX, &mlxParams, 1.0f, amb, frameMLX);
    center = frameMLX[(32/2) + (24/2)*32];
    return true;
}

int main() {
    printf("Initializing system...\n");
    if (!init_bme680() || !init_mlx90640()) {
        printf("Sensor init failed!\n");
        error("Init error");
    }


    // enable motors (simulated)
    enable_motors = 1;
    motor1.setMaxVelocity(1.0f);
    motor2.setMaxVelocity(1.0f);

    const int main_task_period_ms = 20;
    Timer main_task_timer; 

        // sd card logger
    SDLogger sd_logger(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);

    // additional timer to measure time
    Timer logging_timer;
    logging_timer.start();

    // start timer
    main_task_timer.start();


    while (true) {
        main_task_timer.reset();
        switch (state) {
            case WAIT_FOR_START:
                status_led = 0;
                if (start_btn.read() == 0) {
                    ThisThread::sleep_for(50ms);
                    if (start_btn.read() == 0) {
                        printf("Start sequence\n");
                        current_angle = 0.0f;
                        segmentCount  = 0;
                        state = LOG_AND_SCAN;
                    }
                }
                break;

            case LOG_AND_SCAN: {
                status_led = 1;
                float dist = measureDistanceCm();
                float T, H, P, G, amb, cen;
                read_bme680(T, H, P, G);
                read_mlx90640_frame(amb, cen);

                                // measure delta time
                static microseconds time_previous_us{0}; // static variables are only initialized once
                const microseconds time_us = logging_timer.elapsed_time();
                const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
                time_previous_us = time_us;
                
                // log to SD
                sd_logger.write(dtime_us);
                sd_logger.write(current_angle);
                sd_logger.write(dist);
                sd_logger.write(T);
                sd_logger.write(H);
                sd_logger.write(P);
                sd_logger.write(G);
                sd_logger.write(amb);
                sd_logger.write(cen);
                sd_logger.send();


                printf("Ultrasonic: %.2f cm\n", isnan(dist)? -1.0f : dist);
                printf("BME680 -> T: %.2f C, H: %.2f %%RH, P: %.2f hPa, G: %.0f Ohm\n",
                       isnan(T)? -999.0f : T, isnan(H)? -999.0f : H,
                       isnan(P)? -999.0f : P, isnan(G)? -999.0f : G);
                printf("MLX90640 -> Ambient: %.2f C, Center: %.2f C\n",
                       isnan(amb)? -999.0f : amb, isnan(cen)? -999.0f : cen);

                Timer to; to.start();
                while ((isnan(dist) || isnan(T) || isnan(amb)) && to.elapsed_time() < sensorTimeout) {}



                state = ROTATE_SENSOR;
                break;
            }

            case ROTATE_SENSOR:
                status_led = 1;
                printf("Moving to %.1f degrees\n", current_angle + step_deg);
                //ThisThread::sleep_for(200ms);  // simulate rotation
                current_angle += step_deg;
                state = (current_angle >= total_rotation) ? ADVANCE_ROBOT : LOG_AND_SCAN;
                break;

            case ADVANCE_ROBOT:
                status_led = 1;
                printf("Advancing segment %d of %d\n", segmentCount+1, totalSegments);
                //ThisThread::sleep_for(500ms);  // simulate drive
                segmentCount++;
                current_angle = 0.0f;
                state = (segmentCount < totalSegments) ? LOG_AND_SCAN : RETURN_HOME;
                break;

            case RETURN_HOME:
                status_led = 1;
                printf("Returning home\n");
                //ThisThread::sleep_for(1000ms); // simulate return
                state = WAIT_FOR_START;
                break;
        }

        sd_logger.send();
        int buffer_size = sd_logger.BUFFER_SIZE;
        //printf("Buffer Size %d \n", buffer_size);
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        if (elapsed < main_task_period_ms) {
            thread_sleep_for(main_task_period_ms - elapsed);
        } else {
            printf("Warning: main loop too long\n");
        }
    }
}