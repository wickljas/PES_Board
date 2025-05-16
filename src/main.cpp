#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DCMotor.h"
#include "bme68x.h"
#include "bme680_mbed.h"
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"

#ifndef M_PI
  #define M_PI 3.14159265358979323846f
#endif

// Tire geometry (for linear position)
const float wheel_radius_cm        = 0.3;
const float wheel_circumference_cm = 2.0f * M_PI * wheel_radius_cm;

using namespace std::chrono_literals;

// I2C bus for sensors
I2C i2c(PB_9, PB_8);

// SDLogger for logging


// HC-SR04 ultrasonic sensor pins (D3 trig, D2 echo)
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

// Motor enable and objects
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
const float gear_ratio  = 156.0f;
const float kn          = 89.0f/12.0f;
const float voltage_max = 12.0f;
DCMotor motor1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
DCMotor motor2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

// Button and LED
DigitalIn  start_btn(BUTTON1, PullUp);
DigitalOut status_led(LED1);

// Sequence parameters
const float step_deg         = 10.0f;
const float turns_per_step   = step_deg / 360.0f;
const float travel_turns     = 0.3f; // wheel turns per segment
const float total_rotation   = 360.0f;
const int   steps            = total_rotation / step_deg;
const int   totalSegments    = 20;    // limited to two turns
const auto  sensorTimeout    = 5000ms;

float degreeToRev(int degrees) {
    return (degrees / 360.0f) * 0.9;
}

// Speeds
const float sensorSpeed = 0.3f;
const float driveSpeed  = 0.5f;

// State machine
enum State { WAIT_FOR_START, LOG_AND_SCAN, ROTATE_SENSOR, ADVANCE_ROBOT, RETURN_HOME };
State state = WAIT_FOR_START;
int current_step = 0;
int segmentCount = 0;
float current_angle = 0.0f;

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
        //error("Init error");
    }
    // enable motors and setup
    enable_motors = 1;
    motor1.setMaxVelocity(1.0f);
    motor2.setMaxVelocity(1.0f);
    motor1.enableMotionPlanner();
    motor2.enableMotionPlanner();

    while (true) {
        switch (state) {
            case WAIT_FOR_START:
                status_led = 0;
                if (start_btn.read() == 0) {
                    ThisThread::sleep_for(50ms);
                    if (start_btn.read() != 0) break;
                    printf("Start sequence\n");
                    current_step = 0;
                    segmentCount = 0;
                    current_angle = 0.0f;
                    state = LOG_AND_SCAN;
                }
                break;

            case LOG_AND_SCAN: {
                status_led = 1;
                float dist = NAN, T = NAN, H = NAN, P = NAN, G = NAN, amb = NAN, cen = NAN;
                // trigger reads
                dist = measureDistanceCm();
                read_bme680(T, H, P, G);
                read_mlx90640_frame(amb, cen);
                float current_position_cm = segmentCount * travel_turns * wheel_circumference_cm;
                // print sensor values
                
                printf("LOGSTART,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,",
                    current_angle,
                    current_position_cm,
                    isnan(dist)? -1.0f : dist,
                    isnan(T)?    -999.0f: T,
                    isnan(H)?    -999.0f: H,
                    isnan(P)?    -999.0f: P,
                    isnan(G)?    -999.0f: G
                );
                for (int i = 0; i < 768; i++) {
                    printf("%.2f", frameMLX[i]);
                    if (i < 767) printf(",");
                    else         printf(",LOGEND\n");
                }
                
                // wait or timeout
                Timer to; to.start();
                //while ((isnan(dist) || isnan(T) || isnan(amb)) && to.elapsed_time() < sensorTimeout) {}
                // log

                state = ROTATE_SENSOR;
                break;
            }

            case ROTATE_SENSOR: {
                status_led = 1;
                float next_angle = current_angle + step_deg;
                if (next_angle < total_rotation - 1e-3f) {
                    float revs = degreeToRev(step_deg);
                    //printf("Moving +%.1f° → %.2f motor revs\n", step_deg, revs);
                    motor2.setRotationRelative(revs);
                    while (fabs(motor2.getRotationTarget() - motor2.getRotation()) > 0.005f) {
                        //printf("Motor Angle: %f\n", motor2.getRotation());
                        ThisThread::sleep_for(20ms);
                    }
                    current_angle = next_angle;
                    state = LOG_AND_SCAN;
                } else {


                    printf("Reversing back 360° (%.2f revs)\n", degreeToRev(total_rotation));
                    motor2.setRotation(0.0f);
                    while (fabs(motor2.getRotation()) > 0.1f)
                        ThisThread::sleep_for(20ms);
                    current_angle = 0.0f;
                    state = ADVANCE_ROBOT;
                }
                break;
            }



            case ADVANCE_ROBOT:
                status_led = 1;
                printf("Advancing segment %d of %d\n", segmentCount+1, totalSegments);
                motor1.setRotationRelative(-travel_turns);
                
                while (fabs(motor1.getRotationTarget() - motor1.getRotation()) > 0.1f)
                    ThisThread::sleep_for(20ms);
                
                segmentCount++;
                current_angle = 0.0f;
                state = (segmentCount < totalSegments) ? LOG_AND_SCAN : RETURN_HOME;
                break;

            case RETURN_HOME:
                status_led = 1;
                printf("Returning home\n");
                motor2.setRotationRelative(-motor2.getRotation());
                while (fabs(motor2.getRotation()) > 0.005f)
                    ThisThread::sleep_for(20ms);
                motor1.setRotationRelative(-motor1.getRotation());
                while (fabs(motor1.getRotation()) > 0.01f)
                    ThisThread::sleep_for(20ms);
                state = WAIT_FOR_START;
                break;
        }
        //printf("Motor Angle: %f\n", motor2.getRotation());
        ThisThread::sleep_for(20ms);
    }
}