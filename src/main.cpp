#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DCMotor.h"
#include "SDLogger.h"
#include "bme68x.h"
#include "bme680_mbed.h"

// MLX90640 driver headers
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"

// For raw file writes
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

// I/O
DigitalIn  start_btn(BUTTON1);
DigitalOut user_led(LED1);
I2C        i2c(PB_9, PB_8);
SDLogger   sd_logger(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);

// Mount block‐device on “fs:”  
SDBlockDevice bd(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);
FATFileSystem  fs("fs", &bd);

// BME680
bme68x_dev        bme;
bme68x_conf       conf;
bme68x_heatr_conf heatr_conf;

// Ultrasonic
DigitalOut  us_trig(PB_6);
DigitalIn   us_echo(PB_7);

// Motors
const float gear_ratio  = 156.0f;
const float kn          = 89.0f/12.0f;
const float voltage_max = 12.0f;
DCMotor motor1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
DCMotor motor2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

// State machine
enum State { WAIT_FOR_START, LOG_AND_SCAN, ROTATE_SENSOR, ADVANCE_ROBOT, RETURN_HOME };
State state = WAIT_FOR_START;

// Scan parameters
const float step_deg        = 5.0f;               // sensor turn per step
const float turns_per_step  = step_deg / 360.0f;   // full motor turns needed
float       current_angle   = 0.0f;               // cumulative scan angle
const float totalTravelTurns = 10.0f; // e.g. 10 full wheel turns = X mm
// Compute how many segments (i.e. how many scans) fit:
static int   totalSegments;           
static int   segmentCounter;


// Forward travel (motor turns)
const float distanceToTravelTurns = 2.0f;
float       robot_start_pos        = 0.0f;
float       robot_target_pos       = 0.0f;

// Heater‐cycle timer
Timer scan_timer;

// MLX90640 buffers & params
#define MLX_ADDR        0x33
static uint16_t eeMLX[832];
static paramsMLX90640 mlxParams;
static uint16_t rawMLX[834];
static float    frameMLX[768];
static int      frameCount = 0;

// ——— BME680 init ———
void init_bme680() {
    bme.intf     = BME68X_I2C_INTF;
    bme.read     = user_i2c_read;
    bme.write    = user_i2c_write;
    bme.delay_us = user_delay_us;
    bme.intf_ptr = nullptr;
    if (bme68x_init(&bme) != BME68X_OK) {
        printf("BME init failed\n"); while (true);
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

// ——— Ultrasonic ———
float measureDistanceCm() {
    us_trig = 0; wait_us(2);
    us_trig = 1; wait_us(10);
    us_trig = 0;
    Timer t;
    while (!us_echo);
    t.start();
    while (us_echo);
    t.stop();
    return (t.elapsed_time().count() * 0.0343f) / 2.0f;
}

// ——— MLX90640 init & read ———
void init_mlx() {
    MLX90640_I2CInit();
    MLX90640_I2CFreqSet(1000);                              // 1 MHz
    MLX90640_DumpEE(MLX_ADDR, eeMLX);                       // EEPROM → eeMLX
    MLX90640_ExtractParameters(eeMLX, &mlxParams);      // parse cal
    MLX90640_SetRefreshRate(MLX_ADDR, 0x05);
    MLX90640_SetChessMode(MLX_ADDR);

    }

bool read_mlx_frame() {
    if (MLX90640_GetFrameData(MLX_ADDR, rawMLX) < 0) return false;
    float Ta = MLX90640_GetTa(rawMLX, &mlxParams);
    // Correct order here:
    MLX90640_CalculateTo(
        rawMLX,
        &mlxParams,
        1.0f,
        Ta,
        frameMLX
    );
    return true;
}


void saveFrameAsPPM(int idx) {
    char fn[32];
    sprintf(fn, "/fs/frame%04d.ppm", idx);
    FILE *f = fopen(fn, "wb");
    if (!f) return;
    // PPM header
    fprintf(f, "P6 32 24 255\n");
    // map 20–40 °C → 0–255
    for (int i = 0; i < 768; i++) {
        float t = frameMLX[i];
        int v = (int)roundf((t - 20.0f) * (255.0f / 20.0f));
        uint8_t c = v < 0 ? 0 : v > 255 ? 255 : v;
        // write RGB triplet
        fputc(c, f);
        fputc(c, f);
        fputc(c, f);
    }
    fclose(f);
}

// ——— main ———
int main() {
    // mount SD
    if (fs.mount(&bd) != 0) {
        printf("ERROR: SD mount failed\n");
        while (1);
    }

    totalSegments   = ceilf(totalTravelTurns / distanceToTravelTurns);
    segmentCounter  = 0;


    enable_motors = 1;
    motor1.setMaxVelocity(0.5f);
    motor2.setMaxVelocity(0.5f);
    motor1.enableMotionPlanner();
    motor2.enableMotionPlanner();

    init_bme680();
    init_mlx();
    scan_timer.start();

    while (true) {
        switch (state) {

        case WAIT_FOR_START:
            user_led = 0;
            if (start_btn.read()) {
                // debounce
                thread_sleep_for(50);
                if (!start_btn.read()) break;

                // 1) write a CSV header via stdio
                {
                    FILE *f = fopen("/fs/data.csv", "w");
                    if (f) {
                        fprintf(f, "angle,dist_cm,temp_C,hum_pct,press_hPa,gas_ohm\n");
                        fclose(f);
                    }
                }

                // 2) compute segments & reset
                totalSegments  = int(totalTravelTurns / distanceToTravelTurns);
                segmentCounter = 0;

                // 3) record home & first targets
                robot_start_pos  = motor1.getRotation();
                robot_target_pos = robot_start_pos + distanceToTravelTurns;

                // 4) reset scan
                current_angle = 0;
                motor2.setRotationRelative(-motor2.getRotation());
                frameCount = 0;

                state = LOG_AND_SCAN;
            }
            break;

        case LOG_AND_SCAN: {
            user_led = 1;
            // 1) IR frame
            if (read_mlx_frame()) {
                saveFrameAsPPM(frameCount++);
            }
            // 2) BME680
            bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
            bme.delay_us(heatr_conf.heatr_dur * 1000, nullptr);
            bme68x_data d; uint8_t n;
            float T=0,H=0,P=0,G=0;
            if (bme68x_get_data(BME68X_FORCED_MODE, &d, &n, &bme)==BME68X_OK && n) {
                T = d.temperature; H = d.humidity;
                P = d.pressure/100.0f; G = d.gas_resistance;
            }
            // 3) Ultrasonic
            float dist = measureDistanceCm();
            // 4) Log CSV: angle, distance, T,H,P,G
            sd_logger.write(current_angle);
            sd_logger.write(dist);
            sd_logger.write(T);
            sd_logger.write(H);
            sd_logger.write(P);
            sd_logger.write(G);
            sd_logger.send();
            state = ROTATE_SENSOR;
            break;
        }

        case ROTATE_SENSOR: {
            motor2.setRotationRelative(turns_per_step);
            while (fabs(motor2.getRotationTarget() - motor2.getRotation()) > (0.5f/360.0f)) {
                thread_sleep_for(2);
            }
            current_angle += step_deg;
            state = (current_angle < 360.0f) ? LOG_AND_SCAN : ADVANCE_ROBOT;
            break;
        }

        case ADVANCE_ROBOT: {
            // drive to the *current* target
            motor1.setRotation(robot_target_pos);
            while (fabs(motor1.getRotationTarget() - motor1.getRotation()) > 0.01f)
                thread_sleep_for(5);

            segmentCounter++;

            if (segmentCounter < totalSegments) {
                // bump the *absolute* target out by another crawl
                robot_target_pos += distanceToTravelTurns;

                // prep next 360° scan
                current_angle = 0;
                motor2.setRotationRelative(-motor2.getRotation());
                state = LOG_AND_SCAN;
            } else {
                state = RETURN_HOME;
            }
            break;
        }


        case RETURN_HOME: {
            // sensor home
            motor2.setRotationRelative(-motor2.getRotation());
            while (fabs(motor2.getRotation()) > (0.5f/360.0f)) {
                thread_sleep_for(2);
            }
            // robot home
            motor1.setRotation(robot_start_pos);
            while (fabs(motor1.getRotation() - robot_start_pos) > 0.01f) {
                thread_sleep_for(5);
            }
            state = WAIT_FOR_START;
            break;
        }
        }

        thread_sleep_for(10);
    }
}
