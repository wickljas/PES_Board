
/**
 * @file UltrasonicSensor.h
 * @brief This file defines the UltrasonicSensor class, used for measuring distances using the Ultrasonic Ranger V2.0.
 *
 * The UltrasonicSensor class provides functionality to measure distances by emitting ultrasonic pulses 
 * and measuring the time taken for the echo to return. It encapsulates the details of interfacing with the
 * sensor hardware and offers a simple interface for obtaining distance measurements in centimeters.
 * Maximum measurment distance is approximately 2 meters (measured 198.1 cm) with a mearuement period of 12000
 * microseconds. If no new valid measurement is available, the read() function returns -1.0f.
 *
 * @dependencies
 * This class relies on the following components:
 * - DigitalInOut: For toggling the ultrasonic sensor's pin between output and input.
 * - InterruptIn: For detecting the echo signal.
 * - Timer: For measuring the time interval of the echo.
 * - Timeout: For managing pulse emission timing.
 * - ThreadFlag: For managing threading and synchronization.
 *
 * Usage:
 * To use the UltrasonicSensor class, create an instance with the pin connected to the sensor.
 * Measure the distance using read(). The class also provides an operator float() for direct reading.
 *
 * Example:
 * ```
 * UltrasonicSensor ultrasonicSensor(PIN_NAME);
 * float distance = ultrasonicSensor.read();
 * // or simply
 * float distance = ultrasonicSensor;
 * ```
 *
 * @author M. E. Peter
 * @date 02.01.2025
 */

#ifndef SD_LOGGER_H_
#define SD_LOGGER_H_

#include <mbed.h>

#include "SDWriter.h"
#include "pesboard-lib/ThreadFlag.h"

#define DEFAULT_PRIORITY osPriorityLow

/**
 * A minimal thread-based SD logger that:
 * - Uses a larger CircularBuffer<float, BUFFER_SIZE>.
 * - Logs data in bursts from a low-priority thread.
 * - Flushes once per second so data is physically written.
 * - Prints a message if the buffer is full or if an SD write fails.
 * - Writes a "m_num_of_floats" byte at the file start (like a header).
 */
class SDLogger
{
public:
    // Increase buffer size for higher throughput / less overflow
    // static const size_t BUFFER_SIZE = 8192; // 8k floats = 32kB
    // static const size_t BUFFER_SIZE = 4096; // 4k floats = 16kB
    static const size_t BUFFER_SIZE = 2048; // 2k floats = 8kB
    // static const size_t BUFFER_SIZE = 1024; // e.g. 4kB

    /**
     * @param mosi          SPI MOSI pin
     * @param miso          SPI MISO pin
     * @param sck           SPI SCK pin
     * @param cs            SPI CS pin
     * @param num_of_floats We'll write this once at file start 
     *                      to indicate how many floats are in each record.
     */
    SDLogger(PinName mosi, PinName miso, PinName sck, PinName cs, u_int8_t num_of_floats);
    ~SDLogger();

    // start the internal thread at a desired priority
    void start(osPriority priority = DEFAULT_PRIORITY);
    // opens a new file on the SD card, writes the "m_num_of_floats" as a header byte
    bool openFile();
    // closes the file
    void closeFile();
    // log some float data (appends to ring buffer)
    void logFloats(const float* data, size_t count);

private:
    static constexpr int64_t PERIOD_MUS = 20000;

    Mutex m_Mutex; // mutex to protect the ring buffer
    CircularBuffer<float, BUFFER_SIZE> m_CircularBuffer;
    SDWriter m_SDWriter;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    uint8_t m_num_of_floats;
    bool m_file_open{false};

    // helper to drain the buffer in chunks
    void flushBuffer();
    // helper to push floats into the ring buffer
    bool pushFloats(const float* data, size_t count);

    void threadTask();
    void sendThreadFlag();
};
#endif /* SD_LOGGER_H_ */
