
/**
 * @file SDLogger.h
 * @brief This file defines the SDLogger class, used for logging float data to an SD card.
 *
 * The SDLogger class provides functionality to queue floating-point samples using a ring buffer
 * and write them to an SD card at low priority in bursts. It encapsulates the details of thread creation,
 * periodic flushing, and synchronization, offering a simple interface for robust logging.
 *
 * Maximum throughput depends on SD card speed and buffer size. When the buffer fills up, additional data
 * is discarded. By default, data is flushed to disk every second to minimize data loss on power failure.
 *
 * @dependencies
 * This class relies on the following components:
 * - SDWriter: A minimal class for handling mount/unmount, file creation, and binary writes on the SD card.
 * - CircularBuffer<float>: For buffering incoming float data.
 * - ThreadFlag and Ticker: For scheduling periodic tasks to pull data from the buffer and write to the SD card.
 * - Mutex: For thread-safe access to the buffer.
 *
 * Usage:
 * To use the SDLogger class, create an instance by specifying the SPI pins for the SD card and the number of floats per record.
 * Then call logFloats(...) to queue up data, and the logger thread writes it to the SD in the background.
 *
 * Example:
 * ```
 * SDLogger logger(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, 50);
 * float myData[50];
 * // fill myData...
 * logger.logFloats(myData, 50);
 * ```
 *
 * @author 
 * @date 02.01.2025
 */

#ifndef SD_LOGGER_H_
#define SD_LOGGER_H_

#include "mbed.h"

#include "SDWriter.h"
#include "ThreadFlag.h"

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
    SDLogger(PinName mosi, PinName miso, PinName sck, PinName cs, uint8_t num_of_floats);
    ~SDLogger();

    // start the internal thread at a desired priority
    void start(osPriority priority = DEFAULT_PRIORITY);
    // opens a new file on the SD card, writes the "m_num_of_floats" as a header byte
    bool openFile();
    // closes the file
    void closeFile();
    // log some float data (appends to ring buffer)
    void logFloats(const float* data);
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
