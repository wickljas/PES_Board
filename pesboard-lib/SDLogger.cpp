#include "SDLogger.h"

SDLogger::SDLogger(PinName mosi,
                   PinName miso,
                   PinName sck,
                   PinName cs,
                   uint8_t num_of_floats) : m_SDWriter(mosi, miso, sck, cs),
                                            m_Thread(DEFAULT_PRIORITY, 4096),
                                            m_num_of_floats(num_of_floats)
{
    // start the thread
    start();

    // open the file
    if (!openFile()) {
        printf("WARNING: Could not open SD file for logging.\n");
    }
}

SDLogger::~SDLogger()
{
    closeFile();
    m_Thread.terminate();
}

void SDLogger::start(osPriority priority)
{
    // m_Thread.start(callback(this, &SDLogger::threadTask));
    // m_Thread.set_priority(priority);

    // start thread
    // m_Thread.set_priority(priority);
    m_Thread.start(callback(this, &SDLogger::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &SDLogger::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

bool SDLogger::openFile()
{
    // attempt to mount & open
    if (!m_SDWriter.mount()) {
        printf("SDLogger: mount failed\n");
        return false;
    }
    if (!m_SDWriter.openNextFile()) {
        printf("SDLogger: openNextFile failed\n");
        return false;
    }
    // write the "m_num_of_floats" as the first byte once
    if (!m_SDWriter.writeByte(m_num_of_floats)) {
        printf("SDLogger: writing num_of_floats byte failed\n");
        return false;
    }
    m_file_open = true;
    return true;
}

void SDLogger::closeFile()
{
    if (m_file_open) {
        m_SDWriter.closeFile();
        m_SDWriter.unmount();
        m_file_open = false;
    }
}

void SDLogger::logFloats(const float* data, size_t count)
{
    if (!m_file_open) {
        printf("SDLogger: File not open—discarding data.\n");
        return;
    }

    // core_util_critical_section_enter(); 
    // bool ok = pushFloats(data, count);
    // core_util_critical_section_exit();

    m_Mutex.lock();
    bool ok = pushFloats(data, count);
    m_Mutex.unlock();

    if (!ok) {
        // buffer is full
        printf("SDLogger: Buffer overflow, lost data!\n");
    }
}

bool SDLogger::pushFloats(const float* data, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (m_CircularBuffer.full()) {
            return false;
        }
        m_CircularBuffer.push(data[i]);
    }
    return true;
}

void SDLogger::threadTask()
{
    Timer flush_timer;
    flush_timer.start();

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // write any pending data
        flushBuffer();

        // flush the file so data is physically on sd card
        if (flush_timer.elapsed_time() >= 2s) {
            flush_timer.reset();

            if (m_file_open) {
                bool ok = m_SDWriter.flush();
                if (!ok) {
                    printf("SDLogger: fflush failed\n");
                }
            }
        }
        // // sleep a bit before next iteration
        // ThisThread::sleep_for(50ms);
    }
}

void SDLogger::flushBuffer()
{
    if (!m_file_open) {
        return;
    }

    // drain in chunks
    static constexpr size_t CHUNK_SIZE = 256;
    float tmp[CHUNK_SIZE];

    while (!m_CircularBuffer.empty()) {
        size_t count_to_pop = 0;

        // core_util_critical_section_enter();
        // while ((count_to_pop < CHUNK_SIZE) && (!m_CircularBuffer.empty())) {
        //     m_CircularBuffer.pop(tmp[count_to_pop]);
        //     count_to_pop++;
        // }
        // core_util_critical_section_exit();

        m_Mutex.lock();
        while ((count_to_pop < CHUNK_SIZE) && (!m_CircularBuffer.empty())) {
            m_CircularBuffer.pop(tmp[count_to_pop]);
            count_to_pop++;
        }
        m_Mutex.unlock();

        // write that chunk
        if (!m_SDWriter.writeFloats(tmp, count_to_pop)) {
            printf("SDLogger: writeFloats failed\n");
            // break (or keep trying)
            break;
        }
    }
}

void SDLogger::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
