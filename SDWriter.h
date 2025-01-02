#ifndef SD_WRITER_H_
#define SD_WRITER_H_

#include <mbed.h>

#include <SDBlockDevice.h>
#include <FATFileSystem.h>

class SDWriter
{
public:
    SDWriter(PinName mosi, PinName miso, PinName sck, PinName cs);
    ~SDWriter();

    bool mount();
    bool unmount();

    // opens a new file like /sd/data/001.bin, /sd/data/002.bin, etc.
    bool openNextFile();
    void closeFile();

    // write a single byte (e.g. "number of floats" header).
    bool writeByte(uint8_t b);

    // write 'count' floats to the file in binary.
    bool writeFloat(float value);
    bool writeFloats(const float* data, size_t count);

    // flush data to SD so it's physically written.
    bool flush();

private:
    SDBlockDevice m_SDBlockDevice;
    FATFileSystem m_FATFileSystem;

    FILE* m_FilePtr{nullptr};
    bool  m_mounted{false};

    int   m_file_counter{0}; // for enumerating new files
    char  m_file_path[64];   // current file path

    bool ensureDirExists(const char* path);
    bool openNumberedFile();
};
#endif /* SD_WRITER_H_ */
