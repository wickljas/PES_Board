/**
 * @file SDWriter.h
 * @brief This file defines the SDWriter class, used for low-level SD card file operations.
 *
 * The SDWriter class provides functionality to mount/unmount an SD card, open sequential files
 * (e.g. /sd/data/001.bin, /sd/data/002.bin), and write binary floats or single bytes. It encapsulates the
 * details of block device initialization and FAT file system operations, providing a streamlined interface
 * for embedded logging.
 *
 * Data is typically written in raw binary for efficiency. The user can also call flush() to ensure
 * data is physically committed, reducing the risk of corruption on power loss.
 *
 * @dependencies
 * This class relies on the following components:
 * - SDBlockDevice: For low-level communication with the SD card over SPI.
 * - FATFileSystem: For file creation, opening, writing, and closing.
 *
 * Usage:
 * To use the SDWriter class, first mount the SD card, open a new file (e.g. openNextFile()), then call
 * writeFloats(...) or writeByte(...) as needed, and closeFile() when finished.
 *
 * Example:
 * ```
 * SDWriter writer(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN);
 * writer.mount();
 * writer.openNextFile();
 * float data[10] = {1,2,3,...};
 * writer.writeFloats(data, 10);
 * writer.closeFile();
 * writer.unmount();
 * ```
 *
 * @author 
 * @date 02.01.2025
 */

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
