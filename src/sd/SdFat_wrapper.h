#include "SdFat.h"
#if SPI_DRIVER_SELECT == 2  // Must be set in SdFat/SdFatConfig.h
#define SD_FAT_TYPE 0
const uint8_t SD_CS_PIN = 25;
const uint8_t SOFT_MISO_PIN = 14;
const uint8_t SOFT_MOSI_PIN = 26;
const uint8_t SOFT_SCK_PIN = 27;
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
#else
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
#endif
#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else
#error Invalid SD_FAT_TYPE
#endif
#else  // SPI_DRIVER_SELECT
#error SPI_DRIVER_SELECT must be two in SdFat/SdFatConfig.h
#endif  //SPI_DRIVER_SELECT

void sd_init(){
    sd.begin(SD_CONFIG);
}

void sd_write(String filename, String data){
    file.open(filename.c_str(), O_RDWR | O_CREAT | O_AT_END);
    file.println(data.c_str());
    file.close();
}