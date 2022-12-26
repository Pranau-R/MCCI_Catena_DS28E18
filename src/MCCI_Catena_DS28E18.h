/*

Module:  MCCI_Catena_DS28E18.h

Function:
        namespace McciCatena, class TxBuffer_t;

Copyright notice:
        See accompanying license file.

Author:
        Dhinesh Kumar Pitchai, MCCI Corporation	October 2022

*/

#ifndef _MCCI_CATENA_DS28E18_H_
#define _MCCI_CATENA_DS28E18_H_ /* prevent multiple includes */

#include <Arduino.h>
#include <stdbool.h>
#include <OneWire.h>
#include <stdint.h>

#define SPU_DELAY_tOP   1       //ms

/// \brief namespace for this library
namespace Mcci_Ds28e18 {

/// \brief instance object for cDs28e18
class cDs28e18
    {
public:
    typedef enum
        {
        standard,       /**< 1-Wire Standard Speed */
        overdrive,        /**< 1-Wire Overdrive Speed */
        } oneWireSpeed;

    typedef enum
        {
        readRom = 0x33,
        matchRom = 0x55,
        searchRom = 0xF0,
        skipRom = 0xCC,
        resume = 0xA5,
        overdriveSkip = 0x3C,
        overdriveMatch = 0x69,
        releaseByte = 0XAA,
        } oneWireRomCommand;

    typedef struct
        {
        oneWireSpeed speed; 	/**< Speed type */
        oneWireRomCommand romCommand;
        unsigned char romId[8];
        } oneWire_t;

    typedef enum
        {
        start = 0x66,
        writeSequencerCmd = 0x11,
        readSequencerCmd = 0x22,
        runSequencerCmd = 0x33,
        writeConfig = 0x55,
        readConfig = 0x6A,
        writeGpioConfig = 0x83,
        readGpioConfig = 0x7C,
        deviceStatusCmd = 0x7A,
        } deviceCommands;

    typedef enum
        {
        //I2C
        i2cStart = 0x02,
        i2cStop = 0x03,
        i2cWriteData = 0xE3,
        i2cReadData = 0xD4,
        i2cReadDataNack = 0xD3,

        // SPI
        spiWriteReadByte = 0xC0,
        spiWriteReadBit = 0xB0,
        spiSlaveSelectHigh = 0x01,
        spiSlaveSelectLow = 0x80,

        // Utility
        utilityDelay = 0xDD,
        utilitySensorVddOn = 0xCC,
        utilitySensorVddOff = 0xBB,
        utilityGpioBufferWrite = 0xD1,
        utilityGpioBufferRead = 0x1D,
        utilityGpioControlWrite = 0xE2,
        utilityGpioControlRead = 0x2E,
        } sequencerCommands;

    typedef enum
        {
        porOccurred = 0x44,
        executionError = 0x55,
        invalidParameter = 0x77,
        nackOccurred = 0x88,
        pass = 0xAA,
        } resultByte;

    typedef enum
        {
        khz100,
        khz400,
        khz1000,
        khz2300,
        } protocolSpeed;

    typedef enum
        {
        dontIgnore,
        Ignore,
        } IgnoreNack;

    typedef enum
        {
        i2c,
        spi,
        } protocol;

    typedef enum
        {
        mode0 = 0x00,
        mode3 = 0x03,
        } spiMode;

    typedef enum
        {
        control = 0x0B,
        buffer = 0x0C,
        } targetConfigurationRegister;

    typedef enum
        {
        delay1,
        delay2,
        delay4,
        delay8,
        delay16,
        delay32,
        delay64,
        delay128,
        delay256,
        delay512,
        delay1024,
        delay2048,
        delay4096,
        delay8192,
        delay16384,
        delay32768,
        } utilityDelays;

    typedef struct
        {
        unsigned char sequencerPacket[512];
        int sequencePacketIdx;
        unsigned int totalSequencerDelayTime;
        } ds28e18_t;

    // 1-Wire Error
    int oneWireCommError  = 0xFF;

    // cfgRegTarget Offset
    int gpioCtrlReg = 0x0B;
    int gpioBufReg = 0x0C;

public:
    cDs28e18();
    cDs28e18(OneWire*);

    // One-Wire functions
    void OneWireInit(void);
    void OneWireSetSpeed(oneWireSpeed spd);
    oneWireSpeed OneWireGetSpeed();
    void OneWireSetROM(oneWireRomCommand rom);
    oneWireRomCommand OneWireGetROM();
    void OneWireSetRomId(unsigned char *romId);
    unsigned char *OneWireGetRomId();
    int OneWireNext(unsigned char *romId);
    int OneWireFirst(unsigned char *romId);
    
    // device address
    bool getAddress(uint8_t* deviceAddress, uint8_t index);
    bool validAddress(const uint8_t* deviceAddress);

    // General functions
    int runCommand(cDs28e18::deviceCommands command, unsigned char *parameters,
                    int parameterSize, int delayPeriod, unsigned char *resultData);
    unsigned int calculateCrc16Byte(unsigned char data, unsigned int crc);
    unsigned int calculateCrc16Block(unsigned char *data, int dataSize, unsigned int crc);

    // High Level Functions
    int begin();
    int setSpeed(oneWireSpeed spd);
    unsigned char *getSequencerPacket();
    int getSequencerPacketSize();
    void clearSequencerPacket();

    // Device Function Commands
    int writeSequencer(unsigned short nineBitStartingAddress, unsigned char *txData, int txDataSize);
    int readSequencer(unsigned short nineBitStartingAddress, unsigned char *rxData,  unsigned short readLength);
    int runSequencer(unsigned short nineBitStartingAddress, unsigned short runLength);
    int writeConfiguration(protocolSpeed spd, IgnoreNack iNack, protocol protocol, spiMode spiMode);
    int readConfiguration(unsigned char *rxData);
    int writeGpioConfiguration(targetConfigurationRegister cfgRegTarget, unsigned char gpioHigh, unsigned char gpioLow);
    int readGpioConfiguration(targetConfigurationRegister cfgRegTarget, unsigned char *rxData);
    int deviceStatus(unsigned char *rxData);

    // Sequencer Commands
    void i2cStartBuildPacket();
    void i2cStopBuildPacket();
    void i2cWriteDataBuildPacket(uint8_t *i2cData, unsigned char i2cDataSize);
    unsigned short i2cReadDataBuildPacket(int readBytes);
    unsigned short i2cReadDataNackEndBuildPacket(int readBytes);
    unsigned short spiWriteReadByteBuildPacket(unsigned char *spiWriteData, unsigned char spiWriteDataSize, int readBytes, bool fullDuplex);
    unsigned short spiWriteReadBitBuildPacket(unsigned char *spiWriteData, unsigned char spiWriteDataSize, int writeBits, int readBits);
    void spiSlaveSelectHighBuildPacket();
    void spiSlaveSelectLowBuildPacket();
    void uitilityDelayBuildPacket(utilityDelays delayTimeInMs);
    void uitilitySensVddOnBuildPacket();
    void uitilitySensVddOffBuildPacket();
    void uitilityGpioBufferWriteBuildPacket(unsigned char gpioBuf);
    unsigned short uitilityGpioBufferReadBuildPacket();
    void uitilityGpioControlWriteBuildPacket(unsigned char gpioCtrlHigh, unsigned char gpioCtrlLow);
    unsigned short uitilityGpioControlReadBuildPacket();

private:
    // Take a pointer to one wire instance
    OneWire* m_wire;
    ds28e18_t m_ds28e18;
    oneWire_t m_oneWire;
    };

} // end namespace Mcci_Ds28e18

#endif /* _MCCI_CATENA_DS28E18_H_ */