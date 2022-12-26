/*

Module:  MCCI_Catena_DS28E18.cpp

Function:
        namespace McciCatena, class TxBuffer_t;

Copyright notice:
        See accompanying license file.

Author:
        Dhinesh Kumar Pitchai, MCCI Corporation	October 2022

*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include <MCCI_Catena_DS28E18.h>

using namespace Mcci_Ds28e18;

/****************************************************************************\
|
|   Manifest constants & typedefs.
|
\****************************************************************************/

/****************************************************************************\
|
|   Read-only data.
|
\****************************************************************************/

/****************************************************************************\
|
|   Variables.
|
\****************************************************************************/

void cDs28e18::OneWireSetSpeed(cDs28e18::oneWireSpeed spd)
    {
    this->m_oneWire.speed = spd;
    }

cDs28e18::oneWireSpeed cDs28e18::OneWireGetSpeed()
    {
    return this->m_oneWire.speed;
    }

void cDs28e18::OneWireSetROM(cDs28e18::oneWireRomCommand rom)
    {
    this->m_oneWire.romCommand = rom;
    this->m_wire->write((uint8_t)rom);
    }

cDs28e18::oneWireRomCommand cDs28e18::OneWireGetROM()
    {
    return this->m_oneWire.romCommand;
    }

void cDs28e18::OneWireSetRomId(unsigned char *romId)
    {
    for (int i = 0; i < 8; i++)
        {
        this->m_oneWire.romId[i] = romId[i];
        }
    }

unsigned char *cDs28e18::OneWireGetRomId()
    {
    return this->m_oneWire.romId;
    }

void cDs28e18::OneWireInit(void)
    {
    this->m_oneWire.speed = standard;
    }

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire bus
// Return true  : device found, ROM number in ROM_NO buffer
//        false : device not found, end of search
//
int cDs28e18::OneWireNext(unsigned char *romId)
    {
    // leave the search state alone
    return this->m_wire->search(romId);
    }

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus
// Return true  : device found, ROM number in ROM_NO buffer
//        false : no device present
//
int cDs28e18::OneWireFirst(unsigned char *romId)
    {
    // reset the search state
    this->m_wire->reset_search();

    return this->m_wire->search(romId);
    }

/*---------------------------------------------------------------------------*/

cDs28e18::cDs28e18() {}

cDs28e18::cDs28e18(OneWire* oneWire)
    {
    this->m_wire = oneWire;
    }

int cDs28e18::begin()
    {
    OneWireInit();

    unsigned char status[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    unsigned char tempRomId[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    this->setSpeed(standard);
    this->m_wire->skip();

    this->writeGpioConfiguration(control, 0xA5, 0x0F);

    if(OneWireFirst(tempRomId))
        {
        OneWireSetRomId(tempRomId);
        OneWireSetROM(matchRom);

        if(!this->writeGpioConfiguration(control, 0xA5, 0x0F))
            {
            return false;
            }

        if(!this->deviceStatus(status))
            {
            return false;
            }
        else
            {
            }

        while (this->m_wire->search(tempRomId))
            {
            OneWireSetRomId(tempRomId);
            memset(status, 0xFF, sizeof(status));	
            getAddress(tempRomId, 0);	
            Serial.print("\nROM ID (read-2): ");
            for (int i = 0; i < 8; i++)
                {
                Serial.print(tempRomId[i], HEX);
                Serial.print(" ");
                }

            if(!this->writeGpioConfiguration(control, 0xA5, 0x0F))
                {
                return false;
                }

            if(!this->deviceStatus(status))
                {
                return false;
                }
            else
                {
                }
            }
        }
    else
        {
        return false;
        }

    this->clearSequencerPacket();

    return true;
    }

bool cDs28e18::validAddress(const uint8_t* deviceAddress)
    {
    return (this->m_wire->crc8(deviceAddress, 7) == deviceAddress[7]);
    }

bool cDs28e18::getAddress(uint8_t* deviceAddress, uint8_t index)
    {
    uint8_t depth = 0;
    bool fStatus = false;

    if ( !this->m_wire->search(deviceAddress)) 
        {
        this->m_wire->reset_search();
        return false;
        }

    if (validAddress(deviceAddress))
        {
        fStatus = true;
        }

    if (deviceAddress[0] != 0x56)
        {
        getAddress(deviceAddress, index);
        }

    if (fStatus)
        OneWireSetRomId(deviceAddress);

    return fStatus;
    }

    //-----------------------------------------------------------------------------
    //Set desired 1-Wire speed between Standard and Overdrive for both, 1-Wire master and slave.
    //Return '0' if at least one device is detected after a 1-Wire reset is performed on new speed.
    //Return '1' otherwise
int cDs28e18::setSpeed(oneWireSpeed spd)
    {
    int error;

    switch (spd)
        {
        case standard:
            //Set host speed to Standard
            OneWireSetSpeed(standard);

            // do a 1-Wire reset in Standard and catch presence result
            error = this->m_wire->reset();

            break;
        case overdrive:
            //From Standard speed, do a 1-wire reset + Overdrive Skip ROM to set every device on the line to Overdrive
            this->m_wire->reset();
            this->m_wire->write((uint8_t)overdriveSkip);
            delay(40);

            //Set host speed to Overdrive
            OneWireSetSpeed(overdrive);

            // do a 1-Wire reset in Overdrive and catch presence result
            error = this->m_wire->reset();

            break;
        default:
            error = 1;
        }
    return error;
    }

unsigned char *cDs28e18::getSequencerPacket()
    {
    return this->m_ds28e18.sequencerPacket;
    }

int cDs28e18::getSequencerPacketSize()
    {
    return this->m_ds28e18.sequencePacketIdx;
    }

void cDs28e18::clearSequencerPacket()
    {
    memset(this->m_ds28e18.sequencerPacket, 0x00, sizeof(this->m_ds28e18.sequencerPacket));
    this->m_ds28e18.sequencePacketIdx = 0;
    }

unsigned int cDs28e18::calculateCrc16Byte(unsigned char data, unsigned int crc)
    {
    const unsigned char oddParity[] = {0, 1, 1, 0, 1, 0, 0, 1,
                                        1, 0, 0, 1, 0, 1, 1, 0};

    unsigned int data16 = (data ^ crc) & 0xff;
    crc = (crc >> 8) & 0xff;

    if (oddParity[data16 & 0xf] ^ oddParity[data16 >> 4])
        {
        crc ^= 0xc001;
        }

    data16 <<= 6;
    crc ^= data16;
    data16 <<= 1;
    crc ^= data16;

    return crc;
    }

unsigned int cDs28e18::calculateCrc16Block(unsigned char *data, int dataSize, unsigned int crc)
    {
    for (int i = 0; i < dataSize; i++)
        {
        crc = calculateCrc16Byte(data[i], crc);
        }
    return crc;
    }

int cDs28e18::runCommand(deviceCommands command, unsigned char *parameters, int parameterSize, int delayPeriod, unsigned char *resultData)
    {
    uint8_t txPacket[3 + parameterSize];
    uint8_t txPacketCrc16[2];
    unsigned int expectedCrc = 0;
    uint8_t headerResponse[2];
    int resultDataLength;
    uint8_t rxPacketCrc16[2];

    txPacket[0] = start;
    txPacket[1] = 1 + parameterSize;
    txPacket[2] = command;
    if (parameterSize)
        {
        memcpy(&txPacket[3], parameters, parameterSize);
        }

    //Reset pulse + presence
    this->m_wire->reset();

    //Execute ROM Command currently set
    switch(OneWireGetROM())
        {
        case readRom:
            return false;
        case matchRom:
            this->m_wire->write((uint8_t)matchRom);
            this->m_wire->write_bytes((uint8_t*)OneWireGetRomId(), 8);
            break;
        case searchRom:
            return false;
        case skipRom:
            this->m_wire->write((uint8_t)skipRom);
            break;
        case resume:
            this->m_wire->write((uint8_t)resume);
            break;
        case overdriveSkip:
            this->m_wire->write((uint8_t)overdriveSkip);
            break;
        case overdriveMatch:
            this->m_wire->write((uint8_t)overdriveMatch);
            this->m_wire->write_bytes((uint8_t*)OneWireGetRomId(), 8);
            break;
        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    //Write command-specific 1-Wire packet, txPacket
    this->m_wire->write_bytes(txPacket, sizeof(txPacket));

    //Read CRC16 of the txPacket
    this->m_wire->read_bytes(txPacketCrc16, sizeof(txPacketCrc16));

    //Verify CRC16
    expectedCrc = calculateCrc16Block(txPacket, sizeof(txPacket), expectedCrc);
    expectedCrc ^= 0xFFFFU;

    if (expectedCrc != (unsigned int)((txPacketCrc16[1] << 8) | txPacketCrc16[0]))
        {
        // Serial.print("\n1. Error: Invalid CRC16");
        return false;
        }

    //Send Release Byte, 0xAA
    this->m_wire->write((uint8_t)releaseByte);

    // Command-specific delay
    delay(delayPeriod);
    // Serial.print("<Delay: ");
    // Serial.print(delayPeriod);
    // Serial.print(" ms>");

    //Read general command specific 1-Wire packet
    this->m_wire->read_bytes(headerResponse, sizeof(headerResponse)); //Dummy Byte + Length Byte;
    resultDataLength = headerResponse[1];

    if (resultDataLength == 0xFF)
        {
        // Serial.print("\nError: 1-Wire Communication Error");
        return false;
        }

    //Read rest of response
    this->m_wire->read_bytes(resultData, resultDataLength); //Result Byte + Result Data

    //Read CRC16 of the rx_packet
    this->m_wire->read_bytes(rxPacketCrc16, sizeof(rxPacketCrc16));

    //Verify CRC16
    expectedCrc = 0;
    expectedCrc = calculateCrc16Block(&headerResponse[1], sizeof(headerResponse) - 1, expectedCrc);
    expectedCrc = calculateCrc16Block(resultData, resultDataLength, expectedCrc);
    expectedCrc ^= 0xFFFFU;
    if (expectedCrc != (unsigned int)((rxPacketCrc16[1] << 8) | rxPacketCrc16[0]))
        {
        // Serial.print("\n2. Error: Invalid CRC16");
        return false;
        }

    return true;
    }

//---------------------------------------------------------------------------
//-------- Device Function Commands -----------------------------------------
//---------------------------------------------------------------------------
/// Device Function Command: Write Sequencer (11h)
///
/// @param nineBitStartingAddress Target write address
/// @param txData Array of data to be written into the sequencer memory starting from the target write address
/// @param txDataSize Number of elements found in txData array
/// @return
/// true - command succesful @n
/// false - command failed
///
/// @note Use Sequencer Commands functions to help build txData array.
int cDs28e18::writeSequencer(unsigned short nineBitStartingAddress, unsigned char *txData, int txDataSize)
    {
    // Serial.print("\n*Write Sequencer*");

    unsigned char parameters[2 + txDataSize];
    unsigned char response[1];
    unsigned char addressLow;
    unsigned char addressHigh;

    addressLow = nineBitStartingAddress & 0xFF;
    addressHigh = (nineBitStartingAddress >> 8) & 0x01;

    parameters[0] = addressLow;
    parameters[1] = addressHigh;
    memcpy(&parameters[2], &txData[0], txDataSize);

    if (!runCommand(writeSequencerCmd, parameters, sizeof(parameters), SPU_DELAY_tOP, response))
        {
        return false;
        }

    // Parse result byte.
    switch (response[0])
        {
        case pass:
            // Success response.
            break;

        case invalidParameter:
            // Serial.print("\nError: Invalid input or parameter");
            return false;

        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    return true;
    }

//---------------------------------------------------------------------------
/// Device Function Command: Read Sequencer (22h)
///
/// @param nineBitStartingAddress Target read address
/// @param readLength Number of data bytes to be read from the sequencer memory starting from the target read address
/// @param[out] rxData Array of data returned from specified memory address
/// @return
/// true - command succesful @n
/// false - command failed
int cDs28e18::readSequencer(unsigned short nineBitStartingAddress, unsigned char *rxData, unsigned short readLength)
    {
    // Serial.print("\n*Read Sequencer*");

    unsigned char parameters[2];
    int responseLength = 1 + readLength;
    unsigned char response[responseLength];
    unsigned char addressLow;
    unsigned char addressHigh;

    if (readLength == 128)
        {
        readLength = 0;
        }

    addressLow = nineBitStartingAddress & 0xFF;
    addressHigh = (nineBitStartingAddress >> 8) & 0x01;

    parameters[0] = addressLow;
    parameters[1] = (readLength << 1) | addressHigh;

    if (!runCommand(readSequencerCmd, parameters, sizeof(parameters), SPU_DELAY_tOP, response))
        {
        return false;
        }

    // Serial.println("DS28E18 Line : 480");
    memcpy(rxData, &response[1], responseLength - 1);

    // Serial.println("DS28E18 Line : 482");
    // Parse result byte.
    switch (response[0])
        {
        case pass:
            // Success response.
            break;

        case invalidParameter:
            // Serial.print("\nError: Invalid input or parameter");
            return false;

        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    // Serial.println("DS28E18 Line : 499");
    return true;
    }

//---------------------------------------------------------------------------
/// Device Function Command: Run Sequencer (33h)
///
/// @param nineBitStartingAddress Target run address
/// @param runLength Number of data bytes to run from the sequencer memory starting from the target run address
/// @return
/// true - command succesful @n
/// false - command failed
int cDs28e18::runSequencer(unsigned short nineBitStartingAddress, unsigned short runLength)
    {
    // Serial.print("\n*Run Sequencer*");

    unsigned char parameters[3];
    int responseLength = 3;
    unsigned char response[responseLength];
    unsigned char addressLow;
    unsigned char addressHigh;
    unsigned char sequencerLengthLow;
    unsigned char sequencerLengthHigh;
    int totalSequencerCommunicationTime = 0;
    int runSequencerDelay;
    int snackLo;
    int snackHi;
    unsigned short nackOffset;

    if (runLength == 512)
        {
        runLength = 0;
        }

    addressLow = nineBitStartingAddress & 0xFF;
    addressHigh = (nineBitStartingAddress >> 8) & 0x01;
    sequencerLengthLow = ((runLength & 0x7F) << 1);
    sequencerLengthHigh = ((runLength >> 7) & 0x03);

    parameters[0] = addressLow;
    parameters[1] = sequencerLengthLow | addressHigh;
    parameters[2] = sequencerLengthHigh;

    for (float i = 0; i < (runLength / 10); i++)  //add 1ms to Run Sequencer delay for every 10 sequencer commands
        {
        totalSequencerCommunicationTime += 1;
        }

    this->m_ds28e18.totalSequencerDelayTime += this->m_ds28e18.totalSequencerDelayTime * 0.05; // Add ~5% to delay option time for assurance

    runSequencerDelay = SPU_DELAY_tOP + this->m_ds28e18.totalSequencerDelayTime + totalSequencerCommunicationTime;

    if (!runCommand(runSequencerCmd, parameters, sizeof(parameters), runSequencerDelay, response))
        {
        return false;
        }

    // Parse result byte.
    switch (response[0])
        {
        case pass:
            // Success response.
            break;

        case porOccurred:
            // Serial.print("\nError: POR occurred resulting in the command sequencer memory being set to zero");
            return false;

        case executionError:
            // Serial.print("\nError: Execution Error (Sequencer Command packet or packets incorrectly formed)");
            return false;

        case invalidParameter:
            // Serial.print("\nError: Invalid input or parameter");
            return false;

        case nackOccurred:
            snackLo = response[1];
            snackHi = response[2];
            nackOffset = snackLo + (snackHi << 8);
            if (nackOffset == 0)
            {
                nackOffset = 512;
            }
            // Serial.print("\nError: NACK occurred on address: ");
            Serial.print(nackOffset, HEX);

            return false;

        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    return true;
    }

//---------------------------------------------------------------------------
/// Device Function Command: Write Configuration (55h)
///
/// @param spd Desired protocol speed from macros
/// @param iNack Desired iNack configuration from macros
/// @param protocol Desired protocol from macros
/// @param spiMode Desired SPI Mode from macros
/// @return
/// true - command succesful @n
/// false - command failed
int cDs28e18::writeConfiguration(protocolSpeed spd, IgnoreNack iNack, protocol protocol, spiMode spiMode)
    {
    // Serial.print("\n*Write Configuration*");

    unsigned char parameters[1];
    unsigned char response[1];

    parameters[0] = (spiMode << 4) | (protocol << 3) | (iNack << 2) | spd;

    if (!runCommand(writeConfig, parameters, sizeof(parameters), SPU_DELAY_tOP, response))
        {
        return false;
        }

    // Parse result byte.
    switch (response[0])
        {
        case pass:
            // Success response.
            break;

        case invalidParameter:
            // Serial.print("\nError: Invalid input or parameter");
            return false;

        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    return true;
    }

//---------------------------------------------------------------------------
/// Device Function Command: Read Configuration (6Ah)
///
/// @return
/// true - command succesful @n
/// false - command failed
int cDs28e18::readConfiguration(unsigned char *rxData)
    {
    // Serial.print("\n*Read Configuration*");

    unsigned char parameters[0]; //no parameters
    int responseLength = 2;
    unsigned char response[responseLength];

    if (!runCommand(readConfig, parameters, 0, SPU_DELAY_tOP, response))
        {
        return false;
        }

    memcpy(rxData, &response[1], responseLength - 1);

    // Parse result byte.
    switch (response[0])
        {
        case pass:
            // Success response.
            break;

        case invalidParameter:
            // Serial.print("\nError: Invalid input or parameter");
            return false;

        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    return true;
    }

//---------------------------------------------------------------------------
/// Device Function Command: Write GPIO Configuration (83h)
///
/// @param cfgRegTarget Desired GPIO Configuration Register to write to
/// @param gpioHigh Control/Buffer register high byte
/// @param gpioLow Control/Buffer register low byte
/// @return
/// true - command succesful @n
/// false - command failed
///
/// @note Use GPIO Configuration functions to help build gpioHigh/gpioLow parameter.
int cDs28e18::writeGpioConfiguration(targetConfigurationRegister cfgRegTarget, unsigned char gpioHigh, unsigned char gpioLow)
    {
    // Serial.print("\n*Write GPIO Configuration*");

    unsigned char parameters[4];
    unsigned char response[1];

    parameters[0] = cfgRegTarget;
    parameters[1] = 0x03;
    parameters[2] = gpioHigh;
    parameters[3] = gpioLow;

    if (!runCommand(writeGpioConfig, parameters, sizeof(parameters), SPU_DELAY_tOP, response))
        {
        return false;
        }

    // Parse result byte.
    switch (response[0])
        {
        case pass:
            // Success response.
            break;

        case invalidParameter:
            // Serial.print("\nError: Invalid input or parameter");
            return false;

        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    return true;
    }

//---------------------------------------------------------------------------
/// Device Function Command: Read GPIO Configuration (7Ch)
///
/// @param cfgRegTarget Desired GPIO Configuration Register from macros to read from
/// @param rxData[out] Array of 2 bytes to be updated with device current GPIO configuration for gpioHigh and gpioLow
/// @return
/// true - command succesful @n
/// false - command failed
int cDs28e18::readGpioConfiguration(targetConfigurationRegister cfgRegTarget, unsigned char *rxData)
    {
    // Serial.print("\n*Read GPIO Configuration*");

    unsigned char parameters[2];
    int responseLength = 3;
    unsigned char response[responseLength];

    parameters[0] = cfgRegTarget;
    parameters[1] = 0x03;

    if (!runCommand(readGpioConfig, parameters, sizeof(parameters), SPU_DELAY_tOP, response))
        {
        return false;
        }

    memcpy(rxData, &response[1], responseLength - 1);

    // Parse result byte.
    switch (response[0])
        {
        case pass:
            // Success response.
            break;

        case invalidParameter:
            // Serial.print("\nError: Invalid input or parameter");
            return false;

        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    return true;
    }

//---------------------------------------------------------------------------
/// Device Function Command: Device Status (7Ah)
///
/// @param rxData[out] Array of 4 bytes to be updated with devices' status information
/// @return
/// true - command succesful @n
/// false - command failed
int cDs28e18::deviceStatus(unsigned char *rxData)
    {
    // Serial.print("\n*Device Status*");

    unsigned char parameters[0]; //no parameters
    int responseLength = 5;
    unsigned char response[responseLength];

    if (!runCommand(deviceStatusCmd, parameters, 0, SPU_DELAY_tOP, response))
        {
        return false;
        }

    memcpy(rxData, &response[1], responseLength - 1);

    // Parse result byte.
    switch (response[0])
        {
        case pass:
            // Success response.
            break;

        case invalidParameter:
            // Serial.print("\nError: Invalid input or parameter");
            return false;

        default:
            // Serial.print("\nError: 1-Wire Communication Error");
            return false;
        }

    return true;
    }

//---------------------------------------------------------------------------
//-------- Sequencer Commands -----------------------------------------------
//---------------------------------------------------------------------------
/// Sequencer Command: Start (02h).
///
/// Add an I2C Start command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::i2cStartBuildPacket()
    {
    unsigned char i2cStartBuf[1];
    i2cStartBuf[0] = i2cStart;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], i2cStartBuf, sizeof(i2cStartBuf));
    this->m_ds28e18.sequencePacketIdx += sizeof(i2cStartBuf);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: Stop (03h).
///
/// Add an I2C Stop command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::i2cStopBuildPacket()
    {
    unsigned char i2cStopBuf[1];
    i2cStopBuf[0] = i2cStop;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], i2cStopBuf, sizeof(i2cStopBuf));
    this->m_ds28e18.sequencePacketIdx += sizeof(i2cStopBuf);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: Write Data (E3h).
///
/// Add an I2C Write Data command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param i2cData Array with data to be transmitted over the I2C bus
/// @param i2cDataSize Number of elements found in i2cData array
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::i2cWriteDataBuildPacket(uint8_t *i2cData, unsigned char i2cDataSize)
    {
    unsigned char i2cWriteDataBuf[2 + i2cDataSize];
    i2cWriteDataBuf[0] = i2cWriteData;
    i2cWriteDataBuf[1] = i2cDataSize;
    memcpy(&i2cWriteDataBuf[2], i2cData, i2cDataSize);

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], i2cWriteDataBuf, sizeof(i2cWriteDataBuf));
    this->m_ds28e18.sequencePacketIdx += sizeof(i2cWriteDataBuf);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: Read Data (D4h).
///
/// Add an I2C Read Data command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param readBytes Number of bytes to read from the I2C bus
/// @return
/// readArrayFFhStartingAddress - Address where I2C slave response will reside
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short cDs28e18::i2cReadDataBuildPacket(int readBytes)
    {
    unsigned short readArrayFFhStartingAddress = this->m_ds28e18.sequencePacketIdx + 2;
    unsigned char i2cReadDataBuf[2 + readBytes];

    i2cReadDataBuf[0] = i2cReadData;
    if (readBytes == 256)
        {
        i2cReadDataBuf[1] = 0;
        }
    else
        {
        i2cReadDataBuf[1] = readBytes;
        }
    memset(&i2cReadDataBuf[2], 0xFF, readBytes);

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], i2cReadDataBuf, sizeof(i2cReadDataBuf));
    this->m_ds28e18.sequencePacketIdx += sizeof(i2cReadDataBuf);

    return readArrayFFhStartingAddress;
    }

//---------------------------------------------------------------------------
/// Sequencer Command: Read Data w/NACK end (D3h).
///
/// Add an I2C Read Data w/NACK end command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param readBytes Number of bytes to read from the I2C bus
/// @return
/// readArrayFFhStartingAddress - Address where I2C slave response will reside
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short cDs28e18::i2cReadDataNackEndBuildPacket(int readBytes)
    {
    unsigned short readArrayFFhStartingAddress = this->m_ds28e18.sequencePacketIdx + 2;
    unsigned char i2cReadDataWithNackEnd[2 + readBytes];

    i2cReadDataWithNackEnd[0] = i2cReadDataNack;
    if (readBytes == 256)
        {
        i2cReadDataWithNackEnd[1] = 0;
        }
    else
        {
        i2cReadDataWithNackEnd[1] = readBytes;
        }
    memset(&i2cReadDataWithNackEnd[2], 0xFF, readBytes);

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], i2cReadDataWithNackEnd, sizeof(i2cReadDataWithNackEnd));
    this->m_ds28e18.sequencePacketIdx += sizeof(i2cReadDataWithNackEnd);

    return readArrayFFhStartingAddress;
    }

//---------------------------------------------------------------------------
/// Sequencer Command: SPI Write/Read Byte (C0h).
///
/// Add a SPI Write/Read Byte command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param spiWriteData Array with data to be transmitted over the SPI bus. Data not important if only reading.
/// @param spiWriteDataSize Number of elements found in spiWriteData array. Set to 0 if only reading.
/// @param readBytes Number of bytes to read from SPI bus. Set to 0 if only writting.
/// @param fullDuplex Set 'true' when interfacing with a full duplex SPI slave. Otherwise, set 'false'
/// @return
/// readArrayFFhStartingAddress - If reading, address where SPI slave response will reside.
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short cDs28e18::spiWriteReadByteBuildPacket(unsigned char *spiWriteData, unsigned char spiWriteDataSize, int readBytes, bool fullDuplex)
    {
    unsigned short readArrayFFhStartingAddress = 0;
    unsigned char spiWriteReadDataByte[255];
    int idx = 0;

    //command
    spiWriteReadDataByte[idx++] = spiWriteReadByte;

    if (spiWriteDataSize != 0 && readBytes != 0)
        {
        //Write Length
        spiWriteReadDataByte[idx++] = spiWriteDataSize;

        //Read Length
        spiWriteReadDataByte[idx] = readBytes;
        if (!fullDuplex)
            {
            spiWriteReadDataByte[idx] += spiWriteDataSize;
            }
        idx++;

        //Write Array
        for (int i = 0; i < spiWriteDataSize; i++)
            {
            spiWriteReadDataByte[idx++] = spiWriteData[i];
            }

        //Read Array
        if (!fullDuplex)
            {
            memset(&spiWriteReadDataByte[idx], 0xFF, spiWriteDataSize);
            idx += spiWriteDataSize;
            }
        readArrayFFhStartingAddress = idx;
        memset(&spiWriteReadDataByte[idx], 0xFF, readBytes);
        idx += readBytes;
        }

    else if(spiWriteDataSize != 0 && readBytes == 0)
        {
        //Write Length
        spiWriteReadDataByte[idx++] = spiWriteDataSize;

        //Read Length
        spiWriteReadDataByte[idx++] = 0;

        //Write Array
        for (int i = 0; i < spiWriteDataSize; i++)
            {
            spiWriteReadDataByte[idx++] = spiWriteData[i];
            }
        }

    else if(spiWriteDataSize == 0 && readBytes != 0)
        {
        //Write Length
        spiWriteReadDataByte[idx++] = 0;

        //Read Length
        spiWriteReadDataByte[idx++] = readBytes;

        //Read Array
        readArrayFFhStartingAddress = idx;
        memset(&spiWriteReadDataByte[idx], 0xFF, readBytes);
        idx += readBytes;
        }

    else
        {
        //Write Length
        spiWriteReadDataByte[idx++] = 0;

        //Read Length
        spiWriteReadDataByte[idx++] = 0;
        }

    readArrayFFhStartingAddress += this->m_ds28e18.sequencePacketIdx;
    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], spiWriteReadDataByte, idx);
    this->m_ds28e18.sequencePacketIdx += idx;

    return readArrayFFhStartingAddress;
    }

//---------------------------------------------------------------------------
/// Sequencer Command: SPI Write/Read Bit (B0h).
///
/// Add a SPI Write/Read Bit command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param spiWriteData Array with data to be transmitted over the SPI bus. Data not important if only reading.
/// @param spiWriteDataSize Number of elements found in spiWriteData array. Set to 0 if only reading.
/// @param writeBits Number of bits to write to SPI bus. Set to 0 if only reading.
/// @param readBits Number of bits to read from SPI bus. Set to 0 if only writting.
/// @return
/// readArrayFFhStartingAddress - If reading, address where SPI slave response will reside.
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short cDs28e18::spiWriteReadBitBuildPacket(unsigned char *spiWriteData, unsigned char spiWriteDataSize, int writeBits, int readBits)
    {
    unsigned char readBitsInBytes = 0;
    unsigned short readArrayFFhStartingAddress = 0;
    unsigned char spiWriteReadDataBit[255];
    int idx = 0;

    if (readBits > 0 && readBits < 9)
        {
        readBitsInBytes = 1;
        }
    else if (readBits >= 9 && readBits < 17)
        {
        readBitsInBytes = 2;
        }
    else if (readBits >= 17 && readBits < 25)
        {
        readBitsInBytes = 3;
        }
    else if (readBits >= 25 && readBits < 33)
        {
        readBitsInBytes = 4;
        }
    else if (readBits >= 33 && readBits < 41)
        {
        readBitsInBytes = 5;
        }
    else if (readBits >= 41 && readBits < 49)
        {
        readBitsInBytes = 6;
        }
    else if (readBits >= 49 && readBits < 57)
        {
        readBitsInBytes = 7;
        }
    else if (readBits >= 57 && readBits < 65)
        {
        readBitsInBytes = 8;
        }

    //command
    spiWriteReadDataBit[idx++] = spiWriteReadBit;

    if (writeBits != 0 && readBits != 0)
        {
        //Write Length
        spiWriteReadDataBit[idx++] = writeBits;

        //Read Length
        spiWriteReadDataBit[idx++] = readBits;

        //Write Array
        for (int i = 0; i < spiWriteDataSize; i++)
            {
            spiWriteReadDataBit[idx++] = spiWriteData[i];
            }

        //Read Array
        readArrayFFhStartingAddress = idx;
        memset(&spiWriteReadDataBit[idx], 0xFF, readBitsInBytes);
        idx += readBitsInBytes;
        }

    else if(writeBits != 0 && readBits == 0)
        {
        //Write Length
        spiWriteReadDataBit[idx++] = writeBits;

        //Read Length
        spiWriteReadDataBit[idx++] = 0;

        //Write Array
        for (int i = 0; i < spiWriteDataSize; i++)
            {
            spiWriteReadDataBit[idx++] = spiWriteData[i];
            }
        }

    else if(writeBits == 0 && readBits != 0)
        {
        //Write Length
        spiWriteReadDataBit[idx++] = 0;

        //Read Length
        spiWriteReadDataBit[idx++] = readBits;

        //Read Array
        readArrayFFhStartingAddress = idx;
        memset(&spiWriteReadDataBit[idx], 0xFF, readBitsInBytes);
        idx += readBitsInBytes;
        }

    else
        {
        //Write Length
        spiWriteReadDataBit[idx++] = 0;

        //Read Length
        spiWriteReadDataBit[idx++] = 0;
        }

    readArrayFFhStartingAddress += this->m_ds28e18.sequencePacketIdx;
    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], spiWriteReadDataBit, idx);
    this->m_ds28e18.sequencePacketIdx += idx;

    return readArrayFFhStartingAddress;
    }

//---------------------------------------------------------------------------
/// Sequencer Command: SPI SS_High (01h).
///
/// Add a SPI SS_High command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::spiSlaveSelectHighBuildPacket()
    {
    unsigned char spiSSHigh[1];
    spiSSHigh[0] = spiSlaveSelectHigh;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], spiSSHigh, sizeof(spiSSHigh));
    this->m_ds28e18.sequencePacketIdx += sizeof(spiSSHigh);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: SPI SS_Low (80h).
///
/// Add a SPI SS_Low command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::spiSlaveSelectLowBuildPacket()
    {
    unsigned char spiSSLow[1];
    spiSSLow[0] = spiSlaveSelectLow;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], spiSSLow, sizeof(spiSSLow));
    this->m_ds28e18.sequencePacketIdx += sizeof(spiSSLow);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: Delay (DDh).
///
/// Add a Delay command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::uitilityDelayBuildPacket(utilityDelays delayTimeInMs)
    {
    switch (delayTimeInMs)
        {
        case delay1:
            this->m_ds28e18.totalSequencerDelayTime += 1;
            break;
        case delay2:
            this->m_ds28e18.totalSequencerDelayTime += 2;
            break;
        case delay4:
            this->m_ds28e18.totalSequencerDelayTime += 4;
            break;
        case delay8:
            this->m_ds28e18.totalSequencerDelayTime += 8;
            break;
        case delay16:
            this->m_ds28e18.totalSequencerDelayTime += 16;
            break;
        case delay32:
            this->m_ds28e18.totalSequencerDelayTime += 32;
            break;
        case delay64:
            this->m_ds28e18.totalSequencerDelayTime += 64;
            break;
        case delay128:
            this->m_ds28e18.totalSequencerDelayTime += 128;
            break;
        case delay256:
            this->m_ds28e18.totalSequencerDelayTime += 256;
            break;
        case delay512:
            this->m_ds28e18.totalSequencerDelayTime += 512;
            break;
        case delay1024:
            this->m_ds28e18.totalSequencerDelayTime += 1024;
            break;
        case delay2048:
            this->m_ds28e18.totalSequencerDelayTime += 2048;
            break;
        case delay4096:
            this->m_ds28e18.totalSequencerDelayTime += 4096;
            break;
        case delay8192:
            this->m_ds28e18.totalSequencerDelayTime += 8192;
            break;
        case delay16384:
            this->m_ds28e18.totalSequencerDelayTime += 16394;
            break;
        case delay32768:
            this->m_ds28e18.totalSequencerDelayTime += 32768;
            break;
        }

    unsigned char utilityDelayBuf[2];
    utilityDelayBuf[0] = utilityDelay;
    utilityDelayBuf[1] = delayTimeInMs;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], utilityDelayBuf, sizeof(utilityDelayBuf));
    this->m_ds28e18.sequencePacketIdx += sizeof(utilityDelayBuf);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: SENS_VDD On (CCh).
///
/// Add a SENS_VDD On command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::uitilitySensVddOnBuildPacket()
    {
    unsigned char utilitySensVddOn[1];
    utilitySensVddOn[0] = utilitySensorVddOn;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], utilitySensVddOn, sizeof(utilitySensVddOn));
    this->m_ds28e18.sequencePacketIdx += sizeof(utilitySensVddOn);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: SENS_VDD Off (BBh).
///
/// Add a SENS_VDD Off command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::uitilitySensVddOffBuildPacket()
    {
    unsigned char utilitySensVddOff[1];
    utilitySensVddOff[0] = utilitySensorVddOff;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], utilitySensVddOff, sizeof(utilitySensVddOff));
    this->m_ds28e18.sequencePacketIdx += sizeof(utilitySensVddOff);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: gpioBuf Write (D1h).
///
/// Add a gpioBuf Write command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param gpioBuf Buffer register high byte.
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::uitilityGpioBufferWriteBuildPacket(unsigned char gpioBuf)
    {
    unsigned char utilityGpioBufWrite[2];
    utilityGpioBufWrite[0] = utilityGpioBufferWrite;
    utilityGpioBufWrite[1] = gpioBuf;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], utilityGpioBufWrite, sizeof(utilityGpioBufWrite));
    this->m_ds28e18.sequencePacketIdx += sizeof(utilityGpioBufWrite);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: gpioBuf Read (1Dh).
///
/// Add a gpioBuf Read command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.

/// @return readArrayFFhStartingAddress - Starting address where configuration data will reside.
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short cDs28e18::uitilityGpioBufferReadBuildPacket()
    {
    unsigned short readArrayFFhStartingAddress = this->m_ds28e18.sequencePacketIdx + 1;
    unsigned char utilityGpioBufRead[2];
    utilityGpioBufRead[0] = utilityGpioBufferRead;
    utilityGpioBufRead[1] = 0xFF; //gpioBuf

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], utilityGpioBufRead, sizeof(utilityGpioBufRead));
    this->m_ds28e18.sequencePacketIdx += sizeof(utilityGpioBufRead);

    return readArrayFFhStartingAddress;
    }

//---------------------------------------------------------------------------
/// Sequencer Command: GPIO_CNTL Write (E2h).
///
/// Add a GPIO_CNTL Write command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.
///
/// @param gpioCtrlHigh Control register high byte.
/// @param gpioCtrlLow Control register low byte.
/// @return Nothing
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
void cDs28e18::uitilityGpioControlWriteBuildPacket(unsigned char gpioCtrlHigh, unsigned char gpioCtrlLow)
    {
    unsigned char utilityGpioCtrlWrite[3];
    utilityGpioCtrlWrite[0] = utilityGpioControlWrite;
    utilityGpioCtrlWrite[1] = gpioCtrlHigh;
    utilityGpioCtrlWrite[2] = gpioCtrlLow;

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], utilityGpioCtrlWrite, sizeof(utilityGpioCtrlWrite));
    this->m_ds28e18.sequencePacketIdx += sizeof(utilityGpioCtrlWrite);
    }

//---------------------------------------------------------------------------
/// Sequencer Command: GPIO_CNTL Read (2Eh).
///
/// Add a GPIO_CNTL Read command to the writeSequencerPacket. Value of sequencerSize
/// reflects the memory address location where the command will reside.

/// @return readArrayFFhStartingAddress - Starting address where configuration data will reside.
/// @note Must run writeSequencer() in order to set the built sequence into devices' memory.
unsigned short cDs28e18::uitilityGpioControlReadBuildPacket()
    {
    unsigned short readArrayFFhStartingAddress = this->m_ds28e18.sequencePacketIdx + 1;
    unsigned char utilityGpioCtrlRead[3];
    utilityGpioCtrlRead[0] = utilityGpioControlRead;
    utilityGpioCtrlRead[1] = 0xFF; //GPIO_CTRL_HI
    utilityGpioCtrlRead[2] = 0xFF; //GPIO_CTRL_LO

    memcpy(&this->m_ds28e18.sequencerPacket[this->m_ds28e18.sequencePacketIdx], utilityGpioCtrlRead, sizeof(utilityGpioCtrlRead));
    this->m_ds28e18.sequencePacketIdx += sizeof(utilityGpioCtrlRead);

    return readArrayFFhStartingAddress;
    }