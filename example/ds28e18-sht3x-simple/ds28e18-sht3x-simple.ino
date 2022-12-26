/*

Module:  MCCI_Catena_DS28E18.h

Function:
        Simple example for DS28E18 sensors.

Copyright notice:
        See accompanying license file.

Author:
        Dhinesh Kumar Pitchai, MCCI Corporation	October 2022

*/

#include <stdio.h>
#include <string.h>
#include <MCCI_Catena_DS28E18.h>

/****************************************************************************\
|
|   Manifest constants & typedefs.
|
\****************************************************************************/

using namespace Mcci_Ds28e18;

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
static constexpr uint8_t kOneWirePin = A2;

OneWire oneWire(kOneWirePin);
cDs28e18 gDs28e18(&oneWire);

unsigned char ds28e18RomId[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t sht3xSlaveAddress = 0x88;

uint8_t sht3xBegin[] = { sht3xSlaveAddress, 0x30, 0xA2};
uint8_t sht3xConfiguration[] = { sht3xSlaveAddress, 0x24, 0x00};
uint8_t sht3xQuery[] = { sht3xSlaveAddress | 0x01 };

unsigned short slaveReadLength;
unsigned short slaveResponseStartingAddress;

typedef uint8_t deviceAddress[8];
deviceAddress rhFlexCable;

unsigned short humidityReadingLocation;
unsigned short temperatureReadingLocation;
unsigned short readingLocation;

/****************************************************************************\
|
|   Code.
|
\****************************************************************************/

void printAddress(deviceAddress deviceAddress)
    {
    for (uint8_t i = 0; i < 8; i++)
        {
        // zero pad the address if necessary
        if (deviceAddress[i] < 16) Serial.print("0");
            Serial.print(deviceAddress[i], HEX);
        }
    }

void setup()
    {
    Serial.begin(115200);

    // wait for USB to be attached.
    while (! Serial)
        yield();

    pinMode(D11, OUTPUT);
    digitalWrite(D11, HIGH);
    delay(50);

    gDs28e18.begin();
    unsigned char *statusData, *configData, *setConfigData;

    Serial.println("DS28E18 Simple Test");
    if(!gDs28e18.getAddress(rhFlexCable, 0))
        {
        Serial.println("Not able to get address of DS28E18");
        while(1);
        }
    else
        {
        Serial.print("RH Flex Address: ");
        printAddress(rhFlexCable);
        Serial.println("\n");
        }

    gDs28e18.deviceStatus(statusData);
    gDs28e18.readConfiguration(configData);
    gDs28e18.writeConfiguration(gDs28e18.khz1000, gDs28e18.dontIgnore, gDs28e18.i2c, gDs28e18.mode0);
    gDs28e18.readConfiguration(setConfigData);

    //Build I2C sensor's sequence using sequencer functions
    gDs28e18.clearSequencerPacket();
    gDs28e18.uitilitySensVddOnBuildPacket();
    gDs28e18.uitilityDelayBuildPacket(gDs28e18.delay16);
    gDs28e18.i2cStartBuildPacket();
    gDs28e18.i2cWriteDataBuildPacket(sht3xBegin, sizeof(sht3xBegin));
    gDs28e18.i2cStopBuildPacket();
    gDs28e18.uitilityDelayBuildPacket(gDs28e18.delay32);
    gDs28e18.i2cStartBuildPacket();
    gDs28e18.i2cWriteDataBuildPacket(sht3xConfiguration, sizeof(sht3xConfiguration));
    gDs28e18.i2cStopBuildPacket();
    gDs28e18.uitilityDelayBuildPacket(gDs28e18.delay64);
    gDs28e18.i2cStartBuildPacket();
    gDs28e18.i2cWriteDataBuildPacket(sht3xQuery, sizeof(sht3xQuery));

    slaveReadLength = 6;
    readingLocation = gDs28e18.i2cReadDataNackEndBuildPacket(slaveReadLength);
    gDs28e18.i2cStopBuildPacket();
    gDs28e18.uitilitySensVddOffBuildPacket();
    }

void loop()
    {
    gDs28e18.writeSequencer(0x000, gDs28e18.getSequencerPacket(), gDs28e18.getSequencerPacketSize());

    unsigned char readSequencerData[gDs28e18.getSequencerPacketSize()];
    double temperature;
    double humidity;
    unsigned char temperatureRaw[3];
    unsigned char humidityRaw[3];
    unsigned char measurementRaw[6];

    gDs28e18.readSequencer(0x000, readSequencerData, sizeof(readSequencerData));

    gDs28e18.runSequencer(0x000, gDs28e18.getSequencerPacketSize());

    gDs28e18.readSequencer(0x00, readSequencerData, sizeof(readSequencerData));

    Serial.print("Raw Data:");
    for(int i = readingLocation; i < readingLocation + 6; i++)
        {
        measurementRaw[i - readingLocation] = readSequencerData[i];
        Serial.print(readSequencerData[i], HEX);
        Serial.print(" ");
        }

    temperature = (double)((measurementRaw[0] << 8) | measurementRaw[1])/65536;
    temperature = -45 + (175 * temperature);
    Serial.print("  T(C):");
    Serial.print(temperature);

    humidity = (double)((measurementRaw[3] << 8) | measurementRaw[4])/65536;
    humidity = humidity * 100;
    Serial.print("  RH:");
    Serial.print(humidity);
    Serial.println("%");

    delay(2000);
    }
