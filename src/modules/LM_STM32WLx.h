#pragma once

#include <RadioLib.h>
#include "LM_Module.h"

class LM_STM32WLx : public LM_Module {
public:
#ifdef ARDUINO
    LM_STM32WLx(); // No arguments needed for STM32WLx module in Arduino
#else
    LM_STM32WLx(STM32WLx_Module* module);
#endif

    // Initialization
    int16_t begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord,
                  int8_t power, int16_t preambleLength) override;

    // Transmission and Reception
    int16_t transmit(uint8_t* buffer, size_t length) override;
    int16_t receive(uint8_t* data, size_t len) override;
    int16_t startReceive() override;

    // Channel Scanning
    int16_t scanChannel() override;
    int16_t startChannelScan() override;
    int16_t standby() override;

    // Module Control
    void reset() override;
    int16_t setCRC(bool crc) override;
    size_t getPacketLength() override;
    float getRSSI() override;
    float getSNR() override;
    int16_t readData(uint8_t* buffer, size_t numBytes) override;
    uint32_t getTimeOnAir(size_t length) override;

    // Interrupt Handlers
    void setDioActionForReceiving(void (*action)()) override;
    void setDioActionForReceivingTimeout(void (*action)()) override;
    void setDioActionForScanning(void (*action)()) override;
    void setDioActionForScanningTimeout(void (*action)()) override;
    void clearDioActions() override;

    // Configuration
    int16_t setFrequency(float freq) override;
    int16_t setBandwidth(float bw) override;
    int16_t setSpreadingFactor(uint8_t sf) override;
    int16_t setCodingRate(uint8_t cr) override;
    int16_t setSyncWord(uint8_t syncWord) override;
    int16_t setOutputPower(int8_t power) override;
    int16_t setPreambleLength(int16_t preambleLength) override;

    // Unsupported functions
    int16_t setGain(uint8_t gain) override;  // Dummy implementation
    int16_t setOutputPower(int8_t power, int8_t useRfo) override;  // Dummy implementation

private:
    STM32WLx* module;
};
