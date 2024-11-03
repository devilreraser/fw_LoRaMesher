#include "LM_STM32WLx.h"

#ifdef ARDUINO
LM_STM32WLx::LM_STM32WLx(STM32WLx_Module* radioModule) {
    module = new STM32WLx(radioModule);
}
#else 
LM_STM32WLx::LM_STM32WLx(Module* mod) {
    module = new STM32WLx(mod);
}
#endif

int16_t LM_STM32WLx::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, int16_t preambleLength) {
    int16_t state = module->begin(freq, bw, sf, cr, syncWord, power, preambleLength);
    if (state == -706 || state == -707)
        // tcxoVoltage – TCXO reference voltage to be set on DIO3. Defaults to 1.6 V. 
        // If you are seeing -706/-707 error codes, it likely means you are using non-0 value for module with XTAL. 
        // To use XTAL, either set this value to 0, or set SX126x::XTAL to true.
        state = module->begin(freq, bw, sf, cr, syncWord, power, preambleLength, 0);

    return state;
}

int16_t LM_STM32WLx::receive(uint8_t* data, size_t len) {
    return module->receive(data, len);
}

int16_t LM_STM32WLx::startReceive() {
    return module->startReceive();
}

int16_t LM_STM32WLx::scanChannel() {
    return module->scanChannel();
}

int16_t LM_STM32WLx::startChannelScan() {
    return module->startChannelScan();
}

int16_t LM_STM32WLx::standby() {
    return module->standby();
}

void LM_STM32WLx::reset() {
    module->reset();
}

int16_t LM_STM32WLx::setCRC(bool crc) {
    if (crc)
        return module->setCRC(2);
    else
        return module->setCRC(0);
}

size_t LM_STM32WLx::getPacketLength() {
    return module->getPacketLength();
}

float LM_STM32WLx::getRSSI() {
    return module->getRSSI();
}

float LM_STM32WLx::getSNR() {
    return module->getSNR();
}

int16_t LM_STM32WLx::readData(uint8_t* buffer, size_t numBytes) {
    return module->readData(buffer, numBytes);
}

int16_t LM_STM32WLx::transmit(uint8_t* buffer, size_t length) {
    return module->transmit(buffer, length);
}

uint32_t LM_STM32WLx::getTimeOnAir(size_t length) {
    return module->getTimeOnAir(length);
}

void LM_STM32WLx::setDioActionForReceiving(void (*action)()) {
    module->setDio1Action(action);
}

void LM_STM32WLx::setDioActionForReceivingTimeout(void(*action)()) {
    return;
}

void LM_STM32WLx::setDioActionForScanning(void(*action)()) {
    return;
}

void LM_STM32WLx::setDioActionForScanningTimeout(void(*action)()) {
    return;
}

void LM_STM32WLx::clearDioActions() {
    module->clearDio1Action();
}

int16_t LM_STM32WLx::setFrequency(float freq) {
    return module->setFrequency(freq);
}

int16_t LM_STM32WLx::setBandwidth(float bw) {
    return module->setBandwidth(bw);
}

int16_t LM_STM32WLx::setSpreadingFactor(uint8_t sf) {
    return module->setSpreadingFactor(sf);
}

int16_t LM_STM32WLx::setCodingRate(uint8_t cr) {
    return module->setCodingRate(cr);
}

int16_t LM_STM32WLx::setSyncWord(uint8_t syncWord) {
    return module->setSyncWord(syncWord);
}

int16_t LM_STM32WLx::setOutputPower(int8_t power) {
    return module->setOutputPower(power);
}

int16_t LM_STM32WLx::setPreambleLength(int16_t preambleLength) {
    return module->setPreambleLength(preambleLength);
}

int16_t LM_STM32WLx::setGain(uint8_t gain) {
    return 0;
}

int16_t LM_STM32WLx::setOutputPower(int8_t power, int8_t useRfo) {
    return module->setOutputPower(power);
}