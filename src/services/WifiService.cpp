#include "WiFiService.h"

#ifdef ARDUINO
#ifndef ESP32
#include "stm32wlxx_hal.h"

void GetUID(uint8_t *uidBuffer) {
    uint32_t uidPart1 = *(uint32_t *)0x1FFF7580;  // First 32 bits
    uint32_t uidPart2 = *(uint32_t *)0x1FFF7584;  // Second 32 bits
    uint32_t uidPart3 = *(uint32_t *)0x1FFF7588;  // Third 32 bits

    // Combine into a MAC-like structure
    uidBuffer[5] = (uidPart1 >>  0) & 0xFF;
    uidBuffer[4] = (uidPart1 >>  8) & 0xFF;
    uidBuffer[3] = (uidPart1 >> 16) & 0xFF;
    uidBuffer[2] = (uidPart1 >> 24) & 0xFF;
    uidBuffer[1] = (uidPart2 >>  0) & 0xFF;
    uidBuffer[0] = (uidPart2 >>  8) & 0xFF;

    ESP_LOGI(LM_TAG, "UID @ 0x1FFF7580: 0x%08X", uidPart1);
    ESP_LOGI(LM_TAG, "UID @ 0x1FFF7584: 0x%08X", uidPart2);
    ESP_LOGI(LM_TAG, "UID @ 0x1FFF7588: 0x%08X", uidPart3);
}

#else
#include "WiFi.h"
#endif
#else
#include "hal/efuse_hal.h"
#include "esp_mac.h"
#endif

void WiFiService::init() {
    uint8_t mac[6];
#ifdef ARDUINO
#ifndef ESP32
    GetUID(mac);
#else
    WiFi.macAddress(mac);
#endif
#else
    efuse_hal_get_mac(mac);
#endif
    localAddress = (mac[4] << 8) | mac[5];
    ESP_LOGI(LM_TAG, "Local LoRa address (from WiFi MAC): %X", localAddress);
}

uint16_t WiFiService::getLocalAddress() {
    if (localAddress == 0)
        init();
    return localAddress;
}

uint16_t WiFiService::localAddress = 0;
