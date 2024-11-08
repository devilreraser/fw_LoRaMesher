#ifndef _LORAMESHER_BUILD_OPTIONS_H
#define _LORAMESHER_BUILD_OPTIONS_H

#ifndef ESP32
// #define ESP_LOGV(tag, format, ...) printf("[VERB] %s: " format "\r\n", tag, ##__VA_ARGS__)
// #define ESP_LOGD(tag, format, ...) printf("[DEBG] %s: " format "\r\n", tag, ##__VA_ARGS__)
// #define ESP_LOGI(tag, format, ...) printf("[INFO] %s: " format "\r\n", tag, ##__VA_ARGS__)
// #define ESP_LOGW(tag, format, ...) printf("[WARN] %s: " format "\r\n", tag, ##__VA_ARGS__)
// #define ESP_LOGE(tag, format, ...) printf("[FAIL] %s: " format "\r\n", tag, ##__VA_ARGS__)

#if CORE_DEBUG_LEVEL >= 5
#define ESP_LOGV(tag, format, ...) printf("[VERB] %s: " format "\r\n", tag, ##__VA_ARGS__)
#else
#define ESP_LOGV(tag, format, ...)
#endif

#if CORE_DEBUG_LEVEL >= 4
#define ESP_LOGD(tag, format, ...) printf("[DEBG] %s: " format "\r\n", tag, ##__VA_ARGS__)
#else
#define ESP_LOGD(tag, format, ...)
#endif

#if CORE_DEBUG_LEVEL >= 3
#define ESP_LOGI(tag, format, ...) printf("[INFO] %s: " format "\r\n", tag, ##__VA_ARGS__)
#else
#define ESP_LOGI(tag, format, ...)
#endif

#if CORE_DEBUG_LEVEL >= 2
#define ESP_LOGW(tag, format, ...) printf("[WARN] %s: " format "\r\n", tag, ##__VA_ARGS__)
#else
#define ESP_LOGW(tag, format, ...)
#endif

#if CORE_DEBUG_LEVEL >= 1
#define ESP_LOGE(tag, format, ...) printf("[FAIL] %s: " format "\r\n", tag, ##__VA_ARGS__)
#else
#define ESP_LOGE(tag, format, ...)
#endif

#if CORE_DEBUG_LEVEL == 0
#undef ESP_LOGV
#undef ESP_LOGD
#undef ESP_LOGI
#undef ESP_LOGW
#undef ESP_LOGE

#define ESP_LOGV(tag, format, ...)
#define ESP_LOGD(tag, format, ...)
#define ESP_LOGI(tag, format, ...)
#define ESP_LOGW(tag, format, ...)
#define ESP_LOGE(tag, format, ...)
#endif


#include "FreeRTOS.h"
#include "portmacro.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#endif

#ifdef ARDUINO
#include "Arduino.h"
#else
#include <freertos/FreeRTOS.h>
#include <string.h>
#include <string>
#include <cstdint>
#include <math.h>
#include <esp_log.h>
#include <esp_heap_caps.h>

using namespace std;

// adjust SPI pins as needed
#ifndef SPI_SCK
#define SPI_SCK 9
#endif

#ifndef SPI_MOSI
#define SPI_MOSI 10
#endif

#ifndef SPI_MISO
#define SPI_MISO 11
#endif


#define LOW (0x0)
#define HIGH (0x1)
#define INPUT (0x01)
#define OUTPUT (0x03)
#define RISING (0x01)
#define FALLING (0x02)

#define String std::string
#define F(string_literal) (string_literal)

unsigned long millis();
long random(long howsmall, long howbig);

#endif

size_t getFreeHeap();

extern const char* LM_TAG;
extern const char* LM_VERSION;

// LoRa band definition
// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
#define LM_BAND 868.100F
#define LM_BANDWIDTH 125.0
#define LM_LORASF 7U
#define LM_CODING_RATE 7U
#define LM_PREAMBLE_LENGTH 8U

#if defined STM32
#define LM_POWER 22 /* LoRa-E5 */
#else
#define LM_POWER 6
#endif

#define LM_DUTY_CYCLE 100

//Syncronization Word that identifies the mesh network
#define LM_SYNC_WORD 19U

// Comment this line if you want to remove the crc for each packet
#define LM_ADDCRC_PAYLOAD

// Comment this line if not needed to retransmit broadcast packets inside the library
//#define LM_RETRANSMIT_BROADCASTS


// Routing table max size
#define RTMAXSIZE 256

//MAX packet size per packet in bytes. It could be changed between 13 and 255 bytes. Recommended 100 or less bytes.
//If exceed it will be automatically separated through multiple packets 
//In bytes (226 bytes [UE max allowed with SF7 and 125khz])
//MAX payload size for hello packets = LM_MAX_PACKET_SIZE - 7 bytes of header
//MAX payload size for data packets = LM_MAX_PACKET_SIZE - 7 bytes of header - 2 bytes of via
//MAX payload size for reliable and large packets = LM_MAX_PACKET_SIZE - 7 bytes of header - 2 bytes of via - 3 of control packet
#define LM_MAX_PACKET_SIZE 100

// Packet types
#define NEED_ACK_P  0b00000001 // If any packet contains this bit, the receiver must send an ACK back
#define DATA_P      0b00000010
#define ROUTING_P   0b00000100
#define ACK_P       0b00001010
#define XL_DATA_P   0b00010011
#define LOST_P      0b00100010
#define SYNC_P      0b01000011
#define HELLO_P     0b10000000
#define ROUTING_REQUEST_P 0b00000101

const char* getPacketType(uint8_t type);

// Packet configuration
#define BROADCAST_ADDR 0xFFFF
#define DEFAULT_PRIORITY 20
#define MAX_PRIORITY 40

//Definition Times in seconds
#define HELLO_PACKETS_DELAY 30
#define LM_RT_TIMEOUT HELLO_PACKETS_DELAY*10
#define MIN_TIMEOUT 20

//Maximum times that a sequence of packets reach the timeout
#define MAX_TIMEOUTS 10
#define MAX_RESEND_PACKET 3     /* for one send packets loop */
#define MAX_TIMES_RESEND_PACKET 3     /* for this packet */
#define MAX_TRY_BEFORE_SEND 5

// Routing Table Configuration
#define LM_QUALITY_WINDOWS_SIZE 100 // [0, 255]
#define LM_MAX_HOPS 10
#define LM_REDUCED_FACTOR_HOP_COUNT 0.97
#define LM_MAX_METRIC 255
#define LM_MAX_NODES 255

//Role Types
#define ROLE_DEFAULT 0b00000000
#define ROLE_GATEWAY 0b00000001
//Free Role Types from 0b00000010 to 0b10000000

// Define if is testing
// #define LM_TESTING

#endif