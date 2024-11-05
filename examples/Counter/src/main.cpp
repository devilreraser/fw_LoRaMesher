#include <Arduino.h>
#include "LoraMesher.h"

#if defined(ESP32)
/*  */
#elif defined(STM32)
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_subghz.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#endif



#include <vector>

#define TAG "main"

#define USE_AS_CONCENTRATOR   0       /* 0:endpoint 1:concentrator */

//priority higher to lower
#define USE_SBC_NodeMCU_ESP32       1
#define USE_CS_1274                 1

#if defined(STM32)

#elif USE_SBC_NodeMCU_ESP32
//Using SBC-NodeMCU-ESP32
#define RGB_RED     26     
#define RGB_ANODE   25     
#define RGB_GREEN   33     
#define RGB_BLUE    32    
#define LORA_ENABLE 15    
#define ENABLE_ON   HIGH
#define ENABLE_OFF  LOW
#define BOARD_LED   13       /* ? */
#define LED_ON      HIGH
#define LED_OFF     LOW
#elif USE_CS_1274 
//Using CS-1274 GW ESP32 
#define BOARD_LED   4       /* I2C_SCL */
#define LED_ON      LOW
#define LED_OFF     HIGH
#else  
//Using LILYGO TTGO T-BEAM v1.1 
#define BOARD_LED   4
#define LED_ON      LOW
#define LED_OFF     HIGH
#endif  


uint16_t id_concentrator = BROADCAST_ADDR;

uint16_t id_list[] = 
{
    0x8C18,
    0x7200,
    0xCD00,
    0x5384,
};

uint16_t rgb_mask[] = 
{   //rgb
    0b100,
    0b010,
    0b001,
    0b110,
};

uint16_t messages[] = 
{
    0,
    0,
    0,
    0,
};


                            //rgb
uint16_t rgb_mask_unknown = 0b101;

#ifdef BOARD_LED
QueueHandle_t uint16LedQueue = nullptr;
#endif


#ifdef RGB_RED
QueueHandle_t uint16LedRGBQueue = nullptr;
#endif

LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
struct dataPacket {
    uint32_t counter = 0;
    uint32_t id_sender = 0;
};

dataPacket* helloPacket = new dataPacket;


// Limit the vector to store a maximum of 16 unique broadcast payloads
std::vector<dataPacket> previousBroadcastPayloads;
const size_t maxBroadcastPayloads = 16;


//below not used used default Serial - left here only for notes how to do this
// #ifndef ESP32
// HardwareSerial Serial1(PB7, PB6); // RX, TX pins
// HardwareSerial& SerialDBG = Serial1; // Create a reference to Serial1
// #else
// HardwareSerial& SerialDBG = Serial; // Create a reference to Serial
// #endif



#ifndef ESP32

// Sub-GHz handle
SUBGHZ_HandleTypeDef hsubghz;


#define LED_RED PA9

// Queue handle and semaphore handle
QueueHandle_t uartQueue;
SemaphoreHandle_t uartSemaphore;

void serial_printf(const char *format, ...);
#endif


void loopTask(void *pvParameters);

#if 0
extern "C" int _write(int file, char *ptr, int len) {
    // Loop through the string and send each character to Serial
    for (int i = 0; i < len; i++) {
        Serial.write(ptr[i]);
    }
    return len; // Return the number of characters written
}
#else
extern "C" int _write(int file, char *ptr, int len) {
    // Copy the buffer to ensure null-termination
    char buffer[129];  // 128 characters + 1 for null-termination
    int copyLen = (len < 128) ? len : 128;  // Limit to buffer size
    memcpy(buffer, ptr, copyLen);
    buffer[copyLen] = '\0';  // Null-terminate the string

    // Use serial_printf to output the buffer
    serial_printf("%s", buffer);

    return len;  // Return the number of characters written
}
#endif

#ifndef ESP32

// Custom printf function that uses the semaphore for thread-safe access
void serial_printf(const char *format, ...) {
    // Wait for semaphore before accessing Serial
    if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
        char buffer[128];  // Buffer to hold the formatted string
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        Serial.print(buffer);  // Send the formatted string to Serial

        // Release the semaphore after printing
        xSemaphoreGive(uartSemaphore);
    }
}

void blinkTask(void *pvParameters) {
    const int ledPin = LED_RED;

    pinMode(ledPin, OUTPUT);
    serial_printf("LED Init\r\n");

    while (1) {
        digitalWrite(ledPin, HIGH);
        serial_printf("LED On\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        //vTaskDelay(10 / portTICK_PERIOD_MS);
        //vTaskDelay(10000);

        digitalWrite(ledPin, LOW);
        serial_printf("LED Off\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        //vTaskDelay(10 / portTICK_PERIOD_MS);
        //vTaskDelay(10000);
    }
}

// Task to dequeue and print data from the UART queue
void printTask(void *pvParameters) {
    char* receivedData;

    while (1) {
        // Wait to receive a pointer to the data from the queue
        if (xQueueReceive(uartQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
            serial_printf("Received: %s\r\n", receivedData);  // Print the received data as a string

            // Free the allocated memory after processing
            vPortFree(receivedData);
        }
    }
}



void initialize_blink_and_print(void)
{
    Serial.begin(115200);
    while (!Serial);      // Wait for Serial to initialize
    Serial.println("");
    Serial.println("");
    Serial.println("UART Initialized.");

    // Create the semaphore for UART access
    uartSemaphore = xSemaphoreCreateMutex();
    if (uartSemaphore == NULL) {
        Serial.println("Error creating semaphore.");
        while (1);  // Halt if semaphore creation fails
    }

    // Initialize the queue to hold pointers to data buffers
    uartQueue = xQueueCreate(10, sizeof(char*));  // Queue holds up to 10 pointers
    if (uartQueue == NULL) {
        Serial.println("Error creating queue.");
        while (1);  // Halt if queue creation fails
    }

    // Create tasks for LED blinking and Serial data handling
    xTaskCreate(blinkTask, "Blink Task", 128, NULL, 1, NULL);
    xTaskCreate(printTask, "Print Task", 256, NULL, 1, NULL);
}

#endif

#ifdef BOARD_LED
//Led flash
void led_Flash(uint16_t flashes, uint16_t delaymS) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
        digitalWrite(BOARD_LED, LED_ON);
        delay(delaymS);
        digitalWrite(BOARD_LED, LED_OFF);
        delay(delaymS);
    }
}
#endif

void printRouteNode(RouteNode* routeNode) {
    Serial.print("Destination: ");
    Serial.print(routeNode->networkNode.address, HEX);  // Destination node address in hexadecimal

    Serial.print(" | Metric: ");
    Serial.print(routeNode->networkNode.metric);  // Metric associated with this route

    Serial.print(" | Next Hop (via): ");
    Serial.print(routeNode->via, HEX);  // Next hop address in hexadecimal

    Serial.print(" | Hop Count: ");
    Serial.print(routeNode->networkNode.hop_count);  // Number of hops to the destination

    Serial.print(" | Received SNR: ");
    Serial.print(routeNode->receivedSNR);  // Signal-to-noise ratio for received packets

    Serial.print(" | Sent SNR: ");
    Serial.print(routeNode->sentSNR);  // Signal-to-noise ratio for sent packets

    Serial.print(" | Received Link Quality: ");
    Serial.print(routeNode->received_link_quality);  // Link quality for received packets

    Serial.print(" | Transmitted Link Quality: ");
    Serial.print(routeNode->transmitted_link_quality);  // Link quality for transmitted packets

    Serial.print(" | SRTT: ");
    Serial.print(routeNode->SRTT);  // Smoothed round-trip time

    Serial.print(" | RTTVAR: ");
    Serial.print(routeNode->RTTVAR);  // Round-trip time variation

    Serial.print(" | Received Metric: ");
    Serial.println(routeNode->receivedMetric);  // Received metric for this route
}

void printRoutingTable() {
    // Get a copy of the routing table
    LM_LinkedList<RouteNode>* routingTable = radio.routingTableListCopy();

    if (routingTable->getLength() > 0) {
        Serial.println("Routing Table:");
        routingTable->each(printRouteNode);  // Apply the print function to each route node
    } else {
        Serial.println("Routing Table is empty.");
    }

    // Delete the copied routing table to prevent memory leaks
    delete routingTable;
    Serial.println();  // Add a blank line for readability
}

/**
 * @brief Print the counter of the packet
 *
 * @param data
 */
void printPacket(dataPacket data) {
    Serial.printf("Packet receive from %04X Nº %3d index\r\n", data.id_sender, data.counter);
}

/**
 * @brief Iterate through the payload of the packet and print the counter of the packet
 *
 * @param packet
 */
void printDataPacket(AppPacket<dataPacket>* packet) {
    Serial.printf("Packet arrived from %04X sz %3d bytes\r\n", packet->src, packet->payloadSize);

    //Get the payload to iterate through it
    dataPacket* dPacket = packet->payload;
    size_t payloadLength = packet->getPayloadLength();

    for (size_t i = 0; i < payloadLength; i++) {
        //Print the packet
        printPacket(dPacket[i]);
    }
}
void printMessageCounts() {
    uint16_t id_spare = 0;
    uint16_t messages_spare = 0;
    Serial.printf("----------------------------------------------\r\n");
    Serial.printf("|  %04X  |  %04X  |  %04X  |  %04X  |  %04X  |\r\n", id_list[0], id_list[1], id_list[2], id_list[3], id_spare);
    Serial.printf("|  %04d  |  %04d  |  %04d  |  %04d  |  %04d  |\r\n", messages[0], messages[1], messages[2], messages[3], messages_spare);
    Serial.printf("----------------------------------------------\r\n");
}
/**
 * @brief Function that process the received packets
 *
 */
void processReceivedPackets(void*) {
    for (;;) {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);

        //Iterate through all the packets inside the Received User Packets Queue
        while (radio.getReceivedQueueSize() > 0) {
            ESP_LOGV(TAG, "ReceivedUserData_TaskHandle notify received");
            ESP_LOGV(TAG, "Queue receiveUserData size: %d", radio.getReceivedQueueSize());

            //Get the first element inside the Received User Packets Queue
            AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();

            //Print the data packet
            printDataPacket(packet);

            bool isUnique = false;

            // Check if the packet is a broadcast message when endpoint
            #if USE_AS_CONCENTRATOR == 0
            if (packet->dst != BROADCAST_ADDR) 
            {
                isUnique = true;
                //to do remove duplicated if is concentrator and not broadcast because could come from several endpoints re-transmitted
            }
            else
            #endif
            {
                // Flag to track if the payload is unique
                isUnique = true;

                dataPacket* helloPacketReceived = packet->payload;

                // Compare with previous broadcast payloads
                for (const auto& previousPayload : previousBroadcastPayloads) {
                    if (memcmp(helloPacketReceived, &previousPayload, sizeof(dataPacket)) == 0) {
                        isUnique = false;
                        ESP_LOGI(TAG, "Duplicate broadcast message detected, skipping retransmission.");
                        break;
                    }
                }
                
                // If unique, retransmit if needed and store in the list
                if (isUnique) {
                    #if USE_AS_CONCENTRATOR == 0
                    ESP_LOGI(TAG, "Retransmitting unique broadcast message...");
                    radio.createPacketAndSend(BROADCAST_ADDR, packet->payload, packet->getPayloadLength());
                    #endif
                    
                    // Check the size of previousBroadcastPayloads and maintain the limit
                    if (previousBroadcastPayloads.size() >= maxBroadcastPayloads) {
                        previousBroadcastPayloads.erase(previousBroadcastPayloads.begin());  // Remove the oldest entry
                    }
                    // Store the new unique payload
                    previousBroadcastPayloads.push_back(*helloPacketReceived);
                }
            }



            if (isUnique)
            {
                uint16_t packet_size = packet->payloadSize;
                uint16_t packet_sender = 0;
                dataPacket* helloPacketReceived = packet->payload;

                if (packet_size < sizeof(dataPacket))
                {
                    ESP_LOGE(TAG, "Error Received packet_size %d < sizeof(dataPacket) %d", packet_size, sizeof(dataPacket));
                }
                else
                {
                    packet_sender = helloPacketReceived->id_sender;
                    ESP_LOGI(TAG, "Received Data from sender. %X", packet_sender);

                    if (packet->dst == BROADCAST_ADDR) {
                        id_concentrator = packet_sender;    //set id_concentrator from broadcast
                    }
                    
                    uint16_t rgb_code = rgb_mask_unknown;
                    for (int i = 0; i < sizeof(id_list)/sizeof(id_list[0]); i++)
                    {
                        if (packet_sender == id_list[i])
                        {
                            if (i < sizeof(rgb_mask)/sizeof(rgb_mask[0]))
                            {
                                rgb_code = rgb_mask[i];
                            }
                            if (i < sizeof(messages)/sizeof(messages[0]))
                            {
                                messages[i]++;
                            }
                            break;
                        }
                    }
                    #ifdef RGB_RED
                    if (xQueueSend(uint16LedRGBQueue, &rgb_code, pdMS_TO_TICKS(0)) == pdPASS) {
                        ESP_LOGV(TAG, "Data sent to queue. %X", rgb_code);
                    } else {
                        ESP_LOGE(TAG, "Failed to send data to queue.");
                    }
                    #endif
                }
            }



            //Delete the packet when used. It is very important to call this function to release the memory of the packet.
            radio.deletePacket(packet);
        }
        // Print message counts for each ID
        printMessageCounts();
    }
}

TaskHandle_t receiveLoRaMessage_Handle = NULL;

/**
 * @brief Create a Receive Messages Task and add it to the LoRaMesher
 *
 */
void createReceiveMessages() {
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void*) 1,
        2,
        &receiveLoRaMessage_Handle);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Error: Receive App Task creation gave error: %d", res);
    }
}

#ifdef RGB_RED
/**
 * @brief Function that process the led indication
 *
 */
void processLedIndcation(void*) {

    for (;;) {
        uint16_t rgb_code =0b000;
        if (xQueueReceive(uint16LedRGBQueue, &rgb_code, pdMS_TO_TICKS(100)) == pdPASS) {
            ESP_LOGV(TAG, "Data received from Led RGB queue: %X", rgb_code);

            //digitalWrite(BOARD_LED, LED_ON);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, LOW);
            if (rgb_code & 0b010) digitalWrite(RGB_GREEN, LOW);
            if (rgb_code & 0b001) digitalWrite(RGB_BLUE, LOW);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, HIGH);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, LOW);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, HIGH);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, LOW);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, HIGH);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, LOW);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, HIGH);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, LOW);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (rgb_code & 0b100) digitalWrite(RGB_RED, HIGH);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            digitalWrite(RGB_GREEN, HIGH);
            digitalWrite(RGB_BLUE, HIGH);
            //digitalWrite(BOARD_LED, LED_OFF);

            vTaskDelay(200 / portTICK_PERIOD_MS);
        } else {
            //Serial.println("No data received.");
        }


    }
    
}

TaskHandle_t ledIndcation_Handle = NULL;

/**
 * @brief Create a Led indication Task
 *
 */
void createLedIndication() {
    int res = xTaskCreate(
        processLedIndcation,
        "Receive App Task",
        4096,
        (void*) 1,
        2,
        &ledIndcation_Handle);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Error: Led Indication Task creation gave error: %d", res);
    }
}
#endif


#ifdef BOARD_LED
/**
 * @brief Function that process the led Send indication
 *
 */
void processLedSendIndcation(void*) {

    for (;;) {
        uint16_t code =0b000;
        if (xQueueReceive(uint16LedQueue, &code, pdMS_TO_TICKS(100)) == pdPASS) {
            ESP_LOGV(TAG, "Data received from Led Send queue: %X", code);

            digitalWrite(BOARD_LED, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            digitalWrite(BOARD_LED, LED_OFF);

            vTaskDelay(200 / portTICK_PERIOD_MS);
        } else {
            //Serial.println("No data received.");
        }


    }
    
}

TaskHandle_t ledSendIndcation_Handle = NULL;
/**
 * @brief Create a Led Send indication Task
 *
 */
void createLedSendIndication() {
    int res = xTaskCreate(
        processLedSendIndcation,
        "Receive App Task",
        4096,
        (void*) 1,
        2,
        &ledSendIndcation_Handle);
    if (res != pdPASS) {
        Serial.printf("Error: Led Send Indication Task creation gave error: %d\r\n", res);
    }
}
#endif


/**
 * @brief Initialize LoRaMesher
 *
 */
void setupLoraMesher() {
    //Get the configuration of the LoRaMesher
    LoraMesher::LoraMesherConfig config = LoraMesher::LoraMesherConfig();

    #if USE_SBC_NodeMCU_ESP32
    //Set the configuration of the LoRaMesher (SBC-NodeMCU-ESP32)
    config.loraCs = 5;      
    config.loraRst = 18;
    config.loraIrq = 2;     /* LoRA_G0 */    
    config.loraIo1 = -1;    /* LoRA_G1 */

    config.loraMISO = 16;   
    config.loraMOSI = 17;   
    config.loraSCK = 4;     
    #elif USE_CS_1274
    //Set the configuration of the LoRaMesher (CS-1274 GW ESP32)
    config.loraCs = 5;      /* ESP32_VSPI_CS */
    config.loraRst = 2;     /* 2 - CAN_TX */
    config.loraIrq = 34;   /* LoRA_G0 */    
    config.loraIo1 = 23;   /* LoRA_G1 */

    config.loraMISO = 36;   /* ESP32_VSPI_MISO */
    config.loraMOSI = 33;   /* ESP32_VSPI_MOSI */
    config.loraSCK = 13;    /* ESP32_VSPI_SCLK */
    #else  
    //Set the configuration of the LoRaMesher (TTGO T-BEAM v1.1)
    config.loraCs = 18;
    config.loraRst = 23;
    config.loraIrq = 26;
    config.loraIo1 = 33;
    config.loraMISO = 12;  
    config.loraMOSI = 13;  
    config.loraSCK = 14;   
    #endif 


    #ifdef STM32
    config.module = LoraMesher::LoraModules::STM32WL_MOD; // For STM32 LoRa module
    #else
    config.module = LoraMesher::LoraModules::SX1276_MOD; // For ESP32 LoRa module
    #endif


    // #ifdef LORA_E5_DEV_BOARD
    // // set RF switch configuration for Nucleo WL55JC1
    // // NOTE: other boards may be different!
    // //       Some boards may not have either LP or HP.
    // //       For those, do not set the LP/HP entry in the table.
    // static const uint32_t rfswitch_pins[] =
    //                         {PC3,  PC4,  PC5, RADIOLIB_NC, RADIOLIB_NC};
    // static const Module::RfSwitchMode_t rfswitch_table[] = {
    //     {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
    //     {STM32WLx::MODE_RX,    {HIGH, HIGH, LOW}},
    //     {STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH}},
    //     {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},
    //     END_OF_MODE_TABLE,
    // };

    // // set RF switch control configuration
    // // this has to be done prior to calling begin()
    // radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    // #endif


    //Init the loramesher with a configuration
    radio.begin(config);


    //Create the receive task and add it to the LoRaMesher
    createReceiveMessages();

    //Set the task handle to the LoRaMesher
    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);


    #if USE_SBC_NodeMCU_ESP32
    // digitalWrite(LORA_ENABLE, ENABLE_OFF);
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    // digitalWrite(LORA_ENABLE, ENABLE_ON);
    #endif


    //Start LoRaMesher
    radio.start();

    Serial.println("Lora initialized");
}


void setup() {
    Serial.begin(115200);


    // HAL initialization
    //HAL_Init();
    //SystemClock_Config();
    // Enable the Sub-GHz radio
    //__HAL_RCC_SUBGHZSPI_CLK_ENABLE();

    // Initialize the Sub-GHz radio
    //HAL_SUBGHZ_Init(&hsubghz);

    #ifndef ESP32
    initialize_blink_and_print();
    #endif


    Serial.println("initBoard");

#ifdef BOARD_LED  
    pinMode(BOARD_LED, OUTPUT); //setup pin as output for indicator LED
#endif

    #if USE_SBC_NodeMCU_ESP32
#ifdef RGB_RED  
    // pinMode(LORA_ENABLE, OUTPUT); //setup pin as output for LoRa EN
    pinMode(RGB_RED, OUTPUT); //setup pin as output for indicator LED
    pinMode(RGB_ANODE, OUTPUT); //setup pin as output for indicator LED
    pinMode(RGB_GREEN, OUTPUT); //setup pin as output for indicator LED
    pinMode(RGB_BLUE, OUTPUT); //setup pin as output for indicator LED  
    digitalWrite(RGB_ANODE, HIGH);
    // delay(100); //in ms
    digitalWrite(RGB_RED, HIGH);
    digitalWrite(RGB_GREEN, HIGH);
    digitalWrite(RGB_BLUE, HIGH);
#endif
    #endif

#ifdef BOARD_LED
    led_Flash(2, 125);          //two quick LED flashes to indicate program start
#endif

#ifdef RGB_RED  
    uint16LedRGBQueue = xQueueCreate(10, sizeof(uint16_t));
    if (uint16LedRGBQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create queue RGB.");
    } else {
        ESP_LOGV(TAG, "Queue RGB created successfully.");

        uint16_t rgb_code = 0b111;  //white
        if (xQueueSend(uint16LedRGBQueue, &rgb_code, pdMS_TO_TICKS(0)) == pdPASS) {
            ESP_LOGV(TAG, "Data sent to queue RGB. %X", rgb_code);
        } else {
            ESP_LOGE(TAG, "Failed to send data to queue RGB.");
        }
    }

    createLedIndication();
#endif


#ifdef BOARD_LED
    uint16LedQueue = xQueueCreate(10, sizeof(uint16_t));
    if (uint16LedQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create queue LED.");
    } else {
        ESP_LOGV(TAG, "Queue LED created successfully.");

        uint16_t code = 0b1; //on
        if (xQueueSend(uint16LedQueue, &code, pdMS_TO_TICKS(0)) == pdPASS) {
            ESP_LOGV(TAG, "Data sent to queue LED. %X", code);
        } else {
            ESP_LOGE(TAG, "Failed to send data to queue LED.");
        }
    }
    createLedSendIndication();
#endif


    //setupLoraMesher();



#ifdef RGB_RED  
    uint16_t rgb_code = 0b111;  //white
    if (xQueueSend(uint16LedRGBQueue, &rgb_code, pdMS_TO_TICKS(0)) == pdPASS) {
        ESP_LOGV(TAG, "Data sent to queue RGB. %X", rgb_code);
    } else {
        ESP_LOGE(TAG, "Failed to send data to queue RGB.");
    }
#endif


#ifdef BOARD_LED
    uint16_t code = 0b1;  //on
    if (xQueueSend(uint16LedQueue, &code, pdMS_TO_TICKS(0)) == pdPASS) {
        ESP_LOGV(TAG, "Data sent to queue LED. %X", code);
    } else {
        ESP_LOGE(TAG, "Failed to send data to queue LED.");
    }
#endif

    helloPacket->id_sender = radio.getLocalAddress();


    // Create the main task
    xTaskCreate(
        loopTask,          // Task function
        "LoopTask",        // Name of the task
        1024,              // Stack size (adjust as needed)
        NULL,              // Parameters (not used here)
        1,                 // Task priority (1 is low priority)
        NULL               // Task handle (optional, not needed here)
    );


    #ifndef ESP32
    vTaskStartScheduler();
    #endif

}

void loopTask(void *pvParameters) {
    // This replaces the `loop()` function
    bool concentrator = USE_AS_CONCENTRATOR;
    Serial.printf("USE_AS_CONCENTRATOR: %s\r\n", (concentrator)?"TRUE":"FALSE");
    Serial.printf("LORA_IDENTIFICATION: 0x%04X\r\n", radio.getLocalAddress());
    for (;;) {
        if (concentrator)
        {
            ESP_LOGV(TAG, "Send packet %d\r\n", dataCounter);

            helloPacket->counter = dataCounter++;

            //Create packet and send it.
            radio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);
            #ifdef BOARD_LED
            uint16_t code = 0b1;  //on
            if (xQueueSend(uint16LedQueue, &code, pdMS_TO_TICKS(0)) == pdPASS) {
                ESP_LOGV(TAG, "Data sent to queue LED. %X", code);
            } else {
                ESP_LOGE(TAG, "Failed to send data to queue LED.");
            }
            #endif
            // Check the size of previousBroadcastPayloads and maintain the limit
            if (previousBroadcastPayloads.size() >= maxBroadcastPayloads) {
                previousBroadcastPayloads.erase(previousBroadcastPayloads.begin());  // Remove the oldest entry
            }
            // Store the new unique payload
            previousBroadcastPayloads.push_back(*helloPacket);
            ESP_LOGV(TAG, "Data sent to queue from this sender: %X", helloPacket->id_sender);

        }
        else
        {
            if (id_concentrator != BROADCAST_ADDR)
            {
                ESP_LOGV(TAG, "Send packet %d\r\n", dataCounter);

                helloPacket->counter = dataCounter++;

                //Create packet and send it.
                radio.createPacketAndSend(id_concentrator, helloPacket, 1);
                #ifdef BOARD_LED
                uint16_t code = 0b1;  //on
                if (xQueueSend(uint16LedQueue, &code, pdMS_TO_TICKS(0)) == pdPASS) {
                    ESP_LOGV(TAG, "Data sent to queue LED. %X", code);
                } else {
                    ESP_LOGE(TAG, "Failed to send data to queue LED.");
                }
                #endif
            }
            else
            {
                ESP_LOGV(TAG, "Data sent to queue skipped");
            }
        }


        printRoutingTable();

        ESP_LOGV(TAG, "routingTableSize                 %d", radio.routingTableSize());
        ESP_LOGV(TAG, "getLocalAddress                  %X", radio.getLocalAddress());
        ESP_LOGV(TAG, "getReceivedDataPacketsNum        %d", radio.getReceivedDataPacketsNum());
        ESP_LOGV(TAG, "getSendPacketsNum                %d", radio.getSendPacketsNum());
        ESP_LOGV(TAG, "getReceivedHelloPacketsNum       %d", radio.getReceivedHelloPacketsNum());
        ESP_LOGV(TAG, "getSentHelloPacketsNum           %d", radio.getSentHelloPacketsNum());
        ESP_LOGV(TAG, "getReceivedBroadcastPacketsNum   %d", radio.getReceivedBroadcastPacketsNum());
        ESP_LOGV(TAG, "getForwardedPacketsNum           %d", radio.getForwardedPacketsNum());
        ESP_LOGV(TAG, "getDataPacketsForMeNum           %d", radio.getDataPacketsForMeNum());
        ESP_LOGV(TAG, "getReceivedIAmViaNum             %d", radio.getReceivedIAmViaNum());
        ESP_LOGV(TAG, "getDestinyUnreachableNum         %d", radio.getDestinyUnreachableNum());
        ESP_LOGV(TAG, "getReceivedNotForMe              %d", radio.getReceivedNotForMe());
        ESP_LOGV(TAG, "getReceivedPayloadBytes          %d", radio.getReceivedPayloadBytes());
        ESP_LOGV(TAG, "getReceivedControlBytes          %d", radio.getReceivedControlBytes());
        ESP_LOGV(TAG, "getSentPayloadBytes              %d", radio.getSentPayloadBytes());
        ESP_LOGV(TAG, "getSentControlBytes              %d", radio.getSentControlBytes());

        #if USE_AS_CONCENTRATOR
        //Wait 5 seconds to send the next packet
        vTaskDelay(20000 / portTICK_PERIOD_MS);
        //delay(2000);
        #else
        //Wait 5 seconds to send the next packet
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        //delay(5000);
        #endif

        // // Use semaphore to make Serial output thread-safe
        // if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
        //     Serial.print("Loop: ");
        //     Serial.print(millis());  // Print elapsed time in milliseconds
        //     Serial.println(" ms");
        //     Serial.print("portTICK_PERIOD_MS: ");
        //     Serial.println(portTICK_PERIOD_MS);  
        //     Serial.print("configTICK_RATE_HZ: ");
        //     Serial.println(configTICK_RATE_HZ);  
        //     Serial.print("configCPU_CLOCK_HZ: ");
        //     Serial.println(configCPU_CLOCK_HZ);  
        //     xSemaphoreGive(uartSemaphore);
        // }
    }
}

void loop() {

    for (;;) {
        delay(5000);
        if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.print("Loop: ");
            Serial.print(millis());  // Print elapsed time in milliseconds
            Serial.println(" ms");
            Serial.print("portTICK_PERIOD_MS: ");
            Serial.println(portTICK_PERIOD_MS);  
            Serial.print("configTICK_RATE_HZ: ");
            Serial.println(configTICK_RATE_HZ);  
            Serial.print("configCPU_CLOCK_HZ: ");
            Serial.println(configCPU_CLOCK_HZ);  
            xSemaphoreGive(uartSemaphore);
        }
    }
}