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

/* Section below For Task Debugging - Comment out for normal operation */
//#define DEBUG_NO_USE_RECEIVED_APP_MESSAGES      

#include <vector>

#define TAG "main"

#define USE_AS_CONCENTRATOR   1       /* 0:endpoint 1:concentrator */

/* Comment/Comment out Following */
//#define DEBUG_USE_BLINK_TASK

//priority higher to lower
#define USE_SBC_NodeMCU_ESP32       1
#define USE_CS_1274                 1

#if defined(STM32)
#define USE_RED_LED_SEEED_GROVE_LORA_E5
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

#define USE_BOARDS_DYNAMIC_STATISTICS   1
#define BOARDS_DYNAMIC_COUNT_MAX        10
uint16_t boards_dynamic_count = 0;
uint16_t boards_dynamic_id_list[BOARDS_DYNAMIC_COUNT_MAX] = {0};
uint16_t boards_dynamic_id_mess[BOARDS_DYNAMIC_COUNT_MAX] = {0};


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




/* circular buffer begin */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CIRCULAR_BUFFER_SIZE 2048  // Define the total buffer size

typedef struct {
    char data[CIRCULAR_BUFFER_SIZE];  // Fixed circular buffer
    int head;                // Write position
    int tail;                // Read/free position
    int count;               // Used bytes in buffer
} CircularMallocBuffer;

typedef struct {
    int size;                // Size of the allocated block
    bool in_use;             // Status of the block
} BlockHeader;

static CircularMallocBuffer buffer_uart_printf;

void initBuffer(CircularMallocBuffer* buffer) {
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
}

void* circular_malloc(CircularMallocBuffer* buffer, int size) {
    if (size <= 0 || size + sizeof(BlockHeader) > CIRCULAR_BUFFER_SIZE) {
        return NULL;  // Invalid size or too large for buffer
    }

    int totalSize = size + sizeof(BlockHeader);  // BlockHeader + data
    if (buffer->count + totalSize > CIRCULAR_BUFFER_SIZE) {
        return NULL;  // Not enough space
    }

    // Wrap head if needed
    if (buffer->head + totalSize > CIRCULAR_BUFFER_SIZE) {
        if (buffer->tail > totalSize) {
            buffer->head = 0;
        } else {
            return NULL;  // Can't wrap, not enough space
        }
    }

    // Initialize the block header at the head
    BlockHeader* header = (BlockHeader*)&buffer->data[buffer->head];
    header->size = size;
    header->in_use = true;

    // Calculate the location of the user data
    void* dataPtr = (void*)((uint8_t*)header + sizeof(BlockHeader));

    // Update the head and count
    buffer->head = (buffer->head + totalSize) % CIRCULAR_BUFFER_SIZE;
    buffer->count += totalSize;

    return dataPtr;
}

void circular_free(CircularMallocBuffer* buffer, void* ptr) {
    if (ptr == NULL) {
        return;
    }

    // Get the block header for the given pointer
    BlockHeader* header = (BlockHeader*)((uint8_t*)ptr - sizeof(BlockHeader));

    if (header->in_use) {
        // Mark the block as free
        header->in_use = false;
        buffer->count -= (header->size + sizeof(BlockHeader));

        // Move tail forward if this block is the current tail
        while (buffer->count > 0) {
            BlockHeader* currentHeader = (BlockHeader*)&buffer->data[buffer->tail];
            if (currentHeader->in_use) {
                break;
            }
            buffer->tail = (buffer->tail + currentHeader->size + sizeof(BlockHeader)) % CIRCULAR_BUFFER_SIZE;
        }
    }
}
/* circular buffer final */




/* circular queue start */

#define CIRCULAR_QUEUE_SIZE 16  // Define the maximum number of pointers in the queue

typedef struct {
    void *data[CIRCULAR_QUEUE_SIZE];  // Array to hold pointers
    int head;                         // Index of the next element to dequeue
    int tail;                         // Index where the next element will be enqueued
    int count;                        // Number of items currently in the queue
} CircularQueue;

CircularQueue circularQueue;

// Initialize the queue
void CircularQueue_Init(CircularQueue *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

// Check if the queue is full
bool CircularQueue_IsFull(CircularQueue *queue) {
    return queue->count == CIRCULAR_QUEUE_SIZE;
}

// Check if the queue is empty
bool CircularQueue_IsEmpty(CircularQueue *queue) {
    return queue->count == 0;
}

// Enqueue an item (pointer) into the queue
bool CircularQueue_Enqueue(CircularQueue *queue, void *item) {
    if (CircularQueue_IsFull(queue)) {
        // Handle overflow if needed, e.g., return false or discard oldest item
        return false;
    }

    queue->data[queue->tail] = item;
    queue->tail = (queue->tail + 1) % CIRCULAR_QUEUE_SIZE;
    queue->count++;
    return true;
}

// Dequeue an item (pointer) from the queue
bool CircularQueue_Dequeue(CircularQueue *queue, void **item) {
    if (CircularQueue_IsEmpty(queue)) {
        return false;  // No item to dequeue
    }

    *item = queue->data[queue->head];
    queue->head = (queue->head + 1) % CIRCULAR_QUEUE_SIZE;
    queue->count--;
    return true;
}
/* circular queue final */




#define BUFFER_SIZE 1024
#define QUEUE_LENGTH 96

static QueueHandle_t serialQueue = NULL;
static SemaphoreHandle_t serialSemaphore = NULL;

#define SERIAL_PRINTF_WAIT_QUEUE_SEND_TICKS pdMS_TO_TICKS(0)
static uint32_t u32SkippedPrintfMemMalloc = 0;
static uint32_t u32SkippedPrintfMemMallocBytes = 0;
static uint32_t u32SkippedSerialQueueNull = 0;
static uint32_t u32SkippedSerialQueueSend = 0;
static uint32_t u32PrintfReceiveEventFlag = 0;
static uint32_t u32SkippedPrintfCirMalloc = 0;
static uint32_t u32SkippedPrintfCirMallocBytes = 0;
static uint32_t u32SkippedCircleQueueSend = 0;
static uint32_t u32SkippedPrintfMemMallocFromCircle = 0;
static uint32_t u32SkippedPrintfMemMallocBytesFromCircle = 0;
static uint32_t u32SkippedSerialQueueSendFromCircle = 0;


extern "C" int _write(int file, char *ptr, int len) {
    // Ensure the length does not exceed BUFFER_SIZE
    int copyLen = (len < BUFFER_SIZE) ? len : BUFFER_SIZE;

    bool printOnReceiveFlag = false;

    if (serialQueue == NULL) 
    {
        u32SkippedSerialQueueNull++;
        return -1;
    }
    if (radio.getOnReceiveEventsFlag())
    {
        u32PrintfReceiveEventFlag++;
        //return -1;
        printOnReceiveFlag = true;
    }

    // Allocate memory for the message
    char *buffer = NULL;
    if (printOnReceiveFlag == false)
    {
        buffer = (char *)pvPortMalloc(copyLen + 1); //this halts if from event onReceive
        if (buffer == NULL) {
            // Handle memory allocation failure
            u32SkippedPrintfMemMalloc++;
            u32SkippedPrintfMemMallocBytes += len;
            return -1;
        }
    }
    else
    {
        buffer = (char *)circular_malloc(&buffer_uart_printf, copyLen + 1);
        if (buffer == NULL) {
            // Handle memory allocation failure
            u32SkippedPrintfCirMalloc++;
            u32SkippedPrintfCirMallocBytes += len;
            return -1;
        }
    }

    // Copy data and null-terminate
    memcpy(buffer, ptr, copyLen);
    buffer[copyLen] = '\0';


    if (printOnReceiveFlag == false)
    {
        char *bufferCopyOld = NULL;
        while(CircularQueue_Dequeue(&circularQueue, (void**)&bufferCopyOld) == pdPASS)
        {
            if (bufferCopyOld)
            {
                int copyLenLoop = strlen(bufferCopyOld);
                char* bufferCopyNew = (char *)pvPortMalloc(copyLenLoop + 1); 
                if (bufferCopyNew == NULL) {
                    // Handle memory allocation failure
                    u32SkippedPrintfMemMallocFromCircle++;
                    u32SkippedPrintfMemMallocBytesFromCircle += len;
                    circular_free(&buffer_uart_printf, bufferCopyOld);
                    
                }
                else
                {
                    memcpy(bufferCopyNew, bufferCopyOld, copyLenLoop);
                    bufferCopyNew[copyLenLoop] = '\0';
                    circular_free(&buffer_uart_printf, bufferCopyOld);
                    if (xQueueSend(serialQueue, &bufferCopyNew, SERIAL_PRINTF_WAIT_QUEUE_SEND_TICKS) != pdPASS) {
                        u32SkippedSerialQueueSendFromCircle++;
                        vPortFree(bufferCopyNew);  // Free the buffer
                    }
                }
            }
        }

        if (xQueueSend(serialQueue, &buffer, SERIAL_PRINTF_WAIT_QUEUE_SEND_TICKS) != pdPASS) {
            u32SkippedSerialQueueSend++;
            vPortFree(buffer);  // Free the buffer
            return -1;
        }
    }
    else
    {
        if (CircularQueue_Enqueue(&circularQueue, buffer) != pdPASS) {
            u32SkippedCircleQueueSend++;
            circular_free(&buffer_uart_printf, buffer);  // Free the buffer
            return -1;
        }
    }


    return len;  // Return the number of characters written
}

void serialTask(void *pvParameters) {
    char *msg;



    // Create the semaphore for UART print access
    serialSemaphore = xSemaphoreCreateMutex();
    if (serialSemaphore == NULL) {
        Serial.println("Error creating serial print semaphore.");
        vTaskDelete(NULL);
        return;
    }

    Serial.print("serialTask entered\r\n");

    xSemaphoreGive(serialSemaphore);
    printf("printf through queue Test Message\r\n");

    #define LOOPS_PRINT_SERIAL_TASK_STACK  32
    int print_stack_counter = LOOPS_PRINT_SERIAL_TASK_STACK;

    for (;;) {
        // Wait for a message to be available
        if (xQueueReceive(serialQueue, &msg, portMAX_DELAY) == pdPASS) 
        {
            print_stack_counter++;
            if (print_stack_counter >= LOOPS_PRINT_SERIAL_TASK_STACK)
            {
                print_stack_counter = 0;
                ESP_LOGV("main", "Stack space unused after entering the task serialTask: %d", uxTaskGetStackHighWaterMark(NULL));
                ESP_LOGV("main", "Free heap: %d", getFreeHeap());
            }

            // Output the message
            if (xSemaphoreTake(serialSemaphore, portMAX_DELAY) == pdTRUE) {
                Serial.print(msg);
                xSemaphoreGive(serialSemaphore);
            }

            // Free the allocated memory
            vPortFree(msg);
        }

    }
}



#ifdef DEBUG_USE_BLINK_TASK
void blinkTask(void *pvParameters) {
    const int ledPin = LED_BUILTIN;

    pinMode(ledPin, OUTPUT);

    #define LOOPS_PRINT_BLINK_TASK_STACK    32
    int print_stack_counter = LOOPS_PRINT_BLINK_TASK_STACK;

    while (1) 
    {
        print_stack_counter++;
        if (print_stack_counter >= LOOPS_PRINT_BLINK_TASK_STACK)
        {
            print_stack_counter = 0;
            ESP_LOGV("main", "Stack space unused after entering the task blinkTask: %d", uxTaskGetStackHighWaterMark(NULL));
            ESP_LOGV("main", "Free heap: %d", getFreeHeap());
        }
        digitalWrite(ledPin, HIGH);
        printf("LED On\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));

        digitalWrite(ledPin, LOW);
        printf("LED Off\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif



//below not used used default Serial - left here only for notes how to do this
// #ifndef ESP32
// HardwareSerial Serial1(PB7, PB6); // RX, TX pins
// HardwareSerial& SerialDBG = Serial1; // Create a reference to Serial1
// #else
// HardwareSerial& SerialDBG = Serial; // Create a reference to Serial
// #endif



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






#ifdef USE_RED_LED_SEEED_GROVE_LORA_E5

typedef enum 
{
    CODE_IDLE,
    CODE_RX_MESSAGE,    //1 fast blink
    CODE_TX_MESSAGE,    //1 slow blink
//    CODE_INIT_START,    //2 fast blinks
//    CODE_INIT_FINAL,    //3 fast blinks
}e_LoRaE5IndicationCode;

TaskHandle_t ledLoRaE5Indication_Handle = NULL;
QueueHandle_t uint16LedLoRaE5IndicationQueue = NULL;

void processLedLoRaE5Indication(void*) {
    const int ledPin = LED_BUILTIN;
    const int ledOn  = 0;
    const int ledOff = 1;

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, ledOff);

    for (;;) {
        uint16_t code =0b000;
        if (xQueueReceive(uint16LedLoRaE5IndicationQueue, &code, pdMS_TO_TICKS(100)) == pdPASS) {
            ESP_LOGV(TAG, "Data received from Led LoRa E5 Indication queue: %X", code);

            ESP_LOGV("main", "Stack space unused after entering the task processLedLoRaE5Indication: %d", uxTaskGetStackHighWaterMark(NULL));
            ESP_LOGV("main", "Free heap: %d", getFreeHeap());

            switch(code)
            {
                case CODE_RX_MESSAGE:
                    digitalWrite(ledPin, ledOn);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    digitalWrite(ledPin, ledOff);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    break;

                case CODE_TX_MESSAGE:
                    digitalWrite(ledPin, ledOn);
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    digitalWrite(ledPin, ledOff);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    break;

                // case CODE_INIT_START:
                //     digitalWrite(ledPin, ledOn);
                //     vTaskDelay(100 / portTICK_PERIOD_MS);
                //     digitalWrite(ledPin, ledOff);
                //     vTaskDelay(100 / portTICK_PERIOD_MS);
                //     digitalWrite(ledPin, ledOn);
                //     vTaskDelay(100 / portTICK_PERIOD_MS);
                //     digitalWrite(ledPin, ledOff);
                //     vTaskDelay(2000 / portTICK_PERIOD_MS);
                //     break;

                default:
                    //vTaskDelay(100 / portTICK_PERIOD_MS);
                    break;
            }

            //vTaskDelay(1000 / portTICK_PERIOD_MS);
            
        } else {
            //printf("No data received.\r\n");
        }


    }
    
}
void createLedLoRaE5Indication() {
    int res = xTaskCreate(
        processLedLoRaE5Indication,
        "Led LoRa E5 Indication Task",
        256,
        (void*) 1,
        2,
        &ledLoRaE5Indication_Handle);
    if (res != pdPASS) {
        printf("Error: Led LoRa E5 Indication Task creation gave error: %d\r\n", res);
    }
}

void sendLedLoRaE5Indication(e_LoRaE5IndicationCode code)
{
    uint16_t led_code = code;
    if (xQueueSend(uint16LedLoRaE5IndicationQueue, &led_code, pdMS_TO_TICKS(0)) == pdPASS) {
        ESP_LOGV(TAG, "Data sent to queue Led LoRa E5 Indication. %X", led_code);
    } else {
        ESP_LOGE(TAG, "Failed to send data to queue Led LoRa E5 Indication.");
    }
}

void initLedLoRaE5Indication(void)
{
    uint16LedLoRaE5IndicationQueue = xQueueCreate(32, sizeof(uint16_t));
    if (uint16LedLoRaE5IndicationQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create queue Led LoRa E5 Indication.");
    } else {
        ESP_LOGV(TAG, "Queue Led LoRa E5 Indication created successfully.");
        //sendLedLoRaE5Indication(CODE_INIT_START);
    }
    createLedLoRaE5Indication();
}


#endif  /* #ifdef USE_RED_LED_SEEED_GROVE_LORA_E5 */














void printRouteNode(RouteNode* routeNode) {
    printf("Destination: %04X | Metric: %d | Next Hop (via): %04X | Hop Count: %d | Rx SNR: %d | Tx SNR: %d | Rx RSSI: %d | Tx RSSI: %d | SRTT: %d | RTTVAR: %d | Rx Metric: %d\r\n"
        , routeNode->networkNode.address    // Destination node address in hexadecimal
        , routeNode->networkNode.metric     // Metric associated with this route
        , routeNode->via                    // Next hop address in hexadecimal
        , routeNode->networkNode.hop_count  // Number of hops to the destination
        , routeNode->receivedSNR            // Signal-to-noise ratio for received packets
        , routeNode->sentSNR                // Signal-to-noise ratio for sent packets
        , routeNode->received_link_quality  // Link quality for received packets
        , routeNode->transmitted_link_quality   //Link quality for transmitted packets
        , routeNode->SRTT                   // Smoothed round-trip time
        , routeNode->RTTVAR                 // Round-trip time variation
        , routeNode->receivedMetric         // Received metric for this route
        );
}

void printRoutingTable() {
    // Get a copy of the routing table
    LM_LinkedList<RouteNode>* routingTable = radio.routingTableListCopy();
    
    if (routingTable->getLength() > 0) {
        printf("Routing Table:\r\n");
        routingTable->each(printRouteNode);  // Apply the print function to each route node
    } else {
        printf("Routing Table is empty.\r\n");
    }

    // Delete the copied routing table to prevent memory leaks
    delete routingTable;
    printf("\r\n");  // Add a blank line for readability
   
}

/**
 * @brief Print the counter of the packet
 *
 * @param data
 */
void printPacket(dataPacket data) {
    printf("Packet receive from %04X Nº %3d index\r\n", data.id_sender, data.counter);
}

/**
 * @brief Iterate through the payload of the packet and print the counter of the packet
 *
 * @param packet
 */
void printDataPacket(AppPacket<dataPacket>* packet) {
    printf("Packet arrived from %04X sz %3d bytes\r\n", packet->src, packet->payloadSize);

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
    printf("----------------------------------------------\r\n");
    printf("|  %04X  |  %04X  |  %04X  |  %04X  |  %04X  |\r\n", id_list[0], id_list[1], id_list[2], id_list[3], id_spare);
    printf("|  %04d  |  %04d  |  %04d  |  %04d  |  %04d  |\r\n", messages[0], messages[1], messages[2], messages[3], messages_spare);
    printf("----------------------------------------------\r\n");
}
void printDynamicMessageCounts() {
    uint16_t id_spare = 0;
    uint16_t messages_spare = 0;
    char print_line_dash[256] = {0};
    char print_line_idnt[256] = {0};
    char print_line_mess[256] = {0};


    for (int index = 0; index < boards_dynamic_count; index++)
    {
        sprintf(&print_line_dash[strlen(print_line_dash)], "---------");
        sprintf(&print_line_idnt[strlen(print_line_idnt)], "|  %04X  ", boards_dynamic_id_list[index]);
        sprintf(&print_line_mess[strlen(print_line_mess)], "|  %04d  ", boards_dynamic_id_mess[index]);
    }
    sprintf(&print_line_dash[strlen(print_line_dash)], "-\r\n");
    sprintf(&print_line_idnt[strlen(print_line_idnt)], "|\r\n");
    sprintf(&print_line_mess[strlen(print_line_mess)], "|\r\n");

    // printf("%s", print_line_dash);
    // printf("%s", print_line_idnt);
    // printf("%s", print_line_mess);
    // printf("%s", print_line_dash);
    printf("%s%s%s%s", print_line_dash, print_line_idnt, print_line_mess, print_line_dash);

}
/**
 * @brief Function that process the received packets
 *
 */
void processReceivedPackets(void*) {
    for (;;) {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);

        ESP_LOGV("main", "Stack space unused after entering the task processReceivedPackets: %d", uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGV("main", "Free heap: %d", getFreeHeap());


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

                    int board_index = BOARDS_DYNAMIC_COUNT_MAX;
                    for (int i = 0; i < boards_dynamic_count; i++)
                    {
                        if (packet_sender == boards_dynamic_id_list[i])
                        {
                            boards_dynamic_id_mess[i]++;
                            board_index = i;
                            break;
                        }
                    }
                    if (board_index >= BOARDS_DYNAMIC_COUNT_MAX)
                    {
                        if (boards_dynamic_count < BOARDS_DYNAMIC_COUNT_MAX)
                        {
                            board_index = boards_dynamic_count;
                            boards_dynamic_count++;
                            boards_dynamic_id_mess[board_index]++;
                            boards_dynamic_id_list[board_index] = packet_sender;
                        }
                    }




                    #ifdef USE_RED_LED_SEEED_GROVE_LORA_E5
                    sendLedLoRaE5Indication(CODE_RX_MESSAGE);
                    #endif

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
        #if USE_BOARDS_DYNAMIC_STATISTICS 
        printDynamicMessageCounts();
        #else
        printMessageCounts();
        #endif
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
        1024,
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

            ESP_LOGV("main", "Stack space unused after entering the task processLedIndcation: %d", uxTaskGetStackHighWaterMark(NULL));
            ESP_LOGV("main", "Free heap: %d", getFreeHeap());


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
            //printf("No data received.\r\n");
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

            ESP_LOGV("main", "Stack space unused after entering the task processLedSendIndcation: %d", uxTaskGetStackHighWaterMark(NULL));
            ESP_LOGV("main", "Free heap: %d", getFreeHeap());

            digitalWrite(BOARD_LED, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            digitalWrite(BOARD_LED, LED_OFF);

            vTaskDelay(200 / portTICK_PERIOD_MS);
        } else {
            //printf("No data received.\r\n");
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
        "Led Send Indication Task",
        4096,
        (void*) 1,
        2,
        &ledSendIndcation_Handle);
    if (res != pdPASS) {
        printf("Error: Led Send Indication Task creation gave error: %d\r\n", res);
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

    #ifndef DEBUG_NO_USE_RECEIVED_APP_MESSAGES
    //Create the receive task and add it to the LoRaMesher
    createReceiveMessages();

    //Set the task handle to the LoRaMesher
    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
    #endif

    #if USE_SBC_NodeMCU_ESP32
    // digitalWrite(LORA_ENABLE, ENABLE_OFF);
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    // digitalWrite(LORA_ENABLE, ENABLE_ON);
    #endif


    //Start LoRaMesher
    radio.start();

    printf("Lora initialized\r\n");
}

void printSubGHzInterruptPriority() {
    uint32_t priorityGrouping = NVIC_GetPriorityGrouping();
    uint32_t priority = NVIC_GetPriority(SUBGHZ_Radio_IRQn);

    uint32_t preemptPriority = (priority >> (8 - __NVIC_PRIO_BITS)) >> (priorityGrouping & 0x7);
    uint32_t subPriority = (priority & ((1 << (priorityGrouping & 0x7)) - 1));

    printf("SubGHz Interrupt Preempt Priority: %lu, Sub-Priority: %lu\r\n", preemptPriority, subPriority);
        // Replace SUBGHZ_Radio_IRQn with the actual IRQ number for SubGHz
    
    // Print the priority value; adjust the log function as needed
    //printf("SubGHz interrupt priority: %lu\r\n", priority);
    // Or use another logging function, like:
    // ESP_LOGI("DEBUG", "SubGHz interrupt priority: %lu", priority);
}



void MesherTask(void *pvParameters) {
    // This replaces the `loop()` function with main Mesher Function

    #ifdef USE_RED_LED_SEEED_GROVE_LORA_E5
    //sendLedLoRaE5Indication(CODE_INIT_START);
    #endif

    setupLoraMesher();

    #ifdef USE_RED_LED_SEEED_GROVE_LORA_E5
    //sendLedLoRaE5Indication(CODE_INIT_FINAL);
    #endif

    helloPacket->id_sender = radio.getLocalAddress();


    bool concentrator = USE_AS_CONCENTRATOR;
    printf("USE_AS_CONCENTRATOR: %s\r\n", (concentrator)?"TRUE":"FALSE");
    printf("LORA_IDENTIFICATION: 0x%04X\r\n", radio.getLocalAddress());
    for (;;) {

        ESP_LOGV("main", "Stack space unused after entering the task MesherTask: %d", uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGV("main", "Free heap: %d", getFreeHeap());


        if (concentrator)
        {
            ESP_LOGV(TAG, "Send packet %d\r\n", dataCounter);

            helloPacket->counter = dataCounter++;

            //Create packet and send it.
            radio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);
            #ifdef USE_RED_LED_SEEED_GROVE_LORA_E5
            sendLedLoRaE5Indication(CODE_TX_MESSAGE);
            #endif
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
                #ifdef USE_RED_LED_SEEED_GROVE_LORA_E5
                sendLedLoRaE5Indication(CODE_TX_MESSAGE);
                #endif
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
#if 0
#endif
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
        ESP_LOGV(TAG, "getReceivedTotalPackets          %d", radio.getReceivedTotalPacketsNum());
        ESP_LOGV(TAG, "getHelperOnReceiveTriggerNum     %d", radio.getHelperOnReceiveTriggerNum());
        ESP_LOGV(TAG, "------------------------------------");
        ESP_LOGV(TAG, "getOnReceiveEventsCounter        %d", radio.getOnReceiveEventsCounter());
        ESP_LOGV(TAG, "u32SkippedPrintfMemMalloc        %d", u32SkippedPrintfMemMalloc);
        ESP_LOGV(TAG, "u32SkippedPrintfMemMallocBytes   %d", u32SkippedPrintfMemMallocBytes);
        ESP_LOGV(TAG, "u32SkippedSerialQueueNull        %d", u32SkippedSerialQueueNull);
        ESP_LOGV(TAG, "u32SkippedSerialQueueSend        %d", u32SkippedSerialQueueSend);
        ESP_LOGV(TAG, "u32PrintfReceiveEventFlag        %d", u32PrintfReceiveEventFlag);
        ESP_LOGV(TAG, "u32SkippedPrintfCirMalloc        %d", u32SkippedPrintfCirMalloc);
        ESP_LOGV(TAG, "u32SkippedPrintfCirMallocBytes   %d", u32SkippedPrintfCirMallocBytes);
        ESP_LOGV(TAG, "u32SkippedCircleQueueSend        %d", u32SkippedCircleQueueSend);
        ESP_LOGV(TAG, "u32SkippedPrintfMemMallocFromCircle        %d", u32SkippedPrintfMemMallocFromCircle);
        ESP_LOGV(TAG, "u32SkippedPrintfMemMallocBytesFromCircle   %d", u32SkippedPrintfMemMallocBytesFromCircle);
        ESP_LOGV(TAG, "u32SkippedSerialQueueSendFromCircle        %d", u32SkippedSerialQueueSendFromCircle);
        ESP_LOGV(TAG, "------------------------------------");
        printSubGHzInterruptPriority();
        ESP_LOGV(TAG, "------------------------------------");

        #if USE_AS_CONCENTRATOR
        //Wait 5 seconds to send the next packet
        vTaskDelay(20000 / portTICK_PERIOD_MS);
        //delay(2000);
        #else
        //Wait 5 seconds to send the next packet
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        //delay(5000);
        #endif
    }
}

void setup() {

    initBuffer(&buffer_uart_printf);
    CircularQueue_Init(&circularQueue);
    int res;

    Serial.begin(115200);
    while (!Serial);      // Wait for Serial to initialize
    Serial.println("");
    Serial.println("");
    Serial.println("UART Initialized.");




    // Create the queue
    serialQueue = xQueueCreate(QUEUE_LENGTH, sizeof(char *));
    if (serialQueue == NULL) {
        Serial.println("Error creating serial print queue.");
        return;
    }

    Serial.print("serialQueue created\r\n");

    // Create task for printf -> Serial handling
    res = xTaskCreate(serialTask,"SerialTask", 256, NULL, 1, NULL);
    if (res != pdPASS) {
        ESP_LOGE("main", "SerialTask creation gave error: %d", res);
    }

    //Create task for LED blinking handling
    #ifdef DEBUG_USE_BLINK_TASK
    res = xTaskCreate(blinkTask, "Blink Task", 128, NULL, 1, NULL);
    if (res != pdPASS) {
        ESP_LOGE("main", "Blink Task creation gave error: %d", res);
    }
    #endif

    printf("initBoard \r\n");
    ESP_LOGI("main", "configMINIMAL_STACK_SIZE: %d", configMINIMAL_STACK_SIZE);
    ESP_LOGI("main", "configTOTAL_HEAP_SIZE: %d", configTOTAL_HEAP_SIZE);


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

    #ifdef USE_RED_LED_SEEED_GROVE_LORA_E5
    initLedLoRaE5Indication();
    #endif

    // Create task for printf -> Serial handling
    res = xTaskCreate(MesherTask,"MesherTask",1024, NULL, 1, NULL);
    if (res != pdPASS) {
        ESP_LOGE("main", "MesherTask creation gave error: %d", res);
    }




    #ifndef ESP32
    vTaskStartScheduler();
    #endif

}

void loop() {

    // for (;;) {

    //     printf("[loop printf] Loop: %d ms\r\n", millis());  // Print elapsed time in milliseconds
    //     printf("[loop printf] portTICK_PERIOD_MS: %d\r\n", portTICK_PERIOD_MS);
    //     printf("[loop printf] configTICK_RATE_HZ: %d\r\n", configTICK_RATE_HZ);
    //     printf("[loop printf] configCPU_CLOCK_HZ: %d\r\n", configCPU_CLOCK_HZ);
    //     delay(5000);
    // }
    return;
}