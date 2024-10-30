#include <Arduino.h>
#include "LoraMesher.h"

#include <vector>



//priority higher to lower
#define USE_SBC_NodeMCU_ESP32       1
#define USE_CS_1274                 1

#if USE_SBC_NodeMCU_ESP32
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

QueueHandle_t uint16LedRGBQueue = nullptr;


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
    Serial.printf("Hello Counter received nº %d\r\n", data.counter);
}

/**
 * @brief Iterate through the payload of the packet and print the counter of the packet
 *
 * @param packet
 */
void printDataPacket(AppPacket<dataPacket>* packet) {
    Serial.printf("Packet arrived from %X with size %d\r\n", packet->src, packet->payloadSize);

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
            Serial.println("ReceivedUserData_TaskHandle notify received");
            Serial.printf("Queue receiveUserData size: %d\r\n", radio.getReceivedQueueSize());

            //Get the first element inside the Received User Packets Queue
            AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();

            //Print the data packet
            printDataPacket(packet);

            bool isUnique = false;

            // Check if the packet is a broadcast message
            if (packet->dst == BROADCAST_ADDR) {
                // Flag to track if the payload is unique
                isUnique = true;

                dataPacket* helloPacketReceived = packet->payload;

                // Compare with previous broadcast payloads
                for (const auto& previousPayload : previousBroadcastPayloads) {
                    if (memcmp(helloPacketReceived, &previousPayload, sizeof(dataPacket)) == 0) {
                        isUnique = false;
                        Serial.println("Duplicate broadcast message detected, skipping retransmission.");
                        break;
                    }
                }

                // If unique, retransmit and store in the list
                if (isUnique) {
                    Serial.println("Retransmitting unique broadcast message...");
                    radio.createPacketAndSend(BROADCAST_ADDR, packet->payload, packet->getPayloadLength());
                    
                    // Check the size of previousBroadcastPayloads and maintain the limit
                    if (previousBroadcastPayloads.size() >= maxBroadcastPayloads) {
                        previousBroadcastPayloads.erase(previousBroadcastPayloads.begin());  // Remove the oldest entry
                    }
                    // Store the new unique payload
                    previousBroadcastPayloads.push_back(*helloPacketReceived);
                }
            }
            else
            {
                isUnique = true;
            }



            if (isUnique)
            {
                uint16_t packet_size = packet->payloadSize;
                uint16_t packet_sender = 0;
                dataPacket* helloPacketReceived = packet->payload;

                if (packet_size < sizeof(dataPacket))
                {
                    Serial.printf("Error Received packet_size %d < sizeof(dataPacket) %d \r\n", packet_size, sizeof(dataPacket));
                }
                else
                {
                    packet_sender = helloPacketReceived->id_sender;
                    Serial.printf("Received Data from sender. %X\r\n", packet_sender);
                    
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
                    
                    if (xQueueSend(uint16LedRGBQueue, &rgb_code, pdMS_TO_TICKS(0)) == pdPASS) {
                        Serial.printf("Data sent to queue. %X\r\n", rgb_code);
                    } else {
                        Serial.println("Failed to send data to queue.");
                    }
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
        Serial.printf("Error: Receive App Task creation gave error: %d\r\n", res);
    }
}

/**
 * @brief Function that process the led indication
 *
 */
void processLedIndcation(void*) {

    for (;;) {
        uint16_t rgb_code =0b000;
        if (xQueueReceive(uint16LedRGBQueue, &rgb_code, pdMS_TO_TICKS(100)) == pdPASS) {
            Serial.printf("Data received from queue: %X\r\n", rgb_code);

            digitalWrite(BOARD_LED, LED_ON);
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
            digitalWrite(BOARD_LED, LED_OFF);

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
        Serial.printf("Error: Led Indcation Task creation gave error: %d\r\n", res);
    }
}


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




    config.module = LoraMesher::LoraModules::SX1276_MOD;

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

    Serial.println("initBoard");
    pinMode(BOARD_LED, OUTPUT); //setup pin as output for indicator LED
    #if USE_SBC_NodeMCU_ESP32
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
    led_Flash(2, 125);          //two quick LED flashes to indicate program start

    uint16LedRGBQueue = xQueueCreate(10, sizeof(uint16_t));
    if (uint16LedRGBQueue == nullptr) {
        Serial.println("Failed to create queue.");
    } else {
        Serial.println("Queue created successfully.");

        uint16_t rgb_code = 0b111;  //white
        if (xQueueSend(uint16LedRGBQueue, &rgb_code, pdMS_TO_TICKS(0)) == pdPASS) {
            Serial.printf("Data sent to queue. %X\r\n", rgb_code);
        } else {
            Serial.println("Failed to send data to queue.");
        }
    }

    createLedIndication();


    setupLoraMesher();

    uint16_t rgb_code = 0b111;  //white
    if (xQueueSend(uint16LedRGBQueue, &rgb_code, pdMS_TO_TICKS(0)) == pdPASS) {
        Serial.printf("Data sent to queue. %X\r\n", rgb_code);
    } else {
        Serial.println("Failed to send data to queue.");
    }

    helloPacket->id_sender = radio.getLocalAddress();
}



void loop() {
    for (;;) {
        Serial.printf("Send packet %d\r\n", dataCounter);

        helloPacket->counter = dataCounter++;

        //Create packet and send it.
        radio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);
        // Check the size of previousBroadcastPayloads and maintain the limit
        if (previousBroadcastPayloads.size() >= maxBroadcastPayloads) {
            previousBroadcastPayloads.erase(previousBroadcastPayloads.begin());  // Remove the oldest entry
        }
        // Store the new unique payload
        previousBroadcastPayloads.push_back(*helloPacket);
        Serial.printf("Data sent to queue from this sender: %X\r\n", helloPacket->id_sender);

        printRoutingTable();

        Serial.printf("routingTableSize                 %d\r\n", radio.routingTableSize());
        Serial.printf("getLocalAddress                  %X\r\n", radio.getLocalAddress());
        Serial.printf("getReceivedDataPacketsNum        %d\r\n", radio.getReceivedDataPacketsNum());
        Serial.printf("getSendPacketsNum                %d\r\n", radio.getSendPacketsNum());
        Serial.printf("getReceivedHelloPacketsNum       %d\r\n", radio.getReceivedHelloPacketsNum());
        Serial.printf("getSentHelloPacketsNum           %d\r\n", radio.getSentHelloPacketsNum());
        Serial.printf("getReceivedBroadcastPacketsNum   %d\r\n", radio.getReceivedBroadcastPacketsNum());
        Serial.printf("getForwardedPacketsNum           %d\r\n", radio.getForwardedPacketsNum());
        Serial.printf("getDataPacketsForMeNum           %d\r\n", radio.getDataPacketsForMeNum());
        Serial.printf("getReceivedIAmViaNum             %d\r\n", radio.getReceivedIAmViaNum());
        Serial.printf("getDestinyUnreachableNum         %d\r\n", radio.getDestinyUnreachableNum());
        Serial.printf("getReceivedNotForMe              %d\r\n", radio.getReceivedNotForMe());
        Serial.printf("getReceivedPayloadBytes          %d\r\n", radio.getReceivedPayloadBytes());
        Serial.printf("getReceivedControlBytes          %d\r\n", radio.getReceivedControlBytes());
        Serial.printf("getSentPayloadBytes              %d\r\n", radio.getSentPayloadBytes());
        Serial.printf("getSentControlBytes              %d\r\n", radio.getSentControlBytes());

        //Wait 5 seconds to send the next packet
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}