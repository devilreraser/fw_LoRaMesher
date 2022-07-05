#include <Arduino.h>
#include "LoraMesher.h"
#include "display.h"

//Using LILYGO TTGO T-BEAM v1.1 
#define BOARD_LED   4
#define LED_ON      LOW
#define LED_OFF     HIGH

#define DATA_NUM 6

LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
struct dataPacket {
    uint32_t counter[35] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
};

dataPacket helloPackets[DATA_NUM];

/**
 * @brief Flash the lead
 *
 * @param flashes number of flashes
 * @param delaymS delay between is on and off of the LED
 */
void led_Flash(uint16_t flashes, uint16_t delaymS) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
        digitalWrite(BOARD_LED, LED_OFF);
        vTaskDelay(delaymS / portTICK_PERIOD_MS);
        digitalWrite(BOARD_LED, LED_ON);
        vTaskDelay(delaymS / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Print the counter of the packet
 *
 * @param data
 */
void printPacket(dataPacket* data, uint16_t sourceAddress) {
    char text[32];
    snprintf(text, 32, ("%X-> %d" CR), sourceAddress, data->counter[0]);

    Screen.changeLineThree(String(text));
    Log.verboseln(F("Received data nº %d"), data->counter[0]);
}

/**
 * @brief Iterate through the payload of the packet and print the counter of the packet
 *
 * @param packet
 */
void printDataPacket(AppPacket<dataPacket>* packet) {
    Log.traceln(F("Packet arrived from %X with size %d bytes"), packet->src, packet->payloadSize);

    //Get the payload to iterate through it
    dataPacket* dPacket = packet->payload;
    size_t payloadLength = packet->getPayloadLength();

    Log.traceln(F("---- Payload ---- Payload length in dataP: %d "), payloadLength);
    Log.setShowLevel(false);

    for (size_t i = 0; i < payloadLength; i++) {
        Log.verbose(F("Received data nº %d" CR), i);
        Log.verbose(F("%d -- "), i);

        for (size_t j = 0; j < 35; j++) {
            Log.verbose(F("%d, "), dPacket[i].counter[j]);

        }
        Log.verbose(F("" CR));
    }

    Log.setShowLevel(true);
    Log.traceln(F("---- Payload Done ---- "));

}

/**
 * @brief Function that process the received packets
 *
 */
void processReceivedPackets(void*) {
    for (;;) {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        led_Flash(1, 100); //one quick LED flashes to indicate a packet has arrived

        //Iterate through all the packets inside the Received User Packets FiFo
        while (radio.getReceivedQueueSize() > 0) {
            Log.traceln(F("ReceivedUserData_TaskHandle notify received"));
            Log.traceln(F("Queue receiveUserData size: %d"), radio.getReceivedQueueSize());

            //Get the first element inside the Received User Packets FiFo
            AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();

            //Print the data packet
            printDataPacket(packet);

            //Delete the packet when used. It is very important to call this function to release the memory of the packet.
            radio.deletePacket(packet);

        }
    }
}

/**
 * @brief Initialize LoRaMesher
 *
 */
void setupLoraMesher() {
    //Create a loramesher with a processReceivedPackets function
    radio.init(processReceivedPackets);

    Log.verboseln("LoRaMesher initialized");
}

/**
 * @brief Displays the address in the first line
 *
 */
void printAddressDisplay() {
    char addrStr[15];
    snprintf(addrStr, 15, "Id: %X\r\n", radio.getLocalAddress());

    Screen.changeLineOne(String(addrStr));
}

/**
 * @brief Print the routing table into the display
 *
 */
void printRoutingTableToDisplay() {

    //Set the routing table list that is being used and cannot be accessed (Remember to release use after usage)
    LM_LinkedList<RouteNode>* routingTableList = radio.routingTableList();

    routingTableList->setInUse();

    Screen.changeSizeRouting(radio.routingTableSize());

    char text[15];
    for (int i = 0; i < radio.routingTableSize(); i++) {
        RouteNode* rNode = (*routingTableList)[i];
        NetworkNode node = rNode->networkNode;
        snprintf(text, 15, ("|%X(%d)->%X"), node.address, node.metric, rNode->via);
        Screen.changeRoutingText(text, i);
    }

    //Release routing table list usage.
    routingTableList->releaseInUse();

    Screen.changeLineFour();
}


/**
 * @brief Every 20 seconds it will send a counter to a position of the dataTable
 *
 */
void sendLoRaMessage(void*) {
    int dataTablePosition = 0;

    for (;;) {

        if (radio.routingTableSize() == 0) {
            vTaskDelay(20000 / portTICK_PERIOD_MS);
            continue;
        }

        if (radio.routingTableSize() <= dataTablePosition)
            dataTablePosition = 0;

        LM_LinkedList<RouteNode>* routingTableList = radio.routingTableList();

        uint16_t addr = (*routingTableList)[dataTablePosition]->networkNode.address;

        Log.traceln(F("Send data packet nº %d to %X (%d)"), dataCounter, addr, dataTablePosition);

        dataTablePosition++;

        //Send packet reliable
        radio.sendReliable(addr, helloPackets, DATA_NUM);

        //Print second line in the screen
        Screen.changeLineTwo("Send " + String(dataCounter));

        //Increment the dataCounter
        dataCounter++;

        //Increment data counter of all the packets
        // for (int i = 0; i < DATA_NUM; i++)
        //     helloPackets[i].counter[0] = dataCounter;

        //Print routing Table to Display
        printRoutingTableToDisplay();

        break;

        //Wait 120 seconds to send the next packet
        vTaskDelay(120000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

/**
 * @brief Setup the Task to create and send periodical messages
 *
 */
void createSendMessages() {
    TaskHandle_t sendLoRaMessage_Handle = NULL;
    BaseType_t res = xTaskCreate(
        sendLoRaMessage,
        "Send LoRa Message routine",
        4098,
        (void*) 1,
        1,
        &sendLoRaMessage_Handle);
    if (res != pdPASS) {
        Log.errorln(F("Send LoRa Message task creation gave error: %d"), res);
        vTaskDelete(sendLoRaMessage_Handle);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(BOARD_LED, OUTPUT); //setup pin as output for indicator LED

    Screen.initDisplay();

    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.verboseln("Board Init");

    led_Flash(2, 125);          //two quick LED flashes to indicate program start
    setupLoraMesher();
    printAddressDisplay();

    createSendMessages();
}

void loop() {
    vTaskPrioritySet(NULL, 1);
    Screen.drawDisplay();
}