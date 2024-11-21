#include "LoraMesher.h"
#include <string>

#include "debug_root.h"

#ifndef ARDUINO
#include "EspHal.h"
#endif

/* Section below For Task Debugging - Comment out for normal operation */
//#define DEBUG_NO_USE_QUEUE_MANAGER
//#define DEBUG_NO_USE_ROUTE_MANAGER
//#define DEBUG_NO_USE_RECEIVE_DATA
//#define DEBUG_NO_USE_RECEIVE_PACKET
//#define DEBUG_NO_USE_RECEIVE_PACKET_NOTIFY_RECEIVE_DATA
#define ON_RECEIVE_NOTIFY_WITH_BOOL


#ifdef ON_RECEIVE_NOTIFY_WITH_BOOL
bool b_on_receive_notify = false;
#endif


const char* PP_TAG = "ProcPackets";

uint32_t processPacketsState = 0;
uint32_t processPacketsLoop = 0;
uint32_t processPacketsLoopLast = 0;

int last_error_radio = 0;
int startReceivingError = 0;
int resetRadioOnSendingErrorTimes = 0;
int last_error_radio_on_send = 0;

uint8_t dummy_buffer[256];

LoraMesher::LoraMesher() {}


void LoraMesher::begin(LoraMesherConfig config) {
    ESP_LOGV(LM_TAG, "Initializing LoraMesher v%s", LM_VERSION);

    // Set the configuration
    *loraMesherConfig = config;
    initConfiguration();

    // Initialize the radio
    initializeLoRa();

    // Recalculate the max time on air
    recalculateMaxTimeOnAir();

    // Initialize the queues
    initializeSchedulers();
}

void LoraMesher::standby() {
    //Get actual priority
    UBaseType_t prevPriority = uxTaskPriorityGet(NULL);

    //Set max priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    int res = radio->standby();
    if (res != 0)
        ESP_LOGE(LM_TAG, "Standby gave error: %d", res);

    //Clear Dio Actions
    clearDioActions();

    //Suspend all tasks
    #ifndef DEBUG_NO_USE_RECEIVE_PACKET
    if (ReceivePacket_TaskHandle)
        vTaskSuspend(ReceivePacket_TaskHandle);
    #endif
    vTaskSuspend(Hello_TaskHandle);
    #ifndef DEBUG_NO_USE_RECEIVE_DATA
    vTaskSuspend(ReceiveData_TaskHandle);
    #endif
    vTaskSuspend(SendData_TaskHandle);
    #ifndef DEBUG_NO_USE_ROUTE_MANAGER
    vTaskSuspend(RoutingTableManager_TaskHandle);
    #endif
    #ifndef DEBUG_NO_USE_QUEUE_MANAGER
    vTaskSuspend(QueueManager_TaskHandle);
    #endif

    //Set previous priority
    vTaskPrioritySet(NULL, prevPriority);
}

void LoraMesher::start() {
    // Get actual priority
    UBaseType_t prevPriority = uxTaskPriorityGet(NULL);

    // Set max priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    // Resume all tasks
    #ifndef DEBUG_NO_USE_RECEIVE_PACKET
    if (ReceivePacket_TaskHandle)
        vTaskResume(ReceivePacket_TaskHandle);
    #endif
    vTaskResume(Hello_TaskHandle);
    #ifndef DEBUG_NO_USE_RECEIVE_DATA
    vTaskResume(ReceiveData_TaskHandle);
    #endif
    vTaskResume(SendData_TaskHandle);
    #ifndef DEBUG_NO_USE_ROUTE_MANAGER
    vTaskResume(RoutingTableManager_TaskHandle);
    #endif
    #ifndef DEBUG_NO_USE_QUEUE_MANAGER
    vTaskResume(QueueManager_TaskHandle);
    #endif

    // Start Receiving
    startReceivingError = startReceiving(1);

    // Set previous priority
    vTaskPrioritySet(NULL, prevPriority);
}

LoraMesher::~LoraMesher() {
    #ifndef DEBUG_NO_USE_RECEIVE_PACKET
    if (ReceivePacket_TaskHandle)
        vTaskDelete(ReceivePacket_TaskHandle);
    #endif
    vTaskDelete(Hello_TaskHandle);
    #ifndef DEBUG_NO_USE_RECEIVE_DATA
    vTaskDelete(ReceiveData_TaskHandle);
    #endif
    vTaskDelete(SendData_TaskHandle);
    #ifndef DEBUG_NO_USE_ROUTE_MANAGER
    vTaskDelete(RoutingTableManager_TaskHandle);
    #endif
    #ifndef DEBUG_NO_USE_QUEUE_MANAGER
    vTaskDelete(QueueManager_TaskHandle);
    #endif

    ToSendPackets->Clear();
    delete ToSendPackets;
    ReceivedPackets->Clear();
    delete ReceivedPackets;
    ReceivedAppPackets->Clear();
    delete ReceivedAppPackets;

    clearDioActions();
    radio->reset();

    delete radio;
}

void LoraMesher::setConfig(LoraMesherConfig config) {
    standby();

    *loraMesherConfig = config;
    initConfiguration();
    recalculateMaxTimeOnAir();

    restartRadio();

    start();
}

void LoraMesher::restartRadio() {
    radio->reset();
    initializeLoRa();
    incRestartRadioCount();

    ESP_LOGI(LM_TAG, "Restarting radio DONE");
}

void LoraMesher::initConfiguration() {
    ESP_LOGV(LM_TAG, "Initializing Configuration");

    PacketFactory::setMaxPacketSize(loraMesherConfig->max_packet_size);
}

void LoraMesher::initializeLoRa() {
    ESP_LOGV(LM_TAG, "Initializing RadioLib");

    LoraMesherConfig config = *loraMesherConfig;

    ESP_LOGI(LM_TAG, "LoRaMesher Configuration:");
    ESP_LOGI(LM_TAG, "LoRa Module: %d", config.module);
    ESP_LOGI(LM_TAG, "LoRa CS: %d", config.loraCs);
    ESP_LOGI(LM_TAG, "LoRa IRQ: %d", config.loraIrq);
    ESP_LOGI(LM_TAG, "LoRa RST: %d", config.loraRst);
    ESP_LOGI(LM_TAG, "LoRa IO1: %d", config.loraIo1);

#ifdef ARDUINO
    #if STM32WL
    if (config.module == LoraModules::STM32WL_MOD) {
        // Handle STM32WL initialization
        if (radio == nullptr) {
            ESP_LOGV(LM_TAG, "Using STM32WL module");
            radio = new LM_STM32WLx();  // Pass specific module as needed

            #ifdef LORA_E5_DEV_BOARD
            // set RF switch configuration for Nucleo WL55JC1
            // NOTE: other boards may be different!
            //       Some boards may not have either LP or HP.
            //       For those, do not set the LP/HP entry in the table.
            static const uint32_t rfswitch_pins[] =
                                    {PC3,  PC4,  PC5, RADIOLIB_NC, RADIOLIB_NC};
            static const Module::RfSwitchMode_t rfswitch_table[] = {
                {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
                {STM32WLx::MODE_RX,    {HIGH, HIGH, LOW}},
                {STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH}},
                {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},
                END_OF_MODE_TABLE,
            };

            // set RF switch control configuration
            // this has to be done prior to calling begin()
            radio->setRfSwitchTable(rfswitch_pins, rfswitch_table);
            #endif
        }




    } 
    #else
    if (config.spi == nullptr) {
        //SPI.begin();
        SPI.begin(config.loraSCK, config.loraMISO, config.loraMOSI);
        config.spi = &SPI;
    }

    if (radio == nullptr) {
        switch (config.module) {
            case LoraModules::STM32WL_MOD:
                ESP_LOGV(LM_TAG, "Using STM32WL module");
                radio = new LM_STM32WLx(config.radioModule);
                break;
            case LoraModules::SX1276_MOD:
                ESP_LOGV(LM_TAG, "Using SX1276 module");
                radio = new LM_SX1276(config.loraCs, config.loraIrq, config.loraRst, config.spi);
                break;
            case LoraModules::SX1262_MOD:
                ESP_LOGV(LM_TAG, "Using SX1262 module");
                radio = new LM_SX1262(config.loraCs, config.loraIrq, config.loraRst, config.loraIo1, config.spi);
                break;
            case LoraModules::SX1278_MOD:
                ESP_LOGV(LM_TAG, "Using SX1278 module");
                radio = new LM_SX1278(config.loraCs, config.loraIrq, config.loraRst, config.loraIo1, config.spi);
                break;
            case LoraModules::SX1268_MOD:
                ESP_LOGV(LM_TAG, "Using SX1268 module");
                radio = new LM_SX1268(config.loraCs, config.loraIrq, config.loraRst, config.loraIo1, config.spi);
                break;
            case LoraModules::SX1280_MOD:
                ESP_LOGV(LM_TAG, "Using SX1280 module");
                radio = new LM_SX1280(config.loraCs, config.loraIrq, config.loraRst, config.loraIo1, config.spi);
                break;
            default:
                ESP_LOGV(LM_TAG, "Using SX1276 module");
                radio = new LM_SX1276(config.loraCs, config.loraIrq, config.loraRst, config.spi);
                break;
        }
    }
    #endif

#else
    if (config.hal == nullptr)
        config.hal = new EspHal(SPI_SCK, SPI_MISO, SPI_MOSI);

    if (config.hal == nullptr)
        ESP_LOGE(LM_TAG, "Could not create SPI HAL");

    if (radio == nullptr) {
        Module* mod = new Module(config.hal, config.loraCs, config.loraIrq, config.loraRst, config.loraIo1);

        switch (config.module) {
            case LoraModules::SX1276_MOD:
                ESP_LOGV(LM_TAG, "Using SX1276 module");
                radio = new LM_SX1276(mod);
                break;
            case LoraModules::SX1262_MOD:
                ESP_LOGV(LM_TAG, "Using SX1262 module");
                radio = new LM_SX1262(mod);
                break;
            case LoraModules::SX1278_MOD:
                ESP_LOGV(LM_TAG, "Using SX1278 module");
                radio = new LM_SX1278(mod);
                break;
            case LoraModules::SX1268_MOD:
                ESP_LOGV(LM_TAG, "Using SX1268 module");
                radio = new LM_SX1268(mod);
                break;
            case LoraModules::SX1280_MOD:
                ESP_LOGV(LM_TAG, "Using SX1280 module");
                radio = new LM_SX1280(mod);
                break;
            default:
                ESP_LOGV(LM_TAG, "Using SX1276 module");
                radio = new LM_SX1276(mod);
                break;
        }
    }

#endif

    if (radio == NULL) {
        ESP_LOGE(LM_TAG, "RadioLib not initialized properly");
    }

    // Set up the radio parameters
    ESP_LOGV(LM_TAG, "Initializing radio");
    int res = radio->begin(config.freq, config.bw, config.sf, config.cr, config.syncWord, config.power, config.preambleLength);
    ESP_LOGV(LM_TAG, "config.power : %d", config.power);

    if (res != 0) {
        ESP_LOGE(LM_TAG, "Radio module gave error: %d", res);
    }

#ifdef LM_ADDCRC_PAYLOAD
    radio->setCRC(true);
#endif
    ESP_LOGI(LM_TAG, "LoRa module initialization DONE");
}

void LoraMesher::setDioActionsForScanChannel() {
    // set the function that will be called
    // when LoRa preamble is detected
    clearDioActions();
    // radio->setDioActionForScanning(onReceive);
}

void LoraMesher::setDioActionsForReceivePacket() {
    clearDioActions();

    radio->setDioActionForReceiving(onReceive);
}

void LoraMesher::clearDioActions() {
    radio->clearDioActions();
}

int LoraMesher::startReceiving(int times_enter) {
    setDioActionsForReceivePacket();

    int res = radio->startReceive();
    if (res != 0) {
        last_error_radio = res;
        // ESP_LOGE(LM_TAG, "Starting receiving gave error: %d", res);
        // if (res == RADIOLIB_ERR_SPI_WRITE_FAILED) {
        //     ESP_LOGW(LM_TAG, "SPI Write failed, restarting radio");
        //     restartRadio();
        // }

        restartRadio();
        if(times_enter < 3)
        {
            return startReceiving(times_enter + 1);
        }
        else
        {
            ESP_LOGE(LM_TAG, "Max retries reached, aborting startReceiving.");
            return res;  // Exit with the last error code

        }
        
    }
    return res;
}

void LoraMesher::channelScan() {
    setDioActionsForScanChannel();

    int res = radio->scanChannel();

    if (res != RADIOLIB_ERR_NONE) {
        ESP_LOGE(LM_TAG, "Starting new scan failed, code %d", res);
        channelScan();
    }

}

int LoraMesher::startChannelScan() {
    setDioActionsForScanChannel();

    int state = radio->startChannelScan();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(LM_TAG, "Starting new scan failed, code %d", state);
        startChannelScan();
    }

    return state;
}



void LoraMesher::initializeSchedulers() {
    ESP_LOGV(LM_TAG, "Setting up Schedulers");
    int res;

    #ifdef ON_RECEIVE_NOTIFY_WITH_BOOL
    res = xTaskCreate(
        [](void* o) { static_cast<LoraMesher*>(o)->helperRoutine(); },
        "Helper routine",
        256,
        this,
        6,
        &Helper_TaskHandle);
    if (res != pdPASS) {
        ESP_LOGE(LM_TAG, "Helper routine creation gave error: %d", res);
    }
    #endif
    
    #ifndef DEBUG_NO_USE_RECEIVE_PACKET
    res = xTaskCreate(
        [](void* o) { static_cast<LoraMesher*>(o)->receivingRoutine(); },
        "Receiving routine",
        2048 + 2048,  //for sure stack overflow with 2048 bytes when was recursion
        this,
        6,
        &ReceivePacket_TaskHandle);
    if (res != pdPASS) {
        ESP_LOGE(LM_TAG, "Receiving routine creation gave error: %d", res);
    }
    #endif
    res = xTaskCreate(
        [](void* o) { static_cast<LoraMesher*>(o)->sendPackets(); },
        "Sending routine",
        256,
        this,
        5,
        &SendData_TaskHandle);
    if (res != pdPASS) {
        ESP_LOGE(LM_TAG, "Sending Task creation gave error: %d", res);
    }
    res = xTaskCreate(
        [](void* o) { static_cast<LoraMesher*>(o)->sendHelloPacketRoutine(); },
        "Hello routine",
        256,
        this,
        4,
        &Hello_TaskHandle);
    if (res != pdPASS) {
        ESP_LOGE(LM_TAG, "Process Task Hello creation gave error: %d", res);
    }
    #ifndef DEBUG_NO_USE_RECEIVE_DATA
    res = xTaskCreate(
        [](void* o) { static_cast<LoraMesher*>(o)->processPackets(); },
        "Process routine",
        512,  /* ??? not sure was there block if only set to 512 */
        this,
        3,
        &ReceiveData_TaskHandle);
    if (res != pdPASS) {
        ESP_LOGE(LM_TAG, "Process Task RxDat creation gave error: %d", res);
    }
    #endif
    #ifndef DEBUG_NO_USE_ROUTE_MANAGER
    res = xTaskCreate(
        [](void* o) { static_cast<LoraMesher*>(o)->routingTableManager(); },
        "Routing Table Manager routine",
        512,
        this,
        2,
        &RoutingTableManager_TaskHandle);
    if (res != pdPASS) {
        ESP_LOGE(LM_TAG, "Routing Table Manager Task creation gave error: %d", res);
    }
    #endif
    #ifndef DEBUG_NO_USE_QUEUE_MANAGER
    res = xTaskCreate(
        [](void* o) { static_cast<LoraMesher*>(o)->queueManager(); },
        "Queue Manager routine",
        512,
        this,
        2,
        &QueueManager_TaskHandle);
    if (res != pdPASS) {
        ESP_LOGE(LM_TAG, "Queue Manager Task creation gave error: %d", res);
    }
    #endif

    vTaskDelay(5000 / portTICK_PERIOD_MS);
}

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void LoraMesher::onReceive(void) {
    LoraMesher::getInstance().setOnReceiveEventsFlag();
    LoraMesher::getInstance().incOnReceiveEventsCounter();
    ESP_LOGV(LM_TAG, "LoraMesher::onReceive");

    #ifndef DEBUG_NO_USE_RECEIVE_PACKET
    #ifndef DEBUG_NO_USE_RECEIVE_PACKET_NOTIFY_RECEIVE_DATA

    #if defined(ESP32)    
    if (LoraMesher::getInstance().ReceivePacket_TaskHandle)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xHigherPriorityTaskWoken = xTaskNotifyFromISR(
            LoraMesher::getInstance().ReceivePacket_TaskHandle,
            0,
            eSetValueWithoutOverwrite,
            &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR();
    }
    #else
    
    #ifdef ON_RECEIVE_NOTIFY_WITH_BOOL
    b_on_receive_notify = true;
    #else
    #if 0
    if (LoraMesher::getInstance().ReceivePacket_TaskHandle)
    {
        xTaskNotify(LoraMesher::getInstance().ReceivePacket_TaskHandle, 0, eSetValueWithoutOverwrite);
    }
    #else
    if (LoraMesher::getInstance().ReceivePacket_TaskHandle)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xHigherPriorityTaskWoken = xTaskNotifyFromISR(
            LoraMesher::getInstance().ReceivePacket_TaskHandle,
            0,
            eSetValueWithoutOverwrite,
            &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    #endif
    #endif
    #endif

    #endif
    #endif
    LoraMesher::getInstance().clrOnReceiveEventsFlag();
}

#ifdef ON_RECEIVE_NOTIFY_WITH_BOOL
void LoraMesher::helperRoutine() {
    ESP_LOGV(LM_TAG, "Helper routine started");
    //vTaskSuspend(NULL);

    int task_delay_ms = 10;
    int print_delay_ms = 5000;
    int print_timeout_ms = print_delay_ms;
    

    for (;;) {

        DebugTaskCounter(pcTaskGetName(NULL));

        if (startReceivingError)
        {
            startReceivingError = startReceiving(1);
        }

        if (getResetReceiveRoutineTask())
        {
            clrResetReceiveRoutineTask();
            if (ReceivePacket_TaskHandle)
            {
                vTaskDelete(ReceivePacket_TaskHandle);
                int res;
                res = xTaskCreate(
                    [](void* o) { static_cast<LoraMesher*>(o)->receivingRoutine(); },
                    "Receiving routine",
                    2048 + 2048,  //for sure stack overflow with 2048 bytes when was recursion
                    this,
                    6,
                    &ReceivePacket_TaskHandle);

            }
            
        }

        if (b_on_receive_notify)
        {
            incHelperOnReceiveTriggerNum();
            b_on_receive_notify = false;
            if(LoraMesher::getInstance().ReceivePacket_TaskHandle)
            {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xHigherPriorityTaskWoken = xTaskNotifyFromISR(
                    LoraMesher::getInstance().ReceivePacket_TaskHandle,
                    0,
                    eSetValueWithoutOverwrite,
                    &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            
        }

        if (print_timeout_ms >= print_delay_ms)
        {
            print_timeout_ms = 0;
            ESP_LOGV(LM_TAG, "Stack space unused after entering the task helperRoutine: %d", uxTaskGetStackHighWaterMark(NULL));
            ESP_LOGV(LM_TAG, "Free heap: %d", getFreeHeap());
        }
        print_timeout_ms += task_delay_ms;
        //Set a random delay, to avoid some collisions.
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}
#endif

uint32_t packetSizeCreateEmptyTotal = 0;
uint32_t packetSizeCreateEmptyMax = 0;
uint32_t receive_routine_radiolib_err = 0;
uint32_t receive_routine_too_big_packet = 0;
uint32_t receive_routine_create_queue_packet = 0;

void LoraMesher::receivingRoutine() {
    ESP_LOGV(LM_TAG, "Receiving routine started");
    vTaskSuspend(NULL);

    BaseType_t TWres;
    size_t packetSize;
    int8_t rssi, snr;
    int16_t state;

    for (;;) {

        DebugTaskCounter(pcTaskGetName(NULL));

        TWres = xTaskNotifyWait(
            pdTRUE,
            pdFALSE,
            NULL,
            portMAX_DELAY);
        
        if (TWres == pdPASS) {
            ESP_LOGV(LM_TAG, "Stack space unused after entering the task receivingRoutine: %d", uxTaskGetStackHighWaterMark(NULL));
            ESP_LOGV(LM_TAG, "Free heap: %d", getFreeHeap());

            hasReceivedMessage = true;

            incReceivedTotalPackets();


            packetSize = radio->getPacketLength();
            if (packetSize == 0)
                ESP_LOGW(LM_TAG, "Empty packet received");
            else {
                packetSizeCreateEmptyTotal += packetSize;
                if (packetSizeCreateEmptyMax < packetSize)
                {
                    packetSizeCreateEmptyMax = packetSize;
                }
                Packet<uint8_t>* rx = PacketService::createEmptyPacket(packetSize);

                if (rx)
                {
                    rssi = (int8_t) round(radio->getRSSI());
                    snr = (int8_t) round(radio->getSNR());

                    ESP_LOGI(LM_TAG, "Receiving LoRa packet: Size: %d bytes RSSI: %d SNR: %d", packetSize, rssi, snr);

                    size_t max_packet_size = PacketFactory::getMaxPacketSize();
                    if (packetSize > max_packet_size) {
                        ESP_LOGW(LM_TAG, "Received packet with size greater than MAX Packet Size");
                        packetSize = max_packet_size;
                    }

                    state = radio->readData(reinterpret_cast<uint8_t*>(rx), packetSize);

                    if (state != RADIOLIB_ERR_NONE) {
                        ESP_LOGW(LM_TAG, "Reading packet data gave error: %d", state);
                        if (state == RADIOLIB_ERR_SPI_WRITE_FAILED) {
                            ESP_LOGW(LM_TAG, "SPI Write failed, restarting radio");
                            restartRadio();
                        }

                        receive_routine_radiolib_err++;

                        // TODO: Set a count to get the number of CRC errors
                        deletePacket(rx);
                    }
                    else if (packetSize != rx->packetSize) {
                        ESP_LOGW(LM_TAG, "Packet size is different from the size read");
                        receive_routine_too_big_packet++;
                        deletePacket(rx);
                    }
                    else {
                        //Create a Packet Queue element containing the Packet
                        QueuePacket<Packet<uint8_t>>* pq = PacketQueueService::createQueuePacket(rx, 0, 0, rssi, snr);

                        receive_routine_create_queue_packet++;

                        //Add the Packet Queue element created into the ReceivedPackets List
                        ReceivedPackets->Append(pq);

                        //Notify that a packet needs to be process
                        #ifndef DEBUG_NO_USE_RECEIVE_DATA
                        TWres = xTaskNotifyFromISR(
                            ReceiveData_TaskHandle,
                            0,
                            eSetValueWithoutOverwrite,
                            &TWres);
                        #endif
                    }
                }
                else
                {
                    ESP_LOGE(LM_TAG, "Fail Allocate %d bytes for read packet", packetSize);

                    state = radio->readData(dummy_buffer, packetSize);

                    if (state != RADIOLIB_ERR_NONE) {
                        ESP_LOGW(LM_TAG, "Reading packet data gave error: %d", state);
                        if (state == RADIOLIB_ERR_SPI_WRITE_FAILED) {
                            ESP_LOGW(LM_TAG, "SPI Write failed, restarting radio");
                            restartRadio();
                        }
                    }
                }


            }

            startReceivingError = startReceiving(1);
        }
    }
}

uint16_t LoraMesher::getLocalAddress() {
    return WiFiService::getLocalAddress();
}

SinglePoleIIRFilter filter(0.01f);
SinglePoleIIRFilter filterTotal(0.01f);
uint32_t randomDelayTotal = 0;
uint32_t randomDelayTotalFilt = 0;




/**
 *  Region Packet Service
**/

void LoraMesher::waitBeforeSend(uint8_t repeatedDetectPreambles) {
    // TODO: Why did I set this if?
    if (repeatedDetectPreambles > RoutingTableService::routingTableSize())
    {
        return;
    }
        

    hasReceivedMessage = false;

    if (repeatedDetectPreambles == 1)
    {
        randomDelayTotal = 0;
    }



    //Random delay, to avoid some collisions.
    uint32_t randomDelay = getPropagationTimeWithRandom(repeatedDetectPreambles);

    randomDelayTotal += randomDelay;
    incSendRandomTotalWait(randomDelay);

    ESP_LOGV(LM_TAG, "RandomDelay     %d ms", (int) randomDelay);
    uint32_t filteredRandomDelayValue = filter.filter(randomDelay);
    ESP_LOGV(LM_TAG, "RandomDelayFilt %d ms", (int) filteredRandomDelayValue);

    //Set a random delay, to avoid some collisions.
    vTaskDelay(randomDelay / portTICK_PERIOD_MS);

    if (hasReceivedMessage) {
        startReceivingError = startReceiving(1);
        ESP_LOGV(LM_TAG, "Preamble detected while waiting %d", repeatedDetectPreambles);
        waitBeforeSend(repeatedDetectPreambles + 1);
    }

    if (repeatedDetectPreambles == 1)
    {
        ESP_LOGV(LM_TAG, "randomDelayTotal     %d ms", (int) randomDelayTotal);
        uint32_t randomDelayTotalFilt = filterTotal.filter(randomDelayTotal);
        ESP_LOGV(LM_TAG, "randomDelayTotalFilt %d ms", (int) randomDelayTotalFilt);
    }
}

uint32_t LoraMesher::getMaxPropagationTime() {
    return maxTimeOnAir;
}

void LoraMesher::printAllPacketsInSendQueue(int maxPrintCount = 10) {
    ToSendPackets->setInUse();  // Lock the queue for safe access
    int count = 0;  // Initialize packet count

    if (ToSendPackets->moveToStart()) {
        int totalPackets = ToSendPackets->getLength();  // Get the total number of packets in the queue
        ESP_LOGI(LM_TAG, "Total packets in the Send queue: %d", totalPackets);

        do {
            if (count >= maxPrintCount) {
                ESP_LOGI(LM_TAG, "Send Print limit reached. Showing first %d packets only.", maxPrintCount);
                break;  // Stop printing if the maximum print count is reached
            }

            QueuePacket<Packet<uint8_t>>* current = ToSendPackets->getCurrent();
            Packet<uint8_t>* packet = current->packet;

            // Format the payload as a hex string
            std::string payloadStr;
            for (int i = 0; i < packet->packetSize - sizeof(PacketHeader); i++) {
                char hexByte[4];
                snprintf(hexByte, sizeof(hexByte), "%02X ", packet->payload[i]);
                payloadStr += hexByte;
            }

            // Print packet details
            ESP_LOGI(LM_TAG, "Num: %d Pri: %2d Try: %d Sz: %2d, Src: %X, Dst: %X, Type: %3d, Id: %3d, Data: %s",
                    current->number,
                    current->priority,
                    current->resend,
                     packet->packetSize,
                     packet->src,
                     packet->dst,
                     packet->type,
                     packet->id,
                     payloadStr.c_str());

            count++;  // Increment the printed packet count
        } while (ToSendPackets->next());  // Move to next packet
    } else {
        ESP_LOGI(LM_TAG, "No packets in the Send queue.");
    }

    ToSendPackets->releaseInUse();  // Release the queue lock
}

void LoraMesher::printAllPacketsInRecvQueue(int maxPrintCount = 10) {
    ReceivedPackets->setInUse();  // Lock the queue for safe access
    int count = 0;  // Initialize packet count

    if (ReceivedPackets->moveToStart()) {
        int totalPackets = ReceivedPackets->getLength();  // Get the total number of packets in the queue
        ESP_LOGI(LM_TAG, "Total packets in the Recv queue: %d", totalPackets);

        do {
            if (count >= maxPrintCount) {
                ESP_LOGI(LM_TAG, "Recv Print limit reached. Showing first %d packets only.", maxPrintCount);
                break;  // Stop printing if the maximum print count is reached
            }

            QueuePacket<Packet<uint8_t>>* current = ReceivedPackets->getCurrent();
            Packet<uint8_t>* packet = current->packet;

            // Print packet details
            ESP_LOGI(LM_TAG, "Recv Packet -- Size: %d, Src: %X, Dst: %X, Type: %d, Id: %d",
                     packet->packetSize,
                     packet->src,
                     packet->dst,
                     packet->type,
                     packet->id);

            count++;  // Increment the printed packet count
        } while (ReceivedPackets->next());  // Move to next packet
    } else {
        ESP_LOGI(LM_TAG, "No packets in the Recv queue.");
    }

    ReceivedPackets->releaseInUse();  // Release the queue lock
}


void LoraMesher::printAllPacketsInDataQueue(int maxPrintCount = 10) {
    ReceivedAppPackets->setInUse();  // Lock the queue for safe access
    int count = 0;  // Initialize packet count

    if (ReceivedAppPackets->moveToStart()) {
        int totalPackets = ReceivedAppPackets->getLength();  // Get the total number of packets in the queue
        ESP_LOGI(LM_TAG, "Total packets in the Data queue: %d", totalPackets);

        do {
            if (count >= maxPrintCount) {
                ESP_LOGI(LM_TAG, "Data Print limit reached. Showing first %d packets only.", maxPrintCount);
                break;  // Stop printing if the maximum print count is reached
            }

            AppPacket<uint8_t>* current = ReceivedAppPackets->getCurrent();

            // Print packet details
            ESP_LOGI(LM_TAG, "Data Packet -- Size: %d, Src: %X, Dst: %X, Data[0]: %02X, Data[1]: %02X, Data[2]: %02X, Data[3]: %02X",
                     current->payloadSize,
                     current->src,
                     current->dst,
                     current->payload[0],
                     current->payload[1],
                     current->payload[2],
                     current->payload[3]);

            count++;  // Increment the printed packet count
        } while (ReceivedAppPackets->next());  // Move to next packet
    } else {
        ESP_LOGI(LM_TAG, "No packets in the Data queue.");
    }

    ReceivedAppPackets->releaseInUse();  // Release the queue lock
}


bool LoraMesher::sendPacket(Packet<uint8_t>* p) {
    waitBeforeSend(1);

    clearDioActions();

    // Print the packet to be sent
    printHeaderPacket(LM_TAG, p, "TX");

    //Blocking transmit, it is necessary due to deleting the packet after sending it. 
    int resT = radio->transmit(reinterpret_cast<uint8_t*>(p), p->packetSize);

    if ((resT == -707) || (resT == -706)) { //RADIOLIB_ERR_SPI_CMD_FAILED RADIOLIB_ERR_SPI_CMD_INVALID
        last_error_radio_on_send = resT;
        resetRadioOnSendingErrorTimes++;
        restartRadio();        
    }


    //Start receiving again after sending a packet
    startReceivingError = startReceiving(1);

    if (resT != RADIOLIB_ERR_NONE) {
        ESP_LOGE(LM_TAG, "Transmit gave error: %d", resT);
        return false;
    }
    return true;
}

void LoraMesher::sendPackets() {
    ESP_LOGV(LM_TAG, "Send routine started");
    vTaskSuspend(NULL);

    int sendCounter = 0;
    uint8_t sendId = 0;
    uint8_t resendMessage = 0;

#ifdef ARDUINO
    randomSeed(getLocalAddress());
#else
    srand(getLocalAddress());
#endif
    const uint8_t dutyCycleEvery = (100 - LM_DUTY_CYCLE) / portTICK_PERIOD_MS;

    for (;;) {

        DebugTaskCounter(pcTaskGetName(NULL));

        /* Wait for the notification of new packet has to be sent and enter blocking */
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        ESP_LOGV(LM_TAG, "Stack space unused after entering the task sendPackets: %d", uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGV(LM_TAG, "Free heap: %d", getFreeHeap());

        resendMessage = 0;


        while (ToSendPackets->getLength() > 0) {

            ToSendPackets->setInUse();

            ESP_LOGV(LM_TAG, "Size of Send Packets Queue: %d", ToSendPackets->getLength());

            QueuePacket<Packet<uint8_t>>* tx = ToSendPackets->Pop();

            ToSendPackets->releaseInUse();

            if (tx) {
                ESP_LOGV(LM_TAG, "Send n. %d", sendCounter);

                if (tx->packet->src == getLocalAddress())
                    tx->packet->id = sendId++;

                //If the packet has a data packet and its destination is not broadcast add the via to the packet and forward the packet
                if (PacketService::isDataPacket(tx->packet->type) && tx->packet->dst != BROADCAST_ADDR) {
                    uint16_t nextHop = RoutingTableService::getNextHop(tx->packet->dst);

                    //Next hop not found
                    if (nextHop == 0) {
                        ESP_LOGE(LM_TAG, "NextHop Not found from %X, destination %X", tx->packet->src, tx->packet->dst);
                        PacketQueueService::deleteQueuePacketAndPacket(tx);
                        incDestinyUnreachable();
                        continue;
                    }

                    (reinterpret_cast<DataPacket*>(tx->packet))->via = nextHop;
                }

                recordState(LM_StateType::STATE_TYPE_SENT, tx->packet);

                //Send packet
                bool hasSend = sendPacket(tx->packet);

                sendCounter++;

                if (hasSend) {
                    incSendPackets();
                    incSentPayloadBytes(PacketService::getPacketPayloadLengthWithoutControl(tx->packet));
                    incSentControlBytes(PacketService::getControlLength(tx->packet));
                    if (tx->packet->src != getLocalAddress())
                        incForwardedPackets();
                }

                //TODO: If the packet has not been send, add it to the queue and send it again

                bool noLimitResend = (tx->resend < MAX_TIMES_RESEND_PACKET);
                bool noLimitLength = (ToSendPackets->getLength() < MAX_SEND_PACKET_QUEUE_SIZE);
                bool noLimitTimeout = ((int32_t)(tx->timeout - xTaskGetTickCount()) > 0);

                ESP_LOGD(LM_TAG, "tx->timeout:%lu xTaskGetTickCount():%lu", tx->timeout, xTaskGetTickCount());
                ESP_LOGD(LM_TAG, "noLimitResend:%d noLimitLength:%d noLimitTimeout:%d", (int) noLimitResend, (int) noLimitLength, (int) noLimitTimeout);

                if (!hasSend) 
                {
                    ESP_LOGD(LM_TAG, "Message %d Not Sent", (int) sendCounter);

            
                    if(noLimitResend && noLimitLength && noLimitTimeout) {
                        ESP_LOGW(LM_TAG, "Message %d Retry", (int) sendCounter);

                        tx->priority = MAX_PRIORITY;
                        tx->resend++;
                        incReSentPackets();
                        PacketQueueService::addOrdered(ToSendPackets, tx);
                        resendMessage++;
                        continue;
                    }
                    else
                    {
                        ESP_LOGE(LM_TAG, "Message %d No Retry", (int) sendCounter);
                    }
                }
                else
                {
                    ESP_LOGD(LM_TAG, "Message %d Sent", (int) sendCounter);
                }


                //resendMessage = 0;

                uint32_t timeOnAir = radio->getTimeOnAir(tx->packet->packetSize) / 1000;

                TickType_t delayBetweenSend = timeOnAir * dutyCycleEvery;

                ESP_LOGV(LM_TAG, "TimeOnAir %d ms, next message in %d ms", (int) timeOnAir, (int) delayBetweenSend);

                PacketQueueService::deleteQueuePacketAndPacket(tx);

                vTaskDelay(delayBetweenSend / portTICK_PERIOD_MS);
            }
        }
    }
}

bool LoraMesher::isRoutingTableInQueueSend() {
    bool found = false;

    ToSendPackets->setInUse();

    if (ToSendPackets->moveToStart()) {
        do {
            QueuePacket<Packet<uint8_t>>* qp = ToSendPackets->getCurrent();

            if (PacketService::isRoutingTablePacket(qp->packet->type)) {
                found = true;
                break;
            }

        } while (ToSendPackets->next());
    }

    ToSendPackets->releaseInUse();
    return found;
}

void LoraMesher::sendRoutingTablePacket() {
    // Remove the old routing packets
    removePacketsFromSendQueue(ROUTING_P);

    size_t maxNodesPerPacket = (PacketFactory::getMaxPacketSize() - sizeof(RoutePacket)) / sizeof(NetworkNode);

    NetworkNode* nodes = RoutingTableService::getAllNetworkNodes();
    size_t numOfNodes = RoutingTableService::routingTableSize();

    size_t numPackets = (numOfNodes + maxNodesPerPacket - 1) / maxNodesPerPacket;
    numPackets = (numPackets == 0) ? 1 : numPackets;

    for (size_t i = 0; i < numPackets; ++i) {
        ESP_LOGV(LM_TAG, "Creating Routing Packet");

        size_t startIndex = i * maxNodesPerPacket;
        size_t endIndex = startIndex + maxNodesPerPacket;
        if (endIndex > numOfNodes) {
            endIndex = numOfNodes;
        }

        size_t nodesInThisPacket = endIndex - startIndex;

        // Create and send the packet
        RoutePacket* tx = PacketService::createRoutingPacket(
            getLocalAddress(), &nodes[startIndex], nodesInThisPacket, RoleService::getRole(), RoutingTableService::routingTableId
        );
        incSendRoutingTablePacketNum();
        setPackedForSend(reinterpret_cast<Packet<uint8_t>*>(tx), DEFAULT_PRIORITY + 1);
    }

    // Delete the nodes array
    if (numOfNodes > 0)
        delete[] nodes;
}

void LoraMesher::sendHelloPacketRoutine() {
    ESP_LOGV(LM_TAG, "Send Hello Packet routine started");

    vTaskSuspend(NULL);

    //Wait an initial 2 second
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    for (;;) {

        DebugTaskCounter(pcTaskGetName(NULL));

        ESP_LOGV(LM_TAG, "Stack space unused after entering the task hello packet: %d", uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGV(LM_TAG, "Free heap: %d", getFreeHeap());

        sendHelloPacket();

        // Wait for HELLO_PACKETS_DELAY seconds to send the next hello packet
        vTaskDelay(HELLO_PACKETS_DELAY * 1000 / portTICK_PERIOD_MS);
    }
}

// TODO: Hello Packet routine should be a timer reset every time a hello packet is sent
// void LoraMesher::resetHelloPacketTimer() {
//     xTaskNotify(Hello_TaskHandle, 0, eNoAction);
// }

void LoraMesher::sendHelloPacket() {
    // Remove the old hello packets
    removePacketsFromSendQueue(HELLO_P);

    size_t maxNodesPerPacket = (PacketFactory::getMaxPacketSize() - sizeof(HelloPacket)) / sizeof(HelloPacketNode);

    HelloPacketNode* node = nullptr;
    size_t routingTableSize = RoutingTableService::routingTableSize();

    size_t numOfNodes = 0;

    bool correct = RoutingTableService::getAllHelloPacketsNode(&node, &numOfNodes);
    if (!correct) {
        ESP_LOGE(LM_TAG, "Error getting all hello packets node");
        return;
    }

    size_t numPackets = (numOfNodes + maxNodesPerPacket - 1) / maxNodesPerPacket;
    numPackets = (numPackets == 0) ? 1 : numPackets;

    for (size_t i = 0; i < numPackets; ++i) {
        ESP_LOGV(LM_TAG, "Creating Hello Packet with RTId %d and RTSize %d", RoutingTableService::routingTableId, routingTableSize);

        size_t startIndex = i * maxNodesPerPacket;
        size_t endIndex = startIndex + maxNodesPerPacket;
        if (endIndex > numOfNodes) {
            endIndex = numOfNodes;
        }

        size_t nodesInThisPacket = endIndex - startIndex;

        // Create and send the packet
        HelloPacket* tx = PacketService::createHelloPacket(
            getLocalAddress(), &node[startIndex], nodesInThisPacket,
            RoutingTableService::routingTableId, routingTableSize
        );

        if (tx != nullptr) {
            incSendHelloPacketNum();
            setPackedForSend(reinterpret_cast<Packet<uint8_t> *>(tx),
                             DEFAULT_PRIORITY);
        } else {
          ESP_LOGV(LM_TAG, "Could not create packet, Hello Packet not sent");
        }            
    }

    RoutingTableService::clearAllHelloPacketsNode(node);
}



void LoraMesher::processPackets() {
    ESP_LOGV(PP_TAG, "Process routine started");
    vTaskSuspend(NULL);

    processPacketsState = 1;

    for (;;) {

        processPacketsLoop = 0;

        processPacketsState = 2;
        DebugTaskCounter(pcTaskGetName(NULL));

        processPacketsState = 3;
        ESP_LOGV(PP_TAG, "Stack space unused after entering the task processPackets: %d", uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGV(PP_TAG, "Free heap: %d", getFreeHeap());

        processPacketsState = 4;
        /* Wait for the notification of receivingRoutine and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);

        processPacketsState = 5;
        ESP_LOGV(PP_TAG, "Size of Received Packets Queue: %d", ReceivedPackets->getLength());

        processPacketsState = 6;

        while (ReceivedPackets->getLength() > 0) 
        {
            ESP_LOGV(PP_TAG, "Processing packet index %d of %d packets", processPacketsLoop, ReceivedPackets->getLength());
            processPacketsState = 7;
            QueuePacket<Packet<uint8_t>>* rx = ReceivedPackets->Pop();

            processPacketsState = 8;

            if (rx) 
            {
                processPacketsState = 9;
                uint8_t type = rx->packet->type;

#ifdef LM_TESTING
                if (!shouldProcessPacket(rx->packet)) {
                    PacketQueueService::deleteQueuePacketAndPacket(rx);
                    ESP_LOGV(PP_TAG, "TESTING: Packet not for me, deleting it");
                    continue;
                }
#endif
                ESP_LOGV(PP_TAG, "Procesing packet ReceivedPackets->Pop() ok");
                printHeaderPacket(PP_TAG, rx->packet, "RX");

                processPacketsState = 10;
                recordState(LM_StateType::STATE_TYPE_RECEIVED, rx->packet);

                incReceivedPayloadBytes(PacketService::getPacketPayloadLengthWithoutControl(rx->packet));
                incReceivedControlBytes(PacketService::getControlLength(rx->packet));

                processPacketsState = 11;
                if (PacketService::isRoutingTablePacket(type)) 
                {
                    ESP_LOGV(PP_TAG, "Received Routing Table Packet");
                    incReceivedRoutingTablePackets();
                    processPacketsState = 12;

                    bool needSendHelloPacket = RoutingTableService::processRoute(reinterpret_cast<RoutePacket*>(rx->packet), rx->snr);
                    processPacketsState = 13;
                    PacketQueueService::deleteQueuePacketAndPacket(rx);
                    processPacketsState = 14;
                    if (needSendHelloPacket)
                    {
                        ESP_LOGV(PP_TAG, "needSendHelloPacket (processRoute)");
                        processPacketsState = 15;
                        sendHelloPacket();
                    }
                        
                    processPacketsState = 16;
                }
                else if (PacketService::isHelloPacket(type)) 
                {
                    ESP_LOGV(PP_TAG, "Received HelloPacket ok");
                    processPacketsState = 17;
                    incRecHelloPackets();

                    HelloPacket* hello_p = reinterpret_cast<HelloPacket*>(rx->packet);
                    processPacketsState = 18;

                    Packet<uint8_t>* send_packet = nullptr;
                    processPacketsState = 19;
                    bool needSendHelloPacket = RoutingTableService::processHelloPacket(reinterpret_cast<HelloPacket*>(rx->packet), rx->snr, &send_packet);
                    processPacketsState = 20;
                    PacketQueueService::deleteQueuePacketAndPacket(rx);
                    processPacketsState = 21;

                    if (send_packet != nullptr) 
                    {
                        ESP_LOGV(PP_TAG, "send_packet != nullptr ok");
                        processPacketsState = 22;

                        //Ask for the updated routing table
                        incSendRTRequestPacketNum();
                        processPacketsState = 23;
                        setPackedForSend(send_packet, DEFAULT_PRIORITY);
                        processPacketsState = 24;
                    }
                    else if (needSendHelloPacket) 
                    {
                        ESP_LOGV(PP_TAG, "needSendHelloPacket (processHelloPacket)");
                        processPacketsState = 25;
                        sendHelloPacket();
                        processPacketsState = 26;
                    }
                }
                else if (PacketService::isRoutingTableRequestPacket(type)) 
                {
                    ESP_LOGV(PP_TAG, "Received Routing Table Request Packet");
                    processPacketsState = 27;
                    if (rx->packet->dst == getLocalAddress())
                    {
                        processPacketsState = 28;
                        sendRoutingTablePacket();
                        processPacketsState = 29;
                    }
                    processPacketsState = 30;
                        
                    PacketQueueService::deleteQueuePacketAndPacket(rx);
                    processPacketsState = 31;
                }
                else if (PacketService::isDataPacket(type))
                {
                    ESP_LOGV(PP_TAG, "Received Data Packet");
                    processPacketsState = 32;
                    processDataPacket(reinterpret_cast<QueuePacket<DataPacket>*>(rx));
                    processPacketsState = 33;
                }
                    
                else 
                {
                    ESP_LOGV(PP_TAG, "Packet not identified, deleting it");
                    processPacketsState = 34;
                    incReceivedNotForMe();
                    processPacketsState = 35;
                    PacketQueueService::deleteQueuePacketAndPacket(rx);
                    processPacketsState = 36;
                }
            }
            processPacketsLoop++;
        }
        processPacketsLoopLast = processPacketsLoop;
    }
}

void LoraMesher::routingTableManager() {
    ESP_LOGV(LM_TAG, "Routing Table Manager routine started");
    vTaskSuspend(NULL);

    for (;;) {

        DebugTaskCounter(pcTaskGetName(NULL));

        ESP_LOGV(LM_TAG, "Routing Table Manager routine");
        ESP_LOGV(LM_TAG, "Stack space unused after entering the task routingTableManager: %d", uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGV(LM_TAG, "Free heap: %d", getFreeHeap());

        RoutingTableService::manageTimeoutRoutingTable();
        bool send_hello_packet = RoutingTableService::checkReceivedHelloPacket();
        if (send_hello_packet) {
            ESP_LOGV(LM_TAG, "Send Hello Packet from Routing Table Manager");
            sendHelloPacket();
        }

        RoutingTableService::printRoutingTable();

        vTaskDelay(HELLO_PACKETS_DELAY * 1000 / portTICK_PERIOD_MS);
    }
}

void LoraMesher::queueManager() {
    ESP_LOGV(LM_TAG, "Queue Manager routine started");
    vTaskSuspend(NULL);

    for (;;) {

        DebugTaskCounter(pcTaskGetName(NULL));

        ESP_LOGV(LM_TAG, "Stack space unused after entering the task queueManager: %d", uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGV(LM_TAG, "Free heap: %d", getFreeHeap());

        // Record the state for the simulation
        recordState(LM_StateType::STATE_TYPE_MANAGER);

        if (q_WSP->getLength() == 0 && q_WRP->getLength() == 0) {
            ESP_LOGV(LM_TAG, "No packets to send or received");

            // Wait for the notification of send or receive reliable message and enter blocking
            ulTaskNotifyTake(
                pdTRUE,
                portMAX_DELAY);
            continue;
        }

        managerReceivedQueue();
        managerSendQueue();

        // TODO: Calculate the min timeout for the queue manager, get the min timeout from the queues
        vTaskDelay(MIN_TIMEOUT * 1000 / portTICK_PERIOD_MS);
    }
}

void LoraMesher::printHeaderPacket(const char* tag, Packet<uint8_t>* p, String title) {
    bool isDataPacket = PacketService::isDataPacket(p->type);
    bool isControlPacket = PacketService::isControlPacket(p->type);


    //tag == LM_TAG for TX
    //tag == PP_TAG for RX
    ESP_LOGV(tag, "Packet %s -- Size: %d Src: %X Dst: %X Id: %d Type: %s Via: %X Seq_Id: %d Num: %d",
        title.c_str(),
        p->packetSize,
        p->src,
        p->dst,
        p->id,
        getPacketType(p->type),
        isDataPacket ? (reinterpret_cast<DataPacket*>(p))->via : 0,
        isControlPacket ? (reinterpret_cast<ControlPacket*>(p))->seq_id : 0,
        isControlPacket ? (reinterpret_cast<ControlPacket*>(p))->number : 0);
}

void LoraMesher::sendReliablePacket(uint16_t dst, uint8_t* payload, uint32_t payloadSize) {
    // Cannot send an empty packet
    if (payloadSize == 0)
        return;
    if (dst == BROADCAST_ADDR) {
        ESP_LOGW(LM_TAG, "Be aware of sending a reliable packet to the broadcast address");
        size_t numOfNodes = RoutingTableService::routingTableSize();
        if (numOfNodes > 0) {
            NetworkNode* nodes = RoutingTableService::getAllNetworkNodes();
            for (size_t i = 0; i < numOfNodes; i++) {
                NetworkNode* node = &nodes[i];
                sendReliablePacket(node->address, payload, payloadSize);
            }
            delete[] nodes;
        }
        return;
    }
    ESP_LOGV(LM_TAG, "Sending reliable payload with %d bytes to %X", (int) payloadSize, dst);

    // Get the Routing Table node of the destination
    RouteNode* node = RoutingTableService::findNode(dst);

    if (node == NULL) {
        ESP_LOGV(LM_TAG, "Destination not found in the routing table");
        return;
    }

    //Generate a sequence Id for this list of packets
    uint8_t seq_id = getSequenceId();

    //Get the Type of the packet
    uint8_t type = XL_DATA_P;

    //Max payload size per packet
    size_t maxPayloadSize = PacketService::getMaximumPayloadLength(type);

    //Number of packets
    uint16_t numOfPackets = payloadSize / maxPayloadSize + (payloadSize % maxPayloadSize > 0);

    //Create a new Linked list to store the QueuePackets and the payload
    LM_LinkedList<QueuePacket<ControlPacket>>* packetList = new LM_LinkedList<QueuePacket<ControlPacket>>();

    //Add the SYNC configuration packet
    packetList->Append(getStartSequencePacketQueue(dst, seq_id, numOfPackets));


    for (uint16_t i = 1; i <= numOfPackets; i++) {
        //Get the position of the payload
        uint8_t* payloadToSend = reinterpret_cast<uint8_t*>((unsigned long) payload + ((i - 1) * maxPayloadSize));

        //Get the payload Size in bytes
        size_t payloadSizeToSend = maxPayloadSize;
        if (i == numOfPackets)
            payloadSizeToSend = payloadSize - (maxPayloadSize * (numOfPackets - 1));

        ESP_LOGV(LM_TAG, "Payload Size: %d", payloadSizeToSend);

        //Create a new packet with the previous payload
        ControlPacket* cPacket = PacketService::createControlPacket(dst, getLocalAddress(), type, payloadToSend, payloadSizeToSend);
        cPacket->number = i;
        cPacket->seq_id = seq_id;

        //Create a packet queue
        QueuePacket<ControlPacket>* pq = PacketQueueService::createQueuePacket(cPacket, DEFAULT_PRIORITY + 1, i);

        //Append the packet queue in the linked list
        packetList->Append(pq);
    }

    //Create the pair of configuration
    listConfiguration* listConfig = new listConfiguration();
    listConfig->config = new sequencePacketConfig(seq_id, dst, numOfPackets, node);
    listConfig->list = packetList;

    // Set the RTT of the first packet of the sequence
    listConfig->config->calculatingRTT = millis();

    // Set the timeout of the first packet of the sequence
    addTimeout(listConfig->config);

    //Add dataList pair to the waiting send packets queue
    q_WSP->setInUse();
    q_WSP->Append(listConfig);
    q_WSP->releaseInUse();

    //Send the first packet of the sequence (SYNC packet)
    sendPacketSequence(listConfig, 0);

    // Notify the queueManager that a new sequence has been started
    notifyNewSequenceStarted();
}

void LoraMesher::processDataPacket(QueuePacket<DataPacket>* pq) {
    DataPacket* packet = pq->packet;

    incReceivedDataPackets();

    ESP_LOGI(PP_TAG, "Data packet from %X, destination %X, via %X", packet->src, packet->dst, packet->via);
    ESP_LOGV(PP_TAG, "Data packet from %X, destination %X, via %X, Data: %02X %02X %02X %02X", packet->src, packet->dst, packet->via, packet->payload[0], packet->payload[1], packet->payload[2], packet->payload[3]);

    if (packet->dst == getLocalAddress()) {
        ESP_LOGV(PP_TAG, "Data packet from %X for me, type %d", packet->src);
        incDataPacketForMe();
        processDataPacketForMe(pq);
        return;
    }
    else if (packet->dst == BROADCAST_ADDR) {
        ESP_LOGV(PP_TAG, "Data packet from %X BROADCAST", packet->src);
        incReceivedBroadcast();
        processDataPacketForMe(pq);
        return;
    }
    else if (packet->via == getLocalAddress()) {
        ESP_LOGV(PP_TAG, "Data Packet from %X for %X. Via is me. Forwarding it", packet->src, packet->dst);
        incReceivedIAmVia();
        addToSendOrderedAndNotify(reinterpret_cast<QueuePacket<Packet<uint8_t>>*>(pq));
        return;
    }

    ESP_LOGV(PP_TAG, "Packet not for me, deleting it. Packet from %X for %X", packet->src, packet->dst);
    incReceivedNotForMe();
    PacketQueueService::deleteQueuePacketAndPacket(pq);
}

void LoraMesher::processDataPacketForMe(QueuePacket<DataPacket>* pq) {
    DataPacket* p = pq->packet;
    ControlPacket* cPacket = reinterpret_cast<ControlPacket*>(p);

    //By default, delete the packet queue at the finish of this function
    bool deleteQueuePacket = true;

    bool needAck = PacketService::isNeedAckPacket(p->type);

    if (PacketService::isOnlyDataPacket(p->type)) {
        ESP_LOGV(LM_TAG, "Data Packet received");
        //Convert the packet into a user packet
        AppPacket<uint8_t>* appPacket = PacketService::convertPacket(p);

        incTrackConvertedPacketsForMeNum();

        //Add and notify the user of this packet
        notifyUserReceivedPacket(appPacket);
    }
    else if (PacketService::isAckPacket(p->type)) {
        ESP_LOGV(LM_TAG, "ACK Packet received");
        addAck(p->src, cPacket->seq_id, cPacket->number);
    }
    else if (PacketService::isLostPacket(p->type)) {
        ESP_LOGV(LM_TAG, "Lost Packet received");
        processLostPacket(p->src, cPacket->seq_id, cPacket->number);
    }
    else if (PacketService::isSyncPacket(p->type)) {
        ESP_LOGV(LM_TAG, "Synchronization Packet received");
        processSyncPacket(p->src, cPacket->seq_id, cPacket->number);

        needAck = false;
    }
    else if (PacketService::isXLPacket(p->type)) {
        ESP_LOGV(LM_TAG, "Large payload Packet received");
        processLargePayloadPacket(reinterpret_cast<QueuePacket<ControlPacket>*>(pq));
        needAck = false;
        deleteQueuePacket = false;
    }
    else {
        ESP_LOGW(LM_TAG, "Packet not identified, deleting it");
    }

    //Need ack
    if (needAck) {
        //TODO: All packets with this ack will ack?
        ESP_LOGV(LM_TAG, "Previous packet need an ACK");
        sendAckPacket(p->src, cPacket->seq_id, cPacket->number);
    }

    if (deleteQueuePacket)
        //Delete packet queue
        PacketQueueService::deleteQueuePacketAndPacket(pq);
}

void LoraMesher::notifyUserReceivedPacket(AppPacket<uint8_t>* appPacket) {
    if (ReceiveAppData_TaskHandle) {
        ReceivedAppPackets->setInUse();
        //Add the packet inside the receivedUsers Queue
        ReceivedAppPackets->Append(appPacket);

        ReceivedAppPackets->releaseInUse();

        incTrackConvertedPacketsNotifyNum();

        //Notify the received user task handle
        xTaskNotify(
            ReceiveAppData_TaskHandle,
            0,
            eSetValueWithOverwrite);

    }
    else
        deletePacket(appPacket);
}

uint32_t LoraMesher::getPropagationTimeWithRandom(uint8_t multiplayer) {
    // TODO: Use the RTT or other congestion metrics to calculate the time, timeouts...
    uint32_t time = getMaxPropagationTime();
    uint32_t randomTime = random(time, time * 3 + (multiplayer + routingTableSize()) * 100);
    return randomTime;
}

void LoraMesher::recalculateMaxTimeOnAir() {
    maxTimeOnAir = radio->getTimeOnAir(PacketFactory::getMaxPacketSize()) / 1000;
    ESP_LOGV(LM_TAG, "Max Time on Air changed %d ms", (int) maxTimeOnAir);
}

void LoraMesher::recordState(LM_StateType type, Packet<uint8_t>* packet) {
    if (simulatorService == nullptr)
        return;

    simulatorService->addState(ReceivedPackets->getLength(), getSendQueueSize(),
        getReceivedQueueSize(), routingTableSize(), q_WRP->getLength(), q_WSP->getLength(),
        type, packet);
}

#ifdef LM_TESTING
bool LoraMesher::canReceivePacket(uint16_t source) {
    return true;
}
#endif

#ifdef LM_TESTING
bool LoraMesher::isDataPacketAndLocal(DataPacket* packet, uint16_t localAddress) {
    return PacketService::isDataPacket(packet->type) && packet->via == localAddress;
}

bool LoraMesher::shouldProcessPacket(Packet<uint8_t>* packet) {
    return isDataPacketAndLocal(reinterpret_cast<DataPacket*>(packet), getLocalAddress()) || canReceivePacket(packet->src);
}
#endif

/**
 *  End Region Packet Service
**/


/**
 *  Region Routing Table
**/

size_t LoraMesher::routingTableSize() {
    return RoutingTableService::routingTableSize();
}

/**
 *  End Region Routing Table
**/


/**
 *  Region PacketQueue
**/

size_t LoraMesher::getReceivedQueueSize() {
    return ReceivedAppPackets->getLength();
}

size_t LoraMesher::getSendQueueSize() {
    return ToSendPackets->getLength();
}

void LoraMesher::addToSendOrderedAndNotify(QueuePacket<Packet<uint8_t>>* qp) {
    PacketQueueService::addOrdered(ToSendPackets, qp);
    ESP_LOGD(LM_TAG, "Added packet to Q_SP, notifying sender task");

    //Notify the sendData task handle
    xTaskNotify(SendData_TaskHandle, 0, eSetValueWithOverwrite);
}

void LoraMesher::notifyNewSequenceStarted() {
    //Notify the sendData task handle
    #ifndef DEBUG_NO_USE_QUEUE_MANAGER
    xTaskNotify(QueueManager_TaskHandle, 0, eSetValueWithOverwrite);
    #endif
}

/**
 *  End Region PacketQueue
**/


/**
 * Large and Reliable payloads
 */

QueuePacket<ControlPacket>* LoraMesher::getStartSequencePacketQueue(uint16_t destination, uint8_t seq_id, uint16_t num_packets) {
    uint8_t type = SYNC_P;

    //Create the packet
    ControlPacket* cPacket = PacketService::createEmptyControlPacket(destination, getLocalAddress(), type, seq_id, num_packets);

    //Create a packet queue
    return PacketQueueService::createQueuePacket(cPacket, DEFAULT_PRIORITY, 0);
}

void LoraMesher::sendAckPacket(uint16_t destination, uint8_t seq_id, uint16_t seq_num) {
    uint8_t type = ACK_P;

    //Create the packet
    ControlPacket* cPacket = PacketService::createEmptyControlPacket(destination, getLocalAddress(), type, seq_id, seq_num);
    incSendAckPacketNum();
    setPackedForSend(reinterpret_cast<Packet<uint8_t>*>(cPacket), DEFAULT_PRIORITY + 3);
}

void LoraMesher::sendLostPacket(uint16_t destination, uint8_t seq_id, uint16_t seq_num) {
    uint8_t type = LOST_P;

    //Create the packet
    ControlPacket* cPacket = PacketService::createEmptyControlPacket(destination, getLocalAddress(), type, seq_id, seq_num);
    incSendLostPacketNum();
    setPackedForSend(reinterpret_cast<Packet<uint8_t>*>(cPacket), DEFAULT_PRIORITY + 2);
}

bool LoraMesher::sendPacketSequence(listConfiguration* lstConfig, uint16_t seq_num) {
    // Check if the sequence number requested is valid
    if (lstConfig->config->lastAck > seq_num) {
        ESP_LOGE(LM_TAG, "Trying to send packet sequence previously acknowledged Seq_id: %d, Num: %d", lstConfig->config->seq_id, seq_num);
        return false;
    }

    //Get the packet queue with the sequence number
    QueuePacket<ControlPacket>* pq = PacketQueueService::findPacketQueue(lstConfig->list, seq_num);

    if (pq == nullptr) {
        ESP_LOGE(LM_TAG, "NOT FOUND the packet queue with Seq_id: %d, Num: %d", lstConfig->config->seq_id, seq_num);
        return false;
    }

    //Create the packet
    Packet<uint8_t>* p = PacketService::copyPacket(pq->packet, pq->packet->getPacketLength());

    //Add the packet to the send queue
    incSendPacketSequenceNum();
    setPackedForSend(p, DEFAULT_PRIORITY);

    return true;
}

void LoraMesher::addAck(uint16_t source, uint8_t seq_id, uint16_t seq_num) {
    listConfiguration* config = findSequenceList(q_WSP, seq_id, source);
    if (config == nullptr) {
        ESP_LOGE(LM_TAG, "NOT FOUND the sequence packet config in add ack with Seq_id: %d, Source: %d", seq_id, source);
        return;
    }

    //If all packets has been arrived to the destiny
    //Delete this sequence
    if (config->config->number == seq_num) {
        ESP_LOGI(LM_TAG, "All the packets has been arrived to the seq_Id: %d", seq_id);
        findAndClearLinkedList(q_WSP, config);
        return;
    }

    if (config->config->lastAck > seq_num) {
        ESP_LOGE(LM_TAG, "ACK received that has been yet acknowledged Seq_id: %d, Num: %d", config->config->seq_id, seq_num);
        return;
    }

    //Set has been received some ACK
    config->config->firstAckReceived = 1;

    // TODO: Check for repeated ACKs and packets.

    //Add the last ack to the config packet
    config->config->lastAck = seq_num;

    // Recalculate the RTT
    actualizeRTT(config->config);

    //Reset the timeouts
    resetTimeout(config->config);

    ESP_LOGV(LM_TAG, "Sending next packet after receiving an ACK");

    //Send the next packet sequence
    sendPacketSequence(config, seq_num + 1);
}

bool LoraMesher::processLargePayloadPacket(QueuePacket<ControlPacket>* pq) {
    ControlPacket* cPacket = pq->packet;

    listConfiguration* configList = findSequenceList(q_WRP, cPacket->seq_id, cPacket->src);
    if (configList == nullptr) {
        ESP_LOGE(LM_TAG, "NOT FOUND the sequence packet config in Process Large Payload with Seq_id: %d, Source: %d", cPacket->seq_id, cPacket->src);
        PacketQueueService::deleteQueuePacketAndPacket(pq);
        return false;
    }

    if (configList->config->lastAck + 1 != cPacket->number) {
        ESP_LOGE(LM_TAG, "Sequence number received in bad order in seq_Id: %d, received: %d expected: %d", cPacket->seq_id, cPacket->number, configList->config->lastAck + 1);
        sendLostPacket(cPacket->src, cPacket->seq_id, configList->config->lastAck + 1);

        PacketQueueService::deleteQueuePacketAndPacket(pq);
        return false;
    }

    configList->config->lastAck++;

    configList->list->setInUse();
    configList->list->Append(pq);
    configList->list->releaseInUse();

    //Send ACK
    sendAckPacket(cPacket->src, cPacket->seq_id, cPacket->number);

    // Recalculate the RTT
    actualizeRTT(configList->config);

    // Reset the timeouts
    resetTimeout(configList->config);

    //All packets has been arrived, join them and send to the user
    if (configList->config->lastAck == configList->config->number) {
        joinPacketsAndNotifyUser(configList);
        return true;
    }

    return true;
}

void LoraMesher::joinPacketsAndNotifyUser(listConfiguration* listConfig) {
    ESP_LOGV(LM_TAG, "Joining packets seq_Id: %d Src: %X", listConfig->config->seq_id, listConfig->config->source);

    LM_LinkedList<QueuePacket<ControlPacket>>* list = listConfig->list;

    list->setInUse();
    if (!list->moveToStart()) {
        list->releaseInUse();
        return;
    }

    //TODO: getPacketPayloadLength could be done when adding the packets inside the list
    size_t payloadSize = 0;
    size_t number = 1;

    do {
        ControlPacket* currentP = reinterpret_cast<ControlPacket*>(list->getCurrent()->packet);

        if (number != (currentP->number))
            //TODO: ORDER THE PACKETS if they are not ordered?
            ESP_LOGE(LM_TAG, "Wrong packet order");

        number++;
        payloadSize += PacketService::getPacketPayloadLength(currentP);
    } while (list->next());

    //Move to start again
    list->moveToStart();

    ControlPacket* currentP = list->getCurrent()->packet;

    uint32_t appPacketLength = sizeof(AppPacket<uint8_t>);

    //Packet length = size of the packet + size of the payload
    uint32_t packetLength = appPacketLength + payloadSize;

    AppPacket<uint8_t>* p = static_cast<AppPacket<uint8_t>*>(pvPortMalloc(packetLength));

    ESP_LOGV(LM_TAG, "Large Packet Packet length: %d Payload Size: %d", (int) packetLength, payloadSize);

    if (p) {
        DebugHeapAllocateCount((void*)p, packetLength);
        #if USE_ALLOCATION_APP_PACKET
        DebugHeapOnAllocation(ALLOCATION_APP_PACKET, (void*)p, packetLength);
        #else
        DebugHeapOnAllocationSkipAppPacket();
        #endif
        //Copy the payload into the packet
        unsigned long actualPayloadSizeDst = appPacketLength;

        do {
            currentP = list->getCurrent()->packet;

            size_t actualPayloadSizeSrc = PacketService::getPacketPayloadLength(currentP);

            memcpy(reinterpret_cast<void*>((unsigned long) p + (actualPayloadSizeDst)), currentP->payload, actualPayloadSizeSrc);
            actualPayloadSizeDst += actualPayloadSizeSrc;
        } while (list->next());
    }
    else
    {
        #if USE_ALLOCATION_APP_PACKET
        DebugHeapOnAllocationFail(ALLOCATION_APP_PACKET, packetLength);
        #else
        DebugHeapOnAllocationFailSkipAppPacket();
        #endif
    }

    list->releaseInUse();

    //Set values to the AppPacket
    p->payloadSize = payloadSize;
    p->src = listConfig->config->source;
    p->dst = getLocalAddress();

    //TODO: When finished, clear everything? Or maintain the config until timeout?
    findAndClearLinkedList(q_WRP, listConfig);

    notifyUserReceivedPacket(p);
}

void LoraMesher::processSyncPacket(uint16_t source, uint8_t seq_id, uint16_t seq_num) {
    //Check for repeated sequence lists
    listConfiguration* listConfig = findSequenceList(q_WRP, seq_id, source);

    if (listConfig == nullptr) {
        // Get the Routing Table node of the destination
        RouteNode* node = RoutingTableService::findNode(source);

        if (node == nullptr) {
            ESP_LOGW(LM_TAG, "Node not found in the routing table");
            return;
        }

        //Create the pair of configuration
        listConfig = new listConfiguration();
        listConfig->config = new sequencePacketConfig(seq_id, source, seq_num, node);
        listConfig->list = new LM_LinkedList<QueuePacket<ControlPacket>>();

        // Starting to calculate RTT
        actualizeRTT(listConfig->config);

        //Add list configuration to the waiting received packets queue
        q_WRP->setInUse();
        q_WRP->Append(listConfig);
        q_WRP->releaseInUse();

        // Reset the timeout
        addTimeout(listConfig->config);

        // Notify the queueManager that a new sequence has been started
        notifyNewSequenceStarted();

        //Change the number to send the ack to the correct one
        //cPacket->number in SYNC_P specify the number of packets and it needs to ACK the 0
        sendAckPacket(source, seq_id, 0);
    }
}

void LoraMesher::processLostPacket(uint16_t destination, uint8_t seq_id, uint16_t seq_num) {
    //Find the list config
    listConfiguration* listConfig = findSequenceList(q_WSP, seq_id, destination);

    if (listConfig == nullptr) {
        ESP_LOGE(LM_TAG, "NOT FOUND the sequence packet config in ost packet with Seq_id: %d, Source: %d", seq_id, destination);
        return;
    }

    //TODO: Check for duplicate consecutive lost packets, set a timeout to resend the lost packet.
    // Recalculate the RTT
    actualizeRTT(listConfig->config);

    // Reset the timeout
    resetTimeout(listConfig->config);

    // First ack received is set to 1, this counts as a ack received. 
    // If the first sync is received but the first ack is not, then the receiver will send a first lost packet.
    listConfig->config->firstAckReceived = 1;

    //Send the packet sequence that has been lost
    if (sendPacketSequence(listConfig, seq_num)) {
        listConfig->config->numberOfTimeouts++;
        //Reset the timeout of this sequence packets inside the q_WSP
        recalculateTimeoutAfterTimeout(listConfig->config);
    }
}

void LoraMesher::addTimeout(LM_LinkedList<listConfiguration>* queue, uint8_t seq_id, uint16_t source) {
    listConfiguration* config = findSequenceList(q_WSP, seq_id, source);
    if (config == nullptr) {
        ESP_LOGE(LM_TAG, "NOT FOUND the sequence packet config in add timeout with Seq_id: %d, Source: %d", seq_id, source);
        return;
    }

    addTimeout(config->config);
}

void LoraMesher::resetTimeout(sequencePacketConfig* configPacket) {
    configPacket->numberOfTimeouts = 0;
    addTimeout(configPacket);
}

void LoraMesher::actualizeRTT(sequencePacketConfig* config) {
    if (config->calculatingRTT == 0) {
        //Set the first RTT received time
        config->calculatingRTT = millis();
        ESP_LOGV(LM_TAG, "Starting to calculate RTT seq_Id: %d Src: %X",
            config->seq_id, config->source);
        return;
    }

    RouteNode* node = config->node;

    if (node == nullptr) {
        ESP_LOGW(LM_TAG, "Node not found in the routing table");
        return;
    }

    unsigned long actualRTT = millis() - config->calculatingRTT;

    // First time RTT is calculated for this node (RFC 6298)
    if (node->SRTT == 0) {
        node->SRTT = actualRTT;
        node->RTTVAR = actualRTT / 2;
    }
    else {
        unsigned long absRTT = (node->SRTT > actualRTT) ? (node->SRTT - actualRTT) : (actualRTT - node->SRTT);
        node->RTTVAR = std::min((node->RTTVAR * 3 + absRTT) / 4, 100000UL);
        node->SRTT = std::min((node->SRTT * 7 + actualRTT) / 8, 100000UL);
    }

    config->calculatingRTT = millis();

    ESP_LOGV(LM_TAG, "Updating RTT (%u ms), SRTT (%u), RTTVAR (%u) seq_Id: %d Src: %X",
        (unsigned int) actualRTT, (unsigned int) node->SRTT, (unsigned int) node->RTTVAR, config->seq_id, config->source);
}

void LoraMesher::clearLinkedList(listConfiguration* listConfig) {
    LM_LinkedList<QueuePacket<ControlPacket>>* list = listConfig->list;
    list->setInUse();
    ESP_LOGI(LM_TAG, "Clearing list configuration Seq_Id: %d Src: %X", listConfig->config->seq_id, listConfig->config->source);

    size_t listSize = list->getLength();

    ESP_LOGV(LM_TAG, "List size: %d", listSize);

    for (int i = 0; i < listSize; i++) {
        QueuePacket<ControlPacket>* current = list->getCurrent();
        PacketQueueService::deleteQueuePacketAndPacket(current);
        list->DeleteCurrent();
    }

    delete list;
    delete listConfig->config;
    delete listConfig;
}

void LoraMesher::findAndClearLinkedList(LM_LinkedList<listConfiguration>* queue, listConfiguration* listConfig) {
    queue->setInUse();

    if (!queue->Search(listConfig)) {
        ESP_LOGE(LM_TAG, "Not found list config");
        queue->releaseInUse();
    }

    clearLinkedList(listConfig);

    queue->DeleteCurrent();

    queue->releaseInUse();
}

LoraMesher::listConfiguration* LoraMesher::findSequenceList(LM_LinkedList<listConfiguration>* queue, uint8_t seq_id, uint16_t source) {
    queue->setInUse();

    if (queue->moveToStart()) {
        do {
            listConfiguration* current = queue->getCurrent();

            if (current->config->seq_id == seq_id && current->config->source == source) {
                queue->releaseInUse();
                return current;
            }

        } while (queue->next());
    }

    queue->releaseInUse();

    return nullptr;

}

void LoraMesher::managerReceivedQueue() {
    managerTimeouts(q_WRP, QueueType::WRP);
}

void LoraMesher::managerSendQueue() {
    managerTimeouts(q_WSP, QueueType::WSP);
}

void LoraMesher::managerTimeouts(LM_LinkedList<listConfiguration>* queue, QueueType type) {
    String queueName;
    if (type == QueueType::WRP) {
        queueName = F("Waiting Received Queue");
    }
    else {
        queueName = F("Waiting Send Queue");
    }

    ESP_LOGV(LM_TAG, "Checking %s timeouts. Open connections %d", queueName.c_str(), queue->getLength());

    queue->setInUse();

    if (queue->moveToStart()) {
        do {
            listConfiguration* current = queue->getCurrent();

            // Get Config packet
            sequencePacketConfig* configPacket = current->config;

            // If Config packet has reached timeout
            if (configPacket->timeout < millis()) {
                // Increment number of timeouts
                configPacket->numberOfTimeouts++;

                // Description of the timeout:
                // The number of the packet would be the following: 
                // If it is a sender it starts from 0 to n + 1 packets, that includes the sync packet: If num = 0, it is that the sync packet has been lost, if num > 0, it is that the packet num - 1 has been lost
                // For the the receiver it starts from 0 to n packets
                ESP_LOGW(LM_TAG, "%s timeout reached, Src: %X, Seq_Id: %d, Num: %d, N.TimeOuts %d",
                    queueName.c_str(), configPacket->source, configPacket->seq_id, configPacket->lastAck + configPacket->firstAckReceived, configPacket->numberOfTimeouts);

                // If number of timeouts is greater than Max timeouts, erase it
                if (configPacket->numberOfTimeouts >= MAX_TIMEOUTS) {
                    ESP_LOGE(LM_TAG, "%s, MAX TIMEOUTS reached, erasing Id: %d", queueName.c_str(), configPacket->seq_id);
                    clearLinkedList(current);
                    queue->DeleteCurrent();
                    continue;
                }

                // Recalculate the timeout
                recalculateTimeoutAfterTimeout(configPacket);

                if (type == QueueType::WRP) {
                    // Send Last ACK + 1 (Request this packet)
                    sendLostPacket(configPacket->source, configPacket->seq_id, configPacket->lastAck + 1);
                    // TODO: Penalize the node if is at one hop, we need to know witch via node are we using to penalize it
                    // bool updated_rt = RoutingTableService::penalizeNodeReceivedLinkQuality(configPacket->source);
                    // if (updated_rt) {
                    //     sendHelloPacket();
                    // }
                }
                else {
                    // Repeat the configPacket ACK
                    if (configPacket->firstAckReceived == 0) {
                        // Send the first packet of the sequence (SYNC packet)
                        sendPacketSequence(current, 0);
                        // TODO: Penalize the node if is at one hop, we need to know witch via node are we using to penalize it
                        // bool updated_rt = RoutingTableService::penalizeNodeReceivedLinkQuality(configPacket->source);
                        // if (updated_rt) {
                        //     sendHelloPacket();
                        // }
                    }
                }
            }

            vTaskDelay(1);

        } while (queue->next());
    }

    queue->releaseInUse();
}

unsigned long LoraMesher::getMaximumTimeout(sequencePacketConfig* configPacket) {
    uint8_t hops = configPacket->node->networkNode.hop_count;
    if (hops == 0) {
        ESP_LOGE(LM_TAG, "Find next hop in add timeout");
        return 100000;
    }

    return 60000 + hops * 5000;//LM_RT_TIMEOUT * 1000 * hops;
}

unsigned long LoraMesher::calculateTimeout(sequencePacketConfig* configPacket) {
    //TODO: This timeout should account for the number of send packets waiting to send + how many time between send packets?
    //TODO: This timeout should be a little variable depending on the duty cycle. 
    //TODO: Account for how many hops the packet needs to do
    //TODO: Account for how many packets are inside the Q_SP
    uint8_t hops = configPacket->node->networkNode.hop_count;
    if (hops == 0) {
        ESP_LOGE(LM_TAG, "Find next hop in add timeout");
        return MIN_TIMEOUT * 1000;
    }

    if (configPacket->node->SRTT == 0)
        // TODO: The default timeout should be enough smaller to prevent unnecessary timeouts.
        // TODO: Testing the default value
        return MIN_TIMEOUT * 1000 + hops * 5000;

    unsigned long calculatedTimeout = configPacket->node->SRTT + 4 * configPacket->node->RTTVAR;
    unsigned long maxTimeout = getMaximumTimeout(configPacket);

    if (calculatedTimeout > maxTimeout)
        return maxTimeout;

    return (unsigned long) max((int) calculatedTimeout, (int) MIN_TIMEOUT * 1000 + hops * 5000);
}

void LoraMesher::addTimeout(sequencePacketConfig* configPacket) {
    unsigned long timeout = calculateTimeout(configPacket);

    configPacket->timeout = millis() + timeout;
    configPacket->previousTimeout = timeout;

    ESP_LOGV(LM_TAG, "Timeout set to %u s", (unsigned int) (timeout / 1000));
}

void LoraMesher::recalculateTimeoutAfterTimeout(sequencePacketConfig* configPacket) {
    unsigned long timeout = calculateTimeout(configPacket);
    // TODO: The following prevTimeout function should be tested
    unsigned long prevTimeout = log(configPacket->numberOfTimeouts + 1) * 50000 + ToSendPackets->getLength() * 3000;

    if (prevTimeout > timeout)
        timeout = prevTimeout;

    unsigned long maxTimeout = getMaximumTimeout(configPacket);
    if (timeout > maxTimeout)
        timeout = maxTimeout;

    // if (timeout < configPacket->previousTimeout) {
    //     timeout = configPacket->previousTimeout * 2;

    //     unsigned long maxTimeout = getMaximumTimeout(configPacket);
    //     if (timeout > maxTimeout)
    //         timeout = maxTimeout;
    // }

    configPacket->timeout = millis() + timeout;
    configPacket->previousTimeout = timeout;

    ESP_LOGV(LM_TAG, "Timeout recalculated to %u s", (unsigned int) (timeout / 1000));
}

uint8_t LoraMesher::getSequenceId() {
    if (sequence_id == 255) {
        sequence_id = 0;
        return 255;
    }

    uint8_t seqId = sequence_id;
    sequence_id++;

    return seqId;
}

void LoraMesher::rolesUpdated() {
    RoutingTableService::routingTableId++;

    sendHelloPacket();
}

void LoraMesher::removePacketsFromSendQueue(uint8_t type) {
    ToSendPackets->setInUse();
    if (ToSendPackets->moveToStart()) {
        do {
            QueuePacket<Packet<uint8_t>>* current = ToSendPackets->getCurrent();
            Packet<uint8_t>* packet = current->packet;
            if (packet->type == type) {
                ESP_LOGV(LM_TAG, "Deleting packet from the send queue of type %d", type);
                PacketQueueService::deleteQueuePacketAndPacket(current);
                ToSendPackets->DeleteCurrent();
                if (ToSendPackets->getCurrent() == nullptr)
                    break;
            }
        } while (ToSendPackets->next());
    }
    ToSendPackets->releaseInUse();
}

/**
 * End Large and Reliable payloads
 */
