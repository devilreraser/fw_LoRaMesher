#include "PacketQueueService.h"




void PacketQueueService::addOrdered(LM_LinkedList<QueuePacket<Packet<uint8_t>>>* list, QueuePacket<Packet<uint8_t>>* qp) {
    list->setInUse();
    if (list->moveToStart()) {
        do {
            QueuePacket<Packet<uint8_t>>* current = list->getCurrent();
            if (current->priority < qp->priority) {
                list->addCurrent(qp);
                list->releaseInUse();
                return;
            }
        } while (list->next());
    }

    list->Append(qp);

    list->releaseInUse();
}

// #define MAX_QUEUE_SIZE 32  // Define the maximum allowed size for the queue
// void PacketQueueService::addOrdered(LM_LinkedList<QueuePacket<Packet<uint8_t>>>* list, QueuePacket<Packet<uint8_t>>* qp) {
//     list->setInUse();

//     // Insert the new packet in the correct order based on priority
//     if (list->moveToStart()) {
//         do {
//             QueuePacket<Packet<uint8_t>>* current = list->getCurrent();
//             if (current->priority < qp->priority) {
//                 list->addCurrent(qp);
//                 break;
//             }
//         } while (list->next());
//     } else {
//         // If list is empty or qp has the lowest priority, append to the end
//         list->Append(qp);
//     }

//     // memory leak
//     // // Check if the queue exceeds the maximum size and remove the oldest element if necessary
//     // if (list->getLength() > MAX_QUEUE_SIZE) {
//     //     ESP_LOGW(LM_TAG, "Queue size exceeded %d. Removing the oldest packet.", MAX_QUEUE_SIZE);
//     //     list->moveToStart();  // Move to the oldest packet
//     //     list->DeleteCurrent();  // Remove the oldest packet
//     // }

//     list->releaseInUse();
// }
