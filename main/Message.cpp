#include "Message.h"

#include "Utils.h"


Message::Message(MessageType type, const uint8_t* ext_data, size_t length) : type(type), length(length) {
        if (length > MAX_MESSAGE_DATA_SIZE) {
            length = MAX_MESSAGE_DATA_SIZE; // clip message data
            printDebug(String("ERROR: Clipping of message data! Length: ") + length);
        } 
        memcpy(data, ext_data, length);
    }

Message Message::decode(const uint8_t* buffer, size_t length) { //replace with polymorphic
    printDebugTelemetry(String("Decoding message of byte length: ") + length);

    if (length < 2) { // Should NEVER occur
      printDebug("ERROR: Empty message initialised with incorrect length!");
      return Message();
    }

    MessageType t = static_cast<MessageType>(buffer[1]);
    if (length = 2) {
        return Message(t);
    }

    return Message(t, buffer + 2, length - 2); // buffer[2] is the start of the data, and data length is len - 2
}


bool Message::isEmpty() {
    return type == MessageType::EMPTY;
}
