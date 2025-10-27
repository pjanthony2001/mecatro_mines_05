#ifndef MESSAGE_H
#define MESSAGE_H

#include "Parameters.h"

#include <Arduino.h>


enum class MessageType : uint8_t {
  EMPTY     = 0x00,
  HEARTBEAT = 0x01,
  CONTROL   = 0x02,
  DATA      = 0x03,
};

class Message {

    // MESSAGE STRUCTURE: [MESSAGE_LENGTH: 1 Byte] [MESSAGE_TYPE: 1 Byte] [MESSAGE_DATA: (MESSAGE_LENGTH - 2) Bytes]
    public:
        Message() : type(MessageType::EMPTY) {}
        Message(MessageType type) : type(type) {}
        Message(MessageType type, const uint8_t* ext_data, size_t length);


        static Message decode(const uint8_t* buffer, size_t length);
        bool isEmpty();
        MessageType type; // should make private
        
    private:
        uint8_t data[MAX_MESSAGE_DATA_SIZE];
        size_t length;
};


#endif

