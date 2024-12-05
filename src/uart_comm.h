// uart_comm.h
#pragma once

#include <Arduino.h>

class UARTComm {
public:
    void begin(unsigned long baudRate = 115200);
    void sendMessage(const String& message);
    bool receiveMessage(String& message);
};