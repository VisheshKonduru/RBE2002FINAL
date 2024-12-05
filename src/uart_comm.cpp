// uart_comm.cpp
#include "uart_comm.h"

#include "uart_comm.h"

void UARTComm::begin(unsigned long baudRate) {
  Serial1.begin(baudRate);
}

void UARTComm::sendMessage(const String& message) {
  Serial1.println(message);
}

bool UARTComm::receiveMessage(String& message) {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      return true; // Message complete
    } else {
      message += c;
    }
  }
  return false;
}