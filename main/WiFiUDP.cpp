#include "WiFiUDP.h"

#include "Parameters.h"
#include "Utils.h"

#include <stdio.h>


WiFiUDP_AP::WiFiUDP_AP(
    const char* SSID, 
    const char* password) : SSID(SSID), password(password), apStatus(WL_IDLE_STATUS), sendInterval(0), sequence(0), tcpServer(5000) {
    }

void WiFiUDP_AP::begin() {
    printDebug(String("Creating access point named: ") + SSID);
    
    int status = WiFi.beginAP(SSID, password);
    while (status != WL_AP_LISTENING) {
      printDebug("AP failed to start, retrying after 2 seconds");
      unsigned long timenow = millis();
      while (millis() - timenow < 2000) ;;
      status = WiFi.beginAP(SSID, password);
    }
    printDebug("AP started");
    IPAddress ip = WiFi.localIP();

    printDebug(String("IP Address: ") + ip.toString());
    delay(1000);

    if (isAPOnline()) { // CHANGE THIS DISGUSTANG
      printDebug("AP is online !");
    }  else {
      printDebug("AP is offline ! Error occurred, blocking all");
      while(true) ;;
    }

    tcpServer.begin();
    udpServer.begin(LOCAL_UDP_PORT);
    udpServer.flush(); // Clear the udp buffer before reading

    printDebug("Servers started. Waiting for client to connect");
}


void WiFiUDP_AP::sendBatch(CircularBuffer<Sample>& buffer) {
  // The client should be connected before sending a batch
  Sample batch[SAMPLE_BATCH_SIZE];
  if (buffer.available() >= SAMPLE_BATCH_SIZE) {
      buffer.popBatch(batch, SAMPLE_BATCH_SIZE);
      udpServer.beginPacket(clientIP, CLIENT_UDP_PORT);
      udpServer.write((uint8_t*)batch, sizeof(batch));
      udpServer.endPacket();
  }    


};

void WiFiUDP_AP::updateClientIP() {
  // The client should be connected before finding the client IP
    clientIP = tcpClient.remoteIP();
    clientTcpPort = tcpClient.remotePort();

    if (static_cast<uint32_t>(clientIP) == 0) {
      printDebug("ERROR: Tying to update client IP without any acknoledgement");
    }

    printDebug(String("Client IP: ") + clientIP.toString());
};



bool WiFiUDP_AP::isAPOnline() { // CHANGE ALL THIS
  int currStatus = WiFi.status();
  if (currStatus != apStatus) {
    apStatus = currStatus;
  }
  return apStatus == WL_AP_LISTENING || apStatus == WL_AP_CONNECTED;
}




Message WiFiUDP_AP::rxMessage() {
  tcpClient = tcpServer.available();
  
  if (!tcpClient || !tcpClient.connected()) {
    printDebugTelemetry("No client connected !");
    return Message();
  }

  if (tcpClient.available() < 2) {
    printDebugTelemetry("No message has been sent ! ");
    return Message();
  }

  
  int length = tcpClient.peek();
  if (tcpClient.available() < length) { // Message not fully recieved
    return Message();
  }

  uint8_t recvBuf[MAX_MESSAGE_SIZE];
  int bytesRead = tcpClient.read(recvBuf, length); // assert bytesRead == length
  return Message::decode(recvBuf, length);

}


void WiFiUDP_AP::messageCheck() {
  unsigned long timeNow = millis();
  if (connectionStatus && (timeNow - lastHeartbeat > HEARTBEAT_TIMEOUT)) { // The logic could be changed to be better
      updateConnectionStatus(false);
      lastHeartbeat = 0;
  } else {
    Message msgRecieved = rxMessage();
    while (!msgRecieved.isEmpty()) {
      handleMessage(msgRecieved);
      msgRecieved = rxMessage();
    }
  }
}

bool WiFiUDP_AP::isConnected() {
  return connectionStatus;
}

void WiFiUDP_AP::updateConnectionStatus(bool newStatus) {
  if (newStatus && !connectionStatus) {
    printDebug("Client is connected !");
    updateClientIP();
  } else if (!newStatus && connectionStatus) {
    printDebug("Client is disconnected !");
  }

  connectionStatus = newStatus;
}

void WiFiUDP_AP::handleMessage(const Message& msg) {
  switch (msg.type) {
      case MessageType::HEARTBEAT : {
        updateConnectionStatus(true);
        lastHeartbeat = millis();
        printDebug(String("HEARTBEAT RECIEVED at: ") + lastHeartbeat);
        break;
      }
      // OTHER CASES IN EXPANSION

      case MessageType::EMPTY:
        // MAYBE DO SOMETHING 
        break;

      default:
        break;
  }
}
