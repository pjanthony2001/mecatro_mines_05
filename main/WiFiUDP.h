#ifndef WIFI_UDP_STREAMER_AP_H
#define WIFI_UDP_STREAMER_AP_H

#include "Message.h"
#include "CircularSampleBuffer.h"
#include "Sample.h"

#include <WiFiS3.h>
#include <WiFiUdp.h>


class WiFiUDP_AP {
public:
    WiFiUDP_AP(const char* SSID, 
        const char* password);

    void begin();
    void sendBatch(CircularBuffer<Sample>& buffer);
    void checkHandshake();
    bool statusChanged();
    bool isAPOnline();
    void updateClientIP();

    Message rxMessage();
    void messageCheck();
    bool isConnected();

    void updateConnectionStatus(bool newStatus);
    void handleMessage(const Message& msg);
    
\
private:
    const char* SSID;
    const char* password;

    WiFiUDP udpServer;
    WiFiServer tcpServer;

    int apStatus;
    bool connectionStatus;

    IPAddress clientIP;
    int clientTcpPort;
    WiFiClient tcpClient;

    unsigned long lastHeartbeat;
    unsigned long sendInterval;
    unsigned long sequence;



};

#endif
