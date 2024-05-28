#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include "arduino_secrets.h"

#define VALUE_SIZE 4
#define NUM_CAMERA_PORTS 3
#define MESSAGE_SIZE NUM_CAMERA_PORTS * VALUE_SIZE

struct I2CCommand {
    int port;
};

unsigned int I2C_ADDRESS = 52;

char wifiSSID[] = SECRET_WIFI_SSID;
char wifiPassword[] = SECRET_WIFI_PASSWORD;

unsigned int udpPort = 57121;
String macAddress;

WiFiUDP udp;
uint8_t udpPacketBuffer[MESSAGE_SIZE] = {0};
float cameraData[NUM_CAMERA_PORTS] = {0.0f, 0.0f, 0.0f};
I2CCommand command;

void printWiFiParameters() {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("Local IP Address: ");
    Serial.println(ip);

    long rssi = WiFi.RSSI();
    Serial.print("Signal strength: ");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void failureLoop() {
    Serial.println("Error occurred, intentionally looping infinitely.");
    while(true);
}

void handleI2CEvent(int numBytes) {
    // Serial.print("Received I2C event ");
    if (numBytes > 0) {
        command.port = Wire.read();
    }
}

inline float sig_fminf(float a, float b) {
    float r;
    r = (a < b) ? a : b;
    return r;
}

inline float sig_fmaxf(float a, float b) {
    float r;
    r = (a > b) ? a : b;
    return r;
}

inline float sig_clamp(float value, float min, float max) {
    return sig_fminf(sig_fmaxf(value, min), max);
}

int16_t floatToInt16(float value) {
    return (int16_t) value;
    // float scaled = value > 0.0f ? value * 32768.0f : value * 32767.0f;
    // int16_t converted = (((int16_t) (scaled + 32768.5)) - 32768);

    // return converted;
}


float bytesToFloat(uint8_t *bytes, bool big_endian) {
    float f;
    uint8_t *f_ptr = (uint8_t *) &f;

    if (big_endian) {
        f_ptr[3] = bytes[0];
        f_ptr[2] = bytes[1];
        f_ptr[1] = bytes[2];
        f_ptr[0] = bytes[3];
    } else {
        f_ptr[3] = bytes[3];
        f_ptr[2] = bytes[2];
        f_ptr[1] = bytes[1];
        f_ptr[0] = bytes[0];
    }

    return f;
}

void handleI2CRequest() {
    int port = command.port;
    // Serial.print("Received I2C request for port " );
    // Serial.println(port);

    float cameraDataForPort = port < NUM_CAMERA_PORTS ? cameraData[port] : 0.0f;
    int16_t dataToSend = floatToInt16(cameraDataForPort);
    const uint8_t msb = dataToSend >> 8;
    const uint8_t lsb = dataToSend & 0xff;

    Wire.write(msb);
    Wire.write(lsb);
}

void setupMACAddress() {
    unsigned char macAddressBytes[6];
    WiFi.macAddress(macAddressBytes);
    macAddress = macAddressToString(macAddressBytes);
}

void connectWiFi() {
    int wifiStatus = WL_IDLE_STATUS;

    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        failureLoop();
    }

    setupMACAddress();
    Serial.print("MAC Address: ");
    Serial.println(macAddress.c_str());

    while (wifiStatus != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(wifiSSID);
        wifiStatus = WiFi.begin(wifiSSID, wifiPassword);
        delay(5000);
    }

    Serial.println("Connected to WiFi.");
    printWiFiParameters();
}

void setupI2C() {
    Serial.print("Setting up as an I2C peripheral at address ");
    Serial.println(I2C_ADDRESS);

    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(handleI2CEvent);
    Wire.onRequest(handleI2CRequest);
}

void setupUDPListener() {
    int result = udp.begin(udpPort);
    if (!result) {
        Serial.print("Error opening local UDP socket on port ");
        Serial.println(udpPort);
        failureLoop();
    }
}

void setup() {
    Serial.begin(9600);
    delay(2500);

    setupI2C();
    connectWiFi();
    setupUDPListener();
}

String macAddressToString(unsigned char* macAddress) {
    return String(macAddress[5], HEX) +
        String(macAddress[4], HEX) +
        String(macAddress[3], HEX) +
        String(macAddress[2], HEX) +
        String(macAddress[1], HEX) +
        String(macAddress[0], HEX);
}

void loop() {
    // TODO: Periodically check the Wifi and UDP connection status
    // and reconnect if needed.
    if (udp.parsePacket()) {
        udp.read(udpPacketBuffer, MESSAGE_SIZE);

        for (int i = 0; i < NUM_CAMERA_PORTS; i++) {
            size_t offset = i * VALUE_SIZE;
            float value = bytesToFloat(udpPacketBuffer + offset, false);
            cameraData[i] = value;
        }

        // Serial.print("Camera data received from ");
        // Serial.print(udp.remoteIP());
        // Serial.print(": ");
        // Serial.print(cameraData[0]);
        // Serial.print(", ");
        // Serial.print(cameraData[1]);
        // Serial.print(", ");
        // Serial.println(cameraData[2]);
    }
}

