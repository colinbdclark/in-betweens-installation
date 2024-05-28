#include <Wire.h>

void setupI2C() {
    Wire.begin();
}

void setup() {
    Serial.begin(9600);
    delay(2500);

    setupI2C();
}

void loop() {
    unsigned int num = random(0, 256);
    Wire.beginTransmission(52);
    Wire.write(num);
    Wire.endTransmission();
    delay(500);
}
