// #include "Ev3ColorSensor.h"
#include "NewPing.h"

// Ev3ColorSensor sensor(8, 7);
NewPing sonar(9, 10, 127);

void sendMsg(uint8_t value, bool color) {
    value &= 0b01111111;
    value |= color ? 0b10000000 : 0;
    Serial.println(value, BIN);
}

void setup(){
    Serial.begin(115200);
    // sensor.begin();
}

void loop(){
    // Ev3ColorResult color = sensor.read();
    uint8_t clr = (uint8_t)(random() % 127);
    // uint8_t clr = color.color;
    sendMsg(clr, true);

    unsigned int us = sonar.ping_median(5);
    uint8_t dis = (uint8_t)sonar.convert_cm(us);
    sendMsg(dis, false);
}
