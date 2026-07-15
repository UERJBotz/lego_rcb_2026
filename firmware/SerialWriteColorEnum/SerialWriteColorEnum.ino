#include <Wire.h>
#include <math.h>
#include "Adafruit_TCS34725.h"
#include "Ev3ColorSensor.h"
#include "NewPing.h"

#define MAX_DIST 127

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_300MS, TCS34725_GAIN_1X);

NewPing sonar(9, 10, MAX_DIST);
Ev3ColorSensor sensor(8, 7);

enum Cor {
  NENHUMA,
  PRETO,
  AZUL,
  VERDE,
  AMARELO,
  VERMELHO,
  BRANCO,
  MARROM
};

void rgbToHsv(int r, int g, int b, int &h, int &s, int &v) {
  float rf = r / 255.0;
  float gf = g / 255.0;
  float bf = b / 255.0;

  float cmax = max(rf, max(gf, bf));
  float cmin = min(rf, min(gf, bf));
  float delta = cmax - cmin;

  if (delta == 0) h = 0;
  else if (cmax == rf) h = 60 * fmod(((gf - bf) / delta), 6);
  else if (cmax == gf) h = 60 * (((bf - rf) / delta) + 2);
  else h = 60 * (((rf - gf) / delta) + 4);

  if (h < 0) h += 360;

  if (cmax == 0) s = 0;
  else s = (delta / cmax) * 255;

  v = cmax * 255;
}

Cor classificarCor(int h, int s, int v) {
  if (s <= 100 && v >= 5000) return BRANCO;
  if (h >= 50 && h <= 150 && s <= 90) return PRETO;

  if (h >= 20 && h <= 50 && v <= 2000) return MARROM;

  if ((h >= 0 && h <= 10) || (h >= 330 && h <= 360)) return VERMELHO;
  if (h >= 20 && h <= 50) return AMARELO;
  if (h >= 60 && h <= 150) return VERDE;
  if (h >= 180 && h <= 260) return AZUL;

  return NENHUMA;
}

int normalizar(int valor, int min, int max) {
  int v = 255 * (valor - min) / (max - min);
  return constrain(v, 0, 255);
}

void mandar_u8(uint8_t id, uint8_t valor) {
  const uint8_t inicio = 0xAA;
  Serial.write(inicio);
  Serial.write(id);
  Serial.write(valor);
}

void setup(void) {
  Serial.begin(115200);
  sensor.begin();

  if (tcs.begin()) {
    Serial.println("Sensor de cor TCS34725 encontrado");
  } else {
    Serial.println("Sensor de cor TCS34725 não encontrado... espero que não seja durante a partida");
    //! ASSERT(False);
  }
}

void loop() {
  static uint8_t sensor_atual = 0xFF;
  while (Serial.available()) {
    int req = Serial.read();
    sensor_atual = req < 0 ? sensor_atual
                           : req;
  }

  switch (sensor_atual) {
    case 0:
      {
        Ev3ColorResult color = sensor.read();
        mandar_u8(0, color.color);
        break;
      }
    case 1:
      {
        uint16_t r, g, b, c;
        tcs.getRawData(&r, &g, &b, &c);

        int h, s, v;
        rgbToHsv(r, g, b, h, s, v);

        mandar_u8(1, classificarCor(h, s, v));
        break;
      }
    case 2:
      {
        unsigned int us = sonar.ping_median(5);
        mandar_u8(2, sonar.convert_cm(us));
        break;
      }
  }

  delay(50);
}
