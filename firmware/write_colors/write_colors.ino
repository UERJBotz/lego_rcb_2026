#include <Wire.h>
#include <math.h>
#include "Adafruit_TCS34725.h"

// Initialise with specific int time and gain values
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_300MS, TCS34725_GAIN_1X);

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

  if      (delta == 0) h = 0;
  else if (cmax == rf) h = 60 * fmod(((gf - bf) / delta), 6);
  else if (cmax == gf) h = 60 * (((bf - rf) / delta) + 2);
  else                 h = 60 * (((rf - gf) / delta) + 4);

  if (h < 0) h += 360;

  if (cmax == 0) s = 0;
  else           s = (delta / cmax) * 255;

  v = cmax * 255;
}

Cor classificarCor(int h, int s, int v) {
  if (s <= 100 && v >= 5000)         return BRANCO;
  if (h >= 50  && h <= 150 && s<=90) return PRETO;

  if (h >= 20  && h <= 50  && v<=2000) return MARROM;

  if ((h >= 0  && h <= 10) || (h >= 330 && h <= 360)) return VERMELHO;
  if (h >= 20  && h <= 50)  return AMARELO;
  if (h >= 60  && h <= 150) return VERDE;
  if (h >= 180 && h <= 260) return AZUL;

  return NENHUMA;
}

int normalizar(int valor, int min, int max) {
  int v = 255 * (valor - min) / (max - min);
  return constrain(v, 0, 255);
}

void send_cor(uint8_t id, uint8_t cor){
  uint8_t inicio = 0xAA;
  Serial.write(inicio);
  //Serial.print(inicio, HEX);
  Serial.write(id);
  //Serial.print(id);
  Serial.write(cor);
  //Serial.print(cor);
  //Serial.println("");
}

void setup(void) {
  Serial.begin(115200);

  // Initiate TCS3472 IC
  if (tcs.begin()) {
    Serial.println("Found TCS3472 sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}

void loop(void) {
  uint16_t r, g, b, c;
  int h, s, v;
  tcs.getRawData(&r, &g, &b, &c);
  rgbToHsv(r, g, b, h, s, v);
  /*
  Serial.print("h: "); Serial.print(h, DEC); Serial.print(" ");
  Serial.print("s: "); Serial.print(s, DEC); Serial.print(" ");
  Serial.print("v: "); Serial.print(v, DEC); Serial.print(" ");
  Serial.println(" ");
  */
  Cor cor = classificarCor(h, s, v);
  /*
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.print("COR: ");
  Serial.println(cor);
  */
  send_cor(1, cor); 
  delay(50);
 }
