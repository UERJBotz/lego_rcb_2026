"""
Código de leitura de cores largamente adaptado da equipe Titãs da Robótica
"""

from cores import cor
from comum import LOG
import math

class TCS34725:
    TCS_ADDR    = 0x29 # Endereço i2c fixo do sensor TCS34725
    COMMAND_BIT = 0x80
    ENABLE      = 0x00
    ATIME       = 0x01
    CONTROL     = 0x0F
    CDATAL      = 0x14

    def __init__(self, i2c):
        self.i2c = i2c
        try:
            self.i2c.writeto_mem(self.TCS_ADDR, self.COMMAND_BIT | self.ENABLE, b'\x03')
            # Tempo de integração (2.4ms × (256 - ATIME)) → ATIME = 0xC0 → ~60ms
            self.i2c.writeto_mem(self.TCS_ADDR, self.COMMAND_BIT | self.ATIME, b'\xC0')
            # Ganho (1x, 4x, 16x, 60x) → 0x01 = 4x (1x rodou melhor no arduino)
            self.i2c.writeto_mem(self.TCS_ADDR, self.COMMAND_BIT | self.CONTROL, b'\x00')
        except Exception as e:
            LOG(f"Problema na inicialização do TCS34725: {e}... espero que não seja durante a partida")

    #! usar struct.unpack("<HHHH", readfrom_mem(..., 2*4))
    def rgbc(self):
        clear = self._read_u16(self.CDATAL)
        red   = self._read_u16(self.CDATAL + 2)
        green = self._read_u16(self.CDATAL + 4)
        blue  = self._read_u16(self.CDATAL + 6)
        return (red, green, blue, clear)

    def hsv(self):
        return self._rgbc_to_hsv(self.rgbc())

    def cor(self):
        return self._classificar_cor(self.hsv())

    #! tirar
    def _read_u16(self, reg):
        i2c = self.i2c
        low  = i2c.readfrom_mem(self.TCS_ADDR, self.COMMAND_BIT | (reg), 1)
        high = i2c.readfrom_mem(self.TCS_ADDR, self.COMMAND_BIT | (reg + 1), 1)

        if not low or not high: return 0
        return (high[0] << 8) | low[0]

    @staticmethod
    def _rgbc_to_hsv(rgbc: tuple):
        r, g, b, c = rgbc
        if c == 0: return (0, 0, 0)

        r, g, b = r/c, g/c, b/c

        cmax = max(r, g, b)
        cmin = min(r, g, b)
        delta = cmax - cmin

        if   delta == 0: h = 0
        elif  cmax == r: h = 60 * (((g - b) / delta) % 6)
        elif  cmax == g: h = 60 * (((b - r) / delta) + 2)
        else:            h = 60 * (((r - g) / delta) + 4)

        if h < 0: h += 360

        s = 0 if cmax == 0 else (delta / cmax) * 255
        v = cmax * 255

        return (int(h), int(s), int(v))

    @staticmethod
    def _classificar_cor(hsv: tuple) -> cor:
        h, s, v = hsv

        if (v <= 90 and s <= 90): return cor.BRANCO
        if (h >= 20 and h <= 150) and (s <= 110 and v <= 100): return cor.PRETO

        if (h >= 20 and h <= 50) and (s <= 150):            return cor.MARROM
        if (h >= 0 and h <= 10) or (h >= 330 and h <= 360): return cor.VERMELHO

        if (h >= 20  and h <= 50):  return cor.AMARELO
        if (h >= 60  and h <= 150): return cor.VERDE
        if (h >= 180 and h <= 260): return cor.AZUL

        return cor.NENHUMA

