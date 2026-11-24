"""
Código de leitura de cores largamente adaptado da equipe Titãs da Robótica
"""

from cores import cor
from comum import LOG
import math
import struct

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

    def rgbc(self):
        clear, red, green, blue = self._read_crgb()
        return (red, green, blue, clear)

    def hsv(self): return self._rgbc_to_hsv(self.rgbc())

    def cor(self, cacamba = False): return self._classificar_cor(self.hsv(), cacamba)

    def _read_crgb(self):
        return struct.unpack("<HHHH", self._read_bytes(self.CDATAL, 2*4))

    def _read_bytes(self, reg, sz):
        return self.i2c.readfrom_mem(self.TCS_ADDR, self.COMMAND_BIT | reg, sz)

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
    def _classificar_cor(hsv: tuple, cacamba) -> cor:
        h, s, v = hsv
        if cacamba:
            if (s <= 75 and v <= 120): return cor.PRETO
        else:
            if (h >= 50 and h <= 91) and (v >= 90 and v <= 96): return cor.BRANCO
            if (h >= 30 and h <= 150) and (s <= 110 and v <= 110): return cor.PRETO

        if (h >= 20 and h <= 50) and (s <= 130):            return cor.MARROM
        if (h >= 0 and h <= 10) or (h >= 330 and h <= 360): return cor.VERMELHO

        if (h >= 20  and h <= 60):  return cor.AMARELO
        if (h >= 60  and h <= 150): return cor.VERDE
        if (h >= 180 and h <= 260): return cor.AZUL

        return cor.NENHUMA
        
# class intervalo_hsv:
#     def range(m=None, M=None, /, *, sup=1000):
#         if m is None: return range(0, sup+1)
#         if M is None: return range(0,   m+1)
#         else:         return range(m,   M+1)
# 
#     def __init__(self, h=None, s=None, v=None):
#         self.h = self.range(*h) if h else self.range()
#         self.s = self.range(*s) if s else self.range()
#         self.v = self.range(*v) if v else self.range()
# 
#     def __contains__(self, hsv):
#         h, s, v = hsv
#         return (
#             (h in self.h) and (s in self.s) and (v in self.v)
#         )
# 
# if hsv in mapa[cor.PRETO]: return cor.PRETO
# 
# mapa = {
#     cor.PRETO: intervalo_hsv(s=(75), v=(120)),
# }
