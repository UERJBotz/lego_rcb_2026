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

    def __init__(self, i2c, mapa):
        self.i2c = i2c
        self.mapa=mapa
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

    def cor(self): return self._classificar_cor(self.hsv(), self.mapa)

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
    def _classificar_cor(hsv: tuple, mapa: dict) -> cor:
        if hsv in mapa[cor.BRANCO  ]: return cor.BRANCO
        if hsv in mapa[cor.PRETO   ]: return cor.PRETO
        if hsv in mapa[cor.MARROM  ]: return cor.MARROM
        if hsv in mapa[cor.VERMELHO]: return cor.VERMELHO
        if hsv in mapa[cor.AMARELO ]: return cor.AMARELO
        if hsv in mapa[cor.VERDE   ]: return cor.VERDE
        return cor.NENHUMA

class intervalo_hsv:
    class range_i:
        def __init__(self, m, M=None):
            if M is None: self.m, self.M = 0, m
            else:         self.m, self.M = m, M
        def __contains__(self, h):
            return h in range(self.m, self.M+1)

    class range_h(range_i):
        def __init__(self, m, M=None, sup=360):
            super().__init__(m,M)
            if self.m > self.M:
                self.__contains__ = self.__contains_h__
            self.sup = sup

        def __contains_h__(self, h):
            return (h - self.m) % self.sup in range(self.M - self.m, self.sup+1)

    def __init__(self, h=None, s=None, v=None):
        self.h = self.range_h(*h) if h else self.range_i(360)
        self.s = self.range_i(*s) if s else self.range_i(1000)
        self.v = self.range_i(*v) if v else self.range_i(1000)

    def __contains__(self, hsv):
        h, s, v = hsv
        return (
            (h in self.h) and (s in self.s) and (v in self.v)
        )
