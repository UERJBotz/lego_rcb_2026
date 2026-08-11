from machine import Pin, SoftI2C
from lib.polyfill import rgb_to_hsv
from lib.cores import Cor

class TCS34725:
    MUX_ADDR = 0x70  # Endereço do TCA9548A
    TCS_ADDR = 0x29  # Endereço do sensor TCS34725
    COMMAND_BIT = 0x80
    ENABLE = 0x00
    ATIME = 0x01
    CONTROL = 0x0F
    ID = 0x12
    CDATAL = 0x14

    def __init__(self, sda_pin=21, scl_pin=22):
        self.i2c = SoftI2C(scl=Pin(scl_pin), sda=Pin(sda_pin), freq=100000)
        try:
            self.i2c.writeto_mem(self.TCS_ADDR, self.COMMAND_BIT | self.ENABLE, b'\x03')
            # Tempo de integração (2.4ms × (256 - ATIME)) → ATIME = 0xC0 → ~60ms
            self.i2c.writeto_mem(self.TCS_ADDR, self.COMMAND_BIT | self.ATIME, b'\xC0')
            # Ganho (1x, 4x, 16x, 60x) → 0x01 = 4x (deixei 1x pq era o que rodou melhor no arduino)
            self.i2c.writeto_mem(self.TCS_ADDR, self.COMMAND_BIT | self.CONTROL, b'\x00')
        except Exception as e:
            print("Sensor TCS34725 não encontrado... espero que não seja durante a partida", e)

    def _read_word(self, reg):
        low = self.i2c.readfrom_mem(self.TCS_ADDR, self.COMMAND_BIT | reg, 1)
        high= self.i2c.readfrom_mem(self.TCS_ADDR, self.COMMAND_BIT | (reg + 1), 1)
        if not low or not high:
            return 0
        return (high[0] << 8) | low[0]

    def le_rgbc(self):
        clear = self._read_word(self.CDATAL)
        red   = self._read_word(self.CDATAL + 2)
        green = self._read_word(self.CDATAL + 4)
        blue  = self._read_word(self.CDATAL + 6)
        return red, green, blue, clear 
    
    def rgbc_to_rgb255(red, green, blue, clear):
        if clear > 0:
            r_255 = int((red / clear) * 255)
            g_255 = int((green / clear) * 255)
            b_255 = int((blue / clear) * 255)
        else:
            r_255, g_255, b_255 = 0, 0, 0
        return r_255, g_255, b_255

    def ler_cor(self):
        red, green, blue, clear = self.le_rgbc()
        r_255, g_255, b_255     = rgbc_to_rgb255(red, green, blue, clear)

        hsv = rgb_to_hsv((r_255, g_255, b_255))
        return Cor(hsv=hsv)
