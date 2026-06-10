import blt

from time import ticks_ms as millis, sleep_ms as delay
from machine import I2C, Pin
from bleradio import BLERadio
from tcs3472 import TCS34725

from lib.polyfill import Enum

from comum import globais, coringa
from comum import LOG, ERRO, ASSERT


#! from cores import cor as Cor
Cor = Enum("Cor", ["NENHUMA",
                   "PRETO",
                   "AZUL",
                   "VERDE",
                   "AMARELO",
                   "VERMELHO",
                   "BRANCO",
                   "MARROM"])

class Led(Pin):
    def __init__(self, pin):
        super().__init__(pin, Pin.OUT)
    def on(self):  self.value(0)
    def off(self): self.value(1)
    def toggle(self):
        self.value(self.value()^1)

class NoneHub(coringa): #! fazer direito
    def __init__(self, broadcast_channel=None,
                       observe_channels=[]):
        self.ble = BLERadio(broadcast_channel,
                            observe_channels)

    class system(coringa):
        def name(): return "esp32"

    class speaker(coringa):
        def beep(frequency=500, duration=100):
            LOG(f"speaker.beep: {frequency=}")

    class light(coringa):
        def blink(color, durations): #!
            LOG(f"light.blink: {color=} {durations=}")


def setup():
    global hub, timer, led, sensor_caçamba, sensor_cubo

    hub = NoneHub(broadcast_channel=blt.TX_RABO,
                  observe_channels=[blt.TX_CABECA])

    globais.init(hub, True, True, nome="rabo")

    led = Led(2)
    led.on()

    i2c = [
        I2C(0, scl=Pin(19), sda=Pin(21), freq=100000),
        I2C(1, scl=Pin(25), sda=Pin(26), freq=100000),
    ]

    sensor_caçamba = TCS34725(i2c[0])
    sensor_cubo    = TCS34725(i2c[1])

    LOG(f"i2c: caçamba{sensor_caçamba.i2c.scan()},",
                f"cubo{sensor_cubo.i2c.scan()}")

    timer = millis()
    return hub

def main(hub):
    global timer

    cmd = None
    cor_caçamba = cor_garra = 0
    while True:
        cor_garra   = sensor_cubo.cor()
        cor_caçamba = sensor_caçamba.cor()

        LOG(f"caçamba: {Cor(cor_caçamba)}\t",
              f"garra: {Cor(cor_garra)}")

        if (millis() - timer) > 1000:
            timer = millis()
            led.toggle()

        antes, cmd = cmd, hub.ble.observe(blt.TX_CABECA)
        if cmd is not None: #! levar isso em consideração
            comando, *args = cmd
        else: continue

        if cmd != antes:
            LOG(f"{blt.cmd(comando)}{args}")

        if   comando == blt.cmd.ver_cor_sensor_braco: #! nomes
            blt.enviar_resposta(blt.rsp.cor_sensor_braco, cor_caçamba) #!
        elif comando == blt.cmd.ver_cor_sensor_rabo: #!
            blt.enviar_resposta(blt.rsp.cor_sensor_rabo, cor_garra) #!


if __name__ == "__main__":
    while True:
        try:
            hub = setup()
            main(hub)
        except Exception as e:
            LOG(f"{e}")
            continue
