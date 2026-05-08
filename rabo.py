import blt

from time import ticks_ms as millis, sleep_ms as delay
from machine import UART, Pin
from bleradio import BLERadio

from lib.polyfill import Enum

from comum import globais, coringa
from comum import LOG, ERRO, ASSERT


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

class NoneHub():
    def __init__(self, broadcast_channel=None,
                       observe_channels=[]):
        self.ble = BLERadio(broadcast_channel,
                            observe_channels)

    class system:
        name = lambda: "supermini0"

    def __get_attr__(self, *args, **kwargs): #! fazer direito
        return coringa()
    def __getattr__(self, *args, **kwargs): #! fazer direito
        return coringa()


def setup():
    global hub, timer, uart, led

    uart = UART(1, 115200) 
    uart.init(115200, tx=21, rx=10)

    led = Led(8)
    led.on()

    hub = NoneHub(broadcast_channel=blt.TX_RABO,
                  observe_channels=[blt.TX_CABECA])

    globais.init(hub, True, True, nome="rabo")

    timer = millis()
    return hub

def main(hub):
    global timer

    cmd = None
    cor_caçamba = cor_garra = 0
    while True:
        if uart.any():
            leitura = read_sensor()
            if not leitura: continue

            id, valor = leitura
            LOG("id: ", id, "valor: ", Cor(valor)) #! vai ter que mudar pro ultra
            if   id == 0: cor_garra   = valor
            elif id == 1: cor_caçamba = valor

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
            blt.enviar_comando(blt.rsp.cor_sensor_braco, cor_caçamba) #!
        elif comando == blt.cmd.ver_cor_sensor_rabo: #!
            blt.enviar_comando(blt.rsp.cor_sensor_rabo, cor_garra) #!


def read_sensor():
    inicio = b'\xaa'
    if uart.read(1) != inicio:
        return None
    else: return uart.read(2)

if __name__ == "__main__":
    while True:
        try:
            hub = setup()
            main(hub)
        except Exception as e:
            LOG(f"{e}")
            continue
