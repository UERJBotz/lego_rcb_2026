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
    while True:
        if uart.any():
            leitura = read_sensor()
            if leitura:
                id, valor = leitura
                LOG("id: ", id, "valor: ", Cor(valor))
            hub.ble.broadcast((blt.rsp.cor_caçamba, valor))

        if (millis() - timer) > 1000:
            timer = millis()
            led.toggle()

        comando = hub.ble.observe(blt.TX_CABECA)
        if comando is not None:
            LOG(comando)
            #comando, *args = comando
        else: continue

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
