import blt

from time import ticks_ms as millis, sleep_ms as delay
from machine import I2C, Pin, PWM
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

def clamp(x, m,M):
    return min(max(x, m), M)

def map_interval(x, m,M, MIN,MAX):
    return MIN + (x-m)*(MAX-MIN)/(M-m)

def map_interval_clamped(x, m,M, MIN,MAX):
    return clamp(map_interval(x, m,M, MIN,MAX), MIN, MAX)

class Servo(PWM):
    def __init__(self, pin, min_angle=0,   max_angle=90,
                            min_pulse=300, max_pulse=1000):
        super().__init__(pin)
        self.freq(50)
        self.duty(0)

        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.min_angle = min_angle
        self.max_angle = max_angle

    def angle(self, ang):
        self.duty_ns(int(map_interval_clamped(ang, self.min_angle, self.max_angle,
                                                   self.min_pulse, self.max_pulse))*1000)
    def off(self):
        self.duty(0)

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
    global hub, timer, led, sensor_caçamba, sensor_cubo, servos

    hub = NoneHub(broadcast_channel=blt.TX_RABO,
                  observe_channels=[blt.TX_CABECA])

    globais.init(hub, True, True, nome="rabo")

    led = Led(2)
    led.on()

    i2c = [
        I2C(0, scl=Pin(19), sda=Pin(21), freq=100000),
        I2C(1, scl=Pin(33), sda=Pin(25), freq=100000),
    ]

    sensor_caçamba = TCS34725(i2c[0])
    sensor_cubo    = TCS34725(i2c[1])

    LOG(f"i2c: caçamba{sensor_caçamba.i2c.scan()},",
                f"cubo{sensor_cubo.i2c.scan()}")

    servos = [
        Servo(Pin(17)), Servo(Pin(5)), Servo(Pin(18)),
    ]
    for s in servos: s.angle(0)

    while True:
        dt = 500
        for s in servos: s.angle(0)
        delay(dt)
        for s in servos: s.angle(90)
        delay(dt)

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
