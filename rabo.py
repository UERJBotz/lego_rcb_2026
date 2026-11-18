import blt

from time import ticks_ms as millis, sleep_ms as delay
from machine import I2C, Pin, PWM
from bleradio import BLERadio
from tcs3472 import TCS34725
from vl53l0x import VL53L0X
from neopixel import NeoPixel

from lib.polyfill import Enum

from comum import globais, coringa
from comum import LOG, ERRO, ASSERT

ANG_GARRA_FECHADA = 90
ANG_GARRA_ABERTA  = 0

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

def cor_to_rgb(cor):
    return {
        Cor.NENHUMA:  (0, 0, 0),
        Cor.PRETO:    (1, 1, 1),
        Cor.AZUL:     (0, 0, 255),
        Cor.VERDE:    (0, 255, 0),
        Cor.AMARELO:  (255, 150, 0),
        Cor.VERMELHO: (255, 0, 0),
        Cor.BRANCO:   (255, 255, 255),
        Cor.MARROM:   (139, 69, 19),
    }[cor]


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

    def angle(self, ang, wait=None):
        self.duty_ns(int(map_interval_clamped(ang, self.min_angle, self.max_angle,
                                                   self.min_pulse, self.max_pulse))*1000)
        if wait: delay(wait)

    def off(self):
        self.duty(0)

class Led(Pin):
    def __init__(self, pin, invert=True):
        super().__init__(pin, Pin.OUT)
        self.invert = invert

    def on(self):  self.value(1 ^ self.invert)
    def off(self): self.value(0 ^ self.invert)
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
    global hub, timer, led, servo_garra, barra_led
    global sensor_caçamba, sensor_cubo, sensor_dist_cubo

    hub = NoneHub(broadcast_channel=blt.TX_RABO,
                  observe_channels=[blt.TX_CABECA])

    globais.init(hub, True, True, nome="rabo")

    led = Led(2, invert=False); led.on()

    i2c_cubo    = I2C(1, scl=Pin(33), sda=Pin(25), freq=100000)
    i2c_caçamba = I2C(0, scl=Pin(19), sda=Pin(21), freq=100000)

    LOG(f"i2c: caçamba{i2c_caçamba.scan()},",
                f"cubo{i2c_cubo.scan()}", end=" ")

    sensor_dist_cubo = VL53L0X(i2c_cubo, custom_address=0x30)
    sensor_cubo      = TCS34725(i2c_cubo)
    sensor_caçamba   = TCS34725(i2c_caçamba)

    LOG(f"-> i2c: caçamba{i2c_caçamba.scan()},",
                   f"cubo{i2c_cubo.scan()}")

    servo_garra, *_ = servos = [
        Servo(Pin(17)), Servo(Pin(18))
    ]
    servo_garra.angle(ANG_GARRA_ABERTA, wait=200)
    for s in servos: s.off()

    barra_led = NeoPixel(Pin(5), 12)

    timer = millis()
    return hub

def main(hub):
    global timer

    cmd = None
    cor_caçamba = cor_garra = 0
    while True:
        cor_cubo    = sensor_cubo.cor()
        cor_caçamba = sensor_caçamba.cor()
        dist_cubo   = sensor_dist_cubo.ping()

        LOG(f"caçamba: {Cor(cor_caçamba)}\t",
              f"garra: {Cor(cor_cubo)}\t",
               f"dist: {dist_cubo}mm")

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
            blt.enviar_resposta(blt.rsp.cor_sensor_rabo, cor_cubo) #!
        elif comando == blt.cmd.fecha_garra:
            servo_garra.angle(ANG_GARRA_FECHADA)
            blt.enviar_resposta(blt.rsp.fechei)
        elif comando == blt.cmd.abre_garra:
            servo_garra.angle(ANG_GARRA_ABERTA)
            blt.enviar_resposta(blt.rsp.abri)
        elif comando == blt.cmd.mostrar_cor:
            cor, = args
            barra_led.fill(cor_to_rgb(cor))
            barra_led.write()
            blt.enviar_comando
        #LOG(cor_to_rgb(cor_cubo))(blt.rsp.mostrei_cor)


if __name__ == "__main__":
    if False: # DEBUG
        LOG("MODO DEBUG"); main(setup());
        raise KeyboardInterrupt

    while True:
        try:
            hub = setup()
            main(hub)
        except Exception as e:
            LOG(f"{e}")
            continue
