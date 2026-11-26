import blt

from time import ticks_ms as millis, sleep_ms as delay
from machine import I2C, SoftI2C, Pin, PWM
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

def barra_led_cor(led, cor):
    barra_led[led] = cor_to_rgb(cor)
    barra_led.write()


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

class SensorCor(TCS34725): pass
class SensorDist(VL53L0X):
    def __init__(self, i2c, offset=0, *args, **kwargs):
        super().__init__(i2c, *args, **kwargs)
        self.offset = offset

    def ping(self): return super().ping() + self.offset
    def dist(self): return self.ping()

class SensorCombinado:
    def __init__(self, i2c, offset=0):
        self.i2c = i2c
        self.sensor_dist = SensorDist(i2c, offset=offset, custom_address=0x30)
        self.sensor_cor  = SensorCor(i2c)

    def cor_se_perto(self, perto=30, cacamba=False):
        if (dist := self.dist()) >= perto: return Cor.NENHUMA

        if (cor := self.cor(cacamba)) == Cor.NENHUMA:
            return Cor.BRANCO #! ver se fazer assim mesmo
        return cor

    def cor(self, cacamba=False): return self.sensor_cor.cor(cacamba)
    def dist(self): return self.sensor_dist.ping()

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
    global sensor_caçamba, sensor_cubo, sensor_inimigo

    hub = NoneHub(broadcast_channel=blt.TX_RABO,
                  observe_channels=[blt.TX_CABECA])

    globais.init(hub, True, True, nome="rabo")

    led = Led(2, invert=False); led.on()

    #! i2c_extra   = SoftI2C(   scl=Pin(13), sda=Pin(27), freq=100000)
    #! ^ às vezes tem que reiniciar depois de fazer upload pra ele funcionar
    i2c_caçamba =     I2C(0, scl=Pin(33), sda=Pin(25), freq=100000)
    i2c_cubo    =     I2C(1, scl=Pin(19), sda=Pin(21), freq=100000)
    i2c_inimigo = SoftI2C(   scl=Pin(22), sda=Pin(23), freq=100000)

    LOG(f"i2c: caçamba{i2c_caçamba.scan()},",
                f"cubo{i2c_cubo.scan()}", end=" ")

    sensor_cubo    = SensorCombinado(i2c_cubo,    offset=-100)#! investigar pq offset
    sensor_caçamba = SensorCombinado(i2c_caçamba, offset=-50) #! investigar pq offset
    sensor_inimigo = SensorDist(i2c_inimigo,      offset=-70) #! investigar pq offset

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
        cor_cubo     = sensor_cubo.cor_se_perto()
        cor_caçamba  = sensor_caçamba.cor_se_perto(cacamba=True)

        dist_cubo    = sensor_cubo.dist()
        dist_caçamba = sensor_caçamba.dist()
        dist_inimigo = sensor_inimigo.dist()

        barra_led_cor(6, cor_cubo)
        barra_led_cor(9, cor_caçamba)

        if (millis() - timer) > 250:
            timer = millis()
            led.toggle()

            LOG(f"caçamba: {Cor(cor_caçamba)} a {dist_caçamba}mm", 
                  f"garra: {Cor(cor_cubo)} a {dist_cubo}mm",
                f"inimigo: a {dist_inimigo}mm", sep="\t")

        antes, cmd = cmd, hub.ble.observe(blt.TX_CABECA)
        if cmd is not None: #! levar isso em consideração
            comando, *args = cmd
        else: continue

        if cmd != antes:
            LOG(f"{blt.cmd(comando)}{args}")

        if   comando == blt.cmd.ver_cor_sensor_caçamba:
            blt.enviar_resposta(blt.rsp.cor_sensor_caçamba, cor_caçamba)
        elif comando == blt.cmd.ver_dist_sensor_caçamba:
            blt.enviar_resposta(blt.rsp.dist_sensor_caçamba, dist_caçamba)
        elif comando == blt.cmd.ver_cor_sensor_cubo:
            blt.enviar_resposta(blt.rsp.cor_sensor_cubo, cor_cubo)
        elif comando == blt.cmd.ver_dist_sensor_cubo:
            blt.enviar_resposta(blt.rsp.dist_sensor_cubo, dist_cubo)
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
            LOG(f"mostrando cor tudo {Cor(cor)}")
            blt.enviar_resposta(blt.rsp.mostrei_cor)
        elif comando == blt.cmd.mostra_caçambas:
            cores = args
            for i, cor in enumerate(cores):
                barra_led_cor(i-2, cor)
                LOG(f"mostrando caçamba {i} com cor {Cor(cor)}")
            blt.enviar_resposta(blt.rsp.mostrei_caçambas)
        
        elif comando == blt.cmd.mostra_cubos:
            cores = args
            for i, cor in enumerate(cores):
                barra_led_cor(-i - 4, cor)
                barra_led_cor(+i + 4, cor)
                barra_led.write()
                LOG(f"mostrando cubo {i} com cor {Cor(cor)}")
            blt.enviar_resposta(blt.rsp.mostrei_cubos)



if __name__ == "__main__":
    if True: # DEBUG
        LOG("MODO DEBUG"); main(setup());
        raise KeyboardInterrupt

    while True:
        try:
            hub = setup()
            main(hub)
        except Exception as e:
            LOG(f"{e}")
            continue
