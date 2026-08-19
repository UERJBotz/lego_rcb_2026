try:
    from pybricks.tools import wait
    from pybricks.parameters import Color
except ImportError:
    from time import sleep_ms as wait

    class Color:
        ORANGE = None


class globais:
    # cabeça
    rodas = None

    # braço
    motor_garra = None
    motor_vertical = None

    # comum
    hub = None
    ble = None
    name = None
    nome = None

    @classmethod
    def init(self, hub, teste, debug, nome, ble=None):
        self.TESTE = teste
        self.DEBUG = debug

        self.hub = hub
        self.ble = ble or hub.ble

        self.name = hub.system.name()
        self.nome = nome
        self.globalizar()

        LOG(f"{self.name}: {hub.battery.voltage()}mV")

        esperado = None
        if   self.nome == "braço":  esperado = "spike0"
        elif self.nome == "cabeça": esperado = "spike1"
        elif self.nome == "rabo":   esperado = "esp32"
        else: ASSERT(False, "nome de robô desconhecido")

        while name != esperado:
            ASSERT(False, "nome de robô inesperado")
            hub.speaker.beep(frequency=1024); wait(200)
        else:
            hub.light.blink(Color.ORANGE, [100,50,200,100])

    @classmethod
    def globalizar(self):
        global TESTE, DEBUG
        TESTE = self.TESTE
        DEBUG = self.DEBUG
        global hub, ble
        hub = self.hub
        ble = self.ble
        global name, nome
        name = self.name
        nome = self.nome

class bipes:
    inicio = lambda: hub.speaker.beep(frequency=500, duration=100)
    final  = lambda: hub.speaker.beep(frequency=250, duration=250)

    separador  = lambda: hub.speaker.beep(frequency=600, duration=200)
    calibracao = lambda: hub.speaker.beep(frequency=300, duration=100)
    cabeca     = lambda: hub.speaker.beep(frequency=600, duration=100)

    falha      = lambda: (hub.speaker.beep(frequency=800, duration=500), wait(200),
                          hub.speaker.beep(frequency=800, duration=500),)

class luzes:
    @staticmethod
    def mostrar(color):
        if color == Color.BLACK:
            globais.hub.light.blink(Color.WHITE, [100, 100])
        else:
            globais.hub.light.on(color)

    @staticmethod
    def inicial():
        globais.hub.light.blink(Color.GREEN, [100, 100])

    @staticmethod
    def reset():
        globais.hub.light.blink(Color.MAGENTA, [100, 100])

class coringa:
    def __init__(self, *args, **kwargs): pass
    def __call__(*args, **kwargs): pass

    def __get_attr__(*args, **kwargs):
        return coringa()
    def __getattr__(*args, **kwargs):
        return coringa()

def LOG(*args, print=print, **kwargs):
    print(f"{globais.nome}:", *args, **kwargs)

def ERRO(*args, bipar=True):
    LOG("ERRO:", *args);
    if bipar: bipes.falha()

def ASSERT(cond, texto=None):
    if DEBUG: assert cond, texto
    elif not cond:
        ERRO(f"assert '{texto}' falhou", bipar=DEBUG)
    return cond

