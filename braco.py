from pybricks.hubs import PrimeHub

from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Side
from pybricks.tools      import wait, StopWatch
from cores import Cor
from comum import globais, bipes, luzes
from comum import LOG, ERRO, ASSERT

import bluetooth as blt

import cores
import garra


def setup():
    global garra_fechada, garra_levantada, garra_altura_sensor, plataforma_estendida
    global sensor_dist_dir

    garra_levantada = False
    garra_fechada   = False
    garra_altura_sensor = False
    plataforma_estendida= False

    hub = PrimeHub(broadcast_channel=blt.TX_BRACO,
                   observe_channels=[blt.TX_CABECA])
    hub.display.orientation(Side.BOTTOM)
    globais.init(hub, TESTE, DEBUG, nome="braço")

    #! globais.motor_garra    = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    #! globais.motor_vertical = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    globais.motor_guindaste = Motor(Port.C, Direction.COUNTERCLOCKWISE)
    globais.motor_plataforma = Motor(Port.D, Direction.CLOCKWISE)

    LOG("ligando arduino")
    try:
        from pybricks.pupdevices import DCMotor
        arduino = DCMotor(Port.F); arduino.dc(100)
        LOG("arduino ligado")
    except OSError:
        ERRO("ARDUINO NÃO CONECTADO!")

    hub.system.set_stop_button((Button.CENTER,))
    return hub

def main():
    global garra_fechada, garra_levantada, garra_altura_sensor, plataforma_estendida

    cmd = None
    while True:
        antes, cmd = cmd, hub.ble.observe(blt.TX_CABECA)
        if cmd is not None:
            comando, *args = cmd
        else: continue

        if cmd != antes:
            LOG(f"{blt.cmd(comando)}{args}")

        elif comando == blt.cmd.levanta_garra:
            if not garra_levantada:
                LOG("levantando")
                garra.levanta_garra()
                garra_levantada = True
            blt.enviar_comando(blt.rsp.levantei)
        elif comando == blt.cmd.abaixa_garra:
            if garra_levantada:
                LOG("abaixando")
                garra.abaixa_garra()
                garra_levantada = False
            blt.enviar_comando(blt.rsp.abaixei)

        elif comando == blt.cmd.levanta_garra_dist_sensor:
            if not garra_levantada and not garra_altura_sensor:
                LOG("levantando")
                garra.levanta_garra_dist_sensor()
                garra_altura_sensor = True
            blt.enviar_comando(blt.rsp.levantei_dist_sensor)

        elif comando == blt.cmd.ver_dist_sensor_braco:
            dist = sensor_dist_dir.distance()
            blt.enviar_comando(blt.rsp.dist_sensor_braco, dist)
        elif comando == blt.cmd.mostrar_cor:
            cor, = args
            luzes.mostrar(Cor(cor = cor).color)
            blt.enviar_comando(blt.rsp.mostrei_cor)

        elif comando == blt.cmd.estende_plataforma:
            if not plataforma_estendida:
                LOG("estendendo")
                garra.estende_plataforma()
                plataforma_estendida = True
            blt.enviar_comando(blt.rsp.estendi)
        elif comando == blt.cmd.retrai_plataforma:
            if plataforma_estendida:
                LOG("retraindo")
                garra.retrai_plataforma()
                plataforma_estendida = False
            blt.enviar_comando(blt.rsp.retrai)

def test():
    ... # testar coisas aqui sem mudar o resto do código
    from pybricks.parameters import Stop

    motor_guindaste = Motor(Port.C, Direction.COUNTERCLOCKWISE)

    cm_grau = 63/6000 #.0105 cm/º
    grau_cm = 6000/63 # 95.2 º/cm

    ANG_SUBIR_GARRA = 63*grau_cm
    VEL_SUBIR_GARRA = 800   

    def levanta_garra(vel=VEL_SUBIR_GARRA):
        motor_guindaste.run_angle(vel, ANG_SUBIR_GARRA, then=Stop.HOLD, wait=True)

    def abaixa_garra(vel=VEL_SUBIR_GARRA):
        motor_guindaste.run_angle(-vel, ANG_SUBIR_GARRA, then=Stop.HOLD, wait=True)

    levanta_garra()
    abaixa_garra()

def menu_calibração(*sensores, botao_parar=Button.BLUETOOTH,
                               botao_aceitar=Button.CENTER,
                               botao_anterior=Button.LEFT,
                               botao_proximo=Button.RIGHT):
    import gui

    hub.system.set_stop_button(
        (Button.CENTER, Button.BLUETOOTH)
    )
    bipes.calibracao()
    mapa_hsv = cores.mapa_hsv_frente.copy()

    selecao = 0

    wait(150)
    while True:
        botões = gui.tela_escolher_cor(selecao)

        if   botao_proximo  in botões:
            selecao = (selecao + 1) % len(cores.cor)
            wait(100)
        elif botao_anterior in botões:
            selecao = (selecao - 1) % len(cores.cor)
            wait(100)

        elif botao_aceitar in botões:
            [wait(100) for _ in gui.mostrar_palavra("CAL..")]
            mapa_hsv[selecao] = (
                cores.coletar_valores(botao_aceitar, *sensores)
            )
            wait(150)
        elif botao_parar   in botões:
            wait(100)
            break

    hub.system.set_stop_button(Button.CENTER)
    return mapa_hsv


if __name__ == "__main__":
    try:    TESTE == True
    except: TESTE = False
    try:    DEBUG == True
    except: DEBUG = False

    hub = setup()
    while True:
      try:
        bipes.inicio()
        if TESTE: test()
        else:     main()
        bipes.final()
      except RuntimeError as e:
        LOG(f"runtime error {e}")
        continue
      except UnicodeError as e:
        LOG(f"unicode error {e}")
        continue
      except Exception as e:
        if DEBUG: raise e
        else:
            ERRO(f"{e}")
            continue

