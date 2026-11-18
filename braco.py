from pybricks.hubs import PrimeHub

from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Side, Stop
from pybricks.tools      import wait, StopWatch

from cores import Cor
from comum import globais, bipes, luzes
from comum import LOG, ERRO, ASSERT

import bluetooth as blt

import cores
import garra


grau_mm = 6000/630 # 9.52º/mm -> 95.2º/cm
DIST_DESCER_GARRA = 160#mm
ANG_DESCER_GARRA  = DIST_DESCER_GARRA*grau_mm
VEL_DESCER_GARRA  = 800

ALTURA_PLATAFORMA     = 100#mm
ANG_ALTURA_PLATAFORMA = ALTURA_PLATAFORMA*grau_mm

RAIO_ENGRENAGEM = 9#mm
DIST_PLATAFORMA = 60#mm
ANG_PLATAFORMA  = DIST_PLATAFORMA*RAIO_ENGRENAGEM


def setup():
    global garra_fechada, garra_levantada, garra_altura_sensor, plataforma_estendida

    garra_levantada = True
    garra_fechada   = False
    garra_altura_sensor = False
    plataforma_estendida= False

    hub = PrimeHub(broadcast_channel=blt.TX_BRACO,
                   observe_channels=[blt.TX_CABECA])
    hub.display.orientation(Side.BOTTOM)
    globais.init(hub, TESTE, DEBUG, nome="braço")

    globais.motor_guindaste  = Motor(Port.C, Direction.CLOCKWISE)
    globais.motor_plataforma = Motor(Port.D, Direction.CLOCKWISE)

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
                levanta_garra()
                garra_levantada = True
            blt.enviar_comando(blt.rsp.levantei)
        elif comando == blt.cmd.abaixa_garra:
            if garra_levantada:
                LOG("abaixando")
                abaixa_garra()
                garra_levantada = False
            blt.enviar_comando(blt.rsp.abaixei)

        elif comando == blt.cmd.levanta_garra_dist_sensor:
            if (not garra_levantada and
                not garra_altura_sensor):
                LOG("levantando")
                levanta_garra_dist_sensor()
                garra_altura_sensor = True
            blt.enviar_comando(blt.rsp.levantei_dist_sensor)

        elif comando == blt.cmd.mostrar_cor:
            cor, = args
            luzes.mostrar(Cor(cor = cor).color)
            blt.enviar_comando(blt.rsp.mostrei_cor)

        elif comando == blt.cmd.estende_plataforma:
            if not plataforma_estendida:
                LOG("estendendo")
                estende_plataforma()
                plataforma_estendida = True
            blt.enviar_comando(blt.rsp.estendi)
        elif comando == blt.cmd.retrai_plataforma:
            if plataforma_estendida:
                LOG("retraindo")
                retrai_plataforma()
                plataforma_estendida = False
            blt.enviar_comando(blt.rsp.retrai)

def test():
    ... # testar coisas aqui sem mudar o resto do código
    LOG("retrai_plataforma")
    retrai_plataforma()
    bipes.separador()

    LOG("abrir_garra")
    blt.abrir_garra()
    LOG("abaixa_garra")
    abaixa_garra()
    bipes.separador()

    LOG("fechar_garra")
    blt.fechar_garra()
    levanta_garra()
    bipes.separador()

    LOG("estende_plataforma")
    estende_plataforma()
    LOG("abrir_garra")
    blt.abrir_garra()
    LOG("retrai_plataforma")
    retrai_plataforma()
    bipes.separador()

def abaixa_garra(vel=VEL_DESCER_GARRA):
    globais.motor_guindaste.run_angle(vel, ANG_DESCER_GARRA, then=Stop.COAST, wait=True)

def levanta_garra(vel=VEL_DESCER_GARRA):
    globais.motor_guindaste.run_angle(-vel, ANG_DESCER_GARRA, then=Stop.COAST, wait=True)

def levanta_garra_dist_sensor(vel=VEL_DESCER_GARRA): #! errado tudo nome e oq faz
    globais.motor_guindaste.run_angle(-vel, ANG_ALTURA_PLATAFORMA, then=Stop.COAST, wait=True)

def estende_plataforma(vel=240):
    globais.motor_plataforma.run_angle(vel, ANG_PLATAFORMA, then=Stop.COAST, wait=True)

def retrai_plataforma(vel=240):
    globais.motor_plataforma.run_until_stalled(-vel, then=Stop.COAST, duty_limit=None)

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

