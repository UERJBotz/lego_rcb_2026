from pybricks.hubs import PrimeHub

from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Side, Stop
from pybricks.tools      import wait, StopWatch

from cores import Cor
from comum import globais, bipes, luzes
from comum import LOG, ERRO, ASSERT

import bluetooth as blt

import cores


grau_mm = 6000/630 # 9.52º/mm -> 95.2º/cm
DESCIDA_MAX_GARRA  = 230#mm
DESCIDA_PLATAFORMA = 120#mm
VEL_DESCER_GARRA   = 800

RAIO_ENGRENAGEM = 9#mm
DIST_PLATAFORMA = 60#mm
ANG_PLATAFORMA  = DIST_PLATAFORMA*RAIO_ENGRENAGEM
VEL_RETRAIR_PLATAFORMA = 240


def setup():
    hub = PrimeHub(broadcast_channel=blt.TX_BRACO,
                   observe_channels=[blt.TX_CABECA])
    hub.display.orientation(Side.BOTTOM)
    globais.init(hub, TESTE, DEBUG, nome="braço")

    globais.motor_guindaste  = Motor(Port.C, Direction.CLOCKWISE)
    globais.motor_plataforma = Motor(Port.D, Direction.CLOCKWISE)

    reseta_plataforma()
    reseta_guindaste()

    hub.system.set_stop_button((Button.CENTER,))
    return hub

def main():
    global plataforma_estendida, descida_atual_garra

    cmd = None
    while True:
        antes, cmd = cmd, hub.ble.observe(blt.TX_CABECA)
        if cmd is not None:
            comando, *args = cmd
        else: continue

        if cmd != antes:
            LOG(f"{blt.cmd(comando)}{args}")

        elif comando == blt.cmd.levanta_garra:
            levanta_garra()
            blt.enviar_comando(blt.rsp.levantei_garra)
        elif comando == blt.cmd.media_garra:
            media_garra()
            blt.enviar_comando(blt.rsp.mediei_garra)
        elif comando == blt.cmd.abaixa_garra:
            abaixa_garra()
            blt.enviar_comando(blt.rsp.abaixei_garra)

        elif comando == blt.cmd.levanta_garra_dist_sensor:
            ASSERT(False, "levanta_garra_dist_sensor não existe")
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
        elif comando == blt.cmd.sacode_plataforma:
            num_sacudidas, = args
            LOG("sacodindo")
            sacode_plataforma(num_sacudidas)
            blt.enviar_comando(blt.rsp.sacodi)

def test():
    ... # testar coisas aqui sem mudar o resto do código

    LOG("retrai_plataforma")
    retrai_plataforma()
    bipes.separador()

    LOG("abaixa_garra")
    abaixa_garra()
    bipes.separador()

    LOG("levanta_garra")
    levanta_garra()
    bipes.separador()

    LOG("estende_plataforma")
    estende_plataforma()
    LOG("media_garra")
    media_garra()
    bipes.separador()

    LOG("retrai_plataforma")
    retrai_plataforma()
    bipes.separador()

    LOG("abaixa_garra")
    abaixa_garra()
    LOG("media_garra")
    media_garra()
    LOG("estende_plataforma")
    estende_plataforma()
    bipes.separador()


def reseta_guindaste(vel=VEL_DESCER_GARRA):
    global descida_atual_garra
    descida_atual_garra = 0
    globais.motor_guindaste.run_until_stalled(-vel, then=Stop.COAST, duty_limit=None)
    mexer_guindaste(10, vel=vel)

def mexer_guindaste(descida=DESCIDA_MAX_GARRA, vel=VEL_DESCER_GARRA):
    globais.motor_guindaste.run_angle(
        vel, descida*grau_mm, then=Stop.COAST, wait=True
    )

def garra_na_altura(altura, vel=VEL_DESCER_GARRA):
    global descida_atual_garra
    descida = altura - descida_atual_garra
    descida_atual_garra += descida
    mexer_guindaste(descida, vel=vel)

def abaixa_garra(vel=VEL_DESCER_GARRA):
    if descida_atual_garra < DESCIDA_MAX_GARRA:
        LOG("abaixando")
    garra_na_altura(DESCIDA_MAX_GARRA, vel=vel)

def media_garra(vel=VEL_DESCER_GARRA):
    if descida_atual_garra > DIST_PLATAFORMA:
        LOG("levantando médio")
    if descida_atual_garra < DIST_PLATAFORMA:
        LOG("abaixando médio")
    garra_na_altura(DESCIDA_PLATAFORMA, vel=vel)

def levanta_garra(vel=VEL_DESCER_GARRA):
    if descida_atual_garra > 0:
        LOG("levantando")
    garra_na_altura(0, vel=vel)


def levanta_garra_dist_sensor(vel=VEL_DESCER_GARRA): #! errado tudo nome e oq faz
    ASSERT(False, "levanta_garra_dist_sensor não existe")


def reseta_plataforma(vel=VEL_RETRAIR_PLATAFORMA):
    global plataforma_estendida
    plataforma_estendida = False
    retrai_plataforma(vel=vel)

def estende_plataforma(vel=VEL_RETRAIR_PLATAFORMA):
    globais.motor_plataforma.run_angle(vel, ANG_PLATAFORMA, then=Stop.COAST, wait=True)

def retrai_plataforma(vel=VEL_RETRAIR_PLATAFORMA):
    globais.motor_plataforma.run_until_stalled(-vel, then=Stop.COAST, duty_limit=None)

def sacode_plataforma(num_sacudidas, vel=VEL_RETRAIR_PLATAFORMA):
    for _ in range(num_sacudidas):
        globais.motor_plataforma.run_angle(vel, ANG_PLATAFORMA/4, then=Stop.HOLD, wait=True)
        retrai_plataforma(vel=vel*10)

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

