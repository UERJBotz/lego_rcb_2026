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


#! se a gente girar a garra na mão, mesmo resetando a cabeça, o estado da garra no braço se mantém e a gente se fode.

def setup():
    global garra_fechada, garra_levantada, garra_altura_sensor
    global sensor_cor_frente, sensor_dist_dir

    garra_levantada = False
    garra_fechada   = False
    garra_altura_sensor = False

    hub = PrimeHub(broadcast_channel=blt.TX_BRACO,
                   observe_channels=[blt.TX_CABECA])
    hub.display.orientation(Side.BOTTOM)
    globais.init(hub, TESTE, DEBUG, nome="braço")

    globais.motor_garra    = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    globais.motor_vertical = Motor(Port.A, Direction.COUNTERCLOCKWISE)

    sensor_cor_frente = ColorSensor(Port.D)
    sensor_dist_dir = UltrasonicSensor(Port.E)

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
    global garra_fechada, garra_levantada, garra_altura_sensor

    cmd = None
    while True:
        antes, cmd = cmd, hub.ble.observe(blt.TX_CABECA)
        if cmd is not None:
            comando, *args = cmd
        else: continue

        if cmd != antes:
            LOG(f"{blt.cmd(comando)}{args}")

        if   comando == blt.cmd.fecha_garra:
            if not garra_fechada:
                LOG("fechando")
                ang = garra.fecha_garra()
                garra_fechada = True
            blt.enviar_comando(blt.rsp.fechei, ang)
        elif comando == blt.cmd.abre_garra:
            if garra_fechada:
                LOG("abrindo")
                garra.abre_garra()
                garra_fechada = False
            blt.enviar_comando(blt.rsp.abri)

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

        elif comando == blt.cmd.ver_cor_sensor_braco: #! isso é uma gambiarrinha, devia tar em Cor
            cor = sensor_cor_frente.color()
            cor = cores.Color2cor(cor)
            hsv = sensor_cor_frente.hsv()
            if   cor == cores.cor.NENHUMA:
                cor = cores.identificar(hsv, sensor="frente")#"chao")
            elif cor == cores.cor.VERMELHO:
                cor = cores.identificar(hsv, sensor="frente")
            blt.enviar_comando(blt.rsp.cor_sensor_braco, cor)
        elif comando == blt.cmd.ver_hsv_sensor_braco:
            cor = sensor_cor_frente.hsv()
            blt.enviar_comando(blt.rsp.hsv_sensor_braco, *cores.Color2tuple(cor))

        elif comando == blt.cmd.ver_dist_sensor_braco:
            dist = sensor_dist_dir.distance()
            blt.enviar_comando(blt.rsp.dist_sensor_braco, dist)
        elif comando == blt.cmd.mostrar_cor:
            cor, = args
            luzes.mostrar(Cor(cor = cor).color)
            blt.enviar_comando(blt.rsp.mostrei_cor)
def test():
    ... # testar coisas aqui sem mudar o resto do código
    mapa_hsv = menu_calibração(sensor_cor_frente)
    cores.repl_calibração(mapa_hsv, lado="frente")

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

