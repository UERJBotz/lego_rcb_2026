from pybricks.hubs import PrimeHub

from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Side, Axis, Direction, Button, Color

from pybricks.tools      import wait, StopWatch
from pybricks.robotics   import DriveBase

from lib.caminhos import achar_movimentos, achar_caminhos, tipo_movimento
from lib.caminhos import coloca_obstaculo, tira_obstaculo
from lib.caminhos import pegar_celulas_incertas, imprimir_mapa
from lib.caminhos import MAPA_X_MAX, MAPA_Y_MAX

from urandom import choice

import bluetooth as blt
import gui
import cores

from cores import Cor

from comum import globais, bipes, luzes
from comum import LOG, ERRO, ASSERT


NUM_CUBOS_PEGÁVEIS = 2

ANG_LIMIAR_GARRA_FECHADA = 145
VEL_ALINHAR = 80
VEL_ANG_ALINHAR = 20
GIRO_MAX_ALINHAR = 90 #70

TAM_QUARTEIRÃO = 300
TAM_BLOCO = TAM_QUARTEIRÃO//2
TAM_FAIXA = 20

NUM_CAÇAMBAS = 5
TAM_CAÇAMBA = 160
TAM_CUBO = 50

DIST_EIXO_SENSOR = 45
DIST_EIXO_SENSOR_FRENTE = 110
DIST_EIXO_SENSOR_TRAS = 110
DIST_CRUZAMENTO_CUBO = TAM_BLOCO - DIST_EIXO_SENSOR_FRENTE
DIST_VOLTAR_CUBO = TAM_CUBO//2
DIST_SENSORES_CENTRO = 20

DIST_BORDA_CAÇAMBA = 130
DIST_CAÇAMBA = 100
DIST_VERDE_CAÇAMBA = 80
VEL_SEGUIR_LINHA = 100 #mm/s
TAM_PISTA_TODA = TAM_QUARTEIRÃO*6
BLOCO_MEIO = 4

DISTS_CAÇAMBAS = [ DIST_BORDA_CAÇAMBA +
                   TAM_CAÇAMBA*i + DIST_CAÇAMBA*i
                   for i in range(NUM_CAÇAMBAS) ]
cubos_caçambas = [0 for i in range(NUM_CAÇAMBAS)]
cores_caçambas = []


#! 2024: checar stall: jogar exceção
#! 2024: checar cor errada no azul

#! quando sofrer exceção e DEBUG == False, entrar num estado que só espera o reset

#! a procura para(va) no meio quando vê um cubo desconhecido e acha que é catástrofe

#! na varredura, quando vê azul, parar de ir e reclamar do inimigo

class dir_linha:
    ESQ = -1
    DIR = +1

    mul = DIR

def setup():
    global hub, rodas
    global sensor_cor_esq, sensor_cor_centro, sensor_cor_dir
    global orientação_estimada, pos_estimada, na_grade
    global rodas_conf_padrão, vels_padrão, vel_padrão, vel_ang_padrão #! fazer um dicionário e concordar com mudar_velocidade

    hub = PrimeHub(broadcast_channel=blt.TX_CABECA,
                   observe_channels=[blt.TX_BRACO, blt.TX_RABO],
                   front_side=Axis.X, top_side=Axis.Z)
    hub.system.set_stop_button(Button.CENTER)
    globais.init(hub, TESTE, DEBUG, nome="cabeça")

    orientação_estimada = ""

    sensor_cor_esq    = ColorSensor(Port.D)
    sensor_cor_centro = ColorSensor(Port.E)
    sensor_cor_dir    = ColorSensor(Port.C)

    roda_esq = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
    roda_dir = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
    rodas    = DriveBase(roda_esq, roda_dir,
                         wheel_diameter=88,
                         axle_track=145.5) #! recalibrar
    rodas.use_gyro(True)

    na_grade = False

    rodas_conf_padrão = rodas.settings() #! CONSTANTIZAR
    vel_padrão     = rodas_conf_padrão[0]
    vel_ang_padrão = rodas_conf_padrão[2]
    vels_padrão    = vel_padrão, vel_ang_padrão

    return hub

quarteirão_atual = 0
def main():
    global orientação_estimada, pos_estimada, quarteirão_atual, cores_caçambas
    global na_grade
    na_grade = True

    blt.SILENCIOSO = True
    luzes.inicial()

    while True:
        LOG("main: resetou", conta_reset, "vezes")
        blt.resetar_garra()
        blt.abaixar_garra()
        posicionamento_inicial()
        pos_estimada = (0,0)

        if not cores_caçambas:
            descobrir_cor_caçambas()
            acertar_orientação("L")
            achar_azul_alinhado()
            dar_ré_alinhar_primeiro_bloco()
            posicionamento_inicial()

        while quarteirão_atual <= MAPA_Y_MAX//2:
            LOG(f"main: varrendo: quarteirão {quarteirão_atual}")

            achar_não_verde_alinhado()
            rodas.straight(DIST_EIXO_SENSOR)
            cor, pos_estimada = varredura(pos_estimada, cores_caçambas)

            quarteirão_atual += 1
            quarteirão_atual %= (MAPA_Y_MAX//2 + 1) #! nunca vai sair do loop
            if not cor: pass #!
            else:
                colocar_cubo_na_caçamba(cor)
                dar_ré(DIST_VERDE_CAÇAMBA)

            dist_quarteirão = TAM_QUARTEIRÃO + TAM_FAIXA//2

            posicionamento_inicial()
            for _ in range(quarteirão_atual):
                acertar_orientação("S")
                rodas.straight(dist_quarteirão)
                acertar_orientação("L")
                achar_não_verde_alinhado()
                dar_ré_meio_quarteirão()
            pos_estimada = (quarteirão_atual*2, 0)
        else:
            LOG("main: procurando")
            achar_não_verde_alinhado()
            rodas.straight(DIST_EIXO_SENSOR)
            cor, pos_estimada = procura(pos_estimada, cores_caçambas)

            caminho_volta = achar_caminhos(pos_estimada, (0,0))
            seguir_caminho(caminho_volta)

            colocar_cubo_na_caçamba(cor)
            dar_ré(DIST_VERDE_CAÇAMBA)

def test():
    global orientação_estimada, pos_estimada, na_grade, cores_caçambas
    ... # testar coisas aqui sem mudar o resto do código
    blt.SILENCIOSO = True
    
    while False:
        cor = blt.ver_cor_cubo()
        print(cor)
        blt.mostrar_cor(cor)

    while False:
        andar_dist_linha(TAM_BLOCO)
        bipes.separador()

    while False:
        cor = (blt.ver_cor_caçamba())
        print(cor, cor.cor, cor.color)
        achar_cruzamento_linha(dist_max=TAM_CAÇAMBA+DIST_CAÇAMBA)
        bipes.separador()

    while False:
        esq    = sensor_cor_esq.reflection()
        centro = sensor_cor_centro.reflection()
        dir    = sensor_cor_dir.reflection()
        print(esq, centro, dir)

    while False:
        blt.abrir_garra()
        ang = blt.fechar_garra()
        print(ang)

    if False: testes.imprimir_cor_cubo_para_sempre()
    if False: testes.imprimir_cor_caçamba_para_sempre()

    while False:
        if True: vel = None
        else:    vel, *_ = rodas.settings()

        for _ in range(5):
            achar_cruzamento_linha(vel)
            bipes.separador()
            curva_linha_esquerda()

        achar_cruzamento_linha(vel)
        bipes.separador()
        curva_linha_esquerda()

        achar_cruzamento_linha(vel)
        bipes.separador()
        curva_linha_direita()

    if False: tira_obstaculo((0,2))
    if False: tira_obstaculo((0,4))
    if False: tira_obstaculo((1,5))
    if False: tira_obstaculo((2,4))
    if False: tira_obstaculo((3,3))
    if False: tira_obstaculo((4,4))

    if True:
        cores_caçambas = [
            Cor.enum.NENHUMA for _ in range(NUM_CAÇAMBAS)
        ]
        if True: cores_caçambas[0] = Cor.enum.VERMELHO
        if True: cores_caçambas[1] = Cor.enum.AMARELO
        if True: cores_caçambas[2] = Cor.enum.AZUL
        if True: cores_caçambas[3] = Cor.enum.VERDE
        if True: cores_caçambas[4] = Cor.enum.PRETO

    if False: orientação_estimada = "N"
    if False: orientação_estimada = "S"
    if False: orientação_estimada = "L"
    if False: orientação_estimada = "O"

    if False:
        pos_estimada = (0,0)
        orientação_estimada = "L"

        achar_não_verde_alinhado()
        rodas.straight(DIST_EIXO_SENSOR)

        if False:
            cor, pos_estimada = varredura(pos_estimada, cores_caçambas)
        else:
            na_grade = True
            cor, pos_estimada = procura(pos_estimada, cores_caçambas)
            caminho_volta = achar_caminhos(pos_estimada, (0,0))
            seguir_caminho(caminho_volta)
            colocar_cubo_na_caçamba(cor)
            dar_ré(DIST_VERDE_CAÇAMBA)
        return

    if True:
        LOG("tenta verde:")
        achar_não_verde()

        LOG("tenta alinhar:")
        bipes.separador()
        alinhar()

        LOG("vira direita:")
        bipes.separador()
        virar_direita()
        # main()

    LOG("fim do teste")


class mudar_velocidade():
    """
    gerenciador de contexto (bloco with) para (automaticamente):
    1. mudar a velocidade do robô
    2. restaurar a velocidade do robô
    """
    def __init__(self, vel, vel_ang=None, drive_base=None): 
        self.rodas = drive_base or rodas
        self.vel   = vel
        self.vel_ang = vel_ang
 
    def __enter__(self): 
        self.conf_anterior = self.rodas.settings()
        [_, *conf_resto]   = self.conf_anterior
        if self.vel_ang:
            conf_resto[1] = self.vel_ang
        self.rodas.settings(self.vel, *conf_resto)
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback): 
        self.rodas.settings(*self.conf_anterior)

def SucessoOuCatástrofe(*args):
    ERRO("SUCESSO OU CATÁSTROFE", *args) #! if DEBUG: raise ...

def inverte_orientação(ori=None):
    if ori == None: ori = orientação_estimada

    if ori == "N": return "S"
    if ori == "S": return "N"
    if ori == "L": return "O"
    if ori == "O": return "L"

    return ori #!< não é pra chegar aqui

def dar_meia_volta():
    global orientação_estimada
    rodas.turn(180)

    orientação_estimada = inverte_orientação()
    LOG(f"dar_meia_volta: {orientação_estimada=}")

def dar_meia_volta_linha():
    global orientação_estimada

    if   dir_linha.mul == dir_linha.ESQ: rodas.curve(TAM_FAIXA//2, +180)
    elif dir_linha.mul == dir_linha.DIR: rodas.curve(TAM_FAIXA//2, -180)

    orientação_estimada = inverte_orientação()
    LOG(f"dar_meia_volta_linha: {orientação_estimada=}")

#! separar esses ifelses de orientação pra girar_orientação esq e dir que nem inverte_orientação
##! aí fazer as funções de curva também chamarem essa nova também

def virar_direita():
    global orientação_estimada
    rodas.turn(90)

    if   orientação_estimada == "N": orientação_estimada = "L"
    elif orientação_estimada == "S": orientação_estimada = "O"
    elif orientação_estimada == "L": orientação_estimada = "S"
    elif orientação_estimada == "O": orientação_estimada = "N"
    LOG(f"virar_direita: {orientação_estimada=}")

def virar_esquerda():
    global orientação_estimada
    rodas.turn(-90)

    if   orientação_estimada == "N": orientação_estimada = "O"
    elif orientação_estimada == "S": orientação_estimada = "L"
    elif orientação_estimada == "L": orientação_estimada = "N"
    elif orientação_estimada == "O": orientação_estimada = "S"
    LOG(f"virar_esquerda: {orientação_estimada=}")

DIST_PARAR=0.4 #! checar valor
def parar():
    rodas.straight(-DIST_PARAR)
    rodas.stop()
ANG_PARAR=0.0
def parar_girar():
    rodas.turn(-ANG_PARAR)
    rodas.stop()

def dar_ré(dist):
    rodas.straight(-dist)

def dar_ré_alinhar_primeiro_bloco():
    dar_ré(TAM_BLOCO - DIST_EIXO_SENSOR - TAM_FAIXA//2)
    LOG("dar_ré_meio_quarteirão: ré")

def dar_ré_meio_quarteirão():
    dar_ré(TAM_BLOCO - DIST_EIXO_SENSOR)
    LOG("dar_ré_meio_quarteirão: ré")

#! provavelmente mudar andar_até pra receber uma fn -> bool e retornar só bool, dist (pegar as informações extras na própria função)

def ver_não_pista() -> tuple[bool, tuple[Color, hsv], tuple[Color, hsv]]: # type: ignore
    esq, dir = cores.todas(sensor_cor_esq, sensor_cor_dir)
    return ((not esq.pista() or not dir.pista()), esq, dir)

def ver_não_verde() -> tuple[bool, tuple[Color, hsv], tuple[Color, hsv]]: # type: ignore
    esq, dir = cores.todas(sensor_cor_esq, sensor_cor_dir)
    if not esq.verde() or not dir.verde(): LOG(f"ver_não_verde: {esq}, {dir}")
    return ((not esq.area_livre() or not dir.area_livre()), esq, dir)

def verificar_cor(func_cor) -> Callable[None, tuple[bool, int]]: # type: ignore
    def f():
        esq, dir = cores.todas(sensor_cor_esq, sensor_cor_dir)
        return (func_cor(esq) or func_cor(dir), esq, dir)
    return f


def ver_cubo_perto() -> bool:
    cor = blt.ver_cor_cubo()
    return cor != Cor.enum.NENHUMA

def andar_até_idx(*conds_parada: Callable, dist_max=TAM_PISTA_TODA) -> tuple[bool, tuple[Any]]: # type: ignore
    rodas.reset()
    rodas.straight(dist_max, wait=False)
    while not rodas.done():
        for i, cond_parada in enumerate(conds_parada):
            chegou, *retorno = cond_parada()
            if not chegou: continue
            else:
                parar()
                return i+1, retorno
    return 0, (rodas.distance(),)

nunca_parar   = (lambda: (False, False))
ou_manter_res = (lambda res, ext: (res, ext))

def andar_até_bool(sucesso, neutro=nunca_parar, fracasso=ver_não_pista,
                            ou=ou_manter_res, dist_max=TAM_PISTA_TODA):
    succ, neut, frac = 1, 2, 3
    while True:
        res, extra = andar_até_idx(sucesso, neutro, fracasso,
                                   dist_max=dist_max)

        if   res == succ: return True, extra
        elif res == frac: return False, extra
        elif res == neut: continue
        elif res == 0:
            LOG("andar_até_bool: andou demais")
            return False, (None,) #ou(res, extra)
        else: 
            LOG(f"andar_até_bool: {res}")
            assert False

def cor_final(retorno):
    achou, extra = retorno

    if achou: return extra
    else:     return cores.todas(sensor_cor_esq, sensor_cor_dir)

def achar_limite() -> tuple[tuple[Color, hsv], tuple[Color, hsv]]: # type: ignore
    return cor_final(andar_até_idx(ver_não_pista))

def achar_não_verde() -> tuple[tuple[Color, hsv], tuple[Color, hsv]]:
    return cor_final(andar_até_idx(ver_não_verde))

def achar_azul() -> tuple[tuple[Color, hsv], tuple[Color, hsv]]:
    return cor_final(andar_até_idx(verificar_cor(Cor.azul)))

def achar_azul_alinhado(*args, **kwargs):
    achar_azul()
    dar_ré(TAM_FAIXA) #!//2?
    return alinhar(*args, **kwargs)

def achar_não_verde_alinhado(*args, **kwargs):
    achar_não_verde()
    dar_ré(TAM_FAIXA) #!//2?
    return alinhar(*args, **kwargs)

def até_cruzamento(dist, esq, centro, dir, preto):
    return preto(esq) and preto(dir)

def até_dist_max(dist_max):
    def func(dist, esq, centro, dir, preto):
        return dist >= dist_max
    return func

def até_dist_max_ou_cruzamento(dist_max):
    def func(dist, esq, centro, dir, preto):
        return (
            até_cruzamento(dist, esq, centro, dir, preto) or
            até_dist_max(dist_max)(dist, esq, centro, dir, preto)
        )
    return func

def dar_ré_linha(dist, **kwargs):
    seguir_linha_até(até_dist_max(dist), vel=-70, **kwargs) #! vel hardcoded

def dar_ré_achar_cruzamento_linha(*, dist_max=TAM_PISTA_TODA, **kwargs):
    seguir_linha_até(até_dist_max_ou_cruzamento(dist_max), vel=-70, **kwargs) #! vel hardcoded

def andar_dist_linha(dist, **kwargs):
    seguir_linha_até(até_dist_max(dist), **kwargs)

def achar_cruzamento_linha(*, dist_max=TAM_PISTA_TODA, **kwargs):
    seguir_linha_até(até_dist_max_ou_cruzamento(dist_max), **kwargs)
    bipes.separador()


def pid(kp, kd=0, ki=0):
    ierro = erro_ant = 0
    def f(erro):
        nonlocal ierro
        nonlocal erro_ant
        derro = erro_ant - erro
        ativ  = erro*kp + derro*kd + ierro*ki

        erro_ant = erro
        ierro   += erro
        return ativ
    return f

pid_linha = pid(kp=0.50)
def seguir_linha_até(parada=até_dist_max_ou_cruzamento(TAM_PISTA_TODA),
                     *, vel=None, pid=None, parar_no_verde=True):
    if vel is None: vel = VEL_SEGUIR_LINHA
    if pid is None: pid = pid_linha

    if False: REFL_MIN, REFL_MAX = 13, 99
    else:     REFL_MIN, REFL_MAX = 22, 99
    REFL_IDEAL = (REFL_MAX + REFL_MIN)/2

    def preto(val): return val <= REFL_MIN + 5

    rodas.reset()
    while True:
        esq    = sensor_cor_esq.reflection()
        centro = sensor_cor_centro.reflection()
        dir    = sensor_cor_dir.reflection()

        erro = dir_linha.mul * (REFL_IDEAL - centro)
        rodas.drive(vel, pid(erro))

        if parada(rodas.distance(), esq, centro, dir, preto): break
        if parar_no_verde:
            esq, dir = cores.todas(sensor_cor_esq, sensor_cor_dir)
            if esq == Cor.enum.VERDE    and dir == Cor.enum.VERDE: break
            if esq == Cor.enum.VERMELHO and dir == Cor.enum.VERMELHO: break #! parar no vermelho

    rodas.stop()

def curva_linha_esquerda():
    dir_linha.mul = dir_linha.DIR
    rodas.curve(DIST_EIXO_SENSOR, -90)

def curva_linha_direita():
    dir_linha.mul = dir_linha.ESQ
    rodas.curve(DIST_EIXO_SENSOR, +90)


def alinha_parede(vel, vel_ang, giro_max=45,
                  func_cor_pista=Cor.area_livre,
                  func_parar_andar=ver_não_verde
                  ) -> bool:
    desalinhado_branco = lambda esq, dir: Cor.branco(esq) ^ Cor.branco(dir)
    alinhado_não_pista = lambda esq, dir: ((not func_cor_pista(esq)) and
                                           (not func_cor_pista(dir)))
    
    alinhado_pista  = lambda esq, dir: func_cor_pista(esq) and func_cor_pista(dir)
    alinhado_parede = lambda esq, dir: alinhado_não_pista(esq, dir) and not desalinhado_branco(esq, dir)

    LOG("alinha_parede")
    with mudar_velocidade(vel, vel_ang):
        parou, extra = andar_até_idx(func_parar_andar, dist_max=TAM_BLOCO)
        if not parou:
            (dist,) = extra
            LOG(f"alinha_parede: reto pista {dist}")
            return False, extra # viu só branco, não sabemos se tá alinhado
    
        (esq, dir) = extra
        if  alinhado_parede(esq, dir):
            LOG(f"alinha_parede: reto não pista {esq}, {dir}")
            return True, extra
        elif not func_cor_pista(dir):
            LOG(f"alinha_parede: torto pra direita {esq}, {dir}")
            GIRO = giro_max
        elif not func_cor_pista(esq):
            LOG(f"alinha_parede: torto pra esquerda {esq}, {dir}")
            GIRO = -giro_max

        rodas.turn(GIRO, wait=False) #! fazer gira_até
        LOG("alinha_parede: girando")
        while not rodas.done():
            extra = cores.todas(sensor_cor_esq, sensor_cor_dir)
            esq, dir = extra
            if  alinhado_parede(esq, dir):
                LOG(f"alinha_parede: alinhado parede: {esq}, {dir}")
                parar_girar()
                return True, extra # deve tar alinhado
            elif alinhado_pista(esq, dir):
                LOG(f"alinha_parede: alinhado pista: {esq}, {dir}")
                parar_girar()
                return False, extra #provv alinhado, talvez tentar de novo
        LOG(f"alinha_parede: girou tudo, {esq}, {dir}")
        return False, extra # girou tudo, não sabemos se tá alinhado

def alinha_giro(max_tentativas=4, virar=True, #! virar=False?
                              vel=VEL_ALINHAR, vel_ang=VEL_ANG_ALINHAR,
                              giro_max=GIRO_MAX_ALINHAR) -> None:
    for _ in range(max_tentativas): #! esqueci mas tem alguma coisa
        rodas.reset()
        alinhou, extra = alinha_parede(vel, vel_ang, giro_max=giro_max)

        ang  = rodas.angle()
        dist = rodas.distance()
        with mudar_velocidade(vel, vel_ang):
            rodas.turn(-ang)
            dar_ré(dist)
            rodas.turn(ang)

        if alinhou: return extra
        else:
            if virar: virar_direita() #! testar agora
            continue
    return extra

def alinhar(max_tentativas=4, virar=True, #! virar=False?
                              vel=VEL_ALINHAR, vel_ang=VEL_ANG_ALINHAR,
                              giro_max=GIRO_MAX_ALINHAR) -> None:
    for _ in range(max_tentativas):
        rodas.reset()

        alinhou, extra = alinha_parede(vel, vel_ang, giro_max=giro_max)
        LOG("alinhar", extra)
        ang  = rodas.angle()
        dist = rodas.distance()
        if alinhou: return extra
        else:
            with mudar_velocidade(80, 30):
                rodas.turn(-ang)
                dar_ré(dist)
                rodas.turn(ang)
    #! às vezes ele retorna como verde verde, tentei fazer isso pra resolver
    #if extra == (Cor.enum.VERDE, Cor.enum.VERDE):
    #    LOG("alinhar: vou na fé")
    #    return achar_não_verde()
    return extra

def alinha_re(max_tentativas=3,
              vel=VEL_ALINHAR, vel_ang=VEL_ANG_ALINHAR,
              giro_max=70) -> None:
    for _ in range(max_tentativas):
        rodas.reset()
        dar_ré_meio_quarteirão()

        alinhou, extra = alinha_parede(vel, vel_ang, giro_max=giro_max)
        ang  = rodas.angle()
        dist = rodas.distance()
        if alinhou: return
        else:
            with mudar_velocidade(80, 30):
                rodas.turn(-ang)
                dar_ré(dist)
                rodas.turn(ang)
    return extra

def seguir_caminho(caminho): #! lidar com outras coisas
    def interpretar_movimento_livre(mov):
        #! fazer run length encoding aqui
        if   mov == tipo_movimento.FRENTE:
            rodas.straight(TAM_BLOCO, then=Stop.COAST)
        elif mov == tipo_movimento.TRAS:
            dar_meia_volta()
            rodas.straight(TAM_BLOCO, then=Stop.COAST)
        elif mov == tipo_movimento.ESQUERDA_FRENTE:
            virar_esquerda()
            rodas.straight(TAM_BLOCO, then=Stop.COAST)
        elif mov == tipo_movimento.DIREITA_FRENTE:
            virar_direita()
            rodas.straight(TAM_BLOCO, then=Stop.COAST)
        elif mov == tipo_movimento.ESQUERDA:
            virar_esquerda()
        elif mov == tipo_movimento.DIREITA:
            virar_direita()

    def interpretar_movimento_cidade(mov):
        #! fazer run length encoding aqui
        if   mov == tipo_movimento.FRENTE:
            achar_cruzamento_linha(dist_max=TAM_BLOCO)
        elif mov == tipo_movimento.TRAS:
            dar_meia_volta_linha()
            achar_cruzamento_linha(dist_max=TAM_BLOCO)
        elif mov == tipo_movimento.ESQUERDA_FRENTE:
            curva_linha_esquerda()
            achar_cruzamento_linha(dist_max=DIST_EIXO_SENSOR)
        elif mov == tipo_movimento.DIREITA_FRENTE:
            curva_linha_direita()
            achar_cruzamento_linha(dist_max=TAM_BLOCO)
        elif mov == tipo_movimento.ESQUERDA:
            curva_linha_esquerda()
        elif mov == tipo_movimento.DIREITA:
            curva_linha_direita()

    if na_grade: interpretar_movimento = interpretar_movimento_cidade
    else:        interpretar_movimento = interpretar_movimento_livre

    def interpretar_caminho(caminho): #! receber orientação?
        for mov in caminho: #! yield orientação nova?
            LOG(f"seguir_caminho: {tipo_movimento(mov)}")
            interpretar_movimento(mov)
            yield rodas.distance()

    movs, ori_final = achar_movimentos(caminho, orientação_estimada)
    LOG(*(tipo_movimento(mov) for mov in movs))

    for _ in interpretar_caminho(movs):
        while not rodas.done(): pass

    if na_grade: acertar_orientação_linha(ori_final)
    else:        acertar_orientação(ori_final)

def acertar_orientação(ori):
    if True:
        cardinais = ["N", "L", "S", "O"]
        idx_ori_final = cardinais.index(ori)
        idx_ori_atual = cardinais.index(orientação_estimada)

        idx_diff = ((idx_ori_atual - idx_ori_final) % len(cardinais)) - 2
        if idx_diff == -2: return
        if idx_diff == -1: virar_esquerda()
        if idx_diff ==  0: dar_meia_volta()
        if idx_diff ==  1: virar_direita()
    else:
        while orientação_estimada != ori:
            LOG(f"{orientação_estimada=}, {ori=}")
            virar_direita()

def acertar_orientação_linha(ori):
    cardinais = ["N", "L", "S", "O"]
    idx_ori_final = cardinais.index(ori)
    idx_ori_atual = cardinais.index(orientação_estimada)

    idx_diff = ((idx_ori_atual - idx_ori_final) % len(cardinais)) - 2
    if idx_diff == -2: return
    if idx_diff == -1: curva_linha_esquerda()
    if idx_diff ==  0: dar_meia_volta_linha()
    if idx_diff ==  1: curva_linha_direita()

def posicionamento_inicial():
    global orientação_estimada

    viu_vermelho = False
    while not (viu_vermelho and orientação_estimada == "L"):
        esq, dir = achar_não_verde_alinhado()

        bipes.separador()
        if   esq.beco() or dir.beco():
            #ASSERT(esq.beco() and dir.beco(), f"pos_ini: {esq} == {dir}")
            dar_ré_alinhar_primeiro_bloco()
            viu_vermelho = True
            virar_direita()
        elif esq.azul() or dir.azul():
            #ASSERT(esq.azul() and dir.azul(), f"pos_ini: {esq} == {dir}")
            dar_ré_meio_quarteirão()
            orientação_estimada = "L"
            virar_esquerda()
        else:
            #ASSERT(esq.amarelo() or dir.amarelo(), f"pos_ini: {esq} e {dir} assumidos amarelo")
            dar_ré_meio_quarteirão()
            orientação_estimada = "O"
            virar_direita()

'''def posicionamento():
    global orientação_estimada

    viu_vermelho = False
    while not (viu_vermelho and orientação_estimada == "L"):
        esq, dir = achar_não_verde_alinhado()

        bipes.separador()
        if   esq.beco() or dir.beco():
            #ASSERT(esq.beco() and dir.beco(), f"pos_ini: {esq} == {dir}")
            dar_ré_alinhar_primeiro_bloco()
            viu_vermelho = True
            virar_esquerda()
        elif esq.azul() or dir.azul():
            #ASSERT(esq.azul() and dir.azul(), f"pos_ini: {esq} == {dir}")
            dar_ré_meio_quarteirão()
            orientação_estimada = "L"
            virar_direita()
        else:
            #ASSERT(esq.amarelo() or dir.amarelo(), f"pos_ini: {esq} e {dir} assumidos amarelo")
            dar_ré_meio_quarteirão()
            orientação_estimada = "O"
            virar_esquerda()'''

def colocar_cubo_na_caçamba(cor_cubo, max_cubos=NUM_CUBOS_PEGÁVEIS):
    global cubos_caçambas
    margem      = TAM_CUBO//2 if max_cubos != 1 else (TAM_CUBO*3)//2
    espaçamento = TAM_CUBO    if max_cubos == 2 else 0

    ASSERT(cor_cubo != Cor.enum.NENHUMA)
    blt.levantar_garra()

    """
    vai até amarelo
    aí vira e vai reto até o vermelho,
    depois vira pro lado contrário (direção das caçambas)
    """
    acertar_orientação("O")
    ASSERT(orientação_estimada == "O", "alinhar_caçambas: é pra ser oeste!")

    achar_não_verde_alinhado()
    LOG("alinhar_caçambas: viu não verde (espera amarelo)")
    dar_ré_meio_quarteirão()

    virar_direita()
    achar_não_verde_alinhado()
    LOG("alinhar_caçambas: viu não verde (espera vermelho)")


    dar_ré(DIST_BORDA_CAÇAMBA + DIST_EIXO_SENSOR)
    dar_meia_volta()

    for i, (cor, dist) in enumerate(zip(cores_caçambas, DISTS_CAÇAMBAS)):
        if cor_cubo == cor:
            if cubos_caçambas[i] > max_cubos: continue

            rodas.straight(dist - DIST_BORDA_CAÇAMBA - margem
                          + cubos_caçambas[i]*(TAM_CUBO + espaçamento))
            blt.abaixar_garra()
            blt.levantar_garra()
            virar_direita()

            achar_não_verde_alinhado(max_tentativas=5, virar=False)

            rodas.straight(DIST_VERDE_CAÇAMBA-20)
            blt.abrir_garra()

            cubos_caçambas[i] += 1
            return

    SucessoOuCatástrofe("sem lugar pro cubo nas caçambas")

#! considerar adversário
def varredura(pos_estimada, caçambas):
    blt.resetar_garra()
    blt.abaixar_garra()

    y, x = pos_estimada #! para procura genérico, usar y também

    achar_cruzamento_linha()
    andar_dist_linha(TAM_FAIXA)
    pos = y, x
    while x < MAPA_X_MAX-2:
        DIST_VER_CUBO = DIST_CRUZAMENTO_CUBO + DIST_EIXO_SENSOR
        andar_dist_linha(DIST_VER_CUBO)
        x += 1 #! para procura genérico, considerar orientação

        ang = blt.fechar_garra()
        cor = blt.ver_cor_cubo()
        if ang > ANG_LIMIAR_GARRA_FECHADA or cor == Cor.enum.NENHUMA: #! constantizar
            LOG("varredura: cel livre fechou tudo")

            tira_obstaculo((y,x))
            blt.abrir_garra()
        else:
            luzes.mostrar(cor.color) #! fzr printar em braco
            break

        achar_cruzamento_linha(dist_max=TAM_QUARTEIRÃO-DIST_CRUZAMENTO_CUBO+10)
        andar_dist_linha(TAM_FAIXA)
        x += 1 #! para procura genérico, considerar orientação
    else:
        cor = None

    deixar = False
    if cor:
        if cor == Cor.enum.BRANCO:
            LOG(f"varredura: cubo branco")
            bipes.cabeca()
            deixar = True
        if ((cor not in cores_caçambas) or
            (cubos_caçambas[cores_caçambas.index(cor)] >= NUM_CUBOS_PEGÁVEIS)):
            LOG(f"varredura: cubo desconhecido cor {cor}")
            deixar = True

    if deixar:
        coloca_obstaculo((y,x))
        dar_ré(DIST_VOLTAR_CUBO) #!
        blt.abrir_garra()
        dar_ré(DIST_CRUZAMENTO_CUBO) #!
        cor = None

    acertar_orientação_linha("O")

    achar_cruzamento_linha()
    andar_dist_linha(TAM_BLOCO*(x+1))

    return cor, (y,0)

def partial(func, *args, **kwargs):
    return lambda *a, **kw: func(*args, *a, **kwargs, **kw)

#! tá empurrando?
def procura(pos_estimada, cores_caçambas):
    cel_incertas = pegar_celulas_incertas()
    cel_atual = pos_estimada

    caminhos_incertas = {
        cel: achar_caminhos(cel_atual, cel) for cel in cel_incertas
    }
    cel_incertas.sort(key=lambda x: (
        len(caminhos_incertas[x]) if caminhos_incertas[x] is not None else 9000
    ))

    blt.resetar_garra()
    blt.abaixar_garra()
    for cel_destino in cel_incertas:
        LOG("tentando de", cel_atual, "até :", cel_destino)

        caminho = achar_caminhos(cel_atual, cel_destino)
        if not caminho:
            LOG("procura: nenhum caminho encontrado")
            continue

        # anda até ficar imediatamente antes da celula destino
        caminho.pop(-1)
        seguir_caminho(caminho)
        cel_atual = caminho[-1]

        # acerta orientação para como se tivesse andado tudo
        restante = [cel_atual, cel_destino]
        _, ori_final = achar_movimentos(restante, orientação_estimada)
        acertar_orientação_linha(ori_final)

        # vê se tem alguma coisa na frente
        andar_dist_linha(DIST_CRUZAMENTO_CUBO)
        ang = blt.fechar_garra()
        if ang > 145: #! constantizar
            LOG("procura: cel livre fechou tudo")
            tira_obstaculo(cel_destino)
            achar_cruzamento_linha(dist_max=TAM_QUARTEIRÃO)
            cel_atual = cel_destino
            blt.abrir_garra()
            continue

        cor = blt.ver_cor_cubo()
        blt.mostrar_cor(cor)
        if cor == Cor.enum.BRANCO:
            bipes.cabeca()
        if cor == Cor.enum.NENHUMA:
            LOG("procura: cel livre viu NENHUM")
            tira_obstaculo(cel_destino)
            achar_cruzamento_linha(dist_max=TAM_QUARTEIRÃO)
            cel_atual = cel_destino
            continue
        if ((cor not in cores_caçambas) or
            (cubos_caçambas[cores_caçambas.index(cor)] >= NUM_CUBOS_PEGÁVEIS)):
            LOG(f"procura: cubo desconhecido cor {cor}")
            coloca_obstaculo(cel_destino)
            dar_ré(DIST_VOLTAR_CUBO) #!
            blt.abrir_garra()
            dar_ré(DIST_CRUZAMENTO_CUBO)
            continue

        imprimir_mapa()

        LOG(f"procura: cubo cor {cor}")
        tira_obstaculo(cel_destino)
        dar_ré(DIST_CRUZAMENTO_CUBO)
        return cor, cel_atual

    imprimir_mapa()

    SucessoOuCatástrofe("sem caminhos possíveis")

def descobrir_cor_caçambas():
    global cores_caçambas
    if not cores_caçambas:
        cores_caçambas = [Cor(cor=Cor.enum.NENHUMA) for i in range(NUM_CAÇAMBAS)]

    DIST_ALINHO = TAM_BLOCO//2

    acertar_orientação("O")
    achar_não_verde_alinhado()
    rodas.straight(DIST_SENSORES_CENTRO)
    acertar_orientação("S")

    #gambiarra ver o 1 com cor incerta
    pid_caçamba = pid(kp=0.5)
    andar_dist_linha(DIST_ALINHO, vel=20, pid=pid_caçamba)
    andar_dist_linha(DIST_BORDA_CAÇAMBA - DIST_ALINHO, pid=pid_caçamba)

    for i in range(NUM_CAÇAMBAS):
        cores_caçambas[i] = blt.ver_cor_caçamba()
        bipes.cabeca()
        dist = blt.ver_dist_caçamba()

        #! rataria cores caçambas
        if cores_caçambas[i].cor == Cor.enum.MARROM:
            cores_caçambas[i] = Cor(cor=Cor.enum.AMARELO)
        #if cores_caçambas[i].cor == Cor.enum.PRETO:
        #    if False: cores_caçambas[i] = Cor(cor=Cor.enum.AZUL)
        #    else:    cores_caçambas[i] = Cor(cor=Cor.enum.VERDE)
        if cores_caçambas[i].cor == Cor.enum.NENHUMA:
            if dist < TAM_QUARTEIRÃO:
                cores_caçambas[i] = Cor(cor=Cor.enum.PRETO)

        LOG(f"descobrir_cor_caçamba: caçamba {cores_caçambas[i]} a {dist/10}cm")

        if i+1 < NUM_CAÇAMBAS:
            andar_dist_linha(TAM_CAÇAMBA+DIST_CAÇAMBA, vel = 80)
    LOG(f"descobrir_cor_caçamba: cores_caçambas = {cores_caçambas}")
    rodas.turn(10)
    dar_ré(TAM_BLOCO//4)
    rodas.turn(-20)
   
class testes:
    @staticmethod
    def imprimir_cor_caçamba_para_sempre():
        while True:
            cor = blt.ver_cor_caçamba()
            print(cor)

    @staticmethod
    def imprimir_dist_caçamba_pra_sempre():
        while True:
            dist = blt.ver_dist_caçamba()
            print(f"{dist=}")

    @staticmethod
    def imprimir_cor_cubo_para_sempre():
        blt.SILENCIOSO = True
        while True:
            hsv = None #blt.ver_hsv_cubo()
            cor = blt.ver_cor_cubo()
            print(f"hsv: {hsv}, cor: {cor}")

    @staticmethod
    def imprimir_caçamba_para_sempre():
        blt.SILENCIOSO = True
        while True:
            dist = blt.ver_dist_caçamba()
            cor  = blt.ver_cor_caçamba()
            print(f"dist: {dist}, cor: {cor}")


conta_reset = 0
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
        except SystemExit:
            hub.system.set_stop_button(None)
            LOG("pedindo parada")
            rodas.reset()
            luzes.reset()
            bipes.cabeca()
            bipes.inicio()
            bipes.cabeca()
            conta_reset += 1

            wait(300)
            bots = set()
            while Button.CENTER not in bots:
                bots = hub.buttons.pressed()
                if Button.LEFT  in bots: pass
                if Button.RIGHT in bots: pass

            LOG("reiniciando")
            hub.system.set_stop_button(Button.CENTER)
            continue
        except Exception as e:
            bipes.falha()
            raise e
