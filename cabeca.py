from pybricks.hubs import PrimeHub

from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Side, Axis, Direction, Button, Color

from pybricks.tools      import wait as delay
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


BOTÃO_PAUSA    = Button.CENTER
BOTÃO_CONTINUA = Button.CENTER
BOTÕES_DESLIGA = {Button.LEFT}

NUM_CUBOS_PEGÁVEIS = 2

VEL_ALINHAR = 80
VEL_ANG_ALINHAR = 20
GIRO_MAX_ALINHAR = 90

TAM_QUARTEIRÃO = 300
TAM_BLOCO = TAM_QUARTEIRÃO//2
TAM_FAIXA = 20

NUM_CAÇAMBAS = 5
TAM_CAÇAMBA = 160
TAM_CUBO = 50

TAM_SENSOR_FRONTAL = 20
DIST_EIXO_SENSOR = 45
DIST_EIXO_SENSOR_FRENTE = 110
DIST_EIXO_SENSOR_TRAS = 110
DIST_CRUZAMENTO_CUBO = TAM_BLOCO - DIST_EIXO_SENSOR_FRENTE
DIST_VOLTAR_CUBO = 0
DIST_SENSORES_CENTRO = 20
DIST_ALINHAR_PID_CACAMBA = 60

DIST_BORDA_CAÇAMBA = 130
DIST_CAÇAMBA = 100
DIST_VERDE_CAÇAMBA = 80
<<<<<<< HEAD
=======
VEL_SEGUIR_LINHA = 380 #mm/s
VEL_MIN_SEGUIR_LINHA = 100
K_VEL = 2
>>>>>>> 0498367 (tentativa vel adaptativa(+))
TAM_PISTA_TODA = TAM_QUARTEIRÃO*6
BLOCO_MEIO = 4

REFL_MIN, REFL_MAX = 22, 99
REFL_IDEAL = (REFL_MAX + REFL_MIN)/2
VEL_SEGUIR_LINHA = 100 #mm/s

DISTS_CAÇAMBAS = [ DIST_BORDA_CAÇAMBA +
                   TAM_CAÇAMBA*i + DIST_CAÇAMBA*i
                   for i in range(NUM_CAÇAMBAS) ]
cubos_caçambas = [0 for i in range(NUM_CAÇAMBAS)]
cores_caçambas = []
cubos_no_robo = [Cor.NENHUMA,Cor.NENHUMA]


#! 2024: checar stall: jogar exceção
#! 2024: checar cor errada no azul

#! quando sofrer exceção e DEBUG == False, entrar num estado que só espera o reset

#! a procura para(va) no meio quando vê um cubo desconhecido e acha que é catástrofe

#! na varredura, quando vê azul, parar de ir e reclamar do inimigo

def WAIT(ms):
    if TESTE:
        LOG("DEBUG: wait", ms)
        delay(ms)

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
    hub.system.set_stop_button(BOTÃO_PAUSA)
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
        blt.levantar_garra()
        blt.retrair_plataforma()
        posicionamento_inicial()
        pos_estimada = (0,0)

        if not cores_caçambas:
            descobrir_cor_caçambas()
            acertar_orientação("L")
            achar_azul_alinhado()
            dar_ré_alinhar_primeiro_bloco()
            posicionamento_inicial()

        dir_linha.mul = dir_linha.DIR
        while False:
            LOG(f"main: varrendo: quarteirão {quarteirão_atual}")

            achar_não_verde_alinhado()
            rodas.straight(DIST_EIXO_SENSOR)
            dir_linha.mul = achar_direção_linha()
            rodas.turn(-rodas.angle())
            cor, pos_estimada = varredura(pos_estimada, cores_caçambas)
            orientação_estimada = 'L'

            y,x = pos_estimada
            imprimir_mapa()

            dar_meia_volta_linha()
            for _ in range(x):
                achar_cruzamento_linha(dist_max=TAM_BLOCO)
                achar_não_cruzamento()
            LOG("voltando último cruzamento")

            # caminho = achar_caminhos(pos_estimada, (y, 0))
            # LOG(caminho)
            # seguir_caminho(caminho)
            rodas.straight(DIST_EIXO_SENSOR)

            y,x = pos_estimada = ((y + 2) % MAPA_Y_MAX, x)
            if cor != Cor.NENHUMA:
                colocar_cubo_na_caçamba(cor)
                dar_ré(DIST_VERDE_CAÇAMBA)

            dist_quarteirão = TAM_QUARTEIRÃO + TAM_FAIXA//2

            posicionamento_inicial()
            for _ in range(y//2):
                acertar_orientação("S")
                rodas.straight(dist_quarteirão)
                acertar_orientação("L")
                achar_não_verde_alinhado()
                dar_ré_meio_quarteirão()
            pos_estimada = (y, 0)
        else:
            LOG("main: procurando")
            achar_não_verde_alinhado()
            rodas.straight(DIST_EIXO_SENSOR)
            dir_linha.mul = achar_direção_linha()
            rodas.turn(-rodas.angle())

            cor, pos_estimada = procura(pos_estimada, cores_caçambas)
            LOG(f"main: tinha cubo {Cor(cor)} na frente de {pos_estimada}") #! LOG de procura antigo

            caminho_volta = achar_caminhos(pos_estimada, (0,0))
            LOG(f"main: seguindo caminho de volta: {caminho_volta}")
            seguir_caminho(caminho_volta)
            LOG(f"main: pulando o azul")
            rodas.straight(DIST_EIXO_SENSOR)

            LOG(f"main: indo colocar cubo")
            colocar_cubo_na_caçamba(cor)
            dar_ré(DIST_VERDE_CAÇAMBA)

def test():
    global orientação_estimada, pos_estimada, na_grade, cores_caçambas
    ... # testar coisas aqui sem mudar o resto do código

    blt.SILENCIOSO = True
    if False:
        global BOTÕES_DESLIGA
        BOTÕES_DESLIGA = {Button.CENTER}

    if True:
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
            if True: cores_caçambas[0] = Cor.AZUL
            if True: cores_caçambas[1] = Cor.VERMELHO
            if True: cores_caçambas[2] = Cor.MARROM
            if True: cores_caçambas[3] = Cor.VERDE
            if True: cores_caçambas[4] = Cor.PRETO
            blt.mostrar_caçambas(cores_caçambas)

        if False: orientação_estimada = "N"
        if False: orientação_estimada = "S"
        if False: orientação_estimada = "L"
        if False: orientação_estimada = "O"

        if True:
            main()

    if False:
        orientação_estimada = "L"
        dir_linha.mul = dir_linha.DIR
        for obs in [(8,8),
                    (8,6),
                    (8,4),
                    (8,2),
                    (7,1),
                    (2,6),
                    (4,6),
                    (6,6)]:
            tira_obstaculo(obs)

        caminho = achar_caminhos((8,8), (6,6))
        seguir_caminho_inclusivo(caminho)

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
        blt.fechar_garra()

    if False:
        cubos_no_robo = [Cor.VERDE,Cor.AZUL]
        blt.mostrar_cubos(cubos_no_robo)

    if False: testes.imprimir_cor_cubo_para_sempre()
    if False: testes.imprimir_cor_caçamba_para_sempre()

    while False:
        if True: vel = 50 # mudar quando testar
        else:    vel, *_ = rodas.settings()

        for _ in range(5):
            achar_cruzamento_linha(vel=vel)
            bipes.separador()
            curva_linha_esquerda()

        achar_cruzamento_linha(vel=vel)
        bipes.separador()
        curva_linha_esquerda()

        achar_cruzamento_linha(vel=vel)
        bipes.separador()
        curva_linha_direita()

    while False:
        contador = 0

        for _ in range(5):
            achar_cruzamento_linha()
            luzes.mostrar(Color.BLUE)
            contador += 1

            andar_dist_linha(TAM_BLOCO)
            luzes.mostrar(Color.RED)

            LOG("cruzamentos: ", contador)


    while False:
        caca = blt.ver_dist_caçamba()
        cubo = blt.ver_dist_cubo()
        LOG(caca, cubo)

    if False:
        pos_inicial  = (0,0)
        pos_final    = (3,3)

        pos_estimada = pos_inicial
        orientação_estimada = "L"

        achar_não_verde_alinhado()
        rodas.straight(DIST_EIXO_SENSOR)

        na_grade = True
        caminho_ida = [(0,0), (0,1), (0,2), (1,2)]
                      #achar_caminhos(pos_inicial, pos_final)
        LOG(caminho_ida)
        seguir_caminho(caminho_ida)
        caminho_volta = list(reversed(caminho_ida))
                        #achar_caminhos(pos_final, pos_inicial)
        LOG(caminho_volta)
        seguir_caminho(caminho_volta)

    if False:
        blt.abrir_garra()
        blt.fechar_garra()
        bipes.separador()
    if False:
        blt.abaixar_garra()
        blt.levantar_garra()
        bipes.separador()
    if False:
        blt.retrair_plataforma()
        blt.estender_plataforma()
        blt.retrair_plataforma()
        bipes.separador()

# Logica colocar um cubo na plataforma.


    if True:
        LOG("retrai_plataforma")
        blt.retrair_plataforma()
        bipes.separador()

        LOG("abaixa_garra")
        blt.abrir_garra()
        blt.abaixar_garra()
        bipes.separador()

        LOG("levanta_garra")
        blt.fechar_garra()
        blt.levantar_garra()
        bipes.separador()
        blt.mediar_garra()
        blt.abrir_garra()
        blt.levantar_garra()      

# Logica colocar um cubo na plataforma.  

    if False:
        #Logica colocar um cubo na plataforma.
        LOG("retrai_plataforma")
        blt.retrair_plataforma()
        bipes.separador()

        LOG("abaixa_garra")
        blt.abrir_garra()
        blt.abaixar_garra()
        bipes.separador()

        LOG("levanta_garra")
        blt.fechar_garra()
        blt.levantar_garra()
        bipes.separador()

        LOG("estender_plataforma")
        blt.estender_plataforma()
        LOG("mediar_garra")
        blt.mediar_garra()
        blt.abrir_garra()
        blt.levantar_garra()
        bipes.separador()

        LOG("retrai_plataforma")
        blt.retrair_plataforma()
        blt.sacodir_plataforma(2)
        bipes.separador()


        #Colocou um cubo na plataforma.

        LOG("abaixa_garra")
        blt.abaixar_garra()
        bipes.separador()

        LOG("levanta_garra")
        blt.fechar_garra()
        blt.levantar_garra()
        bipes.separador()

        #Pegou segundo cubo.

        LOG("Colocando segundo cubo na caçamba")
        blt.mediar_garra()
        blt.abrir_garra()
        blt.levantar_garra()
        bipes.separador()

        LOG("Colocando primeiro cubo na caçamba")
        blt.estender_plataforma()
        blt.mediar_garra()
        blt.fechar_garra()
        blt.levantar_garra()
        blt.retrair_plataforma()
        blt.mediar_garra()
        blt.abrir_garra()

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
    dar_ré(TAM_BLOCO - DIST_EIXO_SENSOR - TAM_FAIXA)
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

def girar_até_dist(giro_max=90):
    with mudar_velocidade(80, 30):
        rodas.reset()
        rodas.turn(giro_max, wait=False)
        while not rodas.done():
            dist = blt.ver_dist_cubo()
            if dist <= TAM_BLOCO:
                rodas.stop()
                LOG("Achei o cubo e estou indo para ele.")
                #WAIT(2000)
                return
        LOG("Não achei o cubo.")
        rodas.turn(-rodas.angle())
        return



def girar_até_linha(margem=0.1, giro_max=360):
    raio_refl = REFL_IDEAL - REFL_MIN
    margem_refl = raio_refl*margem

    centro = sensor_cor_centro.reflection()
    if abs(centro - REFL_IDEAL) <= margem_refl:
        LOG(f"Ja estou em cima da linha")
        if dir_linha.mul * giro_max <= 0:
            LOG(f"Girando pro lado oposto do dir_linha atual.")
            rodas.straight(TAM_FAIXA)
        else:
            return dir_linha.mul

    with mudar_velocidade(80, 30):
        rodas.reset()
        rodas.turn(giro_max-30, wait=False)
        while not rodas.done():
            centro = sensor_cor_centro.reflection()
            if abs(centro - REFL_IDEAL) <= margem_refl and giro_max > 0:
                rodas.stop()
                LOG("Achei a linha e estou seguindo a esquerda dela.", centro)
                #WAIT(2000)
                return dir_linha.ESQ
            elif abs(centro - REFL_IDEAL) <= margem_refl and giro_max < 0:
                rodas.stop()
                LOG("Achei a linha e estou seguindo a direita dela.", centro)
                #WAIT(2000)
                return dir_linha.DIR

        LOG("Não achei a linha.")
        return dir_linha.mul

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

def achar_direção_linha(margem=0.1, giro_max=45):
    raio_refl = REFL_IDEAL - REFL_MIN
    margem_refl = raio_refl*margem

    centro = sensor_cor_centro.reflection()
    if abs(centro - REFL_IDEAL) <= margem_refl:
        rodas.turn(-10)
        LOG("Ja estou em cima da linha, saindo para achar ela de novo.")
        return achar_direção_linha(giro_max=giro_max)

    with mudar_velocidade(80, 30):
        rodas.reset()
        rodas.turn(giro_max, wait=False)
        while not rodas.done():
            centro = sensor_cor_centro.reflection()
            if abs(centro - REFL_IDEAL) <= margem_refl:
                rodas.stop()
                LOG("Achei a linha e estou seguindo a esquerda dela.", centro)
                return dir_linha.ESQ
        rodas.turn(-2*giro_max, wait=False)
        while not rodas.done():
            centro = sensor_cor_centro.reflection()
            if abs(centro - REFL_IDEAL) <= margem_refl:
                rodas.stop()
                LOG("Achei a linha e estou seguindo a direita dela.", centro)
                return dir_linha.DIR
        LOG("Não achei a linha.")
        return dir_linha.mul

def preto(val): return val <= REFL_MIN + 5

def até_não_cruzamento(dist,esq,centro,dir, preto):
    return not (preto(esq) and preto(dir))

def até_dist_max_ou_não_cruzamento(dist_max):
    def func(dist, esq, centro, dir, preto):
        return (
            até_não_cruzamento(dist, esq, centro, dir,
                preto=(lambda luz: (luz < REFL_IDEAL)) #! hardcode
            ) or até_dist_max(dist_max)(dist, esq, centro, dir, preto)
        )
    return func

def achar_não_cruzamento(*, dist_max=TAM_PISTA_TODA, **kwargs):
    LOG("Pulando Cruzamento.")
    seguir_linha_até(até_dist_max_ou_não_cruzamento((dist_max), **kwargs))
    LOG("Cruzamento pulado.")

def seguir_até(parada=até_dist_max(TAM_PISTA_TODA),
                     *, vel=None, pid=None, parar_no_verde=False, caçamba=False):
    if vel is None: vel = VEL_SEGUIR_LINHA

    def preto(val): return val <= REFL_MIN + 5

    rodas.reset()
    rodas.drive(vel)
    while not parada(rodas.distance(), esq, centro, dir, preto):
        esq    = sensor_cor_esq.reflection()
        centro = sensor_cor_centro.reflection()
        dir    = sensor_cor_dir.reflection()

    rodas.stop()


def pid(kp, kd=0, ki=0):
    ierro = erro_ant = 0
    def f(erro):
        nonlocal ierro
        nonlocal erro_ant
        derro = erro - erro_ant
        ativ  = erro*kp + derro*kd + ierro*ki

        erro_ant = erro
        ierro   += erro
        return ativ
    return f

pid_linha = pid(kp=0.50)
def seguir_linha_até(parada=até_dist_max_ou_cruzamento(TAM_PISTA_TODA),
                     *, vel=None, pid=None, parar_no_verde=False, caçamba=False):
    if vel is None: vel = VEL_SEGUIR_LINHA
    if pid is None: pid = pid_linha

    def preto(val): return val <= REFL_MIN + 5

    rodas.reset()
    while True:
        esq    = sensor_cor_esq.reflection()
        centro = sensor_cor_centro.reflection()
        dir    = sensor_cor_dir.reflection()

        if caçamba:
            erro = dir_linha.DIR * (REFL_IDEAL - esq)
        else:
            erro = dir_linha.mul * (REFL_IDEAL - centro)

        if vel > 0:
            vel = max(VEL_MIN_SEGUIR_LINHA, vel - K_VEL*abs(erro))
        rodas.drive(vel, pid(erro))

        if parada(rodas.distance(), esq, centro, dir, preto): break
        if parar_no_verde: #! isso era o erro do bipe eterno, ver se precisa
            esq, dir = cores.todas(sensor_cor_esq, sensor_cor_dir)
            if esq == Cor.enum.VERDE    and dir == Cor.enum.VERDE: break
            if esq == Cor.enum.VERMELHO and dir == Cor.enum.VERMELHO: break #! parar no vermelho

    rodas.stop()

def girar_orientação(passos):
    global orientação_estimada
    #+ pra horário e - pra antihorário
    cardinais = ["N", "L", "S", "O"]
    i = cardinais.index(orientação_estimada)
    orientação_estimada = cardinais[(i + passos) % 4]

def curva_linha_esquerda():
    dir_linha.mul = dir_linha.DIR
    rodas.curve(DIST_EIXO_SENSOR, -90)
    girar_orientação(-1)

def curva_linha_direita():
    dir_linha.mul = dir_linha.ESQ
    rodas.curve(DIST_EIXO_SENSOR, +90)
    girar_orientação(+1)

def virar_esquerda_linha():
    alinhar_eixo_cruzamento()
    rodas.turn(-60)
    girar_até_linha(giro_max=-115)
    dir_linha.mul = dir_linha.DIR
    #dar_ré(DIST_EIXO_SENSOR)
    girar_orientação(-1)

def virar_direita_linha():
    alinhar_eixo_cruzamento()
    rodas.turn(+60)
    girar_até_linha(giro_max=+115)
    dir_linha.mul = dir_linha.ESQ
    #dar_ré(DIST_EIXO_SENSOR)
    girar_orientação(+1)

def alinhar_eixo_cruzamento():
    rodas.reset()
    achar_não_cruzamento()
    if rodas.distance() == 0:
        seguir_linha_até(até_dist_max(DIST_EIXO_SENSOR - TAM_FAIXA))
    else:
        seguir_linha_até(até_dist_max(DIST_EIXO_SENSOR - rodas.distance()))

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

def seguir_caminho_inclusivo(caminho):
    cel_destino = caminho.pop(-1)
    cel_final   = caminho[-1]

    ## acerta orientação para como se tivesse andado tudo
    restante = [cel_final, cel_destino]
    _, ori_final = achar_movimentos(restante, orientação_estimada)

    seguir_caminho(caminho)
    LOG("seguir_caminho_inclusivo: acertar_orientação_linha")
    acertar_orientação_linha(ori_final)
    return cel_final

def seguir_caminho(caminho): #! lidar com outras coisas
    def interpretar_movimento_livre(i, movs):
        mov = movs[i]
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

    def interpretar_movimento_cidade(i, movs):
        #! fazer run length encoding aqui
        mov = movs[i]
        if   mov == tipo_movimento.FRENTE:
            if movs[i+1:] and movs[i+1] == tipo_movimento.FRENTE:
                achar_cruzamento_linha(dist_max=TAM_BLOCO + 2*DIST_EIXO_SENSOR)
                rodas.reset()
                achar_não_cruzamento(dist_max=TAM_BLOCO)
                if movs[i+2:] and movs[i+2] != tipo_movimento.FRENTE:
                    dar_ré(rodas.distance())
            else:
                achar_cruzamento_linha(dist_max=TAM_BLOCO)
        elif mov == tipo_movimento.TRAS:
            if movs[i+1:] and movs[i+1] == tipo_movimento.FRENTE:
                dar_meia_volta_linha()
                #achar_não_cruzamento()
                achar_cruzamento_linha(dist_max=TAM_BLOCO)
                achar_não_cruzamento()
            else:
                dar_meia_volta_linha()
                achar_cruzamento_linha(dist_max=TAM_BLOCO + 2*DIST_EIXO_SENSOR)
        elif mov == tipo_movimento.ESQUERDA_FRENTE:
            virar_esquerda_linha()
            achar_cruzamento_linha(dist_max=TAM_BLOCO + 2*DIST_EIXO_SENSOR)
        elif mov == tipo_movimento.DIREITA_FRENTE:
            virar_direita_linha()
            achar_cruzamento_linha(dist_max=TAM_BLOCO + 2*DIST_EIXO_SENSOR)
        elif mov == tipo_movimento.ESQUERDA:
            virar_esquerda_linha()
        elif mov == tipo_movimento.DIREITA:
            virar_direita_linha()

    if na_grade: interpretar_movimento = interpretar_movimento_cidade
    else:        interpretar_movimento = interpretar_movimento_livre

    def interpretar_caminho(caminho): #! receber orientação?
        for i, mov in enumerate(caminho): #! yield orientação nova?
            LOG(f"seguir_caminho: {tipo_movimento(mov)}")
            delay(0)
            interpretar_movimento(i,caminho)
            yield rodas.distance()

    movs, ori_final = achar_movimentos(caminho, orientação_estimada)
    if movs:
        LOG(*(tipo_movimento(mov) for mov in movs))

        for _ in interpretar_caminho(movs):
            while not rodas.done(): pass

        #if na_grade:
        #    LOG("Seguir caminho na grade-acertar_orientação_linha")
        #    acertar_orientação_linha(ori_final)
        #else:
        #    LOG("Seguir caminho na grade-acertar_orientação_linha")
        #    acertar_orientação(ori_final)
    else:
        pass #! falhar mais alto

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

def acertar_orientação_linha(ori): #! isso não deveria andar pra frente
    cardinais = ["N", "L", "S", "O"]
    idx_ori_final = cardinais.index(ori)
    idx_ori_atual = cardinais.index(orientação_estimada)

    LOG("Me orientando para frente do cubo.")

    idx_diff = ((idx_ori_atual - idx_ori_final) % len(cardinais)) - 2
    if idx_diff == -2:
        LOG("Cubo na minha frente")
        achar_não_cruzamento()
    if idx_diff == -1:
        LOG("Fazendo curva esquerda para pegar cubo.")
        virar_esquerda_linha()
        girar_até_dist(giro_max=-30)
    if idx_diff ==  0:
        LOG("Dando meia volta para pegar cubo.")
        dar_meia_volta_linha()
    if idx_diff ==  1:
        LOG("Fazendo curva direita para pegar cubo.")
        virar_direita_linha()
        girar_até_dist(giro_max=30)

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
    WAIT(2000)
    dar_ré_meio_quarteirão()

    virar_direita()
    achar_não_verde_alinhado()
    LOG("alinhar_caçambas: viu não verde (espera vermelho)")
    WAIT(2000)

    dar_ré(DIST_BORDA_CAÇAMBA + DIST_EIXO_SENSOR)
    dar_meia_volta()

    LOG("colocar_cubo_na_caçamba: indo até a caçamba")
    for i, (cor, dist) in enumerate(zip(cores_caçambas, DISTS_CAÇAMBAS)):
        if cor_cubo == cor:
            if cubos_caçambas[i] > max_cubos: continue

            LOG(f"colocar_cubo_na_caçamba: colocando na caçamba {Cor(cor)}")
            rodas.straight(dist - DIST_BORDA_CAÇAMBA - margem
                          + cubos_caçambas[i]*(TAM_CUBO + espaçamento))
            virar_direita()

            achar_não_verde_alinhado(max_tentativas=5, virar=False)

            rodas.straight(DIST_VERDE_CAÇAMBA)
            blt.mediar_garra()
            blt.abrir_garra()

            cubos_caçambas[i] += 1
            return

    SucessoOuCatástrofe("sem lugar pro cubo nas caçambas")


def verificar_aresta(dist_cubo_perto=TAM_BLOCO, dist_max=TAM_QUARTEIRÃO, dist_min=DIST_CRUZAMENTO_CUBO//10):
    ## estamos no cruzamento virados pra aresta
    rodas.reset()
    achar_não_cruzamento()
    dist = blt.ver_dist_cubo()
    if dist <= dist_cubo_perto:
        # tem cubo e precisamos ver a cor
        andar_dist_linha(dist)
        cor = blt.ver_cor_cubo()
        if cor == Cor.NENHUMA:
            # leitura falsa de distância ou cor errada ou viu inimigo
            LOG("verificar_aresta: cel livre viu cor.NENHUMA*")
            achar_cruzamento_linha(dist_max=dist_max-rodas.distance())
            return True, Cor.NENHUMA
        else:
            # pode ter cubo pegável
            rejeitado = False
            if cor == Cor.BRANCO:
                LOG(f"verificar_aresta: cubo BRANCO")
                bipes.cabeca()
            if cor not in cores_caçambas:
                LOG(f"verificar_aresta: cubo desconhecido cor {cor}")
                rejeitado = True
            elif cubos_caçambas[cores_caçambas.index(cor)] >= NUM_CUBOS_PEGÁVEIS:
                LOG(f"verificar_aresta: cubo caçamba cheia cor {cor}")
                rejeitado = True

            if rejeitado:
                # cubo impegável
                andado = rodas.distance()
                #dar_ré(andado + DIST_EIXO_SENSOR)
                dar_ré(andado)
                #alinha_eixo_cruzamento() #! tirar pq todas as funções de depois já fazem
                LOG(f"verificar_aresta: {andado=}, {dist=}")
                return False, cor
            else:
                # um cubo pegável
                LOG(f"verificar_aresta: viu cor.{Cor(cor)}")
                blt.abaixar_garra()
                blt.fechar_garra()
                blt.mediar_garra()
                cubos_no_robo[0] = cor #! Mudar a logica do 0 ao pegar dois cubos.
                blt.mostrar_cubos(cubos_no_robo)

                achar_cruzamento_linha(dist_max=dist_max-rodas.distance())
                return True, cor
    else:
        # não tem cubo
        LOG("verificar_aresta: cel livre viu dist longe")
        achar_cruzamento_linha(dist_max=dist_max)
        return True, Cor.NENHUMA

def até_não_cruzamento(dist,esq,centro,dir, preto):
    return not (preto(esq) and preto(dir))

def achar_não_cruzamento(*, dist_max=TAM_PISTA_TODA, **kwargs):
    andar_até_idx(
        lambda: (até_não_cruzamento(rodas.distance(),
                                    sensor_cor_dir.reflection(),
                                    sensor_cor_centro.reflection(),
                                    sensor_cor_esq.reflection(),
                                    preto=(lambda luz: (luz < REFL_IDEAL))),
                 None), dist_max=dist_max, **kwargs)

def alinha_eixo_cruzamento():
    rodas.reset()
    achar_não_cruzamento()
    tam_faixa = rodas.distance()
    LOG(f"alinha_eixo_cruzamento: tamanho faixa lido {tam_faixa} vs {TAM_FAIXA} esperado")
    if tam_faixa:
        seguir_linha_até(até_dist_max(DIST_EIXO_SENSOR - tam_faixa))
    else:
        seguir_linha_até(até_dist_max(DIST_EIXO_SENSOR - TAM_FAIXA))

def atravessa_rua_pegando_cubo_ou_nao(aresta, dist_max=TAM_QUARTEIRÃO):
    def aresta_como_pos(aresta):
        n1, n2 = aresta
        return (n1[0]+n2[0])//2, (n1[1]+n2[1])//2
    def _tira_obstaculo(aresta): #!
        LOG(f"Tirei obstaculo da aresta {aresta}.")
        tira_obstaculo(aresta_como_pos(aresta))
    def _coloca_obstaculo(aresta): #!
        LOG(f"Coloquei obstaculo na aresta {aresta}")
        coloca_obstaculo(aresta_como_pos(aresta))

    passa, cor = verificar_aresta(dist_max=dist_max)
    ini, fim = aresta
    if passa:
        # cubo ou nada
        _tira_obstaculo(aresta)
        return cor, fim
    else:
        # obstáculo
        _coloca_obstaculo(aresta)
        return cor, ini

#! considerar adversário
def varredura(pos_estimada, caçambas):
    y,x = pos_estimada

    ## garra
    blt.retrair_plataforma()
    blt.mediar_garra()
    blt.abrir_garra()

    ## cruzamento inicial
    achar_cruzamento_linha()
    achar_não_cruzamento()
    x += 1

    for _ in range(5):
        prev = (y,x)
        cor, (y,x) = atravessa_rua_pegando_cubo_ou_nao(((y, x), (y, x+2)))

        if (y,x) == prev:
            break
        if cor != Cor.NENHUMA:
            return cor, (y,x)

    return cor, (y,x)

def partial(func, *args, **kwargs):
    return lambda *a, **kw: func(*args, *a, **kwargs, **kw)

#! tá empurrando?
def procura(pos_estimada, cores_caçambas):
    ## ordena as células incertas por distância à origem
    cel_incertas = pegar_celulas_incertas()
    cel_atual = pos_estimada

    caminhos_incertas = {
        cel: achar_caminhos(cel_atual, cel) for cel in cel_incertas
    }
    cel_incertas.sort(key=lambda x: (
        len(caminhos_incertas[x]) if caminhos_incertas[x] is not None else 9000
    ))

    ## garra
    blt.retrair_plataforma()
    blt.mediar_garra()
    blt.abrir_garra()

    ## passa por cada um das células incertas
    for cel_destino in cel_incertas:
        LOG("tentando de", cel_atual, "até :", cel_destino)

        caminho = achar_caminhos(cel_atual, cel_destino)
        if not caminho:
            LOG("procura: nenhum caminho encontrado")
            continue

        ## anda até ficar imediatamente antes de procurates da celula destino
        (y0,x0) = seguir_caminho_inclusivo(caminho)
        (y1,x1) = cel_destino
        depois = (y0 + 2*(y1-y0), x0 + 2*(x1-x0))
        cor, cel_atual = atravessa_rua_pegando_cubo_ou_nao(((y0, x0), depois))
        LOG(f"procura: Estou em {cel_atual}.")

        if cor == Cor.NENHUMA: continue
        if cel_atual == (y0,x0): continue

        imprimir_mapa()

        LOG(f"procura: cubo cor {cor}")
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
    rodas.straight(DIST_ALINHAR_PID_CACAMBA)
    acertar_orientação("S")

    #gambiarra ver o 1 com cor incerta
    pid_caçamba = pid(kp=0.5)
    andar_dist_linha(DIST_ALINHO, vel=20, pid=pid_caçamba, caçamba=True)
    andar_dist_linha(DIST_BORDA_CAÇAMBA - DIST_ALINHO, pid=pid_caçamba, caçamba=True)

    for i in range(NUM_CAÇAMBAS):
        cores_caçambas[i] = blt.ver_cor_caçamba()
        blt.mostrar_caçambas(cores_caçambas[:i+1])
        bipes.cabeca()

        LOG(f"descobrir_cor_caçamba: caçamba {cores_caçambas[i]}")

        if i+1 < NUM_CAÇAMBAS:
            andar_dist_linha(TAM_CAÇAMBA+DIST_CAÇAMBA, vel = 80, caçamba=True)
    LOG(f"descobrir_cor_caçamba: cores_caçambas = {cores_caçambas}")
    blt.mostrar_caçambas(cores_caçambas)

    rodas.turn(10)
    dar_ré(TAM_BLOCO)
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

            delay(300)
            bots = set()
            while BOTÃO_CONTINUA not in bots:
                bots = hub.buttons.pressed()
                if BOTÕES_DESLIGA <= bots:
                    raise SystemExit

                if Button.RIGHT     in bots: pass
                if Button.BLUETOOTH in bots: pass

            LOG("reiniciando")
            hub.system.set_stop_button(Button.CENTER)
            continue
        except Exception as e:
            bipes.falha()
            raise e
