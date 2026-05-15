from pybricks.tools import StopWatch

from lib.polyfill import Enum
from cores import Cor

from comum import globais
from comum import ASSERT, LOG as _LOG

SILENCIOSO = False
def LOG(*args, **kwargs):
    if not SILENCIOSO: _LOG(*args, **kwargs)

TX_CABECA = 24
TX_BRACO  = 69
TX_RABO   = 32

cmd = Enum("cmd", [
    "fecha_garra",
    "abre_garra",
    "levanta_garra",
    "abaixa_garra",
    "ver_cor_sensor_braco",
    "ver_hsv_sensor_braco",
    "ver_distancias_deprecado", #! retirar quando mexer na ordem, fazer upload em tudo
    "ver_cor_sensor_rabo",
    "ver_dist_sensor_braco", #! reordenar
    "levanta_garra_dist_sensor", #! reordenar
    "mostrar_cor",
])

rsp = Enum("rsp", [
    "fechei",
    "abri",
    "levantei",
    "abaixei",
    "cor_sensor_braco",
    "hsv_sensor_braco",
    "distancias_deprecado", #! retirar quando mexer na ordem, fazer upload em tudo
    "cor_sensor_rabo",
    "dist_sensor_braco", #! reordenar
    "levantei_dist_sensor", #! reordenar
    "mostrei_cor",
])

def enviar_mensagem(*msg, enum, **kwargs):
    LOG(f"enviar_mensagem: {enum(msg[0])}{msg[1:]}")
    globais.ble.broadcast(tuple(msg))

def enviar_comando(*comando):   enviar_mensagem(*comando, enum=cmd)
def enviar_resposta(*resposta): enviar_mensagem(*resposta, enum=rsp)

#! fazer esperar respostaS
def esperar_resposta(esperado, canal=TX_BRACO, espera=None, coalesce=True):
    resposta = -1
    crono = StopWatch()
    LOG(f"esperar_resposta: {canal} -> {rsp(esperado)}")
    while resposta != esperado:
        if espera and crono.time() > espera: break
        try:
            resposta = globais.ble.observe(canal) or (None,)
            resposta, *args = resposta
            LOG(f"esperar_resposta: -> {rsp(resposta)}{args}")
        except RuntimeError: continue
        except UnicodeError: continue
    if len(args) == 1 and coalesce:
        return args[0]
    return args


def igual(*a): return a
def primeiro(*a): return a[0] if a else []
def Cor2cor(a, *_): return a.cor
def cor2Cor(a, *_): return Cor(cor=a)
def fabricar_comando(comando, resposta, pre_envio=igual,
                                pre_retorno=primeiro,
                                canal=TX_BRACO):
    def enviar_e_receber(*args, espera=None, **kwargs):
        envio = pre_envio(*args)
        try:              enviar_comando(comando, *envio)
        except TypeError: enviar_comando(comando,  envio)
        return pre_retorno(
            *esperar_resposta(resposta, espera=espera,
                              canal=canal, coalesce=False,
                              **kwargs)
        )
    return enviar_e_receber

fechar_garra = fabricar_comando(cmd.fecha_garra, rsp.fechei)
abrir_garra  = fabricar_comando(cmd.abre_garra, rsp.abri)

levantar_garra = fabricar_comando(cmd.levanta_garra, rsp.levantei)
abaixar_garra  = fabricar_comando(cmd.abaixa_garra, rsp.abaixei)

ver_dist_caçamba = fabricar_comando(cmd.ver_dist_sensor_braco, rsp.dist_sensor_braco)

mostrar_cor = fabricar_comando(
    cmd.mostrar_cor, rsp.mostrei_cor,
    pre_envio=Cor2cor,
)

ver_cor_cubo = fabricar_comando(
    cmd.ver_cor_sensor_rabo, rsp.cor_sensor_rabo,
    pre_retorno=cor2Cor, canal=TX_RABO,
)

ver_cor_caçamba = fabricar_comando(
    cmd.ver_cor_sensor_braco, rsp.cor_sensor_braco,
    pre_retorno=cor2Cor#, canal=TX_RABO,
)

def ver_hsv_cubo():
    return ASSERT(False, "o sensor de ev3 do rabo não consegue ler hsv")


def resetar_garra(): resetar_garra_cima()

def resetar_garra_cima():
    levantar_garra()
    fechar_garra()
    abrir_garra()

def resetar_garra_baixo():
    levantar_garra()
    fechar_garra()
    abrir_garra()
    abaixar_garra()

