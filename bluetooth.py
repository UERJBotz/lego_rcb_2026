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


def nada(*a): return a
def cor2Cor(resp, *_): return Cor(cor=resp)
def Cor2cor(args, *_): return args.cor
def função_comando(comando, resposta, pre_envio=nada, pre_retorno=nada, canal=TX_BRACO):
    def enviar_e_receber(*args, espera=None, **kwargs):
        enviar_comando(comando, *pre_envio(*args))
        return pre_retorno(
            *esperar_resposta(resposta, espera=espera, canal=canal, coalesce=False, **kwargs)
        )
    return enviar_e_receber

fechar_garra = função_comando(cmd.fecha_garra, rsp.fechei)
abrir_garra  = função_comando(cmd.abre_garra, rsp.abri)

levantar_garra = função_comando(cmd.levanta_garra, rsp.levantei)
abaixar_garra  = função_comando(cmd.abaixa_garra, rsp.abaixei)

ver_dist_caçamba = função_comando(cmd.ver_dist_sensor_braco, rsp.dist_sensor_braco)

mostrar_cor = função_comando(
    cmd.mostrar_cor, rsp.mostrei_cor,
    pre_envio=Cor2cor,
)

ver_cor_cubo = função_comando(
    cmd.mostrar_cor, rsp.mostrei_cor, canal=TX_RABO,
    pre_retorno=cor2Cor,
)

ver_cor_caçamba = função_comando(
    cmd.ver_cor_sensor_braco, rsp.cor_sensor_braco,
    pre_retorno=cor2Cor,
)

def ver_hsv_cubo():
    return ASSERT(False, "o sensor do rabo não consegue ler hsv")


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

