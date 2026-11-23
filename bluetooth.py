from comum import globais
from comum import LOG, ASSERT

from pybricks.tools import wait

from lib.polyfill import Enum
try: #!
    from cores import Cor
except ImportError:
    LOG("IHH.... sem Cor...")


SILENCIOSO = False

TX_CABECA = 24
TX_BRACO  = 69
TX_RABO   = 32

cmd = Enum("cmd", [
    "fecha_garra",
    "abre_garra",
    "levanta_garra",
    "abaixa_garra",
    "ver_cor_sensor_caçamba",
    "ver_hsv_sensor_caçamba",
    "ver_dist_sensor_cubo",
    "ver_cor_sensor_cubo",
    "ver_dist_sensor_caçamba", #! reordenar
    "levanta_garra_dist_sensor", #! reordenar
    "mostrar_cor",
    "estende_plataforma",
    "retrai_plataforma",
    "media_garra",
    "mostra_caçambas",
    "sacode_plataforma",
    "mostra_cubos",
])

rsp = Enum("rsp", [
    "fechei",
    "abri",
    "levantei_garra",
    "abaixei_garra",
    "cor_sensor_caçamba",
    "hsv_sensor_caçamba",
    "dist_sensor_cubo",
    "cor_sensor_cubo",
    "dist_sensor_caçamba", #! reordenar
    "levantei_dist_sensor", #! reordenar
    "mostrei_cor",
    "estendi",
    "retrai",
    "mediei_garra",
    "mostrei_caçambas",
    "sacodi",
    "mostrei_cubos",
])

#! mudar alguns nomes depois

def enviar_mensagem(*msg, enum):
    if not SILENCIOSO: print(f"enviar_mensagem: {enum(msg[0])}{msg[1:]}")
    globais.ble.broadcast(tuple(msg))

def enviar_comando(*comando):
    enviar_mensagem(*comando, enum=cmd)
def enviar_resposta(*resposta):
    enviar_mensagem(*resposta, enum=rsp)

def esperar_resposta(esperado, canal=TX_BRACO, delay=0):
    resposta = -1
    if not SILENCIOSO: print(f"esperar_resposta: {rsp(esperado)}")
    while resposta != esperado:
        try:
            resposta = globais.ble.observe(canal) or (None,)
            if not SILENCIOSO: print(f"esperar_resposta: recebido({canal}) {rsp(resposta[0])}{resposta[1:]}")
            if resposta is not None:
                resposta, *args = resposta
        except RuntimeError: continue
        except UnicodeError: continue
    if delay: wait(delay)
    if len(args) == 1: return args[0]
    return args

def resetar_garra():
    levantar_garra()
    fechar_garra()
    abrir_garra()

def fechar_garra():
    enviar_comando(cmd.fecha_garra)
    return esperar_resposta(rsp.fechei, canal=TX_RABO)

def abrir_garra():
    enviar_comando(cmd.abre_garra)
    return esperar_resposta(rsp.abri, canal=TX_RABO, delay=200)

def levantar_garra():
    enviar_comando(cmd.levanta_garra)
    return esperar_resposta(rsp.levantei_garra)

def levantar_garra_dist_sensor():
    ASSERT(False, "levanta_garra_dist_sensor não existe")
    return None

def mediar_garra():
    enviar_comando(cmd.media_garra)
    return esperar_resposta(rsp.mediei_garra)

def abaixar_garra():
    enviar_comando(cmd.abaixa_garra)
    return esperar_resposta(rsp.abaixei_garra)

def ver_cor_cubo():
    enviar_comando(cmd.ver_cor_sensor_cubo)
    return Cor(cor=esperar_resposta(rsp.cor_sensor_cubo, canal=TX_RABO))

def ver_hsv_cubo():
    return ASSERT(False, "o sensor do rabo não consegue ler hsv")

def ver_cor_caçamba():
    enviar_comando(cmd.ver_cor_sensor_caçamba)
    return Cor(cor=esperar_resposta(rsp.cor_sensor_caçamba, canal=TX_RABO))

def ver_dist_cubo():
    enviar_comando(cmd.ver_dist_sensor_cubo)
    return esperar_resposta(rsp.dist_sensor_cubo, canal=TX_RABO)

def ver_dist_caçamba():
    enviar_comando(cmd.ver_dist_sensor_caçamba)
    return esperar_resposta(rsp.dist_sensor_caçamba, canal=TX_RABO)

def mostrar_cor(cor):
    enviar_comando(cmd.mostrar_cor, cor.cor)
    return esperar_resposta(rsp.mostrei_cor)

def estender_plataforma():
    enviar_comando(cmd.estende_plataforma)
    return esperar_resposta(rsp.estendi)

def retrair_plataforma():
    enviar_comando(cmd.retrai_plataforma)
    return esperar_resposta(rsp.retrai)

def mostrar_caçambas(cores):
    enviar_comando(cmd.mostra_caçambas, *[cor.cor for cor in cores])
    return esperar_resposta(rsp.mostrei_caçambas, canal=TX_RABO)

def mostrar_cubos(cubos):
    enviar_comando(cmd.mostra_cubos, *[cor.cor for cor in cubos])
    return esperar_resposta(rsp.mostrei_cubos, canal=TX_RABO)    

def sacodir_plataforma(num_sacudidas=3):
    enviar_comando(cmd.sacode_plataforma, num_sacudidas)
    return esperar_resposta(rsp.sacodi)
