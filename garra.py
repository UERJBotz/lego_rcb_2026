from pybricks.parameters import Port, Stop
from pybricks.tools      import wait

from comum import globais


grau_mm = 6000/630 # 9.52º/mm -> 95.2º/cm
DIST_DESCER_GARRA = 150#mm
ANG_DESCER_GARRA = DIST_DESCER_GARRA*grau_mm
VEL_DESCER_GARRA = 800

ALTURA_PLATAFORMA     = 100#mm
ANG_ALTURA_PLATAFORMA = ALTURA_PLATAFORMA*grau_mm

RAIO_ENGRENAGEM = 7#mm
DIST_PLATAFORMA = 60#mm
ANG_PLATAFORMA  = DIST_PLATAFORMA*RAIO_ENGRENAGEM


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
