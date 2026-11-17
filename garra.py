from pybricks.parameters import Port, Stop
from pybricks.tools      import wait

from comum import globais

SENSOR_EV3 = True

RAIO_ENGRENAGEM  = 6#mm

DIST_SUBIR_GARRA = 100#mm
ANG_PLATAFORMA   = 180
#ANG_SUBIR_GARRA = DIST_SUBIR_GARRA*RAIO_ENGRENAGEM

DIST_AJUSTE_SENSOR = -18 if SENSOR_EV3 else 0#mm
ANG_AJUSTE_SENSOR = DIST_AJUSTE_SENSOR*RAIO_ENGRENAGEM

cm_grau = 63/6000 #.0105 cm/º
grau_cm = 6000/63 # 95.2 º/cm
                            
ANG_SUBIR_GARRA = 20*grau_cm
VEL_SUBIR_GARRA = 800   
                            
def levanta_garra(vel=VEL_SUBIR_GARRA):
    globais.motor_guindaste.run_angle(-vel, ANG_SUBIR_GARRA, then=Stop.COAST, wait=True)
 
def abaixa_garra(vel=VEL_SUBIR_GARRA):
    globais.motor_guindaste.run_angle(vel, ANG_SUBIR_GARRA, then=Stop.COAST, wait=True)

def fecha_garra(vel=240):
    globais.motor_garra.run_until_stalled(vel, then=Stop.COAST, duty_limit=None)
    ang = globais.motor_garra.angle()
    globais.motor_garra.run_angle(vel, 30, then=Stop.COAST, wait=False)
    return ang

def abre_garra(vel=240, ang_volta=60): #ang_volta=72
    globais.motor_garra.run_angle(vel, -ang_volta, then=Stop.COAST, wait=True)

def estende_plataforma(vel=240):
    globais.motor_plataforma.run_angle(vel, ANG_PLATAFORMA, then=Stop.COAST, wait=True)

def retrai_plataforma(vel=240):
    globais.motor_plataforma.run_angle(-vel, ANG_PLATAFORMA, then=Stop.COAST, wait=True)

def levanta_garra_dist_sensor(vel=240):
    globais.motor_guindaste.run_angle(-vel, ANG_AJUSTE_SENSOR, then=Stop.COAST, wait=True)
