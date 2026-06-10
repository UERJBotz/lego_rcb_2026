"""
testar coisas aqui sem mudar o resto do código
"""
#import bluetooth as blt
#import cabeca as _
from cabeca import *

def imprimir_cor_caçamba_para_sempre():
    while True:
        cor = blt.ver_cor_caçamba()
        print(cor)

def ver_cores_caçambas_até_vermelho():
    def dar_meia_volta_linha():
        dar_meia_volta()
        dir_linha.mul = -dir_linha.mul

        LOG(f"dar_meia_volta_linha*: {orientação_estimada=}")
    # jeito diferente de entrar na caçambagem
    achar_não_verde_alinhado()
    curva_linha_esquerda(DIST_EIXO_SENSOR/5)

    DIST_ALINHO = TAM_BLOCO//2
    andar_dist_linha(DIST_ALINHO, vel=20)

    dist = 20
    while True:
        tam_total = TAM_BLOCO*10
        num_movs = (tam_total-TAM_BLOCO)//dist
        for _ in range(num_movs):
            bipes.cabeca()
            cor  = blt.ver_cor_caçamba()
            dist = blt.ver_dist_caçamba()
            LOG(f"cor caçamba {cor} a {dist/10}cm")

            if not andar_dist_linha(20, vel=80): break
        dar_ré(DIST_EIXO_SENSOR_TRAS)
        dar_meia_volta_linha()
        seguir_linha_até(até_dist_max(tam_total),
                         vel=100)
        dar_ré(DIST_EIXO_SENSOR_TRAS)
        dar_meia_volta_linha()

def imprimir_dist_caçamba_pra_sempre():
    while True:
        dist = blt.ver_dist_caçamba()
        print(f"{dist=}")

def imprimir_cor_cubo_para_sempre():
    blt.SILENCIOSO = True
    while True:
        hsv = None #blt.ver_hsv_cubo()
        cor = blt.ver_cor_cubo()
        print(f"hsv: {hsv}, cor: {cor}")

def imprimir_caçamba_para_sempre():
    blt.SILENCIOSO = True
    while True:
        dist = blt.ver_dist_caçamba()
        cor  = blt.ver_cor_caçamba()
        print(f"dist: {dist}, cor: {cor}")

def test():
    global orientação_estimada, pos_estimada, na_grade, cores_caçambas

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

    if False: imprimir_cor_cubo_para_sempre()
    if False: imprimir_cor_caçamba_para_sempre()
    if False: ver_cores_caçambas_até_vermelho()

    while False:
        if False: vel = 70 # mudar pra testar
        else:
            vel, *_ = rodas.settings()

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


    if False: tira_obstaculo((0,2))
    if False: tira_obstaculo((0,4))
    if False: tira_obstaculo((1,5))
    if False: tira_obstaculo((2,4))
    if False: tira_obstaculo((3,3))
    if False: tira_obstaculo((4,4))

    if False:
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
        main()

    LOG("fim do teste")
