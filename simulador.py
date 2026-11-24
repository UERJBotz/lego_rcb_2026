import pygame
from lib.grafo import *

CUBOS = {(1, 5): "AZUL", (4, 6): "VERMELHO", (6, 8): "VERDE"} #adicionar cubos na pista aqui
FRAMES_POR_CELULA = 8      
TAM = 48 #tamanho de cada célula na tela

pos = (0, 0)               
passos = 0              
entregues = []             
na_garra = None            
angulo = 0                 
caminho_atual = []

ORIENTACOES = {
    0: "N",
    90: "L",
    180: "S",
    270: "O"
}

X_MIN, X_MAX, Y_MIN, Y_MAX = -3.5, 15.5, -1.3, 9.3      
FINO = 0.14 #espessura das linhas
BRANCO, PRETO, AZUL = (245, 245, 245), (25, 25, 25), (30, 90, 220)
VERDE, AMARELO, VERMELHO = (0, 140, 40), (250, 220, 0), (225, 25, 25)
COR = {"AZUL": AZUL, "VERMELHO": VERMELHO, "VERDE": VERDE, "AMARELO": AMARELO, "PRETO": (60, 60, 60)}

pygame.init()
tela = pygame.display.set_mode((int((X_MAX - X_MIN) * TAM), int((Y_MAX - Y_MIN) * TAM) + 36))
fonte = pygame.font.SysFont("monospace", 16)
relogio = pygame.time.Clock()

def P(x, y):
    return ((x - X_MIN) * TAM, (y - Y_MIN) * TAM)

def ret(superficie, x0, y0, x1, y1, cor):
    ax, ay = P(x0, y0); bx, by = P(x1, y1)
    pygame.draw.rect(superficie, cor, (ax, ay, max(1, bx - ax), max(1, by - ay)))

def desenhar_fundo():
    s = pygame.Surface(tela.get_size()); s.fill((200, 200, 200))
    ret(s, -3.33, -1, 15.33, 9, BRANCO)
    ret(s, -3.33, -1, -1.8, 9, AMARELO); ret(s, -1.8, -1, 0, 9, VERDE)          
    ret(s, 12, -1, 13.8, 9, VERDE); ret(s, 13.8, -1, 15.33, 9, AMARELO)    
    for linha in range(0, 9, 2):                                                
        ret(s, 0, linha-FINO/2, 12, linha+FINO/2, PRETO)
    for coluna in range(1, 12, 2):                                                    
        ret(s, coluna-FINO / 2, 0, coluna+FINO/2, 8, PRETO)
    for x in (0, 12):                                                           
        ret(s, x-FINO / 2, -1, x+FINO / 2, 9, AZUL)
    pygame.draw.rect(s, (30, 30, 30), (0, tela.get_height() - 36, tela.get_width(), 36))   
    return s

fundo = desenhar_fundo()

def desenhar(lin=None, col=None):
    if lin is None: lin, col = pos
    pygame.event.pump()
    tela.blit(fundo, (0, 0))
    for l, linha in enumerate(mapa):                    
        for c, cel in enumerate(linha):
            if cel.estado == estado_celula.INCERTO:
                pygame.draw.circle(tela, (255, 150, 0), P(c, l), TAM // 8)
    if len(caminho_atual) > 1:                              
        pygame.draw.lines(tela, (0, 190, 255), False, [P(c, l) for l, c in caminho_atual], 4)
    lado = TAM // 3
    for (l, c), cor in CUBOS.items():
        x, y = P(c, l)
        pygame.draw.rect(tela, COR[cor], (x - lado / 2, y - lado / 2, lado, lado))
        pygame.draw.rect(tela, (0, 0, 0), (x - lado / 2, y - lado / 2, lado, lado), 2)
    x, y = P(col, lin)
    pygame.draw.circle(tela, (255, 160, 0), (x, y), TAM // 4)
    pygame.draw.circle(tela, (0, 0, 0), (x, y), TAM // 4, 2)
    dx, dy = {0: (1, 0), 90: (0, 1), 180: (-1, 0), -90: (0, -1)}[angulo]
    pygame.draw.line(tela, (0, 0, 0), (x, y), (x + dx * TAM // 3, y + dy * TAM // 3), 3)
    if na_garra:
        pygame.draw.rect(tela, COR[na_garra], (x - 7, y - 7, 14, 14)); pygame.draw.rect(tela, (0, 0, 0), (x - 7, y - 7, 14, 14), 2)
    texto = f"passos: {passos}   entregues: {len(entregues)} {entregues}   cubos na pista: {len(CUBOS)}   (laranja = não olhou)"
    tela.blit(fonte.render(texto, True, (240, 240, 240)), (8, tela.get_height() - 26))
    pygame.display.flip()
    relogio.tick(60)

DIRECOES = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): -90}

def andar(caminho):
    global pos, passos, angulo, caminho_atual
    caminho_atual = caminho
    for (l0, c0), (l1, c1) in zip(caminho, caminho[1:]):
        print("Caminho recebido:", caminho)
        print("Passo atual:", (l0, c0), "->", (l1, c1))
        angulo = DIRECOES[(l1 - l0, c1 - c0)]
        for f in range(1, FRAMES_POR_CELULA + 1):
            k = f / FRAMES_POR_CELULA
            desenhar(l0 + (l1 - l0) * k, c0 + (c1 - c0) * k)
        pos = (l1, c1)
        passos += 1

def olhar(destino):
    global angulo
    angulo = DIRECOES[(destino[0] - pos[0], destino[1] - pos[1])]
    for i in range(15): desenhar()

#def procura():
#    cel_atual = pos
#    incertas = pegar_celulas_incertas()
#    caminhos_incertas = {
#        cel: achar_caminhos(cel_atual, cel, ORIENTACOES[angulo%360])
#        for cel in incertas
#    }
#    incertas.sort(key=lambda cel: (
#        len(caminhos_incertas[cel])
#        if caminhos_incertas[cel] is not None else 9000
#    ))
#
#    for destino in incertas:
#        caminho = achar_caminhos(cel_atual, destino, ORIENTACOES[angulo%360])
#        if not caminho:
#            continue
#
#        caminho.pop(-1)
#        andar(caminho)
#        cel_atual = pos
#
#        olhar(destino)
#        if destino not in CUBOS:
#            tira_obstaculo(destino)
#            andar([cel_atual, destino])
#            cel_atual = destino
#            continue
#
#        cor = CUBOS[destino]
#        tira_obstaculo(destino)
#        return destino
    return None

def procura():
    cel_atual = pos
    incertas = pegar_celulas_incertas_alcancaveis(pos)

    while incertas:
        candidatas = sorted(incertas, key=lambda c: heuristica(cel_atual, c))[:3]
        caminhos = {
            cel: achar_caminhos(cel_atual, cel, ORIENTACOES[angulo%360]) for cel in candidatas
        }
        candidatas.sort(key=lambda x: (
            len(caminhos[x]) if caminhos[x] is not None else 9000
        ))

        destino = candidatas[0]
        incertas.remove(destino)
        caminho = caminhos[destino]

        if not caminho:
            continue

        caminho.pop(-1)
        andar(caminho)
        cel_atual = pos

        olhar(destino)
        if destino not in CUBOS:
            tira_obstaculo(destino)
            andar([cel_atual, destino])
            cel_atual = pos
            continue

        cor = CUBOS[destino]
        tira_obstaculo(destino)
        return destino

    return None

while CUBOS:
    destino = procura()
    if destino is None: break               
    na_garra = CUBOS.pop(destino)              
    andar(achar_caminhos(pos, (0, 0), ORIENTACOES[angulo%360]))              
    entregues.append(na_garra); na_garra = None

print(f"terminou: {passos} passos, entregues: {entregues}")
while True: desenhar()                      
