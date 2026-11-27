import pygame
from lib.caminhos import (
    achar_caminhos,
    estado_celula,
    mapa,
    pegar_celulas_incertas_alcancaveis,
    tira_obstaculo,
    coloca_obstaculo,
    visitar,
)

CUBOS = {(3, 1): "VERDE"}  # cubos na pista: (linha, coluna) -> cor
OBSTACULOS = set()  # coordenadas das ruas bloqueadas
CORES_ACEITAS = None  # None aceita todas; configure com as cores das caçambas reais
CAPACIDADE_POR_COR = 2
quantidades_entregues = {}
FRAMES_POR_CELULA = 8      
TAM = 48 #tamanho de cada célula na tela

pos = (0, 0)               
passos = 0              
entregues = []             
na_garra = None            
angulo = 0
caminho_atual = []
visitar(pos)

ORIENTACOES = {
    0: "L",
    90: "S",
    180: "O",
    270: "N"
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

def processar_eventos():
    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            pygame.quit()
            raise SystemExit
        if evento.type == pygame.KEYDOWN and evento.key == pygame.K_ESCAPE:
            pygame.quit()
            raise SystemExit

def desenhar(lin=None, col=None):
    if lin is None: lin, col = pos
    processar_eventos()
    tela.blit(fundo, (0, 0))
    for l, linha in enumerate(mapa):                    
        for c, cel in enumerate(linha):
            if cel.estado == estado_celula.INCERTO:
                pygame.draw.circle(tela, (255, 150, 0), P(c, l), TAM // 8)
            elif cel.mutável and cel.estado == estado_celula.OCUPADA:
                pygame.draw.circle(tela, (220, 30, 30), P(c, l), TAM // 8)
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

def andar(caminho, registrar_visitadas=True):
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
        if registrar_visitadas:
            visitar(pos)
        passos += 1

def olhar(destino):
    global angulo
    angulo = DIRECOES[(destino[0] - pos[0], destino[1] - pos[1])]
    for i in range(15): desenhar()

def atualizar_estado_celula(destino, ocupada):
    if ocupada: coloca_obstaculo(destino)
    else:       tira_obstaculo(destino)

def verificar_aresta(aresta):
    ini, fim = aresta
    meio = ((ini[0] + fim[0]) // 2, (ini[1] + fim[1]) // 2)
    cor = CUBOS.get(meio)
    if meio in OBSTACULOS or mapa[meio[0]][meio[1]].estado == estado_celula.OCUPADA:
        return False, cor
    if cor is not None:
        if CORES_ACEITAS is not None and cor not in CORES_ACEITAS:
            return False, cor
        if quantidades_entregues.get(cor, 0) >= CAPACIDADE_POR_COR:
            return False, cor
    return True, cor

def atravessa_rua_pegando_cubo_ou_nao(aresta):
    ini, fim = aresta
    meio = ((ini[0] + fim[0]) // 2, (ini[1] + fim[1]) // 2)
    passa, cor = verificar_aresta(aresta)
    atualizar_estado_celula(meio, not passa)
    if passa:
        andar([ini, meio, fim], registrar_visitadas=False)
        visitar(fim)
        return cor, fim
    return cor, ini

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
#    return None

def procura():
    while True:
        incertas = pegar_celulas_incertas_alcancaveis(pos)
        caminhos = {
            cel: achar_caminhos(pos, cel, ORIENTACOES[angulo%360])
            for cel in incertas
        }
        alcancaveis = [cel for cel in incertas if caminhos[cel]]
        if not alcancaveis:
            return None
        destino = min(alcancaveis, key=lambda cel: len(caminhos[cel]))
        caminho = caminhos[destino]
        caminho.pop()
        andar(caminho)
        olhar(destino)
        dy, dx = destino[0] - pos[0], destino[1] - pos[1]
        depois = (pos[0] + 2 * dy, pos[1] + 2 * dx)
        pos_antes = pos
        cor, pos_depois = atravessa_rua_pegando_cubo_ou_nao((pos, depois))
        if cor is not None and pos_depois != pos_antes:
            return destino

while True:
    destino = procura()
    if destino is None: break               
    na_garra = CUBOS.pop(destino)              
    andar(achar_caminhos(pos, (0, 0), ORIENTACOES[angulo%360]))
    entregues.append(na_garra); na_garra = None
    quantidades_entregues[entregues[-1]] = quantidades_entregues.get(entregues[-1], 0) + 1

print(f"terminou: {passos} passos, entregues: {entregues}")
while True: desenhar()                      
