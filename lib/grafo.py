from lib.caminhos import *

# construindo grafo
PESO_GIRO = 1
PESO_RETA_BLOCO = 1

class Noh:
    NORTE = "N"
    SUL = "S"
    LESTE = "L"
    OESTE = "O"

    DIR = {
        NORTE: (-1, 0),
        SUL:   (1, 0),
        LESTE: (0, 1),
        OESTE: (0, -1)
    }

class Node:
    def __init__(self, key):
        self.key = key
        self.adj = []

    def connect(self, node):
        self.adj.append(node)

class Nó(Node):
    def __init__(self, celula, posicao, direcao):
        super().__init__((posicao, direcao))
        self.celula  = celula
        self.direcao = direcao
        self.posicao = posicao
        self.vizinhos = []
        # Cruz -> 4 nós
        # Vert -> 2
        # Hori -> 2

class Aresta:
    def __init__(self, ini, fim, peso):
        self.ini = ini
        self.fim = fim
        self.peso = peso

class Graph:
    def __init__(self):
        self.nós = dict()
        self.arestas = dict()

    def add(self, node):
        self.nós[node.key] = node #! sobrescreve

    def get(self, key):
        return self.nós.get(key)

    def connect(self, ini, fim, peso=1):
        self.nós[ini].connect(self.nós[fim])
        self.arestas[ini, fim] = Aresta(ini, fim, peso)

    def peso(self, ini, fim):
        return self.arestas[ini, fim].peso

class Grafo(Graph):
    def get_by_pos(self, pos):
        return set(map(lambda x: x.key,
                   filter(lambda x: x is not None,
                         (self.get((pos, d)) for d in ('N', 'S', 'L', 'O')))))

class FronteiraPrioridade():
    def __init__(self):
        self.frontier = []
        self.contador = 0

    def add(self, nó, custo):
        heappush(self.frontier, (custo, self.contador, nó))
        self.contador += 1

    def remove(self):
        if self.empty():
            return None
        return heappop(self.frontier)[2]

    def empty(self):
        return len(self.frontier) == 0

class FronteiraAestrela(FronteiraPrioridade):
    def __init__(self, dest):
        super().__init__()
        self.dest = dest

    def add(self, nó, custo):
        peso = custo + heuristica(nó.posicao, self.dest)
        super().add(nó, peso)

class FronteiraFila(FronteiraPrioridade):
    def add(self, nó, custo):
        self.frontier.append(nó)

    def remove(self):
        if self.empty(): return None
        node = self.frontier[0]
        self.frontier = self.frontier[1:]
        return node

class FronteiraPilha(FronteiraFila):
    def remove(self):
        if self.empty(): return None
        return self.frontier.pop()

def nó_a_direção(mapa, pos, direcao):
    i, j = pos
    di, dj = Noh.DIR[direcao]
    ni, nj = i + di, j + dj

    if not ( (0 <= ni < len(mapa)) and 
             (0 <= nj < len(mapa[0]))
           ): return None
    else: vizinha = mapa[ni][nj]

    if direcao in (Noh.NORTE, Noh.SUL):
        if isinstance(vizinha, (Vert, Cruz)): return (ni, nj)
    else:
        if isinstance(vizinha, (Hori, Cruz)): return (ni, nj)
    return None

def construir_grafo(mapa):
    grafo = Grafo()
    arestas_sobrando = []
    for i in range(len(mapa)):
        for j in range(len(mapa[0])):
            celula = mapa[i][j]
            if   isinstance(celula, Cruz): direcoes = [Noh.LESTE, Noh.NORTE, Noh.OESTE, Noh.SUL]
            elif isinstance(celula, Vert): direcoes = [Noh.NORTE, Noh.SUL]
            elif isinstance(celula, Hori): direcoes = [Noh.LESTE, Noh.OESTE]
            else:                          direcoes = []

            subnós = [Nó(celula, (i, j), d) for d in direcoes]
            for nó in subnós: grafo.add(nó)

            if isinstance(celula, Cruz):
                for n1, n2 in zip(subnós, subnós[1:] + subnós[:1]):
                    grafo.connect(n1.key, n2.key, peso=PESO_GIRO)
                    grafo.connect(n2.key, n1.key, peso=PESO_GIRO)

            for nó in subnós:
                if prox := nó_a_direção(mapa, nó.posicao, nó.direcao):
                    arestas_sobrando.append((nó.key, (prox, nó.direcao)))

    for n1, n2 in arestas_sobrando:
        grafo.connect(n1, n2, peso=PESO_RETA_BLOCO)

    return grafo

#def custo_transicao(atual, vizinho): #! ver se vai precisar
#    datual = Noh.DIR[atual.direcao]
#    i1, j1 = Noh.DIR[vizinho.direcao]
#
#    if atual.posicao != vizinho.posicao:
#        if datual == ( i1,  j1): return 0
#        if datual == (-i1, -j1): return 2*PESO_GIRO # 180º
#        
#        return PESO_GIRO # 90º
#    else:
#        return PESO_RETA_BLOCO

def escolher_caminho(grid, grafo, src, dest, orientacao, estrategia='dijkstra'):
    ok = True

    if not dentro_dos_limites(grid, src):
        ok = False; print(f"{estrategia}: origem inválida {src}")
    if not dentro_dos_limites(grid, dest):
        ok = False; print(f"{estrategia}: destino inválido {dest}")
    if not celula_livre(grid, src):
        ok = False; print(f"{estrategia}: origem bloqueada {src}")
    if celula_ocupada(grid, dest):
        ok = False; print(f"{estrategia}: destino bloqueado {dest}")

    if not ok: return None

    # Check if we are already at the destination
    if eh_destino(src, dest):
        print(f"{estrategia}: já no destino {src, dest}")
        return []

    nó_destino_keys = grafo.get_by_pos(dest)
    
    if not nó_destino_keys:
        print(f"{estrategia}: destino sem nós disponíveis")
        return None

    if   estrategia == 'bfs':      fronteira = FronteiraFila()
    elif estrategia == 'dijkstra': fronteira = FronteiraPrioridade() 
    elif estrategia == 'dfs':      fronteira = FronteiraPilha()
    else:                          fronteira = FronteiraAestrela(dest)
    
    pais = dict()
    nó_inicial = grafo.get((src, orientacao))
    if nó_inicial is None:
        print(f"{estrategia}: orientação inicial sem nó em {src}: {orientacao}")
        return None

    passos_dados = {nó_inicial: 0}
    concluidos = set()

    fronteira.add(nó_inicial, 0)
    while not fronteira.empty():
        nó_atual = fronteira.remove()
        if nó_atual in concluidos: continue

        concluidos.add(nó_atual)

        if nó_atual.key in nó_destino_keys: return tracar_caminho(pais, nó_atual, src)

        for vizinho in nó_atual.adj:
            if vizinho in concluidos: continue

            if    celula_ocupada(grid, vizinho.posicao): continue
            if (not celula_livre(grid, vizinho.posicao) and
                not   eh_destino(vizinho.posicao, dest)): continue

            novo_custo = passos_dados[nó_atual] + grafo.peso(nó_atual.key, vizinho.key)

            if novo_custo < passos_dados.get(vizinho, float('inf')):
                passos_dados[vizinho] = novo_custo
                pais[vizinho] = nó_atual
                fronteira.add(vizinho, novo_custo)
    return None
   
def tracar_caminho(pai, atual, src):
    caminho = []

    while atual in pai:
        caminho.append(atual.posicao)
        atual = pai[atual]

    caminho.append(src)
    caminho.reverse()

    caminho_sem_repetir = []

    for pos in caminho:
        if not caminho_sem_repetir or caminho_sem_repetir[-1] != pos:
            caminho_sem_repetir.append(pos)

    return caminho_sem_repetir

def achar_caminhos(src, dest, orientacao):
    return escolher_caminho(mapa, grafo, src, dest, orientacao)

def celulas_alcancaveis(src):
    visitados = []
    pos_alcançáveis = {src}

    fronteira = FronteiraFila()
    for key in grafo.get_by_pos(src):
        fronteira.add(grafo.get(key), 0)

    while not fronteira.empty():
        atual = fronteira.remove()

        if atual in visitados: continue

        visitados.append(atual)
        pos_alcançáveis.add(atual.posicao)

        for vizinho in atual.adj:
            if vizinho not in visitados: 
                fronteira.add(vizinho, 0)

    return pos_alcançáveis

def pegar_celulas_incertas_alcancaveis(src):
    cel_incertas = pegar_celulas_incertas()

    alcancaveis = celulas_alcancaveis(src)

    cel_incertas = [cel for cel in cel_incertas
                    if cel in alcancaveis]
    return cel_incertas

grafo = construir_grafo(mapa)

