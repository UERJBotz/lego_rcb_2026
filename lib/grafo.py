from lib.caminhos import *
# construindo grafo

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

class No:
    def __init__(self, celula, direcao, posicao):
        self.celula = celula
        self.direcao = direcao
        self.posicao = posicao
        self.vizinhos = []
        # Cruz -> 4 nós
        # Vert -> 2
        # Hori -> 2

class FronteiraPrioridade():
    def __init__(self):
        self.frontier = []
        self.contador = 0

    def add(self, noh, custo):
        heappush(self.frontier, (custo, self.contador, noh))
        self.contador += 1

    def contains_noh(self, noh):
        return any(n == noh for _,_,n in self.frontier)

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

    def add(self, noh, custo):
        peso = custo + heuristica(noh.posicao, self.dest)
        super().add(noh, peso)

class FronteiraFila(FronteiraPrioridade):
    def add(self, noh, custo):
        self.frontier.append(noh)
    
    def contains_noh(self, noh):
        return any(n == noh for n in self.frontier)

    def remove(self):
        if self.empty(): return None
        node = self.frontier[0]
        self.frontier = self.frontier[1:]
        return node

class FronteiraPilha(FronteiraFila):
    def remove(self):
        if self.empty(): return None
        return self.frontier.pop()

oposta = {Noh.NORTE: Noh.SUL,
          Noh.SUL:   Noh.NORTE,
          Noh.LESTE: Noh.OESTE,
          Noh.OESTE: Noh.LESTE}


def pode_conectar(mapa, pos, direcao):
    i, j = pos
    di, dj = Noh.DIR[direcao]
    ni, nj = i + di, j + dj

    if not (0 <= ni < len(mapa) and 0 <= nj < len(mapa[0])): return None

    vizinha = mapa[ni][nj]

    if direcao in (Noh.NORTE, Noh.SUL):
        if isinstance(vizinha, (Vert, Cruz)): return (ni, nj)
    elif isinstance(vizinha, (Hori, Cruz)):   return (ni, nj)
    return None

#def construir_nohs(mapa):
#    mapa_de_nohs = { (1, Noh.NORTE): [] }
#    k = 0
#    for i in range(len(mapa))
#        for j in range(len(mapa[0])):
#            celula = mapa[i][j]
#            if isinstance(celula, Cruz):
#                mapa_de_nohs.append({(k+0, Noh.NORTE): [(k+1, Noh.LESTE), (k+3, Noh.OESTE)],
#                                     (k+2, Noh.SUL):   [(k+1, Noh.LESTE), (k+3, Noh.OESTE)],
#                                     (k+1, Noh.LESTE): [(k+0, Noh.NORTE), (k+2, Noh.SUL)],
#                                     (k+3, Noh.OESTE): [(k+0, Noh.NORTE), (k+2, Noh.SUL)]})
#                k += 4
#            elif isintance(celula, Vert):
#                mapa_de_nohs.append({(k+0): [],
#                                     (k+1): []})
#                k += 2
#            elif isintance(celula, Hori):
#                mapa_de_nohs.append([Noh.NORTE,
#                                     Noh.SUL])
#
#    return mapa_de_nohs

#def construir_nohs(mapa):
#    mapa_em_nohs = []
#    k = 0
#    for i in range(len(mapa)):
#        linha_nohs = []
#        for j in range(len(mapa[0])):
#            celula = mapa[i][j]
#            nohs = {}
#            if isinstance(celula, Cruz):
#                if pode_conectar(mapa, (i, j), Noh.NORTE): nohs[(k+0, Noh.NORTE)] = []
#                if pode_conectar(mapa, (i, j), Noh.LESTE): nohs[(k+1, Noh.LESTE)] = []
#                if pode_conectar(mapa, (i, j), Noh.SUL):   nohs[(k+2, Noh.SUL)] = []
#                if pode_conectar(mapa, (i, j), Noh.OESTE): nohs[(k+3, Noh.OESTE)] = []
#                k+=4
#            elif isinstance(celula, Vert):
#                if pode_conectar(mapa, (i, j), Noh.NORTE): nohs[(k+0, Noh.NORTE)] = []
#                if pode_conectar(mapa, (i, j), Noh.SUL):   nohs[(k+1, Noh.SUL)] = []
#                k+=2
#            elif isinstance(celula, Hori):
#                if pode_conectar(mapa, (i, j), Noh.LESTE): nohs[(k+0, Noh.LESTE)] = []
#                if pode_conectar(mapa, (i, j), Noh.OESTE): nohs[(k+1, Noh.OESTE)] = []
#                k+=2
#
#            linha_nohs.append(nohs)
#        mapa_em_nohs.append(linha_nohs)
#    return mapa_em_nohs

def construir_nohs(mapa):
    mapa_em_nohs = []
    for i in range(len(mapa)):
        linha_nohs = []
        for j in range(len(mapa[0])):
            celula = mapa[i][j]
            nohs = {}
            direcoes = []
            if isinstance(celula, Cruz):   direcoes = [Noh.NORTE, Noh.SUL, Noh.LESTE, Noh.OESTE]
            elif isinstance(celula, Vert): direcoes = [Noh.NORTE, Noh.SUL]
            elif isinstance(celula, Hori): direcoes = [Noh.LESTE, Noh.OESTE]

            for direcao in direcoes:
                if (i, j) == (0, 0) and direcao == Noh.NORTE: nohs[direcao] = No(celula, direcao, (i, j))
                elif  pode_conectar(mapa, (i, j), direcao):   nohs[direcao] = No(celula, direcao, (i, j))

            if isinstance(celula, Cruz):
                for noh in nohs.values():
                    for outro in nohs.values():
                        if outro != noh: noh.vizinhos.append(outro)
            elif len(nohs) == 2:
                a, b = nohs.values()
                a.vizinhos.append(b)
                b.vizinhos.append(a)

            linha_nohs.append(nohs)
        mapa_em_nohs.append(linha_nohs)
    return mapa_em_nohs

#def conectar_celulas(mapa, mapa_em_nohs):
#    oposta = {
#        Noh.NORTE: Noh.SUL,
#        Noh.SUL:   Noh.NORTE,
#        Noh.LESTE: Noh.OESTE,
#        Noh.OESTE: Noh.LESTE
#    }
#
#    for i in range(len(mapa_em_nohs)):
#        for j in range(len(mapa_em_nohs[0])):
#            for noh in mapa_em_nohs[i][j]:
#                id, direcao = noh
#
#                pos = pode_conectar(mapa, (i, j), direcao)
#                if pos is None: continue
#                ni, nj = pos
#
#                for vizinho in mapa_em_nohs[ni][nj]:
#                    if vizinho[1] == oposta[direcao]:
#                        mapa_em_nohs[i][j][noh].append(vizinho)
#                        break
def conectar_celulas(mapa, mapa_em_nohs):
    for i in range(len(mapa)):
        for j in range(len(mapa[0])):
            for direcao, noh in mapa_em_nohs[i][j].items():

                pos = pode_conectar(mapa, (i, j), direcao)
                if pos is None: continue

                ni, nj = pos
                vizinho = mapa_em_nohs[ni][nj].get(oposta[direcao])
                if vizinho is not None and vizinho not in noh.vizinhos: noh.vizinhos.append(vizinho)

def construir_grafo(mapa):
    grafo = construir_nohs(mapa)
    conectar_celulas(mapa, grafo)
    return grafo

def custo_virada(ori_atual, ori_final):
    atual = Noh.DIR[ori_atual]
    final = Noh.DIR[ori_final]

    if atual == final: return 0
    if atual == (-final[0], -final[1]): return 2

    #perpendicular
    return 1

def custo_transicao(atual, vizinho):
    if atual.posicao != vizinho.posicao:
        return 1

    orientacao_atual = oposta[atual.direcao]
    orientacao_final = vizinho.direcao

    return custo_virada(orientacao_atual, orientacao_final)

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

    if not ok:
        return None

    # Check if we are already at the destination
    if eh_destino(src, dest):
        print(f"{estrategia}: já no destino {src, dest}")
        return []

    nohs_iniciais = set(grafo[src[0]][src[1]].values())
    nohs_destino  = set(grafo[dest[0]][dest[1]].values()) 
    
    if not nohs_iniciais or not nohs_destino:
        print(f"{estrategia}: origem ou destino sem nós disponíveis")
        return None

    if   estrategia == 'bfs'  : fronteira = FronteiraFila()
    elif estrategia == 'dijkstra': fronteira = FronteiraPrioridade() 
    elif estrategia == 'dfs': fronteira = FronteiraPilha()
    else: fronteira = FronteiraAestrela(dest)
    
    pai = {}
    passos_dados = {}
    concluidos = set()

    for noh in nohs_iniciais:
        custo = custo_virada(orientacao, noh.direcao)

        passos_dados[noh] = custo
        fronteira.add(noh, custo)

    while not fronteira.empty():
        noh_atual = fronteira.remove()
        if noh_atual in concluidos: continue

        concluidos.add(noh_atual)

        if noh_atual in nohs_destino: return tracar_caminho(pai, noh_atual, src)

        for vizinho in noh_atual.vizinhos:
            if vizinho in concluidos:
                continue

            pos_vizinho = pos_vizinho = vizinho.posicao

            if celula_ocupada(grid, pos_vizinho): continue
            if not (celula_livre(grid, pos_vizinho) or eh_destino(pos_vizinho, dest)): continue

            novo_custo = passos_dados[noh_atual] + custo_transicao(noh_atual, vizinho)

            if novo_custo < passos_dados.get(vizinho, float('inf')):
                passos_dados[vizinho] = novo_custo
                pai[vizinho] = noh_atual
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

grafo = construir_grafo(mapa)

