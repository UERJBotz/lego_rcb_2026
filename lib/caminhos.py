from lib.polyfill import Enum, heappop, heappush


class COR:
    VERM = '\033[91m'
    VERD = '\033[92m'
    ENDC = '\033[0m'

def color(col, texto):
    return col + texto + COR.ENDC


direcionalidade = Enum("direcionalidade", ["NENHUMA",
                                           "HORIZONTAL",
                                           "VERTICAL"])
estado_celula = Enum("estado_celula", ["LIVRE",
                                       "OCUPADA",
                                       "INCERTO"])

class Bloco:
    def __init__(self, estado=estado_celula.LIVRE,
                       direc=direcionalidade.NENHUMA, mut=False):
        self.estado  = estado
        self.direc   = direc
        self.mutável = mut

class Nada(Bloco):
    def __init__(self, estado=estado_celula.OCUPADA,
                       direc=direcionalidade.NENHUMA, mut=False):
        super().__init__(estado, direc, mut)
    def __str__(self): return ' '

class Rua(Bloco):
    def __init__(self, estado=estado_celula.INCERTO,
                       direc=direcionalidade.NENHUMA, mut=False):
        super().__init__(estado, direc, mut)

class Cruz(Rua):
    def __init__(self, estado=estado_celula.LIVRE,
                       direc=direcionalidade.NENHUMA, mut=False):
        super().__init__(estado, direc, mut)
    def __str__(self): return '#'

class Vert(Rua):
    def __init__(self, estado=estado_celula.INCERTO,
                       direc=direcionalidade.VERTICAL, mut=True):
        super().__init__(estado, direc, mut)
    def __str__(self): return '|'

class Hori(Rua):
    def __init__(self, estado=estado_celula.INCERTO,
                       direc=direcionalidade.HORIZONTAL, mut=True):
        super().__init__(estado, direc, mut)
    def __str__(self): return '—'

class Safe(Cruz):
    def __init__(self, estado=estado_celula.LIVRE,
                       direc=direcionalidade.HORIZONTAL, mut=False):
        super().__init__(estado, direc, mut)
    def __str__(self): return '*'


mapa = [
    [Safe(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
    [Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert()],
    [Safe(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
    [Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert()],
    [Safe(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
    [Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert()],
    [Safe(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
    [Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert()],
    [Safe(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
]

MAPA_X_MAX = len(mapa[0]) -1
MAPA_Y_MAX = len(mapa)    -1

def coloca_obstaculo(pos):
    y, x = pos; cel = mapa[y][x]
    if cel.mutável: #! falhar mais alto
        cel.estado = estado_celula.OCUPADA

def tira_obstaculo(pos):
    y, x = pos; cel = mapa[y][x]
    if cel.mutável: #! falhar mais alto
        cel.estado = estado_celula.LIVRE

def imprimir_mapa(matriz=mapa):
    """Imprime a matriz de forma alinhada."""
    largura_maxima = 12

    print(end='  ')
    for i in range(len(matriz[0])):
        print((f"{i}     ")[:2], end=' ')
    print()

    for i, linha in enumerate(matriz):
        print(i, end=' ')

        for celula in linha:
            texto = str(celula)
            estado = color(COR.VERM, 'X') if celula.estado == estado_celula.OCUPADA else (
                                     '?'  if celula.estado == estado_celula.INCERTO else (
                     color(COR.VERD, texto)))
            print(estado*2, end=" ")
        print()

direcoes = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Movimentos: direita, baixo, esquerda, cima

def dist_manhatan(posicao_atual, destino):
    atual_x, atual_y = posicao_atual
    dest_x,  dest_y  = destino
    return abs(atual_x - dest_x) + abs(atual_y - dest_y)

def dentro_dos_limites(matriz, cell):
    row, col = cell
    return 0 <= row < len(matriz) and 0 <= col < len(matriz[0])

def celula_ocupada(grid, cell):
    row, col = cell
    return grid[row][col].estado == estado_celula.OCUPADA

def celula_livre(grid, cell):
    row, col = cell
    return grid[row][col].estado == estado_celula.LIVRE

def celula_especialmente_bloqueada(mapa, curr, new):
    i, j = curr
    new_i, new_j = new
    new = mapa[new_i][new_j]
    if new.direc == direcionalidade.NENHUMA: return False

    horizontal = (new_i == i) 
    vertical   = (new_j == j)

    if vertical   and new.direc == direcionalidade.VERTICAL:   return False
    if horizontal and new.direc == direcionalidade.HORIZONTAL: return False

    return True

#!
def eh_destino(src, dest):
    row, col = src
    return row == dest[0] and col == dest[1]

def heuristica(src, dest):
    return dist_manhatan(src, dest)

#funcao que recebe lista de posicoes na matriz e transforma em lista de direcoes
def caminho_relativo(caminho_absoluto: list[tuple[int, int]]):
    if caminho_absoluto is None:
        caminho_absoluto = []
        print("Caminho Errado") #! LOG

    #movimentos
    direita = (0, 1); baixo = (1, 0); esquerda = (0, -1); cima = (-1, 0)
    direcoes = [(0,0)]

    # print(caminho_absoluto)
    for i in range(1, len(caminho_absoluto)):
        dx = caminho_absoluto[i][0] - caminho_absoluto[i - 1][0]
        dy = caminho_absoluto[i][1] - caminho_absoluto[i - 1][1]

        if   dx == 0 and dy == 1:
            direcoes.append(direita)
        elif dx == 1 and dy == 0:
            direcoes.append(baixo)
        elif dx == 0 and dy == -1:
            direcoes.append(esquerda)
        elif dx == -1 and dy == 0:
            direcoes.append(cima)

    return direcoes #! checar no chamador se é vazio

tipo_movimento = Enum("tipo_movimento",
                      ["FRENTE", "DIREITA_FRENTE", "ESQUERDA_FRENTE", "TRAS", "DIREITA", "ESQUERDA"])

def prox_movimento(ori_ini: tipo_movimento, ori_final: tipo_movimento): #type: ignore
        diferenca = (ori_final - ori_ini) % 4
        if   diferenca == 0: return tipo_movimento.FRENTE
        elif diferenca == 1: return tipo_movimento.DIREITA_FRENTE
        elif diferenca == 2: return tipo_movimento.TRAS
        elif diferenca == 3: return tipo_movimento.ESQUERDA_FRENTE
        else:
            assert False


posicao_parede = Enum("posicao_parede", ["N", "L", "S", "O"])

def movimento_relativo(cam_rel, orientacao_ini):
    idx_orientacao = posicao_parede[orientacao_ini]
    nova_orientacao = orientacao_ini

    caminho_relativo = cam_rel.copy()
    caminho_relativo.pop(0)

    movimentos = []
    for movimento in caminho_relativo:
        if   movimento == ( 0, 1): nova_orientacao = "L"
        elif movimento == ( 1, 0): nova_orientacao = "S"
        elif movimento == ( 0,-1): nova_orientacao = "O"
        elif movimento == (-1, 0): nova_orientacao = "N"

        nova_idx_orientacao = posicao_parede[nova_orientacao]

        movimentos.append(
            prox_movimento(idx_orientacao, nova_idx_orientacao)
        )

        idx_orientacao = nova_idx_orientacao

    return movimentos, posicao_parede(idx_orientacao)

def pegar_celulas_incertas():
    incertas = []
    for i in range(len(mapa)):
        for j in range(len(mapa[i])):
            if mapa[i][j].estado == estado_celula.INCERTO:
                incertas.append((i, j))
    return incertas

def achar_movimentos(caminho, orientacao):
    caminho_rel = caminho_relativo(caminho)
    movimentos, ori_fim = movimento_relativo(caminho_rel, orientacao)

    if not caminho: return None, orientacao
    return movimentos, ori_fim

