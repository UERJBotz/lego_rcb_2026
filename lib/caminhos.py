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
                       direc=direcionalidade.NENHUMA, mut=False, visitada=False):
        self.estado  = estado
        self.direc   = direc
        self.mutável = mut
        self.visitada= visitada

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

class Safe(Cruz):
    def __str__(self): return '*'

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


mapa = [
    [Safe(estado=estado_celula.LIVRE), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
    [Vert(estado=estado_celula.LIVRE), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert()],
    [Safe(estado=estado_celula.LIVRE), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
    [Vert(estado=estado_celula.LIVRE), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert()],
    [Safe(estado=estado_celula.LIVRE), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
    [Vert(estado=estado_celula.LIVRE), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert()],
    [Safe(estado=estado_celula.LIVRE), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
    [Vert(estado=estado_celula.LIVRE), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert(), Nada(), Vert()],
    [Safe(estado=estado_celula.LIVRE), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz(), Hori(), Cruz()],
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

def coloca_obstaculo_incerto(pos):
    y, x = pos; cel = mapa[y][x]
    if cel.mutável: #! falhar mais alto
        cel.estado = estado_celula.INCERTO

def visitar(pos):
    y, x = pos; cel = mapa[y][x]
    cel.visitada = True

def visitada(pos):
    y, x = pos; cel = mapa[y][x]
    return cel.visitada

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


## implementação do A* (adaptada de <https://www.geeksforgeeks.org/a-search-algorithm-in-python/>)
class Cell():
    def __init__(self, i_pai=0,
                       j_pai=0,
                       f=float('inf'),
                       g=float('inf'),
                       h=0):
        self.i_pai = i_pai # Parent cell's row index
        self.j_pai = j_pai # Parent cell's column index
        self.f     = f # Total cost of the cell (g + h)
        self.g     = g # Cost from start to this cell
        self.h     = h # Heuristic cost from this cell to destination

orientações = {"L":(0, 1), "S":(1, 0), "O":(0, -1),"N":(-1, 0)}

direcoes = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Movimentos: direita, baixo, esquerda, cima (l, s, o, n)

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

# Trace the path from source to destination
def tracar_caminho(info_celulas, dest):
    print("The Path is ")
    path = []
    row, col = dest

    # Trace the path from destination to source using parent cells
    while not (info_celulas[row][col].i_pai == row and info_celulas[row][col].j_pai == col):
        path.append((row, col))
        temp_row = info_celulas[row][col].i_pai
        temp_col = info_celulas[row][col].j_pai
        row = temp_row
        col = temp_col

    # Add the source cell to the path
    path.append((row, col))
    # Reverse the path to get the path from source to destination
    path.reverse()

    return path

def custo_virada(atual, final):
    if atual == final: return 0
    if atual == (-final[0], -final[1]): return 2
    return 1

# Implement the A* search algorithm
def a_estrela(grid, src, dest, ori, peso_visitada=0.5):
    ok = True
    if not dentro_dos_limites(grid, src):
        ok = False; print(f"a_estrela: origem inválida {src}")
    if not dentro_dos_limites(grid, dest):
        ok = False; print(f"a_estrela: destino inválido {dest}")

    if not celula_livre(grid, src):
        ok = False; print(f"a_estrela: origem bloqueada {src}")
    if celula_ocupada(grid, dest):
        ok = False; print(f"a_estrela: destino bloqueado {dest}")

    if not ok: return None

    # Check if we are already at the destination
    if eh_destino(src, dest):
        print(f"a_estrela: já no destino {src, dest}"); return []

    ROW = len(grid)
    COL = len(grid[0])

    # Initialize the closed list (visited cells)
    closed_list  = [[False for _ in range(COL)]  for _ in range(ROW)]
    # Initialize the details of each cell
    info_celulas = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Initialize the start cell details
    i, j = src
    info_celulas[i][j] = Cell(f=0,
                              g=0,
                              h=0,
                              i_pai=i,
                              j_pai=j)

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heappush(open_list, (0.0, i, j))

    # Initialize the flag for whether destination is found
    achou_dest = False
    dir_ini = orientações[ori] 

    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heappop(open_list)

        # Mark the cell as visited
        i = p[1]
        j = p[2]
        closed_list[i][j] = True
        atual = info_celulas[i][j]
        if atual.i_pai == i and atual.j_pai == j: dir_atual = dir_ini
        else: dir_atual = (i - atual.i_pai, j - atual.j_pai)

        # For each direction, check the successors
        for dir in direcoes:
            new_i = i + dir[0]
            new_j = j + dir[1]
            new = new_i, new_j

            # If the successor is valid, (unblocked or is destiny), and not visited
            if ((dentro_dos_limites(grid, new) and celula_livre(grid, new)) or
                (eh_destino(new, dest) and not closed_list[new_i][new_j])):

                if celula_especialmente_bloqueada(grid, (i,j), new): continue

                # If the successor is the destination
                if eh_destino(new, dest):
                    # Set the parent of the destination cell
                    info_celulas[new_i][new_j].i_pai = i
                    info_celulas[new_i][new_j].j_pai = j
                    print("a_estrela: caminho encontrado")
                    # Trace and print the path from source to destination
                    return tracar_caminho(info_celulas, dest)
                else:
                    # Calculate the new f, g, and h values
                    g_new = info_celulas[i][j].g + 1.0 + custo_virada(dir_atual, dir)
                    if visitada(new): g_new += peso_visitada
                    h_new = heuristica(new, dest)
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if (info_celulas[new_i][new_j].f == float('inf') or
                        info_celulas[new_i][new_j].f > f_new):
                        # Add the cell to the open list
                        heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        info_celulas[new_i][new_j] = Cell(f = f_new,
                                                          g = g_new,
                                                          h = h_new,
                                                          i_pai = i,
                                                          j_pai = j)

    # If the destination is not found after visiting all cells
    if not achou_dest:
        print("a_estrela: não achou")

    return None

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

def pegar_celulas_incertas_alcancaveis(pos_ini):
    vistas     = {pos_ini}
    explorando = [pos_ini]
    incertas   = []

    while explorando:
        i, j = explorando.pop(0)
        for di, dj in direcoes:
            vizinho = (i + di, j + dj)

            if vizinho in vistas: continue
            if not dentro_dos_limites(mapa, vizinho): continue
            if celula_especialmente_bloqueada(mapa, (i, j), vizinho): continue

            vistas.add(vizinho)
            estado = mapa[vizinho[0]][vizinho[1]].estado

            if estado == estado_celula.LIVRE: explorando.append(vizinho)
            elif estado == estado_celula.INCERTO: incertas.append(vizinho)
    return incertas

def achar_caminhos(pos_ini, pos_fim, orientacao):
    caminho = a_estrela(mapa, pos_ini, pos_fim, ori=orientacao)
    return caminho

def achar_movimentos(caminho, orientacao):
    caminho_rel = caminho_relativo(caminho)
    movimentos, ori_fim = movimento_relativo(caminho_rel, orientacao)

    if not caminho: return None, orientacao
    return movimentos, ori_fim

