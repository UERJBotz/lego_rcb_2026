from pybricks.tools      import wait
from pybricks.parameters import Color

from lib.polyfill import Enum, rgb_to_hsv, hsv_to_rgb

from lib.cores_calibradas_ import mapa_hsv, mapa_hsv_frente

from comum import globais

cor = Enum("cor", ["NENHUMA",
                   "PRETO",
                   "AZUL",
                   "VERDE",
                   "AMARELO",
                   "VERMELHO",
                   "BRANCO",
                   "MARROM"])

#! o mapa pro identificar deve tar errado pq a gente mudou a ordem do enum.
##! talvez tenha que mudar pra um dicionário

class Cor:
    enum = cor

    def __init__(self, color=None, hsv=None, cor=None):
        self.color = color or cor2Color(cor)
        self.cor = cor or Color2cor(self.color)
        self.hsv = hsv or self.color
        self.hsv = Color2tuple(self.hsv)

        if self.hsv:
            self.h, self.s, self.v = self.hsv
        else:
            self.h, self.s, self.v = None, None, None

    def __iter__(self): # para poder fazer *Cor
        yield self.color
        yield self.hsv

    def __eq__(self, other): #! testar
        if other is self: return True

        if isinstance(other, Cor):
            return other.hsv == self.hsv and other.cor == self.cor

        #! Color não é uma classe aqui (???) mas Color.X são instâncias da classe Color
        if isinstance(other, type(Color.BLACK)): return other == self.color
        if isinstance(other, tuple):             return other == self.hsv
        if isinstance(other, int):               return other == self.cor #! enum cor
        #! será que devia usar o identificar?

    def __str__(self):
        return f"{self.enum(self.cor)}[{self.h}, {self.s}, {self.v}]"
    def __repr__(self): return self.__str__()

    def preto(self):    return (self.color == Color.PRETO)
    def azul(self):     return (self.color == Color.BLUE)
    def verde(self):    return (self.color == Color.GREEN)
    def amarelo(self):  return ((self.color == Color.YELLOW) or
                                (self.color == Color.WHITE))
    def vermelho(self): return (self.color == Color.RED)
    def branco(self):   return (self.color == Color.WHITE)
    def marrom(self):   return (self.color == Color.BROWN)

    def area_livre(self): return self.verde()
    def pista(self):      return self.branco() or self.verde()
    def parede(self):     return self.preto()
    def beco(self):       return self.vermelho()

    @classmethod
    def init(cls):
        cls.NENHUMA  = Cor(cor=cor.NENHUMA)
        cls.PRETO    = Cor(cor=cor.PRETO)
        cls.AZUL     = Cor(cor=cor.AZUL)
        cls.VERDE    = Cor(cor=cor.VERDE)
        cls.AMARELO  = Cor(cor=cor.AMARELO)
        cls.VERMELHO = Cor(cor=cor.VERMELHO)
        cls.BRANCO   = Cor(cor=cor.BRANCO)
        cls.MARROM   = Cor(cor=cor.MARROM)

def Color2tuple(color): #! falhar mais alto
    try:    return color.h, color.s, color.v
    except: return None #! except

def Color2cor(color): #! falhar mais alto
    res = {
        Color2tuple(Color.BLACK ): Cor.enum.PRETO,
        Color2tuple(Color.BLUE  ): Cor.enum.AZUL,
        Color2tuple(Color.GREEN ): Cor.enum.VERDE,
        Color2tuple(Color.YELLOW): Cor.enum.AMARELO,
        Color2tuple(Color.RED   ): Cor.enum.VERMELHO,
        Color2tuple(Color.WHITE ): Cor.enum.BRANCO,
        Color2tuple(Color.BROWN ): Cor.enum.MARROM,
    }.get(Color2tuple(color))
    return res if res is not None else cor.NENHUMA

def cor2Color(cor): #! falhar mais alto
    res = {
        Cor.enum.NENHUMA:  Color.NONE,
        Cor.enum.PRETO:    Color.BLACK,
        Cor.enum.AZUL:     Color.BLUE,
        Cor.enum.VERDE:    Color.GREEN,
        Cor.enum.AMARELO:  Color.YELLOW,
        Cor.enum.VERMELHO: Color.RED,
        Cor.enum.BRANCO:   Color.WHITE,
        Cor.enum.MARROM:   Color.BROWN
    }.get(cor)
    return res


#hsv = tuple[float, float, float]

def norm_hsv(hsv):
    if type(hsv) != tuple:
        h, s, v = hsv.h, hsv.s, hsv.v
    else:
        h, s, v = hsv
    return (h/360, s/100, v/100)

def unnorm_hsv(hsv):
    h, s, v = hsv
    return (h*360, s*100, v*100)

def todas(sensor_esq, sensor_dir) -> tuple[tuple, tuple]:
    esq = Cor(sensor_esq.color(), sensor_esq.hsv())
    dir = Cor(sensor_dir.color(), sensor_dir.hsv())
    return (esq, dir)

def iter_coleta(botao_parar, sensor):
    minm, maxm = (360, 100, 100), (0, 0, 0)
    soma, cont = (000, 000, 000), 0
    while botao_parar not in globais.hub.buttons.pressed():
        hsv = sensor.hsv()
        hsv = hsv.h, hsv.s, hsv.v

        rgb_norm = hsv_to_rgb(norm_hsv(hsv))

        minm = tuple(map(min, minm, hsv))
        maxm = tuple(map(max, maxm, hsv))

        soma = tuple(map(lambda c,s: c+s, hsv, soma))
        cont += 1

        if 1:
            cor_txt_rgb = tuple(map("{:.2f}".format, rgb_norm))
            cor_txt_hsv = tuple(map("{:.2f}".format, hsv))
            print(cor_txt_hsv, cor_txt_rgb)

        yield (minm, soma, cont, maxm)


def coletar_valores(botao_parar, sensor) -> tuple[hsv, hsv, hsv]: # type: ignore
    wait(200)
    for info in iter_coleta(botao_parar, sensor):
        minm, soma, cont, maxm = info

    med = tuple(map(lambda s: s/cont, soma))
    print("max:", maxm, "med:", med, "min:", minm)

    return (minm, tuple(map(round, med)), maxm)

def identificar_por_intervalo_hsv(hsv, mapa) -> cor: # type: ignore
    h, s, v = hsv
    
    if   v <= 20: #! usar valores do arquivo
        return cor.PRETO
    elif s <= 10 and v >= 90:
        return cor.BRANCO

    for i, (m, mm, M) in enumerate(mapa):
        hm, _, _ = m
        hM, _, _ = M

        if h in range(hm, hM): return i
    return cor.NENHUMA

def identificar_por_hue_medio(hsv, mapa) -> int:
    h, s, v = hsv
    menor_diferenca = float('inf')
    indice_mais_perto = None

    for i, (_, medM, _) in enumerate(mapa):
        # Considera apenas a componente media (hM)
        hm, _, _ = medM
        diferenca = abs(h - hm)  #  representa a média do hue
        if diferenca < menor_diferenca:
            indice_mais_perto = i
            menor_diferenca = diferenca
    
    if indice_mais_perto is None:
        return cor.NENHUMA  # Retorne um valor padrão se não encontrar nada

    return indice_mais_perto  # Retorna o índice da cor mais próxima


def identificar(color, sensor="chao") -> cor: # type: ignore   
    if sensor == "frente":
        identificar_cor = identificar_por_hue_medio
        mapa = mapa_hsv_frente
    elif sensor == "chao":
        identificar_cor = identificar_por_intervalo_hsv
        mapa = mapa_hsv

    try: #! ver jeito melhor
        return identificar_cor(color, mapa)
    except TypeError as e:
        print(f"cores.identificar: {e}")
        hsv = color.h, color.s, color.v
        return identificar_cor(hsv, mapa)


def certificar(sensor_dir, sensor_esq, uni, uni2=None) -> bool:
    if uni2 is None: uni2 = uni
    esq, dir = todas(sensor_esq, sensor_dir)

    res = uni(esq) and uni2(dir)
    print(f"certificar_cor: {res=}")

    return res


#! tinha um motivo pra não guardar cor.NENHUMA?
def repl_calibração(mapa_hsv, lado=""):
    print(f"mapa_hsv{lado} = [")
    for c in Cor.enum:
        print(f"\t{mapa_hsv[Cor.enum[c]]}, #{c}")
    print("]")

## Inicialização da classe Cor
Cor.init()
