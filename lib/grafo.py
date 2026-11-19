# construindo grafo

class Noh:
    NORTE = 1,
    SUL = 2
    LESTE = 3
    OESTE = 4

def construir_nohs(mapa)
    mapa_em_nohs = []
    mapa = { (1, Noh.NORTE): [] }
    k = 0
    for i in range(len(mapa))
        for j in range(len(mapa[0])):
            if isinstance(celula, Cruz):
                mapa_em_nohs.append({(k+0, Noh.NORTE): [(k+1, Noh.LESTE), (k+3, Noh.OESTE)],
                                     (k+2, Noh.SUL):   [(k+1, Noh.LESTE), (k+3, Noh.OESTE)],
                                     (k+1, Noh.LESTE): [(k+0, Noh.NORTE), (k+2, Noh.SUL)],
                                     (k+3, Noh.OESTE): [(k+0, Noh.NORTE), (k+2, Noh.SUL)]})
                k += 4
            elif isintance(celula, Vert):
                mapa_de_nohs.append({(k+0): [],
                                     (k+1): []})
                k += 2
            elif isintance(celula, Hori):
                mapa_de_nohs.append([Noh.NORTE,
                                     Noh.SUL])

    return mapa_de_nohs

def 


class No:
    def __init__(self, celula):
        # Cruz -> 4 nós
        # Vert -> 2
        # Hori -> 2