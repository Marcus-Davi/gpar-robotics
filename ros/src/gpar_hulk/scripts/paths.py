#import matplotlib.pyplot as plt #Visualização dos pontos

def quadrado(lado = 2):
    '''Função que retorna um list() com tuplas correspondendo as 
    coordenadas das arestas de um quadrado de lado "lado" (padrão = 1).
    O robô deve começar no ponto (0,0) e terminar no ponto (0,0).
    '''

    points = list()
    points.append((0,lado))
    points.append((lado,lado))
    points.append((lado,0))
    points.append((0,0))

    return points

