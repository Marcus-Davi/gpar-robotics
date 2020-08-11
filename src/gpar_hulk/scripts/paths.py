from math import pi, cos, sin

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


def circulo(raio = 2):
    '''Função que retorna uma list() com tuplas correspondendo a 10
    pontos de uma circunferencia com raio "raio" (padrão = 1).
    Robo começa no ponto (0,0) que pertence a circunferencia.
    '''

    npoints = 100
    points = list()
    for t in range(100):
        x = t * 2*pi/ npoints
        points.append((cos(x)*raio + raio, sin(x)*raio))

    return points