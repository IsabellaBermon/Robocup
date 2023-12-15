def map_coordinates(x, a, b, c, d):
    """
    Mapea el valor x de un rango [a, b] a un rango [c, d].
    """
    return (x - a) * (d - c) / (b - a) + c