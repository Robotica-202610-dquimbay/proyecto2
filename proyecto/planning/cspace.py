from shapely.geometry import Polygon, Point, MultiPoint, box as shapely_box
from .constants import ROBOT_SQUARE, ROBOT_HALF, CELL_SIZE
import math

def _rect_vertices(x_min, y_min, x_max, y_max):
    """Devuelve los 4 vértices de un obstáculo rectangular."""
    return [
        (x_min, y_min), (x_max, y_min),
        (x_max, y_max), (x_min, y_max),
    ]


def build_c_obstacle(obstacle_verts: list, robot_verts: list) -> Polygon:
    """
    C-obstáculo = B ⊕ (−A)  (suma de Minkowski).
    Para un robot cuadrado fijo (sin rotación relevante en traslación)
    esto equivale a expandir cada obstáculo por ROBOT_HALF en todas direcciones.
    Se calcula igual que en P1 usando el producto cartesiano de vértices.
    """
    neg_robot = [(-x, -y) for x, y in robot_verts]
    cloud = [(px + qx, py + qy)
             for px, py in obstacle_verts
             for qx, qy in neg_robot]
    return MultiPoint(cloud).convex_hull


def build_c_obstacles(scene: dict) -> list:
    """Construye la lista de C-obstáculos para todos los obstáculos de la escena."""
    c_obs_list = []
    for obs in scene["obstacles"]:
        verts = _rect_vertices(*obs)
        c_obs = build_c_obstacle(verts, ROBOT_SQUARE)
        c_obs_list.append(c_obs)

    # También añadimos los márgenes del escenario como C-obstáculos
    # (el robot no puede salir del área)
    w, h = scene["width"], scene["height"]
    margin = ROBOT_HALF
    # Los cuatro bordes como franjas expandidas hacia adentro
    borders = [
        shapely_box(-1.0,        -1.0,       margin, h + 1.0),  # borde izq
        shapely_box(w - margin,  -1.0,       w + 1.0, h + 1.0), # borde der
        shapely_box(-1.0,        -1.0,       w + 1.0, margin),  # borde inf
        shapely_box(-1.0,        h - margin, w + 1.0, h + 1.0), # borde sup
    ]
    c_obs_list.extend(borders)
    return c_obs_list


def _cell_sample_points(col: int, row: int, cs: float) -> list:
    """
    Puntos representativos de una celda: 4 esquinas + centro.
    Igual que en P1 pero parametrizado con CELL_SIZE.
    """
    x0, y0 = col * cs, row * cs
    x1, y1 = x0 + cs, y0 + cs
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (cx, cy)]


def classify_cell(col: int, row: int, c_obs_list: list, cs: float) -> str:
    """
    Clasifica una celda como 'libre', 'semi-libre' u 'ocupada'
    comprobando contra todos los C-obstáculos.
    """
    pts = _cell_sample_points(col, row, cs)
    flags = []
    for pt in pts:
        p = Point(pt)
        inside = any(c_obs.covers(p) for c_obs in c_obs_list)
        flags.append(inside)
    if all(flags):
        return "ocupada"
    if any(flags):
        return "semi-libre"
    return "libre"


def build_classification_map(scene: dict,
                              c_obs_list: list,
                              cs: float = CELL_SIZE) -> dict:
    """
    Construye el mapa completo de clasificación de celdas.
    Retorna dict {(col, row): 'libre'|'semi-libre'|'ocupada'}
    """
    n_cols = math.ceil(scene["width"]  / cs)
    n_rows = math.ceil(scene["height"] / cs)
    return {
        (col, row): classify_cell(col, row, c_obs_list, cs)
        for row in range(n_rows)
        for col in range(n_cols)
    }


def world_to_cell(x: float, y: float, cs: float = CELL_SIZE) -> tuple:
    """Convierte coordenadas mundo (metros) a índice de celda (col, row)."""
    return (int(x / cs), int(y / cs))


def cell_to_world(col: int, row: int, cs: float = CELL_SIZE) -> tuple:
    """Devuelve el centro de una celda en coordenadas mundo."""
    return ((col + 0.5) * cs, (row + 0.5) * cs)