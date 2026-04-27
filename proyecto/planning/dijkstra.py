import math
import heapq
from .constants import CELL_SIZE, _DIRECTIONS
from .cspace import world_to_cell, cell_to_world

def _move_cost(dc: int, dr: int) -> float:
    """Coste de moverse: √2 en diagonal, 1 en cardinal."""
    return math.sqrt(2) if (dc != 0 and dr != 0) else 1.0


def dijkstra(start_cell: tuple,
             goal_cell: tuple,
             classification_map: dict) -> list | None:
    """
    Dijkstra sobre la cuadrícula de celdas.

    Parámetros
    ----------
    start_cell, goal_cell : (col, row)
    classification_map    : dict {(col, row): str}

    Retorna
    -------
    Lista de celdas (col, row) desde start hasta goal (inclusive),
    o None si no existe camino.
    """
    # Verificar que inicio y fin son alcanzables
    if classification_map.get(start_cell, "ocupada") == "ocupada":
        raise ValueError(f"La celda de inicio {start_cell} está ocupada.")
    if classification_map.get(goal_cell, "ocupada") == "ocupada":
        raise ValueError(f"La celda de destino {goal_cell} está ocupada.")

    dist   = {start_cell: 0.0}
    prev   = {start_cell: None}
    # heap: (coste_acumulado, col, row)
    heap   = [(0.0, start_cell[0], start_cell[1])]

    while heap:
        cost, col, row = heapq.heappop(heap)
        current = (col, row)

        if current == goal_cell:
            break   # camino encontrado

        if cost > dist.get(current, math.inf):
            continue   # entrada obsoleta

        for dc, dr in _DIRECTIONS:
            nb = (col + dc, row + dr)
            status = classification_map.get(nb)
            if status is None or status == "ocupada":
                continue
            # Penalizar semi-libre para preferir celdas completamente libres
            penalty = 2.0 if status == "semi-libre" else 1.0
            new_cost = cost + _move_cost(dc, dr) * penalty
            if new_cost < dist.get(nb, math.inf):
                dist[nb] = new_cost
                prev[nb] = current
                heapq.heappush(heap, (new_cost, nb[0], nb[1]))

    if goal_cell not in prev:
        return None   # sin camino

    # Reconstruir ruta
    path = []
    node = goal_cell
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()
    return path


def cells_to_waypoints(cell_path: list,
                       cs: float = CELL_SIZE) -> list:
    """
    Convierte la lista de celdas en waypoints (x, y) en coordenadas mundo,
    usando el centro de cada celda.
    """
    return [cell_to_world(col, row, cs) for col, row in cell_path]


def compute_full_path(scene: dict,
                       classification_map: dict,
                       cs: float = CELL_SIZE) -> list | None:
    """
    Ejecuta Dijkstra desde q0 hasta qf y devuelve la lista de
    waypoints (x, y) en metros.
    
    Robust handling of q0/qf which may be (x,y) or (x,y,theta).
    """
    q0 = scene["q0"]
    qf = scene["qf"]
    
    # Handle both (x, y) and (x, y, theta) formats
    x0, y0 = q0[0], q0[1]
    xf, yf = qf[0], qf[1]
    start = world_to_cell(x0, y0, cs)
    goal  = world_to_cell(xf, yf, cs)
    cell_path = dijkstra(start, goal, classification_map)
    if cell_path is None:
        return None
    return cells_to_waypoints(cell_path, cs)
