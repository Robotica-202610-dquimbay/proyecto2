ROBOT_HALF = 0.15
CELL_SIZE = 0.30

ROBOT_SQUARE = [
    (-ROBOT_HALF, -ROBOT_HALF),
    ( ROBOT_HALF, -ROBOT_HALF),
    ( ROBOT_HALF,  ROBOT_HALF),
    (-ROBOT_HALF,  ROBOT_HALF),
]

_DIRECTIONS = [
    (0,  1),  (1,  1),  (1,  0),  (1, -1),
    (0, -1),  (-1,-1),  (-1, 0),  (-1, 1),
]