import math

def obtener_distancia_angulo(msg_scan, angulo_objetivo_rad):
    """
    Calcula la distancia a un obstáculo en un ángulo específico.
    """
    if msg_scan is None: return float('inf')
    
    # Encontrar el índice correspondiente al ángulo objetivo
    # Normalizamos el ángulo basado en las propiedades del mensaje Lidar
    indice = int((angulo_objetivo_rad - msg_scan.angle_min) / msg_scan.angle_increment)
    
    # Validamos que el índice no se salga de los límites del arreglo
    if 0 <= indice < len(msg_scan.ranges):
        distancia = msg_scan.ranges[indice]
        # Verificamos que la lectura sea confiable (ni muy cerca ni "infinita")
        if msg_scan.range_min <= distancia <= msg_scan.range_max:
            return distancia
            
    return float('inf')

def _normalize_lidar_angle(angle_rad: float, angle_min_rad: float = -math.pi, 
                          angle_max_rad: float = math.pi) -> float:
    """
    Normalizes an angle to a specific range.
    Default range is [-π, π] for ROS Lidar scans.
    """
    while angle_rad > angle_max_rad:
        angle_rad -= 2 * math.pi
    while angle_rad < angle_min_rad:
        angle_rad += 2 * math.pi
    return angle_rad

def obtener_distancias_rango(msg_scan, angulo_min_deg, angulo_max_deg):
    """
    Retorna una lista con las distancias válidas capturadas en un rango de ángulos (en grados).
    
    Handles wraparound at ±180° boundary properly.
    E.g., querying 170° to -170° (crossing ±180°) works correctly.
    """
    if msg_scan is None: return []

    rad_min = math.radians(angulo_min_deg)
    rad_max = math.radians(angulo_max_deg)
    
    # Normalize the input range to [-π, π]
    rad_min = _normalize_lidar_angle(rad_min)
    rad_max = _normalize_lidar_angle(rad_max)
    
    distancias = []
    
    # Check if this is a wraparound range (e.g., 170° to -170° crosses ±180°)
    is_wraparound = rad_min > rad_max
    
    for i, dist in enumerate(msg_scan.ranges):
        angulo = msg_scan.angle_min + (i * msg_scan.angle_increment)
        
        # Normalize scan angle to [-π, π]
        angulo = _normalize_lidar_angle(angulo)
        
        # Check if angle is in range
        in_range = False
        if is_wraparound:
            # Wraparound case: include angles >= rad_min OR <= rad_max
            in_range = (angulo >= rad_min or angulo <= rad_max)
        else:
            # Normal case: include angles in [rad_min, rad_max]
            in_range = (rad_min <= angulo <= rad_max)
        
        if in_range and msg_scan.range_min <= dist <= msg_scan.range_max:
            distancias.append(dist)
            
    return distancias