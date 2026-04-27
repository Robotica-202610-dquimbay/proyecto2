import math

def _angle_to(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Ángulo en grados del vector (x1,y1) → (x2,y2),
    medido desde el eje +X, en rango (-180, 180].
    """
    return math.degrees(math.atan2(y2 - y1, x2 - x1))


def _normalize_angle(deg: float) -> float:
    """
    Normalizes an angle in degrees to the range (-180, 180].
    Uses atan2-based normalization for robustness.
    """
    rad = math.radians(deg)
    normalized_rad = math.atan2(math.sin(rad), math.cos(rad))
    return math.degrees(normalized_rad)


def waypoints_to_configurations(waypoints: list,
                                 theta0_deg: float,
                                 thetaf_deg: float) -> list:
    """
    Convierte la lista de waypoints (x, y) en la secuencia completa de
    configuraciones (x, y, θ) intercalando las rotaciones intermedias.

    Parámetros
    ----------
    waypoints   : lista de (x, y) desde q0 hasta qf  (salida de Dijkstra)
    theta0_deg  : orientación inicial del robot en grados
    thetaf_deg  : orientación final deseada del robot en grados

    Retorna
    -------
    Lista de tuplas (x, y, theta_deg).  Cada par de entradas consecutivas
    representa o bien una rotación en lugar (mismo x,y, distinto θ)
    o bien una traslación hacia adelante (mismo θ, distinto x,y).
    """
    if not waypoints or len(waypoints) < 2:
        # Camino trivial: solo rotación final si es necesario
        x, y = waypoints[0]
        configs = [(x, y, theta0_deg)]
        if abs(_normalize_angle(thetaf_deg - theta0_deg)) > 0.01:
            configs.append((x, y, thetaf_deg))
        return configs

    configs = []
    current_theta = theta0_deg

    for i, (x, y) in enumerate(waypoints):
        if i == 0:
            # Configuración inicial q0 con su orientación original
            configs.append((x, y, current_theta))
            continue

        # Ángulo necesario para apuntar al siguiente waypoint
        x_prev, y_prev = waypoints[i - 1]
        alpha = _angle_to(x_prev, y_prev, x, y)

        # qrot(i-1→i): rotación en el lugar del waypoint anterior
        # Solo añadirla si el ángulo cambia de forma significativa (> 0.5°)
        if abs(_normalize_angle(alpha - current_theta)) > 0.5:
            configs.append((x_prev, y_prev, alpha))
            current_theta = alpha

        # qi: traslación al waypoint actual (misma orientación)
        configs.append((x, y, current_theta))

    # qrotn: rotación final para quedar con la orientación de qf
    xf, yf = waypoints[-1]
    if abs(_normalize_angle(thetaf_deg - current_theta)) > 0.5:
        configs.append((xf, yf, thetaf_deg))

    return configs


def _label_configurations(configurations: list) -> list:
    """
    Genera etiquetas descriptivas para cada configuración de la secuencia:
    q0, qrot0-1, q1, qrot1-2, ..., qn, qrotn
    """
    labels = []
    n = len(configurations)
    q_idx = 0
    i = 0

    while i < n:
        x, y, theta = configurations[i]

        if i == 0:
            labels.append("q0")
            q_idx = 0
            i += 1
            continue

        x_prev, y_prev, _ = configurations[i - 1]
        same_pos = (abs(x - x_prev) < 1e-6 and abs(y - y_prev) < 1e-6)

        if same_pos:
            # Rotación en el lugar → qrot(q_idx → q_idx+1)
            labels.append(f"qrot{q_idx}-{q_idx+1}")
        else:
            # Traslación → siguiente qi
            q_idx += 1
            labels.append(f"q{q_idx}")

        i += 1

    # Corregir la última etiqueta si es una rotación final en qf
    if n > 1:
        x_last, y_last, _ = configurations[-1]
        x_prev, y_prev, _ = configurations[-2]
        same_pos = (abs(x_last - x_prev) < 1e-6 and abs(y_last - y_prev) < 1e-6)
        if same_pos:
            labels[-1] = f"qrot{q_idx}"  # rotación final

    return labels


def save_path_txt(configurations: list,
                  filepath: str,
                  scene: dict = None) -> None:
    """
    Guarda la secuencia de configuraciones en un archivo .txt.
    Formato de cada línea:  x  y  theta_deg  (tabulador, 4 decimales)
    Incluye cabecera comentada y etiquetas para facilitar la lectura.
    El compañero B leerá este archivo para ejecutar los movimientos en ROS2.
    """
    with open(filepath, "w") as f:
        f.write("# Camino geometrico – ISIS4826 Proyecto 2\n")
        if scene:
            x0, y0, th0 = scene["q0"]
            xg, yg, thg = scene["qf"]
            f.write(f"# q0 = ({x0}, {y0}, {th0} deg)\n")
            f.write(f"# qf = ({xg}, {yg}, {thg} deg)\n")
            f.write(f"# Escenario: {scene['width']} x {scene['height']} m\n")
        f.write(f"# Total configuraciones: {len(configurations)}\n")
        f.write("# Formato: x[m]  y[m]  theta[deg]\n")
        f.write("#\n")

        labels = _label_configurations(configurations)

        f.write(f"# {'Etiqueta':<14}  {'x':>9}  {'y':>9}  {'theta':>9}\n")
        f.write("# " + "-" * 45 + "\n")
        for (x, y, theta), label in zip(configurations, labels):
            f.write(f"  {label:<14}  {x:>9.4f}  {y:>9.4f}  {theta:>9.4f}\n")

    print(f"  Archivo guardado: {filepath}  ({len(configurations)} configuraciones)")


def generate_path_file(scene: dict,
                       waypoints: list,
                       output_filepath: str) -> list:
    """
    Función principal de A3: a partir de los waypoints de Dijkstra,
    genera la secuencia completa (x, y, θ) y la guarda en .txt.
    Retorna la lista de configuraciones para que el main pueda imprimirla.
    """
    _, _, theta0 = scene["q0"]
    _, _, thetaf = scene["qf"]
    configurations = waypoints_to_configurations(waypoints, theta0, thetaf)
    save_path_txt(configurations, output_filepath, scene=scene)
    return configurations