def load_scene(filepath: str) -> dict:
    scene = {
        "obstacles": [],
        "points_tmp": {}  # {num: {"p1":(...), "p2":(...)}}
    }

    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split(",")
            tag = parts[0].strip().upper()
            vals = [float(v) for v in parts[1:]]

            if tag == "DIMENSIONES":
                scene["width"], scene["height"] = vals

            elif tag == "Q0":
                scene["q0"] = tuple(vals)

            elif tag == "QF":
                scene["qf"] = tuple(vals)

            elif tag == "DFRENTE":
                scene["d_frente"] = vals[0]

            elif tag == "DDERECHA":
                scene["d_derecha"] = vals[0]

            elif tag.startswith("OBSTACULO"):
                # Ej: OBSTACULO3_PTO1
                tag_clean = tag.replace("OBSTACULO", "")
                num, tipo = tag_clean.split("_")

                if num not in scene["points_tmp"]:
                    scene["points_tmp"][num] = {}

                if "PTO1" in tipo:
                    scene["points_tmp"][num]["p1"] = tuple(vals)
                elif "PTO2" in tipo:
                    scene["points_tmp"][num]["p2"] = tuple(vals)

    # Construir obstáculos correctamente
    for num, pts in scene["points_tmp"].items():
        if "p1" in pts and "p2" in pts:
            x1, y1 = pts["p1"]
            x2, y2 = pts["p2"]

            x_min, x_max = min(x1, x2), max(x1, x2)
            y_min, y_max = min(y1, y2), max(y1, y2)

            scene["obstacles"].append((x_min, y_min, x_max, y_max))

    # Limpieza
    scene.pop("points_tmp")

    return scene

def scene_from_dict(width: float, height: float,
                    q0: tuple, qf: tuple,
                    d_frente: float, d_derecha: float,
                    obstacles: list) -> dict:
    """Crea un escenario directamente desde código (sin archivo)."""
    return {
        "width": width, "height": height,
        "q0": q0, "qf": qf,
        "d_frente": d_frente, "d_derecha": d_derecha,
        "obstacles": obstacles,   # lista de (x_min, y_min, x_max, y_max)
    }
