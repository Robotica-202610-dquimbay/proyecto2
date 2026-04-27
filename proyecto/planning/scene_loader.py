def load_scene(filepath: str) -> dict:
    scene = {
        "obstacles": [],
        "points_tmp": {}  # {num: {"p1":(...), "p2":(...)}}
    }

    with open(filepath) as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split(",")
            if not parts:
                continue
                
            tag = parts[0].strip().upper()
            
            # Safely parse numeric values, skip if malformed
            try:
                vals = [float(v) for v in parts[1:]] if len(parts) > 1 else []
            except ValueError:
                print(f"Warning: Skipping malformed line {line_num}: {line}")
                continue

            if tag == "DIMENSIONES":
                if len(vals) >= 2:
                    scene["width"], scene["height"] = vals[0], vals[1]

            elif tag == "Q0":
                if len(vals) >= 2:
                    scene["q0"] = tuple(vals[:3]) if len(vals) >= 3 else (vals[0], vals[1], 0.0)

            elif tag == "QF":
                if len(vals) >= 2:
                    scene["qf"] = tuple(vals[:3]) if len(vals) >= 3 else (vals[0], vals[1], 0.0)

            elif tag == "DFRENTE":
                if vals:
                    scene["d_frente"] = vals[0]

            elif tag == "DDERECHA":
                if vals:
                    scene["d_derecha"] = vals[0]

            elif tag.startswith("OBSTACULO"):
                # Ej: OBSTACULO3_PTO1
                try:
                    tag_clean = tag.replace("OBSTACULO", "")
                    parts_tag = tag_clean.split("_")
                    if len(parts_tag) != 2:
                        continue
                    num, tipo = parts_tag
                    
                    if len(vals) >= 2:
                        if num not in scene["points_tmp"]:
                            scene["points_tmp"][num] = {}

                        if "PTO1" in tipo:
                            scene["points_tmp"][num]["p1"] = tuple(vals[:2])
                        elif "PTO2" in tipo:
                            scene["points_tmp"][num]["p2"] = tuple(vals[:2])
                except (ValueError, IndexError):
                    print(f"Warning: Skipping malformed obstacle line {line_num}: {line}")
                    continue

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
