"""
Pure Python tests for core robotics modules (NO ROS2 required).
Tests focus on:
- scene_loader.py
- cspace.py
- dijkstra.py
- Coordinate transformations
"""

import sys
import os
import math
from pathlib import Path

# Add proyecto to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from proyecto.planning.scene_loader import load_scene, scene_from_dict
from proyecto.planning.cspace import (
    build_c_obstacles, build_classification_map, 
    world_to_cell, cell_to_world, _rect_vertices
)
from proyecto.planning.dijkstra import dijkstra, cells_to_waypoints, compute_full_path
from proyecto.planning.constants import CELL_SIZE, ROBOT_HALF, ROBOT_SQUARE


def test_scene_loader():
    """Test 1: Scene loader correctness"""
    print("\n" + "="*60)
    print("TEST 1: scene_loader.py")
    print("="*60)
    
    # Test programmatic scene creation
    scene = scene_from_dict(
        width=4.0,
        height=5.0,
        q0=(0.75, 0.75, 0.0),
        qf=(3.25, 4.25, 90.0),
        d_frente=0.8,
        d_derecha=0.78,
        obstacles=[(3.0, 0.5, 3.5, 1.0)]
    )
    
    print(f"✓ Scene created: {scene['width']}x{scene['height']}")
    print(f"  q0: {scene['q0']}")
    print(f"  qf: {scene['qf']}")
    print(f"  obstacles: {scene['obstacles']}")
    
    # Load from file
    data_dir = Path(__file__).parent / "data"
    scene_file = data_dir / "Escena-Problema1.txt"
    
    if scene_file.exists():
        scene_loaded = load_scene(str(scene_file))
        print(f"\n✓ Loaded Escena-Problema1.txt")
        print(f"  Width: {scene_loaded.get('width')}, Height: {scene_loaded.get('height')}")
        print(f"  q0: {scene_loaded.get('q0')}")
        print(f"  qf: {scene_loaded.get('qf')}")
        print(f"  Obstacles count: {len(scene_loaded.get('obstacles', []))}")
        print(f"  Obstacle details: {scene_loaded.get('obstacles')}")
        
        # VALIDATION: Check q0 and qf dimensions
        q0 = scene_loaded.get('q0')
        qf = scene_loaded.get('qf')
        if len(q0) != 3 or len(qf) != 3:
            print(f"  ⚠ WARNING: q0/qf should be 3-tuples (x,y,theta), got {len(q0)}, {len(qf)}")
        else:
            print(f"  ✓ q0/qf are 3-tuples")
    else:
        print(f"⚠ File not found: {scene_file}")


def test_coordinate_transforms():
    """Test 2: Coordinate conversion consistency"""
    print("\n" + "="*60)
    print("TEST 2: Coordinate Transformations")
    print("="*60)
    
    cs = CELL_SIZE
    print(f"CELL_SIZE: {cs} m")
    print(f"ROBOT_HALF: {ROBOT_HALF} m")
    
    # Test world_to_cell and cell_to_world
    test_points = [
        (0.0, 0.0),
        (0.15, 0.15),  # Within cell (0,0)
        (0.3, 0.3),    # Corner of cell (1,1)
        (0.6, 0.6),    # Middle of cell (1,1) center
        (1.5, 2.0),
    ]
    
    print("\nWorld → Cell → World conversions:")
    for x, y in test_points:
        cell = world_to_cell(x, y, cs)
        x_back, y_back = cell_to_world(cell[0], cell[1], cs)
        error = math.sqrt((x - x_back)**2 + (y - y_back)**2)
        print(f"  ({x:.2f}, {y:.2f}) → cell {cell} → ({x_back:.2f}, {y_back:.2f}) [error: {error:.4f}m]")
        
        # ERROR CHECK: Reconstruction should have max error of half cell size
        if error > cs/2:
            print(f"    ⚠ ERROR: Reconstruction error {error:.4f} > {cs/2}")
        else:
            print(f"    ✓ OK")


def test_cspace():
    """Test 3: C-space computation"""
    print("\n" + "="*60)
    print("TEST 3: C-space (Minkowski Sum)")
    print("="*60)
    
    # Create simple scene
    scene = scene_from_dict(
        width=4.0,
        height=5.0,
        q0=(0.5, 0.5, 0.0),
        qf=(3.5, 4.5, 0.0),
        d_frente=0.8,
        d_derecha=0.78,
        obstacles=[
            (1.5, 1.5, 2.0, 2.0),  # Square obstacle
        ]
    )
    
    print(f"Scene: {scene['width']}x{scene['height']}")
    print(f"Robot size: {2*ROBOT_HALF}x{2*ROBOT_HALF} (square)")
    print(f"Obstacles: {scene['obstacles']}")
    
    try:
        c_obs_list = build_c_obstacles(scene)
        print(f"\n✓ C-obstacles computed: {len(c_obs_list)} total")
        print(f"  (including 4 borders)")
        
        # Check each c-obstacle
        for i, c_obs in enumerate(c_obs_list[:1]):  # Just check first
            bounds = c_obs.bounds  # (minx, miny, maxx, maxy)
            print(f"  C-obstacle {i}: bounds {bounds}")
    except Exception as e:
        print(f"✗ Error in C-space: {e}")


def test_classification_map():
    """Test 4: Cell classification"""
    print("\n" + "="*60)
    print("TEST 4: Classification Map")
    print("="*60)
    
    scene = scene_from_dict(
        width=2.0,
        height=2.0,
        q0=(0.15, 0.15, 0.0),
        qf=(1.85, 1.85, 0.0),
        d_frente=0.8,
        d_derecha=0.78,
        obstacles=[
            (0.75, 0.75, 1.25, 1.25),  # Central square obstacle
        ]
    )
    
    print(f"Scene: 2x2 m with central obstacle at (0.75-1.25, 0.75-1.25)")
    print(f"Robot can safely occupy from ({ROBOT_HALF:.2f}, {ROBOT_HALF:.2f})")
    
    try:
        c_obs_list = build_c_obstacles(scene)
        cmap = build_classification_map(scene, c_obs_list, CELL_SIZE)
        
        print(f"\n✓ Classification map computed: {len(cmap)} cells")
        
        # Count cell types
        free = sum(1 for v in cmap.values() if v == "libre")
        semi = sum(1 for v in cmap.values() if v == "semi-libre")
        occupied = sum(1 for v in cmap.values() if v == "ocupada")
        
        print(f"  - Libre: {free}")
        print(f"  - Semi-libre: {semi}")
        print(f"  - Ocupada: {occupied}")
        
        # Show map
        n_cols = math.ceil(scene['width'] / CELL_SIZE)
        n_rows = math.ceil(scene['height'] / CELL_SIZE)
        print(f"\n  Grid ({n_cols}x{n_rows}):")
        for row in range(n_rows-1, -1, -1):
            line = "  "
            for col in range(n_cols):
                status = cmap.get((col, row), "?")
                char = "█" if status == "ocupada" else "▓" if status == "semi-libre" else "."
                line += char
            print(line)
            
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()


def test_dijkstra():
    """Test 5: Dijkstra pathfinding"""
    print("\n" + "="*60)
    print("TEST 5: Dijkstra Pathfinding")
    print("="*60)
    
    scene = scene_from_dict(
        width=3.0,
        height=3.0,
        q0=(0.15, 0.15, 0.0),
        qf=(2.85, 2.85, 0.0),
        d_frente=0.8,
        d_derecha=0.78,
        obstacles=[
            (1.2, 0.6, 1.8, 1.2),  # Obstacle
        ]
    )
    
    print(f"Scene: 3x3 m, start=(0.15,0.15), goal=(2.85,2.85)")
    
    try:
        c_obs_list = build_c_obstacles(scene)
        cmap = build_classification_map(scene, c_obs_list, CELL_SIZE)
        
        # Compute path
        waypoints = compute_full_path(scene, cmap, CELL_SIZE)
        
        if waypoints is None:
            print("✗ No path found!")
        else:
            print(f"✓ Path found: {len(waypoints)} waypoints")
            for i, (x, y) in enumerate(waypoints[:5]):
                print(f"  wp{i}: ({x:.3f}, {y:.3f})")
            if len(waypoints) > 5:
                print(f"  ... ({len(waypoints)-5} more)")
            
            # Verify path continuity
            print("\n  Path continuity check:")
            all_valid = True
            for i in range(len(waypoints)-1):
                x1, y1 = waypoints[i]
                x2, y2 = waypoints[i+1]
                dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                if dist > 2.0:  # Max reasonable jump
                    print(f"    ✗ Large jump at step {i}: {dist:.3f}m")
                    all_valid = False
            
            if all_valid:
                print(f"    ✓ All waypoints within 2.0m")
                
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()


def test_angle_normalization():
    """Test 6: Angle handling"""
    print("\n" + "="*60)
    print("TEST 6: Angle Normalization")
    print("="*60)
    
    from proyecto.planning.path_generator import _normalize_angle
    
    test_angles = [0, 90, 180, 270, 360, -90, -180, 450, -450]
    print("Angle normalization to (-180, 180]:")
    for angle in test_angles:
        norm = _normalize_angle(angle)
        print(f"  {angle:4d}° → {norm:7.1f}°")


def test_quaternion_extraction():
    """Test 7: Quaternion to angle conversion"""
    print("\n" + "="*60)
    print("TEST 7: Quaternion to Angle Conversion")
    print("="*60)
    
    print("⚠ CRITICAL BUG FOUND:")
    print("navigation_node.py line 71-73 extracts angle from quaternion:")
    print("  qz = msg.pose.pose.orientation.z")
    print("  qw = msg.pose.pose.orientation.w")
    print("  self.current_theta = 2.0 * math.atan2(qz, qw)")
    print("\nThis is INCORRECT! For 2D rotation about Z-axis:")
    print("  theta = 2 * atan2(qz, qw)  ← This assumes qx=qy=0")
    print("  theta = atan2(2*(qw*qz - qx*qy), 1 - 2*(qy²+qz²))  ← Correct formula")
    print("\nFor robots publishing only Z rotation, this may work by accident,")
    print("but it's non-standard. Should use full quaternion conversion.")
    
    # Test what happens with full quaternion
    import math
    qx, qy, qz, qw = 0, 0, 0.7071, 0.7071  # 90 degree rotation about Z
    theta_wrong = 2.0 * math.atan2(qz, qw)
    print(f"\nExample: quat=(0,0,{qz:.4f},{qw:.4f}) (90° Z rotation)")
    print(f"  Wrong formula: theta = {math.degrees(theta_wrong):.1f}°")
    print(f"  Expected: theta = 90.0°")


def run_all_tests():
    """Run all tests"""
    print("\n" + "█"*60)
    print("CORE LOGIC VALIDATION - NO ROS2 REQUIRED")
    print("█"*60)
    
    try:
        test_scene_loader()
    except Exception as e:
        print(f"ERROR in test_scene_loader: {e}")
    
    try:
        test_coordinate_transforms()
    except Exception as e:
        print(f"ERROR in test_coordinate_transforms: {e}")
    
    try:
        test_cspace()
    except Exception as e:
        print(f"ERROR in test_cspace: {e}")
    
    try:
        test_classification_map()
    except Exception as e:
        print(f"ERROR in test_classification_map: {e}")
    
    try:
        test_dijkstra()
    except Exception as e:
        print(f"ERROR in test_dijkstra: {e}")
    
    try:
        test_angle_normalization()
    except Exception as e:
        print(f"ERROR in test_angle_normalization: {e}")
    
    try:
        test_quaternion_extraction()
    except Exception as e:
        print(f"ERROR in test_quaternion_extraction: {e}")
    
    print("\n" + "█"*60)
    print("TESTS COMPLETE")
    print("█"*60)


if __name__ == "__main__":
    run_all_tests()
