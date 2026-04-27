#!/usr/bin/env python3
"""
Simple test script to verify path planning works.
No ROS2 required - just pure Python.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from proyecto.planning.scene_loader import load_scene
from proyecto.planning.cspace import build_c_obstacles, build_classification_map
from proyecto.planning.dijkstra import compute_full_path

def test_scene(scene_num):
    """Test loading and planning for a specific scene."""
    
    data_dir = Path(__file__).parent / "data"
    scene_file = data_dir / f"Escena-Problema{scene_num}.txt"
    
    print(f"\n{'='*60}")
    print(f"Testing Scene {scene_num}")
    print(f"{'='*60}")
    
    # Load scene
    print(f"\n1. Loading scene from {scene_file}...")
    if not scene_file.exists():
        print(f"   ERROR: File not found!")
        return False
    
    try:
        scene = load_scene(str(scene_file))
        print(f"   ✓ Loaded: {scene['width']}x{scene['height']}m")
        print(f"   ✓ Start: {scene['q0']}")
        print(f"   ✓ Goal: {scene['qf']}")
        print(f"   ✓ Obstacles: {len(scene['obstacles'])}")
    except Exception as e:
        print(f"   ERROR: {e}")
        return False
    
    # Build C-space
    print(f"\n2. Building C-space...")
    try:
        c_obs = build_c_obstacles(scene)
        print(f"   ✓ C-obstacles: {len(c_obs)}")
    except Exception as e:
        print(f"   ERROR: {e}")
        return False
    
    # Classification map
    print(f"\n3. Building classification map...")
    try:
        cmap = build_classification_map(scene, c_obs)
        free = sum(1 for v in cmap.values() if v == "libre")
        semi = sum(1 for v in cmap.values() if v == "semi-libre")
        occupied = sum(1 for v in cmap.values() if v == "ocupada")
        print(f"   ✓ Cells: {len(cmap)} (free={free}, semi={semi}, occupied={occupied})")
    except Exception as e:
        print(f"   ERROR: {e}")
        return False
    
    # Dijkstra
    print(f"\n4. Computing path with Dijkstra...")
    try:
        waypoints = compute_full_path(scene, cmap)
        if waypoints is None:
            print(f"   ERROR: No path found!")
            return False
        print(f"   ✓ Path found: {len(waypoints)} waypoints")
        for i, (x, y) in enumerate(waypoints[:3]):
            print(f"     wp{i}: ({x:.3f}, {y:.3f})")
        if len(waypoints) > 3:
            print(f"     ... and {len(waypoints)-3} more")
    except Exception as e:
        print(f"   ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    print(f"\n✅ Scene {scene_num} SUCCESS\n")
    return True

if __name__ == "__main__":
    print("\nSimple Path Planning Test")
    print("Testing without ROS2\n")
    
    # Test scene 1
    success = test_scene(1)
    
    if success:
        print("="*60)
        print("✅ ALL TESTS PASS - Ready for ROS2!")
        print("="*60)
        sys.exit(0)
    else:
        print("="*60)
        print("❌ TEST FAILED - Check errors above")
        print("="*60)
        sys.exit(1)
