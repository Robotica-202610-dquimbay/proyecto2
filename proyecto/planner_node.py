import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import os
import math
from ament_index_python.packages import get_package_share_directory

from .planning.scene_loader import load_scene
from .planning.cspace import build_c_obstacles, build_classification_map
from .planning.dijkstra import compute_full_path


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.publisher_ = self.create_publisher(Path, '/plan', 10)

        self.get_logger().info("Planner listo.")

        self.generar_plan(1)


    def generar_plan(self, numero_escena):

        package_path = get_package_share_directory('proyecto')
        ruta = os.path.join(package_path, 'data', f'Escena-Problema{numero_escena}.txt')

        self.get_logger().info(f"Cargando: {ruta}")

        try:
            scene = load_scene(ruta)
        except Exception as e:
            self.get_logger().error(f"Error cargando escena: {e}")
            return

        # =========================
        # PLANIFICACIÓN
        # =========================
        try:
            c_obs = build_c_obstacles(scene)
            cmap = build_classification_map(scene, c_obs)
            waypoints = compute_full_path(scene, cmap)
        except Exception as e:
            self.get_logger().error(f"Error en planificación: {e}")
            return

        if waypoints is None:
            self.get_logger().error("No se encontró camino")
            return

        # =========================
        # CREAR MENSAJE PATH
        # =========================
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = msg.header.stamp

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Compute orientation: direction from previous to current waypoint
            # For first point, use zero orientation
            if i == 0:
                theta = 0.0
            else:
                x_prev, y_prev = waypoints[i-1]
                theta = math.atan2(y - y_prev, x - x_prev)
            
            # Convert angle to quaternion (Z-axis rotation only)
            # q = [sin(θ/2), 0, 0, cos(θ/2)] for Z rotation, but standard is [0, 0, sin(θ/2), cos(θ/2)]
            half_theta = theta / 2.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(half_theta)
            pose.pose.orientation.w = math.cos(half_theta)

            msg.poses.append(pose)

        # =========================
        # PUBLICAR
        # =========================
        self.publisher_.publish(msg)

        self.get_logger().info(f"Plan publicado con {len(waypoints)} puntos")