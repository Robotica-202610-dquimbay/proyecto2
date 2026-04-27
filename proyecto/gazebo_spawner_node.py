#!/usr/bin/env python3
"""
Gazebo Spawner Node
Reads scene file and spawns obstacles in Gazebo.
Also publishes waypoints as visual markers for RViz.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import os
import time
from planning.scene_loader import load_scene

# Try to import Gazebo services (optional)
try:
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity
    GAZEBO_AVAILABLE = True
except ImportError:
    GAZEBO_AVAILABLE = False


class GazeboSpawnerNode(Node):
    def __init__(self):
        super().__init__('gazebo_spawner')
        
        self.gazebo_ready = False
        self.spawn_client = None
        
        # Marker publisher for waypoints (always available)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        if GAZEBO_AVAILABLE:
            self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
            self.get_logger().info('Checking for Gazebo spawn service...')
            
            # Wait up to 3 seconds for Gazebo
            start_time = time.time()
            while time.time() - start_time < 3.0:
                if self.spawn_client.wait_for_service(timeout_sec=0.5):
                    self.gazebo_ready = True
                    self.get_logger().info('✓ Connected to Gazebo')
                    break
            
            if not self.gazebo_ready:
                self.get_logger().warn('✗ Gazebo not available - will only publish markers')
        else:
            self.get_logger().warn('✗ gazebo_msgs not installed - will only publish markers')
        
        # Get scene file from parameter or default
        self.declare_parameter('scene_file', 'data/Escena-Problema1.txt')
        scene_file = self.get_parameter('scene_file').value
        
        # Make path absolute if needed
        if not os.path.isabs(scene_file):
            base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            scene_file = os.path.join(base_path, scene_file)
        
        self.get_logger().info(f'Loading scene: {scene_file}')
        
        try:
            self.scene = load_scene(scene_file)
            if self.gazebo_ready:
                self.spawn_obstacles()
            self.publish_waypoints()
        except Exception as e:
            self.get_logger().error(f'Error loading scene: {e}')
            raise
    
    def spawn_obstacles(self):
        """Spawn all obstacles from scene file in Gazebo."""
        if not self.gazebo_ready:
            return
        
        for i, obs in enumerate(self.scene['obstacles']):
            x_min, y_min, x_max, y_max = obs
            
            # Calculate center and dimensions
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
            width = x_max - x_min
            length = y_max - y_min
            height = 0.5  # Fixed obstacle height
            
            # Create URDF for box obstacle
            urdf = self._create_box_urdf(width, length, height)
            
            # Create pose
            pose = Pose()
            pose.position.x = center_x
            pose.position.y = center_y
            pose.position.z = height / 2  # Center vertically
            pose.orientation.w = 1.0
            
            # Spawn in Gazebo
            from gazebo_msgs.srv import SpawnEntity
            req = SpawnEntity.Request()
            req.name = f'obstacle_{i}'
            req.xml = urdf
            req.initial_pose = pose
            
            try:
                future = self.spawn_client.call_async(req)
                future.add_done_callback(lambda f: self._spawn_callback(i, f))
            except Exception as e:
                self.get_logger().warn(f'Failed to spawn obstacle {i}: {e}')
    
    def _spawn_callback(self, obs_id, future):
        """Callback when obstacle spawn completes."""
        try:
            result = future.result()
            if result.success:
                x_min, y_min, x_max, y_max = self.scene['obstacles'][obs_id]
                self.get_logger().info(
                    f'✓ Spawned obstacle {obs_id} at '
                    f'({(x_min+x_max)/2:.2f}, {(y_min+y_max)/2:.2f})'
                )
            else:
                self.get_logger().warn(f'Failed to spawn obstacle {obs_id}: {result.status_message}')
        except Exception as e:
            self.get_logger().error(f'Spawn error for obstacle {obs_id}: {e}')
    
    def publish_waypoints(self):
        """Publish start and goal as markers for RViz visualization."""
        markers = MarkerArray()
        
        # Start position (green sphere)
        start_marker = Marker()
        start_marker.header.frame_id = 'map'
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.id = 1000
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = float(self.scene['q0'][0])
        start_marker.pose.position.y = float(self.scene['q0'][1])
        start_marker.pose.position.z = 0.2
        start_marker.pose.orientation.w = 1.0
        start_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
        start_marker.color.g = 1.0  # Green
        start_marker.color.a = 0.8
        start_marker.text = "START"
        markers.markers.append(start_marker)
        
        # Goal position (red sphere)
        goal_marker = Marker()
        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.id = 1001
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(self.scene['qf'][0])
        goal_marker.pose.position.y = float(self.scene['qf'][1])
        goal_marker.pose.position.z = 0.2
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
        goal_marker.color.r = 1.0  # Red
        goal_marker.color.a = 0.8
        goal_marker.text = "GOAL"
        markers.markers.append(goal_marker)
        
        self.marker_pub.publish(markers)
        self.get_logger().info(
            f'✓ Published markers: START ({self.scene["q0"][0]:.2f}, {self.scene["q0"][1]:.2f}), '
            f'GOAL ({self.scene["qf"][0]:.2f}, {self.scene["qf"][1]:.2f})'
        )
    
    @staticmethod
    def _create_box_urdf(width: float, length: float, height: float) -> str:
        """Generate a simple box URDF string."""
        return f'''<?xml version="1.0" ?>
<robot name="obstacle">
    <link name="link">
        <collision>
            <geometry>
                <box size="{width} {length} {height}"/>
            </geometry>
        </collision>
        <visual>
            <geometry>
                <box size="{width} {length} {height}"/>
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1.0"/>
            </material>
        </visual>
        <inertial>
            <mass value="100"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
    </link>
</robot>'''


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
