import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import threading
import os

from .logic.lidar import obtener_distancia_angulo, obtener_distancias_rango
from .logic.movement import calcular_rotacion, calcular_movimiento_relativo
from nav_msgs.msg import Path

# Utils
from proyecto.planning.scene_loader import load_scene
from proyecto.planning.cspace import build_c_obstacles, build_classification_map
from proyecto.planning.dijkstra import compute_full_path
from proyecto.planning.path_generator import waypoints_to_configurations
from proyecto.execution_utils import (
    save_path_with_results,
    estimate_position_from_odometry,
    compute_qact_from_lidar,
    normalize_angle_deg
)

from ament_index_python.packages import get_package_share_directory


class NavigationNode(Node):
    def __init__(self):
        super().__init__('student_navigation')
        
        # Suscriptores
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan_raw', self.lidar_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)  # Suscriptor para el planificador global (opcional)
        
        # Publicador
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Variables de estado interno
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_scan = None
        self.estado = "IDLE"
        
        # Memorias de estado para los movimientos relativos
        self.target_theta_relativo = None
        self.pose_inicial_relativa = None 
        
        # Variable para almacenar el texto crudo de la escena
        self.texto_escena = ""
        
        # Variables para comunicar el menú interactivo con el control loop
        self.comando_activo = None
        self.parametros_comando = []

        # Variables para el planificador
        self.plan = []
        self.plan_index = 0
        
        # ===== NEW: STATE MACHINE FOR OPEN-LOOP EXECUTION =====
        self.state = "IDLE"  # IDLE | PLANIFICANDO | EJECUTANDO | FINALIZANDO
        self.configurations = []  # Full (x, y, theta) path
        self.config_index = 0  # Current configuration index
        self.scene = None  # Loaded scene
        self.path_file = None  # Path output file
        
        # Initial position for final estimates
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        
        # ===== ODOMETRY FILTERING =====
        # Track last valid position to reject noise spikes
        self.last_valid_x = None  # None = first reading, accept it
        self.last_valid_y = None
        self.max_jump = 0.5  # Reject jumps > 0.5m as noise
        
        # Final measurements
        self.d_front_final = None
        self.d_right_final = None
        
        # Timer (El loop de control corre 10 veces por segundo)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de Navegación Estudiantil Iniciado.")

        # Iniciamos el menú en un hilo separado para NO bloquear a ROS2
        self.hilo_menu = threading.Thread(target=self.menu_interactivo, daemon=True)
        self.hilo_menu.start()

    # =======================================================
    # CALLBACKS DE ROS2
    # =======================================================
    def odom_callback(self, msg):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        
        # Filter out noise spikes (skip check on first reading)
        if self.last_valid_x is not None:
            jump = math.sqrt((new_x - self.last_valid_x)**2 + (new_y - self.last_valid_y)**2)
            if jump > self.max_jump:
                # Ignore this noisy reading, keep last valid position
                return
        
        # Update valid position
        self.current_x = new_x
        self.current_y = new_y
        self.last_valid_x = new_x
        self.last_valid_y = new_y
        
        # Correct quaternion to 2D angle extraction for Z-axis rotation
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Standard quaternion to 2D angle (Z-axis yaw)
        theta_raw = math.atan2(2.0 * (qw * qz + qx * qy), 
                               1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Accept all angle readings - let calcular_rotacion() handle normalization
        # It uses atan2(sin, cos) which is robust to ±π wrapping
        self.current_theta = theta_raw
        self.last_valid_theta = theta_raw

    def lidar_callback(self, msg):
        self.last_scan = msg

    # Callback para el planificado
    def plan_callback(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.plan_index = 0
        self.get_logger().info(f"Plan recibido con {len(self.plan)} puntos")

    # =======================================================
    # WRAPPERS PARA LOS ESTUDIANTES
    # =======================================================
    def leer_distancia_en_angulo(self, grados):
        """Retorna la distancia (en metros) de un ángulo específico del Lidar."""
        return obtener_distancia_angulo(self.last_scan, math.radians(grados))

    def leer_distancia_direccion(self, direccion):
        """
        Retorna la distancia en una dirección cardinal específica:
        'frente', 'atras', 'izquierda', 'derecha'.
        """
        mapa_direcciones = {
            'frente': 0.0,
            'izquierda': 90.0,
            'derecha': 270.0,
            'atras': 180.0
        }
        
        direccion = direccion.lower()
        if direccion in mapa_direcciones:
            return self.leer_distancia_en_angulo(mapa_direcciones[direccion])
        else:
            self.get_logger().error(f"Dirección '{direccion}' no válida.")
            return float('inf')

    def leer_distancias_en_rango(self, grados_min, grados_max):
        """Retorna una lista con todas las detecciones en un rango visual."""
        return obtener_distancias_rango(self.last_scan, grados_min, grados_max)

    def rotar_relativo(self, grados_relativos, tolerancia=0.05):
        """
        Gira el robot de forma relativa (ej: 90 grados a la izquierda).
        Retorna True si la maniobra ya finalizó.
        """
        if self.target_theta_relativo is None:
            # Capturamos el ángulo base al arrancar la maniobra
            self.target_theta_relativo = self.current_theta + math.radians(grados_relativos)
            
        cmd, completado = calcular_rotacion(self.current_theta, self.target_theta_relativo, tolerancia=tolerancia)
        self.cmd_pub.publish(cmd)
        
        if completado:
            # Reseteamos la meta para que el robot pueda volver a girar en el futuro
            self.target_theta_relativo = None 
            
        return completado

    def mover_relativo(self, distancia_x_metros, distancia_y_metros, cono_vision=30, dist_segura=0.3, vel_lineal=0.4):
        """
        Desplazamiento usando Cinemática de Tiempo (Dead Reckoning).
        Evalúa el obstáculo según la dirección (Adelante, Atrás, Lados).
        """
        # 1. Inicializamos el cronómetro al arrancar la orden
        if self.pose_inicial_relativa is None:
            self.pose_inicial_relativa = True # Lo usamos como bandera de inicio
            self.tiempo_maniobra = 0.0        # Cronómetro en segundos

         # 2. Determinar hacia dónde nos vamos a mover para vigilar ESA dirección
        if abs(distancia_x_metros) >= abs(distancia_y_metros):
            if distancia_x_metros >= 0:
                # Movimiento hacia el FRENTE
                cono_despejado = self.leer_distancias_en_rango(-cono_vision, cono_vision)
            else:
                # Movimiento hacia ATRÁS - use negative angles that wrap properly
                # leer_distancias_en_rango now handles wraparound at ±180°
                cono_despejado = self.leer_distancias_en_rango(180-cono_vision, 180+cono_vision)
        else:
            if distancia_y_metros > 0:
                # Movimiento hacia la IZQUIERDA
                cono_despejado = self.leer_distancias_en_rango(90-cono_vision, 90+cono_vision)
            else:
                # Movimiento hacia la DERECHA
                cono_despejado = self.leer_distancias_en_rango(270-cono_vision, 270+cono_vision)

        # 3. Calcular movimiento basado en tiempo
        cmd, estado = calcular_movimiento_relativo(
            self.tiempo_maniobra,
            distancia_x_metros, distancia_y_metros,
            cono_despejado,
            dist_segura=dist_segura,
            vel_lineal=vel_lineal
        )
        self.cmd_pub.publish(cmd)
        
        # 4. Sumamos el tiempo de este ciclo (nuestro timer general corre a 0.1s)
        self.tiempo_maniobra += 0.1
        
        # 5. Si terminamos o nos bloqueamos, limpiamos todo para el próximo comando
        if estado in ['COMPLETADO', 'BLOQUEADO']:
            self.pose_inicial_relativa = None
            self.tiempo_maniobra = 0.0
            
        return estado

    def cargar_escena(self, numero_escena):
        """
        Lee el archivo de la escena indicada y guarda el texto en self.texto_escena.
        """
        # Calculamos la ruta subiendo un nivel de directorio desde este archivo hasta la carpeta 'data'
        package_path = get_package_share_directory('proyecto')
        ruta_archivo = os.path.join(package_path, '..', 'data', f'Escena-Problema{numero_escena}.txt')
        
        try:
            with open(ruta_archivo, 'r', encoding='utf-8') as archivo:
                self.texto_escena = archivo.read()
            self.get_logger().info(f"Escena {numero_escena} cargada correctamente.")
            # Opcional: imprimir un pedacito para confirmar
            print(f"\n--- Contenido Escena {numero_escena} ---\n{self.texto_escena}\n---------------------------")
        except FileNotFoundError:
            self.get_logger().error(f"No se encontró el archivo: {ruta_archivo}")
        except Exception as e:
            self.get_logger().error(f"Error al leer la escena: {e}")

    # =======================================================
    # BUCLE PRINCIPAL (Área de trabajo del estudiante)
    # =======================================================
    def menu_interactivo(self):
        while rclpy.ok():
            if self.comando_activo is None and self.estado == "IDLE":

                print("\n" + "="*40)
                print("--- MENÚ DE NAVEGACIÓN ---")
                print("1. Leer distancia en un ángulo")
                print("2. Leer distancias en un rango")
                print("3. Rotar grados relativos")
                print("4. Mover relativo (X, Y)")
                print("5. Cargar escena y planificar")
                print("6. Leer distancia por dirección")
                print("0. Salir")
                print("="*40)

                try:
                    opcion = input("Elige una opción: ")

                    # =====================
                    # LIDAR
                    # =====================
                    if opcion == '1':
                        angulo = float(input("Ángulo (grados): "))
                        self.parametros_comando = [angulo]
                        self.comando_activo = 1

                    elif opcion == '2':
                        ang_min = float(input("Ángulo mínimo: "))
                        ang_max = float(input("Ángulo máximo: "))
                        self.parametros_comando = [ang_min, ang_max]
                        self.comando_activo = 2

                    elif opcion == '6':
                        direccion = input("Dirección (frente/atras/izquierda/derecha): ").lower()
                        if direccion in ['frente', 'atras', 'izquierda', 'derecha']:
                            self.parametros_comando = [direccion]
                            self.comando_activo = 6
                        else:
                            print("Dirección inválida.")

                    # =====================
                    # MOVIMIENTO MANUAL
                    # =====================
                    elif opcion == '3':
                        grados = float(input("Grados a rotar: "))
                        self.parametros_comando = [grados]
                        self.comando_activo = 3

                    elif opcion == '4':
                        x = float(input("Movimiento en X (m): "))
                        y = float(input("Movimiento en Y (m): "))
                        self.parametros_comando = [x, y]
                        self.comando_activo = 4

                    # =====================
                    # PLANIFICACIÓN (LO NUEVO)
                    # =====================
                    elif opcion == '5':
                        numero = int(input("Número de escena (1-6): "))
                        self.parametros_comando = [numero]
                        self.comando_activo = 5

                    elif opcion == '0':
                        print("Saliendo...")
                        break

                    else:
                        print("Opción inválida.")

                except ValueError:
                    print("Entrada inválida.")

    # =======================================================
    # BUCLE PRINCIPAL DE CONTROL
    # =======================================================
    def control_loop(self):
        """
        Main control loop implementing open-loop path execution.
        
        State machine:
        IDLE        -> wait for command
        PLANIFICANDO -> load scene, plan path, convert to configurations
        EJECUTANDO   -> execute configurations sequentially (no corrections)
        FINALIZANDO  -> read final LiDAR, compute qact, save results
        """
        
        if self.last_scan is None:
            return
        
        # ===== COMMAND 5: LOAD + PLAN =====
        if self.comando_activo == 5 and self.state == "IDLE":
            numero = self.parametros_comando[0]
            self.state = "PLANIFICANDO"
            self.get_logger().info(f"PLANIFICANDO: Loading scene {numero}...")
            
            try:
                package_path = get_package_share_directory('proyecto')
                ruta = os.path.join(package_path, 'data', f'Escena-Problema{numero}.txt')
                
                self.scene = load_scene(ruta)
                self.get_logger().info(f"  Scene: {len(self.scene['obstacles'])} obstacles")
                
                # Build C-space and plan
                c_obs = build_c_obstacles(self.scene)
                cmap = build_classification_map(self.scene, c_obs)
                waypoints = compute_full_path(self.scene, cmap)
                
                if waypoints is None:
                    raise Exception("No path found")
                
                # Get initial theta from scene
                theta0 = self.scene['q0'][2] if len(self.scene['q0']) > 2 else 0.0
                thetaf = self.scene['qf'][2] if len(self.scene['qf']) > 2 else 90.0
                
                # Convert waypoints to full configurations with headings
                self.configurations = waypoints_to_configurations(waypoints, theta0, thetaf)
                # Start at config_index=1 to skip q0 (we assume we're already at start position)
                # config[0]=q0, config[1]=qrot0-1 (first rotation), config[2]=q1, etc.
                self.config_index = 1
                
                self.get_logger().info(f"  ✓ Path generated: {len(self.configurations)} configurations")
                
                # Save path file
                self.path_file = os.path.join(package_path, f'path_Escena{numero}.txt')
                save_path_with_results(self.configurations, self.path_file)
                self.get_logger().info(f"  ✓ Path saved to {self.path_file}")
                
                # Store initial position for final estimates
                self.x0 = self.current_x
                self.y0 = self.current_y
                self.theta0 = self.current_theta
                
                # Transition to execution
                self.state = "EJECUTANDO"
                self.comando_activo = 7  # Trigger execution
                
            except Exception as e:
                self.get_logger().error(f"Error in PLANIFICANDO: {e}")
                self.state = "IDLE"
                self.comando_activo = None
            
            return
        
        # ===== STATE: EJECUTANDO (OPEN-LOOP EXECUTION) =====
        if self.state == "EJECUTANDO" and self.comando_activo == 7:
            
            # Check if all configurations executed
            if self.config_index >= len(self.configurations):
                self.get_logger().info("✓ EJECUTANDO: All configurations completed!")
                self.cmd_pub.publish(Twist())  # Stop robot
                self.state = "FINALIZANDO"
                return
            
            x_conf, y_conf, theta_conf = self.configurations[self.config_index]
            
            # Determine if this is a rotation or translation
            if self.config_index > 0:
                x_prev, y_prev, _ = self.configurations[self.config_index - 1]
                is_rotation = (abs(x_conf - x_prev) < 1e-6 and abs(y_conf - y_prev) < 1e-6)
            else:
                is_rotation = False
            
            if is_rotation:
                # ROTATION: Rotate in place to target theta
                target_rad = math.radians(theta_conf)
                cmd, completed = calcular_rotacion(self.current_theta, target_rad, tolerancia=0.05)
                
                self.get_logger().info(
                    f"  ⟲ GIRANDO [config {self.config_index}] -> {theta_conf:.1f}° | "
                    f"Actual: {math.degrees(self.current_theta):.1f}°"
                )
                self.cmd_pub.publish(cmd)
                
                if completed:
                    self.config_index += 1
            
            else:
                # TRANSLATION: Move forward in current heading
                dx = x_conf - self.current_x
                dy = y_conf - self.current_y
                dist = math.sqrt(dx**2 + dy**2)
                
                # Tolerance: 20cm
                if dist < 0.2:
                    self.get_logger().info(
                        f"  ✓ Config {self.config_index} reached ({x_conf:.2f}, {y_conf:.2f})"
                    )
                    self.config_index += 1
                    return
                
                # Adaptive velocity: scales with distance but bounded [v_min, v_max]
                # This slows down near target without infinite loops
                v_max = 0.3          # Max velocity (far from target)
                v_min = 0.15         # Min velocity (prevents stuck state)
                k = 0.5              # Proportional gain
                
                # Velocity = k * distance, clamped between v_min and v_max
                cmd = Twist()
                cmd.linear.x = max(v_min, min(v_max, k * dist))
                
                self.get_logger().info(
                    f"  ➜ AVANZANDO [config {self.config_index}] ({x_conf:.2f}, {y_conf:.2f}) | "
                    f"Actual: ({self.current_x:.2f}, {self.current_y:.2f}) | Dist: {dist:.2f}m | v={cmd.linear.x:.2f}m/s"
                )
                self.cmd_pub.publish(cmd)
            
            return
        
        # ===== STATE: FINALIZANDO (FINAL MEASUREMENTS) =====
        if self.state == "FINALIZANDO":
            
            # Read LiDAR measurements
            self.d_front_final = self.leer_distancia_direccion('frente')
            self.d_right_final = self.leer_distancia_direccion('derecha')
            
            # Get expected distances from scene
            d_front_expected = self.scene.get('d_frente', 0.8)
            d_right_expected = self.scene.get('d_derecha', 0.78)
            
            self.get_logger().info(
                f"✓ FINALIZANDO: Final measurements:\n"
                f"  d_front = {self.d_front_final:.3f}m (expected: {d_front_expected:.3f}m)\n"
                f"  d_right = {self.d_right_final:.3f}m (expected: {d_right_expected:.3f}m)"
            )
            
            # Compute estimates
            qf_theoretical = self.scene['qf']
            qf_est = estimate_position_from_odometry(self.x0, self.y0, self.theta0, self.configurations)
            
            qact = compute_qact_from_lidar(qf_est[0], qf_est[1], math.radians(qf_est[2]),
                                          self.d_front_final, self.d_right_final,
                                          d_front_expected, d_right_expected)
            
            self.get_logger().info(
                f"\n{'='*60}\n"
                f"RESULTADOS FINALES\n"
                f"{'='*60}\n"
                f"qf (teórico):  ({qf_theoretical[0]:.4f}, {qf_theoretical[1]:.4f}, {qf_theoretical[2]:.1f}°)\n"
                f"qf_est (odo):  ({qf_est[0]:.4f}, {qf_est[1]:.4f}, {qf_est[2]:.1f}°)\n"
                f"qact (sensor): ({qact[0]:.4f}, {qact[1]:.4f}, {qact[2]:.1f}°)\n"
                f"{'='*60}"
            )
            
            # Save results to file
            save_path_with_results(self.configurations, self.path_file, qf_est, qact)
            self.get_logger().info(f"✓ Results appended to {self.path_file}")
            
            # Reset state
            self.state = "IDLE"
            self.comando_activo = None
            self.cmd_pub.publish(Twist())
            
            return


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Detener motores forzosamente si se presiona Ctrl+C
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()