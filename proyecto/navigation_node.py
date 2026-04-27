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
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Correct quaternion to 2D angle extraction for Z-axis rotation
        # For a quaternion (qx, qy, qz, qw) representing Z-axis rotation:
        # theta = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Standard quaternion to 2D angle (Z-axis yaw)
        self.current_theta = math.atan2(2.0 * (qw * qz + qx * qy), 
                                        1.0 - 2.0 * (qy * qy + qz * qz))

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
        if self.last_scan is None:
            return

        # =====================================
        # COMANDO 5: CARGAR + PLANIFICAR
        # =====================================
        if self.comando_activo == 5:
            numero = self.parametros_comando[0]
            self.get_logger().info(f"Loading scene {numero}...")

            try:
                from ament_index_python.packages import get_package_share_directory

                package_path = get_package_share_directory('proyecto')
                ruta = os.path.join(package_path, 'data', f'Escena-Problema{numero}.txt')

                self.scene = load_scene(ruta)
                self.get_logger().info(f"Scene loaded: {len(self.scene['obstacles'])} obstacles")

            except Exception as e:
                self.get_logger().error(f"Error loading scene: {e}")
                self.comando_activo = None
                return

            self.get_logger().info("Planning path...")

            try:
                c_obs = build_c_obstacles(self.scene)
                cmap = build_classification_map(self.scene, c_obs)
                waypoints = compute_full_path(self.scene, cmap)

            except Exception as e:
                self.get_logger().error(f"Error planning: {e}")
                self.comando_activo = None
                return

            if waypoints is None:
                self.get_logger().error("No path found")
                self.comando_activo = None
                return

            self.path = waypoints
            self.indice_path = 0

            self.get_logger().info(f"Path generated: {len(self.path)} waypoints")

            self.comando_activo = 7
            return


        # =====================================
        # ESTADO 7: EJECUTAR PATH
        # =====================================
        elif self.comando_activo == 7:
            if self.indice_path >= len(self.path):
                self.get_logger().info("Trayectoria completada.")
                self.cmd_pub.publish(Twist())  # detener robot
                self.comando_activo = None
                return

            x_target, y_target = self.path[self.indice_path]

            dx = x_target - self.current_x
            dy = y_target - self.current_y
            dist = math.sqrt(dx**2 + dy**2)

            # Waypoint reached (best effort: 20cm tolerance)
            if dist < 0.2:
                self.indice_path += 1
                return

            # Calculate desired heading to waypoint
            desired_theta = math.atan2(dy, dx)
            
            # Normalize angle error to [-π, π]
            angle_error = desired_theta - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            cmd = Twist()
            
            # Drive while making best effort to align (loose tolerance)
            if abs(angle_error) < 0.3:  # ~17 degrees - allows driving while correcting
                # Drive toward waypoint
                cmd.linear.x = min(0.5 * dist, 0.5)  # Cap at 0.5 m/s
            else:
                # Rotate to face waypoint first (only if misaligned)
                cmd.angular.z = max(min(angle_error, 0.5), -0.5)  # Proportional rotation, capped

            self.cmd_pub.publish(cmd)
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