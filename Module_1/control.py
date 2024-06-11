#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import numpy as np

# Nodo de Controlador
class NodoControlador(Node):
    def __init__(self, posiciones_objetivo):
        super().__init__('nodo_controlador')
        self.publicador_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.suscriptor_pose = self.create_subscription(Pose2D, 'pose', self.actualizar_control, 10)

        # Generamos la lista de las ubicaciones a recorrer
        self.posiciones_objetivo = posiciones_objetivo
        self.indice_objetivo = 0

        # Establecer la primera posición objetivo
        self.objetivo_x, self.objetivo_y = self.posiciones_objetivo[self.indice_objetivo]
        self.get_logger().info(f'Iniciando, moviéndose al primer objetivo: {self.posiciones_objetivo[self.indice_objetivo]}')

        # Constantes de control proporcional
        self.Kp_lineal = 0.1  # Reducida para disminuir la velocidad
        self.Kp_angular = 0.5 # Reducida para disminuir la velocidad

        # Velocidades máximas
        self.velocidad_lineal_maxima = 0.2  # Velocidad lineal máxima
        self.velocidad_angular_maxima = 0.1 # Velocidad angular máxima
        
        # Mensaje de comando de velocidad
        self.comando_velocidad = Twist()

        # Tolerancias para considerar alcanzado un objetivo
        self.tolerancia_lineal = 0.2   # Unidad en metros
        self.tolerancia_angular = 0.2  # Unidad en radianes

        # Temporizador para actualizar el control
        periodo_temporizador = 0.1  # segundos
        self.temporizador = self.create_timer(periodo_temporizador, self.bucle_control)

        # Estado del control
        self.objetivo_alcanzado = False
        self.mensaje_pose = Pose2D()
    
    def bucle_control(self):
        if not self.objetivo_alcanzado:
            self.actualizar_control(self.mensaje_pose)

    def actualizar_control(self, mensaje_pose):
        self.mensaje_pose = mensaje_pose  # Actualizar el mensaje de pose con el último recibido
        
        # Cálculo del error de distancia y ángulo
        error_distancia = np.sqrt((self.objetivo_x - mensaje_pose.x) ** 2 + (self.objetivo_y - mensaje_pose.y) ** 2)
        error_ángulo = np.arctan2(self.objetivo_y - mensaje_pose.y, self.objetivo_x - mensaje_pose.x) - mensaje_pose.theta
        error_ángulo = np.arctan2(np.sin(error_ángulo), np.cos(error_ángulo))  # Normalizar el ángulo

        # Registrar los errores para depuración
        self.get_logger().info(f'Posición actual: x={mensaje_pose.x:.2f}, y={mensaje_pose.y:.2f}, theta={mensaje_pose.theta:.2f}')
        self.get_logger().info(f'Posición objetivo: x={self.objetivo_x:.2f}, y={self.objetivo_y:.2f}')
        self.get_logger().info(f'Error de distancia: {error_distancia:.2f} m, error de ángulo: {error_ángulo:.2f} rad')

        # Comprobar si se ha alcanzado el objetivo
        if error_distancia < self.tolerancia_lineal:
            self.objetivo_alcanzado = True
            self.comando_velocidad.linear.x = 0.0
            self.comando_velocidad.angular.z = 0.0
            self.publicador_cmd_vel.publish(self.comando_velocidad)
            self.get_logger().info(f'Objetivo {self.indice_objetivo + 1} alcanzado: {self.posiciones_objetivo[self.indice_objetivo]}')

            if self.indice_objetivo < len(self.posiciones_objetivo) - 1:
                self.indice_objetivo += 1
                self.objetivo_x, self.objetivo_y = self.posiciones_objetivo[self.indice_objetivo]
                self.objetivo_alcanzado = False
                self.get_logger().info(f'Moviéndose al siguiente objetivo: {self.posiciones_objetivo[self.indice_objetivo]}')
            else:
                self.get_logger().info('Todos los objetivos han sido alcanzados. Deteniendo el robot.')
                self.temporizador.cancelar()  # Detener el temporizador si todos los objetivos han sido alcanzados

        else:
            # Asignar comandos de velocidad basados en el control proporcional
            velocidad_lineal = self.Kp_lineal * error_distancia
            velocidad_angular = self.Kp_angular * error_ángulo

            # Limitar la velocidad para evitar que sea demasiado rápida
            self.comando_velocidad.linear.x = max(min(velocidad_lineal, self.velocidad_lineal_maxima), -self.velocidad_lineal_maxima)
            self.comando_velocidad.angular.z = max(min(velocidad_angular, self.velocidad_angular_maxima), -self.velocidad_angular_maxima)
        
            # Publicar comando de velocidad
            self.publicador_cmd_vel.publish(self.comando_velocidad)
    

def main(args=None):
    rclpy.init(args=args)
    posiciones_objetivo = [(5.0, 0.0), (-2.0, 0.0), (0.0, 3.0), (0.0, -3.0), (5.0, 5.0), (0.0, 0.0)]
    nodo_controlador = NodoControlador(posiciones_objetivo)

    rclpy.spin(nodo_controlador)
    nodo_controlador.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
