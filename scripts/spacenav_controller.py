#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

FACTOR_LIN = 100
FACTOR_ROT = 0

class SpacenavCntroller(Node):
    def __init__(self):
        super().__init__('spacenav_controller')

        # Suscriptor al SpaceMouse
        self.subscription = self.create_subscription(
            Joy,
            '/spacenav/joy',
            self.joy_callback,
            10
        )

        # Publicar al 2 cript
        self.publisher = self.create_publisher(Twist, 'twist', 10)
        self.button_publisher = self.create_publisher(Int8, 'buttons', 10)

        # Posición integrada y última velocidad leída
        #self.position = [0.0, 0.0, 0.0]
        #self.last_velocity = [0.0, 0.0, 0.0]
        self.dt = 0.01  # 10 Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.joy=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.buttons=[0,0]


    def joy_callback(self, msg):
        self.joy = msg.axes
        self.buttons = msg.buttons
        # self.get_logger().debug(f'Velocidad recibida: {self.last_velocity}')

    def timer_callback(self):
        # Integrar velocidad -> nueva posición
        # for i in range(3):
        #    self.position[i] += self.last_velocity[i] * self.dt

        # Publicar posición como coordenadas Twist
        
        msg = Twist()
        msg.linear.x = -self.joy[0] * FACTOR_LIN 
        msg.linear.y = self.joy[1] * FACTOR_LIN 
        msg.linear.z = -self.joy[2] * FACTOR_LIN 
        msg.angular.x = -self.joy[3] * FACTOR_ROT
        msg.angular.y = self.joy[4] * FACTOR_ROT 
        msg.angular.z = -self.joy[5] * FACTOR_ROT  

        msg_buttons = Int8()

        if self.buttons[0] == 1:
            self.get_logger().info("Botón presionado, abriendo pinza")
            msg_buttons.data = 1
        elif self.buttons[1] == 1:
            self.get_logger().info("Botón presionado, cerrando pinza")
            msg_buttons.data = -1
            
        self.button_publisher.publish(msg_buttons)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Posición publicada: {self.joy}')

def main(args=None):
    rclpy.init(args=args)
    node = SpacenavCntroller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()