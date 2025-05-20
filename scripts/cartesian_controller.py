#!/usr/bin/env python3

import numpy as np
import PyKDL as kdl
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

GRIPPER_INCR = 0.01 # rad

class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.subscription_buttons = self.create_subscription(
            Int8,
            'buttons',
            self.buttons_callback,
            10
        )
        self.subscription_buttons
        self.subscription_twist = self.create_subscription(
            Twist,
            'twist',
            self.listener_callback,
            10)
        self.subscription_twist

        self.q = kdl.JntArray(5)
        
        self.q[0] = 0
        self.q[1] = 0
        self.q[2] = 0
        self.q[3] = 0
        self.q[4] = 0
        
        self.q_min = kdl.JntArray(5)
        self.q_max = kdl.JntArray(5)

        self.q_max[0] = 3.14
        self.q_min[0] = -3.14
        self.q_max[1] = 2
        self.q_min[1] = -2
        self.q_max[2] = 3.14
        self.q_min[2] = -3
        self.q_max[3] = 1.57
        self.q_min[3] = -1.5
        self.q_max[4] = 1.57
        self.q_min[4] = -1.57

        self.q_gripper = 0
        self.q_gripper_min = -3.14
        self.q_gripper_max = 3.14

        self.buttons = 0

        a, b, c, d, e  = 140, 106, 140, 100, 0

        self.chain = kdl.Chain()
        #                  A alpha D theta
        H_0_1 = kdl.Frame.DH(0,np.deg2rad(-90),a,0)
        H_1_2 = kdl.Frame.DH(b,0,0,np.deg2rad(-90))
        H_2_3 = kdl.Frame.DH(c,0,0,np.deg2rad(+90))
        H_3_4 = kdl.Frame.DH(0,np.deg2rad(+90),-e,np.deg2rad(+90))
        H_4_5 = kdl.Frame.DH(0,0,d,0)

        axis = kdl.Joint(kdl.Joint.RotZ)

        self.chain.addSegment(kdl.Segment(axis, H_0_1))
        self.chain.addSegment(kdl.Segment(axis, H_1_2))
        self.chain.addSegment(kdl.Segment(axis, H_2_3))
        self.chain.addSegment(kdl.Segment(axis, H_3_4))
        self.chain.addSegment(kdl.Segment(axis, H_4_5))

        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver_vel = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_NR(self.chain, self.fk_solver, self.ik_solver_vel,100, 1e-6)

    def buttons_callback(self, msg):
        self.buttons = msg.data

    def listener_callback(self, msg):
        # self.get_logger().info(f'Recibido objetivo: {msg}')
        #print("q actual:", [self.q[i] for i in range(self.q.rows())])

        tw = kdl.Twist()
        tw.vel = kdl.Vector(msg.linear.x, msg.linear.y, msg.linear.z)
        tw.rot = kdl.Vector(msg.angular.x, msg.angular.y, msg.angular.z)
        #print("Twist recibido:", tw)
        
        q_dot = kdl.JntArray(5)
        ret = self.ik_solver_vel.CartToJnt(self.q, tw, q_dot)

        if ret != 0:
            print(ret)
        # print([q_dot[i] for i in range(q_dot.rows())])

        dt = 0.01
        q_nueva = kdl.JntArray(5)
        en_rango = True

        #corregir limites articulares

        for i in range(5):
            q_temp = self.q[i] + q_dot[i] * dt

            if not (self.q_min[i] <= q_temp <= self.q_max[i]):
                en_rango = False
            else:
                q_nueva[i] = q_temp

            #self.q[i] += q_dot[i]

        if en_rango:
            self.q = q_nueva
            out = Float64MultiArray()
            out.data = [self.q[i] for i in range(self.q.rows())]

            if self.buttons == 1:
                temp = self.q_gripper + GRIPPER_INCR
            elif self.buttons == -1:
                temp = self.q_gripper - GRIPPER_INCR
            else:
                temp = self.q_gripper

            if self.q_gripper_min <= temp <= self.q_gripper_max:
                self.q_gripper = temp

            out.data.append(self.q_gripper)

            self.publisher_.publish(out)
            self.get_logger().info(f'out: {out}')
        else:
            self.get_logger().warn('Movimiento anulado: se exceden los límites articulares.') 

        # # Posición inicial
        # q_init = kdl.JntArray(5)
        # for i in range(5):
        #     q_init[i] = 0
        # # q_a = (0, 24.83, 37.83, 0, 27.34, 0)
        # # for i in range(5):
        # #     q[i] = np.deg2rad(q_a[i])


        # Solvers
        

        
        # # Usar objetivo recibido
        # x, y, z = self.objetivo
        # p_in = kdl.Frame(kdl.Vector(x, y, z))
        # q_result = kdl.JntArray(5)
        # ret = ik_solver.CartToJnt(q_init, p_in, q_result)

        # if ret >= 0:
        #     q_out_list = [q_result[i] for i in range(q_result.rows())]
        #     q_out_deg = np.degrees(q_out_list)

        #     # Publicar en el tópico
        #     msg = Float64MultiArray()
        #     msg.data = q_out_deg.tolist()
        #     self.publisher_.publish(msg)

        #     self.get_logger().info(f'Publicado en topic: {msg.data}')
        # else:
        #     self.get_logger().warn('Falló la resolución IK')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
