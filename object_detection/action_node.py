#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String
import time
import math


class YouBotArmTest(Node):

    def __init__(self):
        super().__init__('youbot_arm_test')

        # --- Subscriber action ---
        self.sub_act = self.create_subscription(String,'/action', self.action_callback, 10)


        # --- Publishers bras ---
        self.pubs = [
            self.create_publisher(Float64, '/youbot/arm/joint0_cmd', 10),
            self.create_publisher(Float64, '/youbot/arm/joint1_cmd', 10),
            self.create_publisher(Float64, '/youbot/arm/joint2_cmd', 10),
            self.create_publisher(Float64, '/youbot/arm/joint3_cmd', 10),
            self.create_publisher(Float64, '/youbot/arm/joint4_cmd', 10),
        ]

        # --- Publishers pince ---
        self.gripper_left  = self.create_publisher(Float64, '/youbot/arm/gripper1_cmd', 10)
        self.gripper_right = self.create_publisher(Float64, '/youbot/arm/gripper2_cmd', 10)

        self.timer = self.create_timer(0.05, self.move_sequence)

        self.goal_reached = False
        self.last_action = ''
        self.current_action = ''
        self.get_logger().info("YouBot arm test started")

    def action_callback(self, msg):
        message = msg.data
        self.last_action = self.current_action
        self.current_action = message


    def send_gripper(self, value):
        """
        value == 1  → ouvre
        value == -1 → ferme
        """
        m1 = Float64()
        m2 = Float64()
        if value == 1 :
            m1.data = 0.02
            m2.data = -0.04
        elif value == -1:
            m1.data = 0.0
            m2.data = 0.0

        self.gripper_left.publish(m1)
        self.gripper_right.publish(m2)


    def move_sequence(self):
        if self.current_action == 'grasp_object':

            # Positions actuelles internes
            positions = [0., 0., 0., 0., 0.]

            # Position cible
            goal = [
                0.0,
                -1.2,
                -1.0,
                -1.0,
                0.0
            ]

            dt = 0.05

            # --- pince OUVERTE pendant le mouvement ---
            self.send_gripper(1)

            while True:

                for i in range(len(goal)):
                    if positions[i] < goal[i]:
                        positions[i] += dt
                    elif positions[i] > goal[i]:
                        positions[i] -= dt

                # publier les positions
                for pub, pos in zip(self.pubs, positions):
                    msg = Float64()
                    msg.data = pos
                    pub.publish(msg)

                time.sleep(0.1)

                # test d'arrêt
                if all(abs(p - g) < 0.03 for p, g in zip(positions, goal)):
                    break

            # --- pince FERMÉE une fois arrivé ---
            self.get_logger().info("Goal reached → closing gripper")
            self.send_gripper(-1)

            time.sleep(1)
            self.get_logger().info("Gripper closed → moving position")
            goal = [0.0, 0.0, 0.0, 0.0, 0.0]
            while True:

                for i in range(len(goal)):
                    if positions[i] < goal[i]:
                        positions[i] += dt
                    elif positions[i] > goal[i]:
                        positions[i] -= dt

                # publier les positions
                for pub, pos in zip(self.pubs, positions):
                    msg = Float64()
                    msg.data = pos
                    pub.publish(msg)

                time.sleep(0.1)

                # test d'arrêt
                if all(abs(p - g) < 0.03 for p, g in zip(positions, goal)):
                    break

            # on arrête la timer
            self.get_logger().info("Grasp success")
            self.timer.cancel()
        



def main(args=None):
    rclpy.init(args=args)
    node = YouBotArmTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
