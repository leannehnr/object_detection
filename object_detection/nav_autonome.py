#!/usr/bin/env python3
import math
import re
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(q) -> float:
    # geometry_msgs/Quaternion -> yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_detection(s: str):
    """
    Accepte:
      "blue 0.01 0.17 0.025"
      "<blue> <0.01> <0.17> <0.025>"
    Retour: (type, lat, dist, z) ou None

    Convention:
      lat  : décalage latéral (m). lat>0 = objet à gauche (vu depuis le bras)
      dist : distance devant (m)
      z    : hauteur/optionnel
    """
    tokens = re.findall(r'[A-Za-z]+|[-+]?\d*\.\d+|[-+]?\d+', s)
    if len(tokens) < 4:
        return None
    obj_type = tokens[0].lower()
    if obj_type not in ("red", "blue"):
        return None
    lat = float(tokens[1])
    dist = float(tokens[2])
    z = float(tokens[3])
    return obj_type, lat, dist, z


class NavAutonomy(Node):
    def __init__(self):
        super().__init__("nav_autonomy_node")

        # ------------------ PARAMS (à ajuster)
        # Approche objet
        self.target_dist = 0.15
        self.dist_tol = 0.01
        self.lat_tol = 0.01

        self.k_vx = 1.2
        self.k_vy = 2.0
        self.k_wz = 1.5

        self.max_vx = 0.1
        self.max_vy = 0.1
        self.max_wz = 1.0

        # Exploration simple
        self.explore_vx = 0.05
        self.explore_wz = 0.35
        self.explore_turn_period = 6.0  # toutes les 6s, change de sens
        self.explore_turn_dir = 1.0

        # Navigation vers panier (odom)
        # (x, y, theta)
        self.basket_red = (-2.125, -2.225, 0.05)
        self.basket_blue = (-1.15, -2.225, 0.05)

        # Stop au panier
        self.basket_pos_tol = 0.15  # m

        # ------------------ ROS I/O
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_action = self.create_publisher(String, "/action", 10)

        # Ton topic de détection (mets le bon nom ici)
        self.sub_det = self.create_subscription(String, "/detected_object", self.on_detection, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.sub_action = self.create_subscription(String, "/action", self.on_action, 10)

        # ------------------ ÉTAT
        self.state = "SEARCH"   # SEARCH -> APPROACH -> GRASP -> GO_BASKET -> DONE
        self.det = None         # (type, lat, dist, z)
        self.last_det_time = 0.0
        self.target_type = None

        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.grasp_sent = False
        self.grasp_start_t = 0.0

        self.search_start_t = time.time()

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        self.get_logger().info("NavAutonomy started: SEARCH")

    # ------------------ Callbacks
    def on_detection(self, msg: String):
        p = parse_detection(msg.data)
        if p is None:
            return
        self.det = p
        self.last_det_time = time.time()

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        self.yaw = yaw_from_quat(q)
        self.have_odom = True

    def on_action(self, msg):
        message = msg.data
        if message == 'grasp_object': 
            self.state = 'GRASP'

    # ------------------ Utils
    def publish_cmd(self, vx: float, vy: float, wz: float):
        vx = max(-self.max_vx, min(self.max_vx, vx))
        vy = max(-self.max_vy, min(self.max_vy, vy))
        wz = max(-self.max_wz, min(self.max_wz, wz))
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = wz
        self.pub_cmd.publish(t)

    def stop(self):
        self.publish_cmd(0.0, 0.0, 0.0)

    def send_action(self, s: str):
        m = String()
        m.data = s
        self.pub_action.publish(m)

    # ------------------ Contrôles
    def control_approach(self, lat: float, dist: float):
        # erreurs
        e_d = dist - self.target_dist
        e_lat = lat

        # bearing approx: angle objet
        bearing = math.atan2(e_lat, max(dist, 1e-6))

        vx = self.k_vx * e_d
        vy = -self.k_vy * e_lat
        wz = -self.k_wz * bearing
        return vx, vy, wz

    def control_go_to(self, gx: float, gy: float, gth: float):
        """
        Go-to en odom:
        - on génère vx/vy dans le repère robot en fonction de la cible
        - wz pour tourner vers la cible (puis orientation finale)
        """
        dx = gx - self.x
        dy = gy - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        heading = math.atan2(dy, dx)
        e_yaw_to_target = wrap_pi(heading - self.yaw)

        k_lin = 0.7
        vx = k_lin * dist * math.cos(e_yaw_to_target)
        vy = k_lin * dist * math.sin(e_yaw_to_target)

        k_ang = 1.6
        wz = k_ang * e_yaw_to_target

        # proche : viser theta final
        if dist < 0.35:
            e_final = wrap_pi(gth - self.yaw)
            wz = 1.2 * e_final
            vx *= 0.5
            vy *= 0.5

        return vx, vy, wz, dist

    # ------------------ FSM loop
    def loop(self):
        now = time.time()
        det_recent = (self.det is not None) and ((now - self.last_det_time) < 0.4)
        self.get_logger().info(f"State : {self.state}")
        if self.state == "SEARCH":
            # si on voit un objet, verrouille le type et approche
            if det_recent:
                obj_type, lat, dist, z = self.det
                self.target_type = obj_type
                self.state = "APPROACH"
                self.get_logger().info(f"Detected {obj_type} -> APPROACH")
                return

            # exploration simple: avance + tourne, change de sens périodiquement
            t = now - self.search_start_t
            if t > self.explore_turn_period:
                self.explore_turn_dir *= -1.0
                self.search_start_t = now
            self.publish_cmd(self.explore_vx, 0.0, self.explore_wz * self.explore_turn_dir)
            return

        if self.state == "APPROACH":
            if not det_recent:
                self.state = "SEARCH"
                self.get_logger().info("Lost detection -> SEARCH")
                return

            obj_type, lat, dist, z = self.det

            # approche & centrage
            vx, vy, wz = self.control_approach(lat, dist)
            self.publish_cmd(vx, vy, wz)

            ready = (abs(dist - self.target_dist) < self.dist_tol) and (abs(lat) < self.lat_tol)
            if ready:
                self.stop()
                self.state = "GRASP"
                self.grasp_sent = False
                self.get_logger().info("Aligned at 14.5cm -> GRASP")
            return

        if self.state == "GRASP":
            self.stop()
            if not self.grasp_sent:
                self.send_action("grasp_object")
                self.grasp_sent = True
                self.grasp_start_t = now
                self.get_logger().info("Action sent: grasp_object")
                return

            # laisse le temps à ton bras de faire sa séquence
            if (now - self.grasp_start_t) > 4.0:
                if self.have_odom:
                    self.state = "GO_BASKET"
                    self.get_logger().info("Assuming grasp OK -> GO_BASKET")
                else:
                    self.state = "DONE"
                    self.get_logger().warning("No /odom -> DONE after grasp")
            return

        if self.state == "GO_BASKET":
            if not self.have_odom:
                self.stop()
                self.state = "DONE"
                return

            if self.target_type == "red":
                gx, gy, gth = self.basket_red
            else:
                gx, gy, gth = self.basket_blue

            vx, vy, wz, d = self.control_go_to(gx, gy, gth)
            self.publish_cmd(vx, vy, wz)

            if d < self.basket_pos_tol:
                self.stop()
                # tu pourras ajouter plus tard: self.send_action("drop_object")
                self.get_logger().info("Reached basket -> DONE (add drop action later)")
                self.state = "DONE"
            return

        if self.state == "DONE":
            self.stop()
            return


def main(args=None):
    rclpy.init(args=args)
    node = NavAutonomy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()