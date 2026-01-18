import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String


import math
import time
import re

def parse_detection(s: str):
    # récupère: type + 3 floats, même si entourés de < >
    tokens = re.findall(r'[A-Za-z]+|[-+]?\d*\.\d+|[-+]?\d+', s)
    if len(tokens) < 4:
        return None
    obj_type = tokens[0].lower()   # "blue" / "red"
    lat  = float(tokens[1])
    dist = float(tokens[2])
    z    = float(tokens[3])
    return obj_type, lat, dist, z



class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # ---- Parameters ----
        self.arm_reach = 0.45      # portée du bras en m        // to modify given real conditions
        self.linear_speed = 0.15  # m/s
        self.angular_speed = 0.6  # rad/s

        # ---- Subscribers ----
        self.sub_obj = self.create_subscription(String,'/detected_object_position',self.object_callback,10)

        # ---- Publishers ----
        self.pub = self.create_publisher(String, '/action', 10)

        # ---- State ----
        self.current_object = None
        self.last_object = None     # (x, y)
        self.state = "IDLE"         # to define

        self.get_logger().info("Decision node started.")

    # =======================================================================
    # CALLBACK          // to modify +/-
    # =======================================================================
    def object_callback(self, msg):
        parsed = parse_detection(msg.data)
        if parsed is None:
            return
        obj_type, lat, dist, z = parsed        
        self.last_object = self.current_object
        self.current_object = (lat, dist)

        # Trigger FSM
        if self.state == "IDLE":
            self.state = "MOVE_TO_OBJECT"

    # =======================================================================
    # MAIN LOOP
    # =======================================================================
    def control_loop(self):

        # Aucun objet détecté
        if self.current_object is None:
            action_msg = String()
            action_msg.data = 'explore'
            if(self.current_object!=self.last_object):
                self.pub.publish(action_msg)
                return
        else :
            x, y = self.current_object
            dist = math.sqrt(x * x + y * y)

            if dist < self.arm_reach and dist>0.0:
                action = 'grasp_object'
            elif (dist>0.0):
                action = f'go_to {x} {y}'
            else : 
                action = 'explore'

            if(self.current_object!=self.last_object):
                action_msg = String()
                action_msg.data = action
                self.pub.publish(action_msg)


# =======================================================================
# MAIN
# =======================================================================
def main(args=None):
    rclpy.init(args=args)

    node = DecisionNode()

    # Timer : 20 Hz loop
    node.create_timer(0.05, node.control_loop)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()