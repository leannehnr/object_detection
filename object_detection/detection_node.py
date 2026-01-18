import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        # Publisher ROS2
        self.pub = self.create_publisher(String, '/detected_object_position', 10)

        # Connect to CoppeliaSim
        client = RemoteAPIClient()
        self.sim = client.getObject('sim')

        # Handles
        self.cam = self.sim.getObject('/objectLocator')
        self.lidar = self.sim.getObject('/LIDAR')

        self.get_logger().info("Connected to CoppeliaSim. Starting detection loop...")

        # Timer = execute @ 30 Hz
        self.create_timer(1/30, self.loop)

    # ============================================================
    #                   FIND CENTERS
    # ============================================================
    def find_centers(self, mask, color_name, img, depth, res, res_depth):
        sim = self.sim
        lidar = self.lidar

        centers = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) > 50:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy))
                    cv2.circle(img, (cx, cy), 6,
                               (0, 0, 255) if color_name == 'red' else (255, 0, 0), -1)

        if len(centers) == 0:
            return None

        # ---- Near/far ----
        _, near = sim.getObjectFloatParameter(lidar, sim.visionfloatparam_near_clipping)
        _, far = sim.getObjectFloatParameter(lidar, sim.visionfloatparam_far_clipping)

        distance_map = near + depth * (far - near)
        distances = distance_map[0, :]

        # ---- FOV camera → lidar ----
        cam_fov = np.deg2rad(60)
        lidar_fov = np.deg2rad(90)

        total_pixels = res_depth[0]
        start_idx = int((total_pixels / 2) - (total_pixels * cam_fov / (2 * lidar_fov)))
        end_idx = int((total_pixels / 2) + (total_pixels * cam_fov / (2 * lidar_fov)))

        subset = distances[start_idx:end_idx]

        cx = centers[0][0]
        prop = cx / res[0]

        subset_index = int(prop * len(subset))
        subset_index = min(subset_index, len(subset) - 1)

        dist = subset[subset_index]

        # ---- Compute position relative robot ----
        angle = (prop - 0.5) * cam_fov

        x = dist * np.sin(angle)
        y = dist

        return x, y

    # ============================================================
    #            MAIN LOOP 
    # ============================================================
    def loop(self):
        sim = self.sim

        # Read camera
        img_raw, res = sim.getVisionSensorImg(self.cam)
        img = np.frombuffer(img_raw, dtype=np.uint8).reshape((res[1], res[0], 3))
        img = cv2.cvtColor(cv2.flip(img, 0), cv2.COLOR_RGB2BGR)

        # Read depth
        depth_raw = sim.getVisionSensorDepthBuffer(self.lidar)
        res_depth = sim.getVisionSensorResolution(self.lidar)
        depth = np.array(depth_raw, dtype=np.float32).reshape((res_depth[1], res_depth[0]))
        depth = np.flipud(depth)

        # HSV masks
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        lower_blue = np.array([80, 150, 150])
        upper_blue = np.array([100, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Detect colors
        pos_red = self.find_centers(mask_red, 'red', img, depth, res, res_depth)
        pos_blue = self.find_centers(mask_blue, 'blue', img, depth, res, res_depth)

        # Publish if valid
        x, y, z = 0.0, 0.0, 0.0
        msg = String()
        if pos_red:
            x, y, z = float(pos_red[0]), float(pos_red[1]), 0.0
            msg = f'red {x} {y} {z}'
            self.pub.publish(msg)

        if pos_blue:
            x, y, z = float(pos_blue[0]), float(pos_blue[1]), 0.0
            msg = f'blue {x} {y} {z}'
            self.pub.publish(msg)

        if not pos_blue and not pos_red: 
            msg = f'wall {x} {y} {z}'
            self.pub.publish(msg)

        # Show camera
        cv2.imshow("Camera", img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
