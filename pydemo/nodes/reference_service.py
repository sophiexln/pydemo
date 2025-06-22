# ~/hy_ws/src/pydemo/pydemo/nodes/reference_service.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion
import cv2
import yaml
import os
import time

class CaptureImageService(Node):
    def __init__(self):
        super().__init__('capture_image_service')
        self.bridge = CvBridge()
        self.image = None
        self.pose = None

        # 경로 설정
        self.image_dir = os.path.expanduser('~/hy_ws/src/pydemo/pydemo/logs/images/reference')
        self.pose_dir = os.path.expanduser('~/hy_ws/src/pydemo/pydemo/config')
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)

        # 토픽 구독
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # 서비스 서버 생성
        self.srv = self.create_service(Trigger, '/capture_image_now', self.capture_callback)
        self.get_logger().info("📸 CaptureImageService ready!")

    def image_callback(self, msg):
        self.image = msg

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def capture_callback(self, request, response):
        self.get_logger().info("📥 Service request received! Waiting for image and pose...")
        timeout = time.time() + 10.0
        while rclpy.ok() and (self.image is None or self.pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > timeout:
                response.success = False
                response.message = "❌ Timeout: image or pose not received"
                return response

        # 저장할 번호 계산
        index = len([f for f in os.listdir(self.image_dir) if f.endswith('.jpg')]) + 1
        image_path = os.path.join(self.image_dir, f"{index}.jpg")
        pose_path = os.path.join(self.pose_dir, f"waypoint{index}.yaml")

        # 이미지 저장
        cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        cv2.imwrite(image_path, cv_image)

        # 포즈 저장
        q = self.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose_data = {
            'pose': {
                'x': self.pose.position.x,
                'y': self.pose.position.y,
                'theta': yaw
            }
        }
        with open(pose_path, 'w') as f:
            yaml.dump(pose_data, f)

        response.success = True
        response.message = f"✅ Saved image to {image_path} and pose to {pose_path}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CaptureImageService()
    rclpy.spin(node)
    rclpy.shutdown()
