import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import os
import yaml
from tf_transformations import euler_from_quaternion

class SimpleCapture(Node):
    def __init__(self):
        super().__init__('simple_capture')

        self.bridge = CvBridge()
        self.pose = None
        self.image = None
        self.saved = False

        self.image_dir = os.path.expanduser('~/hy_ws/src/pydemo/pydemo/logs/images/reference')
        self.pose_dir = os.path.expanduser('~/hy_ws/src/pydemo/pydemo/config')
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)

        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        self.get_logger().info("📸 SimpleCapture node started.")

    def image_callback(self, msg):
        if self.saved:
            return
        self.get_logger().info("📷 Image received")
        self.image = msg
        self.try_save()

    def pose_callback(self, msg):
        if self.saved:
            return
        self.get_logger().info("📡 Pose received")
        self.pose = msg.pose.pose
        self.try_save()

    def try_save(self):
        if self.pose is None or self.image is None:
            return  # 아직 둘 다 안 들어옴

        # 이미지 저장
        image_index = len([f for f in os.listdir(self.image_dir) if f.endswith('.jpg')]) + 1
        image_path = os.path.join(self.image_dir, f"{image_index}.jpg")
        cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f"✅ Saved image: {image_path}")

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
        pose_path = os.path.join(self.pose_dir, f"waypoint{image_index}.yaml")
        with open(pose_path, 'w') as f:
            yaml.dump(pose_data, f)
        self.get_logger().info(f"✅ Saved pose: {pose_path}")

        # 종료
        self.saved = True
        self.get_logger().info("🎉 Done. Shutting down.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCapture()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
