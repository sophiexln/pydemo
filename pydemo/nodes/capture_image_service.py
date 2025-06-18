import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion
import os, cv2, yaml


class CaptureImageService(Node):
    def __init__(self):
        super().__init__('capture_image_service')
        self.bridge = CvBridge()
        self.image_msg = None
        self.pose_msg = None

        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_service(Trigger, 'capture_image', self.capture_cb)

        self.get_logger().info("📸 이미지 서비스 노드 시작됨")

    def image_callback(self, msg): self.image_msg = msg
    def pose_callback(self, msg): self.pose_msg = msg

    def capture_cb(self, req, resp):
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image_msg and self.pose_msg:
                break

        self.get_logger().info("[요청 수신] 이미지 캡처 요청 수신됨")

        img_success = False
        pose_success = False

        try:
            folder = os.path.expanduser('~/hy_ws/src/pydemo/pydemo/logs/images/current')
            os.makedirs(folder, exist_ok=True)
            idx = len(os.listdir(folder)) // 2 + 1

            if self.image_msg:
                img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
                cv2.imwrite(os.path.join(folder, f"{idx}.jpg"), img)
                img_success = True

            if self.pose_msg:
                pose = self.pose_msg.pose.pose
                _, _, yaw = euler_from_quaternion([
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                with open(os.path.join(folder, f"{idx}.yaml"), 'w') as f:
                    yaml.dump({
                        'position': {'x': pose.position.x, 'y': pose.position.y},
                        'orientation': {'yaw': yaw}
                    }, f)
                pose_success = True

            resp.success = img_success and pose_success
            resp.message = f"파일 저장 완료: {idx}.jpg" if resp.success else "일부 저장 실패"

        except Exception as e:
            resp.success = False
            resp.message = f"예외 발생: {str(e)}"

        # 결과 2줄만 출력
        img_msg = "✅" if img_success else "❌"
        pose_msg = "✅" if pose_success else "❌"
        self.get_logger().info(f"[결과] 이미지 저장: {img_msg}  |  포즈 저장: {pose_msg}")

        return resp


def main():
    rclpy.init()
    node = CaptureImageService()
    rclpy.spin(node)
    rclpy.shutdown()
