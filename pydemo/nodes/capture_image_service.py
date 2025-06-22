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

        # 🔧 reference 코드 기반: 이미지와 포즈 토픽 구독
        self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # 필요시 실제 토픽명으로 변경
            self.image_callback,
            10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        # 🔧 Trigger 서비스 생성
        self.create_service(Trigger, 'capture_image', self.capture_cb)

        self.get_logger().info('📸 이미지 서비스 노드 시작됨')

    def image_callback(self, msg):
        # 🔧 최신 이미지 저장
        self.image_msg = msg

    def pose_callback(self, msg):
        # 🔧 최신 포즈 저장
        self.pose_msg = msg.pose.pose

    def capture_cb(self, req, resp):
        # 🔧 요청이 들어올 때마다 최신 메시지를 사용할 수 있도록 초기화는 하지 않고,
        #    이전 구독으로 이미 최신 값이 self.image_msg, self.pose_msg에 저장됨

        # 🔧 최대 5초간 이미지/포즈 대기
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image_msg and self.pose_msg:
                break

        if not self.image_msg or not self.pose_msg:
            self.get_logger().warn('[⚠️] 메시지 수신 불가: 이미지 또는 포즈 없음')
            resp.success = False
            resp.message = '이미지 또는 포즈 수신 실패'
            return resp

        # 🔧 저장 경로 설정
        folder = os.path.expanduser('~/hy_ws/src/pydemo/pydemo/logs/images/current')
        os.makedirs(folder, exist_ok=True)
        # 파일 인덱스 계산 (이미지와 포즈 쌍으로 1부터)
        idx = len([f for f in os.listdir(folder) if f.endswith('.jpg')]) + 1

        # 🔧 이미지 저장
        cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        image_path = os.path.join(folder, f'{idx}.jpg')
        cv2.imwrite(image_path, cv_image)

        # 🔧 포즈 저장 (yaw 추출)
        pose = self.pose_msg
        _, _, yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        pose_data = {
            'position': {'x': pose.position.x, 'y': pose.position.y},
            'orientation': {'yaw': yaw}
        }
        pose_path = os.path.join(folder, f'{idx}.yaml')
        with open(pose_path, 'w') as f:
            yaml.dump(pose_data, f)

        resp.success = True
        resp.message = f'파일 저장 완료: {idx}.jpg'
        self.get_logger().info(f"[결과] 이미지 저장: ✅  |  포즈 저장: ✅")
        return resp


def main():
    rclpy.init()
    node = CaptureImageService()
    rclpy.spin(node)
    rclpy.shutdown()
