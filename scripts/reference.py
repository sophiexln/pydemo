import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import cv2
import yaml
import os
from tf_transformations import euler_from_quaternion

class SimpleNav2Capture(Node):
    def __init__(self):
        super().__init__('simple_nav2_capture')

        # 👉 기본 설정
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_pose = None
        self.image_saved = False

        # 👉 경로 설정
        self.image_dir = os.path.expanduser('~/ros2_ws/src/runner1/logs/images/reference')
        self.pose_dir = os.path.expanduser('~/ros2_ws/src/runner1/config')
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)

        # 👉 구독자 등록
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # 👉 Nav2 ActionClient (결과만 수신)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()
        self.get_logger().info('📡 Waiting for external Nav2 goal to be sent (via Rviz)...')

        # 외부 goal을 기다리고 도달 결과만 감지
        dummy_goal = NavigateToPose.Goal()
        self.nav_client.send_goal_async(dummy_goal).add_done_callback(self._goal_callback)

    def _goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Nav2 Goal rejected")
            return
        self.get_logger().info("🚀 Nav2 Goal accepted, waiting for result...")
        goal_handle.get_result_async().add_done_callback(self._on_goal_done)

    def _on_goal_done(self, future):
        self.get_logger().info("🏁 Nav2 Goal reached!")
        # ✅ 도착 직후 5초 대기 후 저장하도록 타이머 설정
        self.get_logger().info("⏳ Waiting 5 seconds to stabilize before capturing...")
        self.create_timer(5.0, self.save_once)  # 🟢 5초 후 save_once 실행

    def image_callback(self, msg):
        self.latest_image = msg

    def pose_callback(self, msg):
        self.latest_pose = msg.pose.pose

    def save_once(self):
        if self.image_saved:
            return  # ✅ 타이머가 반복 실행되지 않도록 한번만 저장

        if self.latest_image is None or self.latest_pose is None:
            self.get_logger().warn("⚠️ Image or pose not ready. Skipping save.")
            return

        # ✅ 저장 인덱스 계산
        index = len(os.listdir(self.image_dir)) + 1
        image_path = os.path.join(self.image_dir, f'{index}.jpg')
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f'✅ Saved image to {image_path}')

        # ✅ 포즈 저장
        q = self.latest_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose_data = {
            'pose': {
                'x': self.latest_pose.position.x,
                'y': self.latest_pose.position.y,
                'theta': yaw
            }
        }
        pose_path = os.path.join(self.pose_dir, f'waypoint{index}.yaml')
        with open(pose_path, 'w') as f:
            yaml.dump(pose_data, f)
        self.get_logger().info(f'✅ Saved pose to {pose_path}')

        self.image_saved = True
        self.get_logger().info("🎉 Done. Shutting down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNav2Capture()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
