import os
import time
import yaml
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatusArray, GoalStatus
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge


class Nav2CaptureWhenStopped(Node):
    def __init__(self):
        super().__init__('nav2_capture_when_stopped')

        # --- 저장 경로 설정 ---
        self.image_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'logs', 'images', 'reference'))
        self.pose_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'config'))
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)

        # --- 메시지 보관용 변수 ---
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_pose = None
        self.latest_odom = None

        # --- 구독자 설정 ---
        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        # odom은 sensor_data QoS (RELIABLE) 로 설정
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        # status 토픽은 BEST_EFFORT 로 맞춤
        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/status',
            self.status_callback,
            status_qos
        )

        # --- Nav2 액션 클라이언트 ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()
        self.get_logger().info("✅ Connected to Nav2 action server")

        # --- 상태 플래그 ---
        self.goal_succeeded = False
        self.stop_start_time = None
        self.saved = False

        # --- 주기 타이머 (0.2s) ---
        self.timer = self.create_timer(0.2, self.check_stopped_and_save)

    # 콜백들
    def image_callback(self, msg: Image):
        self.latest_image = msg

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = msg.pose.pose

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def status_callback(self, msg: GoalStatusArray):
        # 모든 goal 상태 중 SUCCEEDED 탐지
        for s in msg.status_list:
            if s.status == GoalStatus.STATUS_SUCCEEDED and not self.goal_succeeded:
                self.get_logger().info("🏁 Nav2 goal succeeded!")
                self.goal_succeeded = True
                break

    # 정지 상태 확인 후 저장
    def check_stopped_and_save(self):
        if not self.goal_succeeded or self.saved or self.latest_odom is None:
            return

        vx = self.latest_odom.twist.twist.linear.x
        vz = self.latest_odom.twist.twist.angular.z

        if abs(vx) < 0.01 and abs(vz) < 0.01:
            if self.stop_start_time is None:
                self.stop_start_time = time.time()
                self.get_logger().info("🛑 Robot stopped. Starting 5s timer...")
            elif time.time() - self.stop_start_time >= 5.0:
                self.get_logger().info("📷 5s stable stop complete. Saving...")
                self.save_once()
        else:
            if self.stop_start_time is not None:
                self.get_logger().info("🚶 Robot moved again. Resetting stop timer.")
            self.stop_start_time = None

    # 이미지 및 pose 저장
    def save_once(self):
        if self.latest_image is None or self.latest_pose is None:
            self.get_logger().warn("⚠️ Image or pose not ready.")
            return

        index = len(os.listdir(self.image_dir)) + 1

        # 이미지 저장
        image_path = os.path.join(self.image_dir, f'{index}.jpg')
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f"📷 Image saved to {image_path}")

        # pose 저장
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
        self.get_logger().info(f"📝 Pose saved to {pose_path}")

        self.saved = True


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CaptureWhenStopped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
