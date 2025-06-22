import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import os
import yaml
from tf_transformations import euler_from_quaternion

class ReferenceImageCapture(Node):
    def __init__(self):
        super().__init__('reference_image_capture')

        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # 필요시 다른 이미지 토픽으로 변경 가능
            self.image_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.bridge = CvBridge()
        self.current_pose = None
        self.image_saved = False

        # 🟢 이미지 저장 경로: ~/ros2_ws/src/runner1/logs/images/reference
        self.image_dir = os.path.expanduser('~/hy_ws/src/pydemo/pydemo/logs/images/reference')
        os.makedirs(self.image_dir, exist_ok=True)

        # 🟢 포즈 저장 경로: ~/ros2_ws/src/runner1/config
        self.pose_dir = os.path.expanduser('~/hy_ws/src/pydemo/pydemo/config')
        os.makedirs(self.pose_dir, exist_ok=True)

        self.get_logger().info('📸 ReferenceImageCapture Node Started')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def image_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn('⚠️ Pose not received yet. Skipping image save.')
            return

        if self.image_saved:
            return

        # ✅ 인덱스 계산
        image_index = len([f for f in os.listdir(self.image_dir) if f.endswith('.jpg')]) + 1
        pose_index = len([f for f in os.listdir(self.pose_dir) if f.startswith('waypoint')]) + 1

        # ✅ 이미지 저장
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_path = os.path.join(self.image_dir, f'{image_index}.jpg')
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f'✅ Saved image: {image_path}')

        # ✅ 쿼터니언 → yaw(radian) 변환
        q = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(q)

        # ✅ 포즈 저장
        pose_data = {
            'pose': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'theta': yaw  # radian 값
            }
        }

        pose_path = os.path.join(self.pose_dir, f'waypoint{pose_index}.yaml')
        with open(pose_path, 'w') as f:
            yaml.dump(pose_data, f)
        self.get_logger().info(f'✅ Saved pose: {pose_path}')

        # ✅ 1회성 저장 후 종료
        self.image_saved = True
        self.get_logger().info('🎉 Done. Shutting down node.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ReferenceImageCapture()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
