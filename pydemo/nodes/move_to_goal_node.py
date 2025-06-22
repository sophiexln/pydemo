import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import py_trees
import os
import yaml
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory


class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, index, attempt):
        super().__init__(name=f"MoveToGoal_{index}_{attempt}")
        self.index = index
        self.attempt = attempt
        self.node = rclpy.create_node(f"move_to_goal_{index}_{attempt}")
        self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def update(self):
        # 🛰 Navigation2 서버 연결 확인
        if not self.client.wait_for_server(timeout_sec=5.0):
            print(f"[⏳] Nav2 서버 연결 대기 중 - WP{self.index} 시도 {self.attempt}")  # 🔧 수정: 연결 실패 시 SUCCESS 대신 RUNNING 반환하여 계속 대기
            return py_trees.common.Status.RUNNING

        print(f"[🧭] WP{self.index} 이동 시작 - 시도 {self.attempt}")

        # 🎯 목표 위치 설정 및 전송
        goal = self.load_goal()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            print(f"[❌] WP{self.index} 목표 수락 거부됨")
            return py_trees.common.Status.SUCCESS

        # ⏳ 결과 대기 및 확인
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()

        # 🎉 도착 성공 여부 체크 (수정: SUCCEEDED(4) 확인)
        if result.status == 4:
            print(f"[✅] WP{self.index} 도착 성공 (시도 {self.attempt})")
        else:
            print(f"[⚠️] WP{self.index} 도착 실패 (시도 {self.attempt})")

        return py_trees.common.Status.SUCCESS

    def load_goal(self):
        # 📦 설정된 config 파일에서 waypoint 로드
        pkg_path = get_package_share_directory('pydemo')
        config_path = os.path.join(pkg_path, 'config', f'waypoint{self.index}.yaml')
        with open(config_path, 'r') as f:
            wp = yaml.safe_load(f)

        pose = wp['pose']
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose['x'])
        goal.pose.pose.position.y = float(pose['y'])
        goal.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, float(pose['theta']))
        (goal.pose.pose.orientation.x,
         goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z,
         goal.pose.pose.orientation.w) = q
        return goal
