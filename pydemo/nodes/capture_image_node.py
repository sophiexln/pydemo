import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import py_trees

class CaptureImageNode(py_trees.behaviour.Behaviour):
    def __init__(self, index, attempt):
        label = f"{index}_{attempt}"
        super().__init__(name=f"CaptureImage_{label}")
        self.index = index
        self.attempt = attempt
        self.node = rclpy.create_node(f"capture_image_node_{label}")
        self.client = self.node.create_client(Trigger, 'capture_image')

    def update(self):
        if not self.client.wait_for_service(timeout_sec=3.0):
            print(f"[❌] 서비스 연결 실패 (WP{self.index} 시도 {self.attempt})")
            return py_trees.common.Status.SUCCESS

        print(f"[📸] 캡처 서비스 요청 (WP{self.index} 시도 {self.attempt})")
        req = Trigger.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        if result.success:
            print(f"[✅] 저장 완료: {result.message}")
        else:
            print(f"[⚠️] 저장 실패: {result.message}")

        return py_trees.common.Status.SUCCESS


