# ~/hy_ws/src/pydemo/pydemo/nodes/run_capture_once.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class CaptureClient(Node):
    def __init__(self):
        super().__init__('capture_client')
        self.cli = self.create_client(Trigger, '/capture_image_now')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('🔄 waiting for service...')
        self.req = Trigger.Request()

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"🎉 Success: {future.result().message}")
        else:
            self.get_logger().error(f"🚫 Failed: {future.result().message}")

def main(args=None):
    rclpy.init(args=args)
    client = CaptureClient()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()
