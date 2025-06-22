import time
import rclpy
import py_trees
from std_srvs.srv import Trigger

class CaptureImageNode(py_trees.behaviour.Behaviour):
    def __init__(self, index, attempt):
        super().__init__(name=f"CaptureImage_{index}_{attempt}")
        self.index = index
        self.attempt = attempt
        # 독립 노드로 클라이언트 생성
        self.node = rclpy.create_node(f"capture_cli_{index}_{attempt}")
        self.client = self.node.create_client(Trigger, 'capture_image')
        self._future = None
        self._start_time = None
        self._timeout_sec = 5.0
        self._service_called = False

    def initialise(self):
        # 매 tick마다 초기화
        self._future = None
        self._start_time = None
        self._service_called = False

    def update(self):
        # 1) 아직 서비스 요청 전
        if not self._service_called:
            if not self.client.wait_for_service(timeout_sec=1.0):
                # 서비스 연결 대기 중
                return py_trees.common.Status.RUNNING

            self._future = self.client.call_async(Trigger.Request())
            self._start_time = time.time()
            self._service_called = True
            print(f"[📸] 캡처 요청 보냄 (WP{self.index}, 시도{self.attempt})")
            return py_trees.common.Status.RUNNING

        # 2) 서비스 응답 처리 위해 spin_once() 호출
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # 3) 응답이 왔으면 SUCCESS/FAILURE 구분
        if self._future.done():
            result = self._future.result()
            if result.success:
                print(f"[✅] 결과 (성공) - {result.message}")
            else:
                print(f"[❌] 결과 (실패) - {result.message}")
            # 실패여도 다음으로 넘어가려면 항상 SUCCESS 반환
            return py_trees.common.Status.SUCCESS

        # 4) 타임아웃 검사
        if time.time() - self._start_time > self._timeout_sec:
            print(f"[⚠️] 응답 타임아웃 (WP{self.index}, 시도{self.attempt})")
            # 타임아웃도 다음으로 넘어가도록 SUCCESS
            return py_trees.common.Status.SUCCESS

        # 5) 아직 대기 중이면 RUNNING
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # 취소할 필요 있으면 future.cancel() 등 처리
        if self._future and not self._future.done():
            self._future.cancel()