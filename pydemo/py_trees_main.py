import rclpy
import py_trees
from pydemo.nodes.move_to_goal_node import MoveToGoal
from pydemo.nodes.capture_image_node import CaptureImageNode


def main():
    rclpy.init()

    # 🧹 루트 시퀀스: 전체 순차 행동 트리 (메모리 모드로, 완료된 자식은 다시 실행하지 않음)
    root = py_trees.composites.Sequence("Root", memory=True)

    # 📍 각 waypoint에 대해 이동 2회 + 이미지 캡처 2회 반복
    for i in range(1, 4):  # Waypoint 1~3
        # WP 시퀀스도 메모리 모드로 설정
        wp_seq = py_trees.composites.Sequence(f"WP_{i}", memory=True)

        # 1차 이동 및 캡처
        wp_seq.add_child(MoveToGoal(index=i, attempt=1))         # 🔧 memory=True 덕분에 성공/실패와 관계없이 1회만 실행
        wp_seq.add_child(CaptureImageNode(index=i, attempt=1))   # 🔧 동일하게 1회 캡처

        # 2차 이동 및 캡처
        wp_seq.add_child(MoveToGoal(index=i, attempt=2))         # 🔧 2회차 이동 시도
        wp_seq.add_child(CaptureImageNode(index=i, attempt=2))   # 🔧 2회차 캡처

        root.add_child(wp_seq)

    # 🌿 행동 트리 실행 (tick_tock은 memory=True 덕분에 한번 지나간 자식은 다시 실행하지 않고 다음으로 진행)
    tree = py_trees.trees.BehaviourTree(root)
    print("behavior tree 실행")

    try:
        tree.tick_tock(period_ms=500)
    except KeyboardInterrupt:
        print("종료")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
