import rclpy
import py_trees
from pydemo.nodes.move_to_goal_node import MoveToGoal
from pydemo.nodes.capture_image_node import CaptureImageNode


def main():
    rclpy.init()

    # 🧹 루트 시그니체: 전체 순차 행동 트리
    root = py_trees.composites.Sequence("Root", memory=False)

    # 📍 각 waypoint에 대해 이동 2회 + 이미지 캐프 2회 반복
    for i in range(1, 4):  # Waypoint 1~3
        wp_seq = py_trees.composites.Sequence(f"WP_{i}", memory=False)

        wp_seq.add_child(MoveToGoal(index=i, attempt=1))
        wp_seq.add_child(CaptureImageNode(index=i, attempt=1))

        wp_seq.add_child(MoveToGoal(index=i, attempt=2))
        wp_seq.add_child(CaptureImageNode(index=i, attempt=2))

        root.add_child(wp_seq)

    # 🌿 행동 트리 실행
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
