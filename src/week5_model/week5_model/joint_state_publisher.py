import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Timer를 설정해 주기적으로 퍼블리시
        self.publish_joint_state()

    def publish_joint_state(self):
        joint_state_msg = JointState()

        # 메시지 초기화
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["top_link", "wheel_left_link", "wheel_right_link"]
        joint_state_msg.position = [0.0, 0.0, 0.0]  # 모든 조인트의 초기값을 0으로 설정
        # 퍼블리시
        self.publisher.publish(joint_state_msg)
        self.get_logger().info(f"Published: {joint_state_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
