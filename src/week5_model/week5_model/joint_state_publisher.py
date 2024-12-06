import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)

        # 초기 상태
        self.current_position = 0.0
        self.velocity = 0.01  # 조인트 이동 속도
        self.direction = 1

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['top_joint']
        self.current_position += self.velocity * self.direction

        # 제한 범위 설정
        if self.current_position >= 0.07:
            self.direction = -1
        elif self.current_position <= 0.0:
            self.direction = 1

        msg.position = [self.current_position]
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint position: {self.current_position:.3f}')

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
