import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.current_position = 0.0  # 초기 위치
        self.get_logger().info('Joint State Publisher is ready!')

        # 사용자 입력을 처리하는 타이머
        self.timer = self.create_timer(0.1, self.publish_joint_state)

    def publish_joint_state(self):
        msg = JointState()
        position = self.set_position()
        if position is None:
            return
        elif position > 0.06:
            self.get_logger().error('Invalid position. Must be between 0.0 and 0.07.')
            return
        elif position < 0.0:
            self.get_logger().error('Invalid position. Must be between 0.0 and 0.07.')
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['top_joint']
        msg.position = [position]
        self.publisher.publish(msg)
        self.get_logger().info(f'Published joint position: {self.current_position:.3f}')

    def set_position(self):
        position = float(input('Enter joint position (0.0 ~ 0.07): '))
        if 0.0 <= position <= 0.07:  # 유효 범위 체크
            self.get_logger().info(f'Set position to: {position}')
            return position
        else:
            self.get_logger().error('Invalid position. Must be between 0.0 and 0.07.')

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
