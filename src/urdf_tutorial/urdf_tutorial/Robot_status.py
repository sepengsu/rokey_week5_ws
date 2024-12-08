import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

class FireSuppressionRobot(Node):
    def __init__(self):
        super().__init__('fire_suppression_robot')

        # 카메라 구독 (화재 감지를 위한 이미지 받기)
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # 적절한 카메라 토픽으로 변경
            self.camera_callback,
            10
        )

        # 로봇 제어를 위한 cmd_vel 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 순찰 및 복귀를 위한 위치 목록
        self.patrol_positions = [
            [-0.198, -5.069, 0.0],  # 순찰 위치 1
            [-7.681, -5.314, 0.0],  # 순찰 위치 2
            [-8.152, 0.763, 0.0],   # 순찰 위치 3
            [-8.18, 7.49, 0.0],     # 순찰 위치 4
            [-0.08, 7.7, 0.001],    # 순찰 위치 5
            [-0.019, 0.980, 0.001], # 순찰 위치 6
            [-7.680, 1.416, 0.001], # 순찰 위치 7
            [-7.72, -1.21, 0.001],  # 순찰 위치 8
            [-0.415, -0.458, 0.001] # 순찰 위치 9
        ]
        self.initial_position = [-0.0534, -0.0035, -0.00143]  # 초기 위치 (로봇이 시작하는 위치)
        self.current_position_index = 0  # 순찰 위치의 인덱스 (초기값은 첫 번째 위치)
        self.fire_detected = False  # 화재 감지 여부를 추적하는 플래그
        self.is_returning = False  # 복귀 여부를 추적하는 플래그

        # CvBridge 초기화
        self.bridge = CvBridge()

        # NavigateToPose 액션 클라이언트 (목표 위치로 네비게이션)
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 액션 서버가 준비될 때까지 대기
        while not self.navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('NavigateToPose 액션 서버 대기 중...')

        # 타이머를 사용하여 순찰을 시작
        self.timer = self.create_timer(5.0, self.patrol)  # 5초마다 호출되는 타이머 설정

    def camera_callback(self, msg):
        """카메라 이미지를 받아서 화재를 감지하는 콜백 함수."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")

        # 이미지를 처리하여 화재를 감지
        fire_detected = self.detect_fire(cv_image)

        if fire_detected and not self.fire_detected and not self.is_returning:
            self.get_logger().info("화재 감지됨! 화재 진압을 시작합니다.")
            self.fire_detected = True
            self.stop_robot()  # 로봇 정지
            self.suppress_fire()  # 화재 진압
            self.return_to_base()  # 초기 위치로 복귀

    def detect_fire(self, image):
        """이미지에서 빨간색을 감지하여 화재를 탐지하는 함수."""
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_red = (0, 120, 70)
        upper_red = (10, 255, 255)
        
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        
        red_pixels = cv2.countNonZero(mask)
        total_pixels = mask.size
        red_percentage = (red_pixels / total_pixels) * 100

        self.get_logger().info(f"빨간색 비율: {red_percentage:.2f}%")
        return red_percentage > 70.0  # 화재 감지 기준 70%

    def move_to_position(self, position):
        """로봇을 지정된 위치로 이동시키는 함수 (Nav2 액션 사용)."""
        goal_msg = PoseStamped()

        # 목표 위치 설정
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = position[0]
        goal_msg.pose.position.y = position[1]
        goal_msg.pose.orientation.w = 1.0  # 회전 없이 이동

        self.get_logger().info(f"위치로 이동 중: {position}")

        # 목표 위치를 퍼블리시 (NavigateToPose 액션 호출)
        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal_msg

        # 액션 호출
        future = self.navigate_to_pose_client.send_goal_async(action_goal)
        return future

    def stop_robot(self):
        """로봇의 이동을 멈추는 함수."""
        self.get_logger().info("로봇 정지 중...")
        twist = Twist()  # 속도 0으로 설정
        self.cmd_vel_pub.publish(twist)

    def suppress_fire(self):
        """화재 진압 로직 (예: 물 분사기 활성화)."""
        self.get_logger().info("화재 진압 시스템 활성화!")
        rclpy.sleep(5)  # 5초 동안 화재 진압 시뮬레이션

    def return_to_base(self):
        """화재 진압 후 초기 위치로 복귀하는 함수."""
        self.is_returning = True
        self.get_logger().info("기지로 복귀 중...")
        self.move_to_position(self.initial_position)  # 초기 위치로 이동
        self.get_logger().info("복귀 완료. 순찰을 종료합니다.")
        self.is_returning = False
        self.fire_detected = False
        rclpy.shutdown()  # ROS 2 종료

    def patrol(self):
        """사각형 경로로 순찰을 수행하는 함수."""
        if self.fire_detected:
            return  # 화재 감지 시 순찰을 중단

        # 순찰 위치로 이동
        self.get_logger().info(f"순찰 위치: {self.patrol_positions[self.current_position_index]}")
        self.move_to_position(self.patrol_positions[self.current_position_index])

        # 순찰 위치 인덱스를 업데이트
        self.current_position_index = (self.current_position_index + 1) % len(self.patrol_positions)

def main():
    rclpy.init()
    fire_robot = FireSuppressionRobot()

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    
    executor.add_node(fire_robot)

    try:
        executor.spin()  # spin을 호출하여 노드 실행
    except KeyboardInterrupt:
        pass
    finally:
        fire_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()