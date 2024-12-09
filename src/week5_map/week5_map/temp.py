import rclpy, cv2
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from rclpy.executors import MultiThreadedExecutor
import time
from sensor_msgs.msg import JointState

class FireSuppressionRobot(Node):
    def __init__(self):
        super().__init__('fire_suppression_robot')

        # 카메라 구독 (화재 감지를 위한 이미지 받기)
        self.camera_sub = self.create_subscription(
            Image,
            '/tb1/camera/image_raw',  # 적절한 카메라 토픽으로 변경
            self.camera_callback,
            10
        )

        # 로봇 2로 이동 명령을 보내기 위한 퍼블리셔
        self.move_pub = self.create_publisher(PoseStamped, '/tb2/move', 10)
        # 로봇 제어를 위한 cmd_vel 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/tb1/cmd_vel', 10)

        # 순찰 및 복귀를 위한 위치 목록
        self.patrol_positions = [
            [-0.54429,1.921109,0.002471], # 순찰위치 1
            [-4.00228,1.975084,-0.00143], # 순찰위치 2
            [-7.60197,1.850219,-0.00143], # 순찰위치 3
            [-7.72995,-1.07178,-0.00143], # 순찰위치 4
            [-3.31024,-0.97845,-0.00143], # 순찰위치 5
            [-0.0014805,-0.0060469,0.1952819] # 순찰위치 복귀
        ]
        self.initial_position = [-0.0534, -0.0035, -0.00143]  # 초기 위치 (로봇이 시작하는 위치)
        self.fire_position = [-2.51,-3.41, 0.001]  # 화재 위치 (임의로 설정)
        self.current_position_index = 0  # 순찰 위치의 인덱스 (초기값은 첫 번째 위치)
        self.fire_detected = False  # 화재 감지 여부를 추적하는 플래그
        self.is_returning = False  # 복귀 여부를 추적하는 플래그

        # CvBridge 초기화
        self.bridge = CvBridge()

        # NavigateToPose 액션 클라이언트 (목표 위치로 네비게이션)
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, '/tb1/navigate_to_pose')

        # rob2 로봇을 위한 퍼블리셔
        self.move_pub = self.create_publisher(PoseStamped, '/tb2/move', 10)
        # 액션 서버가 준비될 때까지 대기
        while not self.navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('NavigateToPose 액션 서버 대기 중...')


        # 순찰 시작
        time.sleep(5)  # 로봇이 초기 위치로 이동할 시간을 줌
        self.patrol()

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
            self.return_to_base()  # 초기 위치로 복귀

    def detect_fire(self, image):
        """이미지에서 빨간색을 감지하여 화재를 탐지하는 함수."""
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 빨간색 범위 (두 범위로 나눔)
        lower_red = (0, 50,50)
        upper_red = (10, 255, 255)
        # 두 개의 마스크 생성
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        red_pixels = cv2.countNonZero(mask1)
        total_pixels = mask1.size
        red_percentage = (red_pixels / total_pixels) * 100

        return red_percentage > 10.0  # 화재 감지 기준 10%


    def move_to_position(self, position):
        """로봇을 지정된 위치로 이동시키는 함수 (Nav2 액션 사용)."""
        goal_msg = PoseStamped()

        # 목표 위치 설정
        goal_msg.header.frame_id = "map"

        goal_msg.pose.position.x = position[0]
        goal_msg.pose.position.y = position[1]
        goal_msg.pose.orientation.w = 1.0  # 회전 없이 이동

        # 목표 위치를 퍼블리시 (NavigateToPose 액션 호출)
        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal_msg

        # 액션 호출
        future = self.navigate_to_pose_client.send_goal_async(action_goal)
        future.add_done_callback(self.check_goal_status)
        return future

    def stop_robot(self):
        """로봇의 이동을 멈추는 함수."""
        self.get_logger().info("로봇 정지 중...")
        twist = Twist()  # 속도 0으로 설정
        self.cmd_vel_pub.publish(twist)
        time.sleep(30)


    def suppress_fire(self):
        """화재 진압 로직 (예: 물 분사기 활성화)."""
        msg = PoseStamped()
        msg.header.frame_id = "map"  # 필수: 메시지의 frame_id 설정
        msg.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 추가
        msg.pose.position.x = self.fire_position[0]
        msg.pose.position.y = self.fire_position[1]
        msg.pose.orientation.w = 1.0  # 기본 회전값
        self.move_pub.publish(msg) 
        self.stop_robot()

    def return_to_base(self):
        """화재 진압 후 초기 위치로 복귀하는 함수."""
        self.is_returning = True
        self.get_logger().info("기지로 복귀 중...")
        self.move_to_position(self.initial_position)  # 초기 위치로 이동
        self.get_logger().info("복귀 완료. 순찰을 종료합니다.")
        self.is_returning = False
        self.fire_detected = False

    def check_goal_status(self, future):
        """액션 목표 상태를 확인하는 함수."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표를 수행할 수 없습니다.')
            return
        action_result = goal_handle.get_result_async()
        action_result.add_done_callback(self.on_goal_finished)
                                        
    def on_goal_finished(self, future):
        action_status = future.result().status
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[INFO] Reached goal.")
            self.current_position_index += 1
            if self.current_position_index >= len(self.patrol_positions):
                self.current_position_index = 0
        else:
            self.get_logger().error("[ERROR] Failed to reach goal.")
        if self.current_position_index==0:
            self.get_logger().info("화재 감지!!!!")
            self.suppress_fire()
        else:
            self.patrol()

    def patrol(self):
        """사각형 경로로 순찰을 수행하는 함수."""
        if self.fire_detected:
            return  # 화재 감지 시 순찰을 중단
        # 순찰 위치로 이동
        self.get_logger().info(f"순찰 위치: {self.patrol_positions[self.current_position_index]}")
        self.move_to_position(self.patrol_positions[self.current_position_index])

class Robot2(Node):
    def __init__(self):
        super().__init__('hose_robot')
        self.bridge = CvBridge()
        self.init_pos = [-7.72, -1.21, 0.001]
        # 로봇 제어를 위한 cmd_vel 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/tb2/cmd_vel', 10)

        # 이동 명령을 받기 위한 퍼블리셔
        self.move_sub = self.create_subscription(PoseStamped, '/tb2/move', 
            self.go_to_fire, 10)
        # 조인트 제어를 위한 퍼블리셔
        self.joint_pub = self.create_publisher(PoseStamped, '/tb2/joint_states', 10)
        # 이동 관련 action client
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, '/tb2/navigate_to_pose')
        while not self.navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('NavigateToPose 액션 서버 대기 중...')
        

    def go_to_fire(self, msg):
        """화재 위치로 이동하는 함수."""
        self.get_logger().info("화재 위치 수신 및 이동 중 ...")
        position = [msg.pose.position.x, msg.pose.position.y]
        self.move_to_position(position)
    
    def move_to_position(self, position):
        """로봇을 지정된 위치로 이동시키는 함수 (Nav2 액션 사용)."""
        goal_msg = PoseStamped()

        # 목표 위치 설정
        goal_msg.header.frame_id = "map"

        goal_msg.pose.position.x = position[0]
        goal_msg.pose.position.y = position[1]
        goal_msg.pose.orientation.w = 1.0
        
        # 목표 위치를 퍼블리시 (NavigateToPose 액션 호출)
        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal_msg

        # 액션 호출
        future = self.navigate_to_pose_client.send_goal_async(action_goal)
        future.add_done_callback(self.check_goal_status)
        return future
    
    def check_goal_status(self, future):
        """액션 목표 상태를 확인하는 함수."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표를 수행할 수 없습니다.')
            return
        action_result = goal_handle.get_result_async()
        action_result.add_done_callback(self.on_goal_finished)

    def on_goal_finished(self, future):
        action_status = future.result().status
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[INFO] Reached goal.")
            self.move_to_position(self.init_pos)
        else:
            self.get_logger().error("[ERROR] Failed to reach goal.")

    def pump_up(self):
        """물 분사기 활성화."""
        self.get_logger().info("물 분사기 활성화 중 ...")
        msg = JointState()
        msg.name = ['top_joint']
        msg.position = [0.07]
        msg.velocity = [0.01]
        self.joint_pub.publish(msg)
        
    
    def pump_down(self):
        self.get_logger().info("물 분사기 비활성화 중 ...")
        msg = JointState()
        msg.name = ['top_joint']
        msg.position = [0.0]
        msg.velocity = [0.01]
        self.joint_pub.publish(msg)
        time.sleep(7)

    def watering(self):
        self.get_logger().info("물 분사 중 ...")
        time.sleep(2)
    
    def go_to_init(self):
        position = self.init_pos
        self.move_to_position(position)
    
    def front(self):
        '''
        화재 진압을 위한 전체 프로세스
        1. 위치 이동 
        2. 물 분사기 활성화
        3. 물 분사기 작동
        4. 물 분사기 비활성화
        5. 위치 이동
        '''
        self.go_to_fire()
        self.pump_up()
        self.watering()
        self.pump_down()
        self.go_to_init()


def main():
    rclpy.init()

    # 두 노드 생성
    fire_robot = FireSuppressionRobot()
    robot2 = Robot2()

    # MultiThreadedExecutor 생성
    executor = MultiThreadedExecutor()

    # 두 노드를 실행 환경에 추가
    executor.add_node(fire_robot)
    executor.add_node(robot2)

    try:
        executor.spin()  # 두 노드를 동시에 실행
    except KeyboardInterrupt:
        pass
    finally:
        fire_robot.destroy_node()
        robot2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()