
import threading, time, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
import cv2

import numpy as np
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QFrame
from PyQt5.QtGui import QPixmap, QColor, QImage, QPainter
from PyQt5.QtCore import Qt, QTimer, QThread
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from PyQt5.QtCore import QMetaObject, Qt
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from PIL import Image as PILImage

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # 신뢰성 설정
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # 지속성 설정
    depth=10  # 히스토리 버퍼 크기 설정
)
cam_qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # 신뢰성 설정
    durability=QoSDurabilityPolicy.VOLATILE,   # 지속성 설정 (과거 데이터 저장 안 함)
    depth=10                                   # 기본 히스토리 버퍼 크기
)
class SystemNode(Node):
    def __init__(self):
        super().__init__('system_node')  # Node 이름 설정

        # ROS2 토픽 구독
        self.map_subscription = self.create_subscription(OccupancyGrid,'/map',
            self.map_callback,qos_profile)
        self.tb1_subscription = self.create_subscription(PoseWithCovarianceStamped,'/tb1/amcl_pose',
            self.tb1_callback, qos_profile)
        self.tb2_subscription = self.create_subscription(PoseWithCovarianceStamped,'/tb2/amcl_pose',
            self.tb2_callback, qos_profile)
        self.tb1_camera_subscription = self.create_subscription(Image,'/tb1/camera/image_raw',
            self.tb1_camera_callback, cam_qos_profile)

        # 데이터 저장 및 락 설정
        self.map_data = None
        self.tb1_position = None
        self.tb2_position = None
        self.lock = threading.Lock()
        self.tb1_image = None

    def map_callback(self, msg):
        """맵 데이터를 저장"""
        with self.lock:
            self.map_data = msg
            self.get_logger().info("Map data updated.")

    def get_data(self):
        """맵 데이터를 반환 (스레드 안전)"""
        with self.lock:
            return [self.map_data, self.tb1_position, self.tb2_position, self.tb1_image]

    def tb1_callback(self, msg):
        """tb1의 위치를 저장"""
        with self.lock:
            self.tb1_position = msg.pose.pose.position
        # self.get_logger().info(f"tb1 위치: x={self.tb1_position.x:.2f}, y={self.tb1_position.y:.2f}")

    def tb2_callback(self, msg):
        """tb2의 위치를 저장"""
        with self.lock:
            self.tb2_position = msg.pose.pose.position
        # self.get_logger().info(f"tb2 위치: x={self.tb2_position.x:.2f}, y={self.tb2_position.y:.2f}")
    
    def tb1_camera_callback(self, msg):
        with self.lock:
            try:
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # OpenCV 이미지로 변환
                # 처리할 내용 추가 (예: 저장, 디스플레이 등)
                # self.get_logger().info("tb1 카메라 데이터 수신")
                cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB, cv_image)
                self.tb1_image = cv_image
            except Exception as e:
                self.get_logger().error(f"카메라 데이터 변환 실패: {e}")



class ControlTowerGUI(QWidget):
    def __init__(self, node: SystemNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("화재 관제 시스템")
        self.setGeometry(100, 100, 1000, 800)
        self.setStyleSheet("background-color: white;")

        # 메인 레이아웃
        main_layout = QVBoxLayout()

        # 상단 영역 (지도, 화재 상태)
        top_layout = QHBoxLayout()
        self.create_map_section(top_layout)
        self.create_fire_status_section(top_layout)

        # 하단 영역 (모니터링, 로봇 상태)
        bottom_layout = QHBoxLayout()
        self.create_monitoring_section(bottom_layout)
        self.create_robot_monitoring_section(bottom_layout)

        # 레이아웃 합치기
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)
        self.setLayout(main_layout)

        # QTimer로 주기적으로 업데이트
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_gui)  # 주기적으로 업데이트 함수 호출
        self.timer.start(100)  # 100ms마다 호출

    def create_map_section(self, layout):
        """지도 표시 부분 생성"""
        self.map_label = QLabel("지도 로딩 중...")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
        layout.addWidget(self.map_label)

    def create_fire_status_section(self, layout):
        """실시간 화재 상태 표시 부분 생성"""
        self.fire_status_label = QLabel("화재 상태: 안전")
        self.fire_status_label.setAlignment(Qt.AlignCenter)
        self.fire_status_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
        layout.addWidget(self.fire_status_label)

    def create_monitoring_section(self, layout):
        """화면 모니터링 부분 생성"""
        self.monitoring_label = QLabel("모니터링 화면")
        self.monitoring_label.setAlignment(Qt.AlignCenter)
        self.monitoring_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
        layout.addWidget(self.monitoring_label)

    def create_robot_monitoring_section(self, layout):
        """로봇 실시간 모니터링 표시 부분 생성"""
        self.robot_monitoring_label = QLabel("로봇 상태: 대기 중")
        self.robot_monitoring_label.setAlignment(Qt.AlignCenter)
        self.robot_monitoring_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
        layout.addWidget(self.robot_monitoring_label)


    def update_gui(self):
        """GUI 업데이트 함수"""
        # ROS2 노드에서 데이터를 가져옴
        map_data, tb1_position, tb2_position, tb1_image = self.node.get_data()
        # 맵 데이터 업데이트
        if map_data is not None:
            self.update_display(map_data, tb1_position, tb2_position)
        else:
            self.map_label.setText("맵 데이터 없음")

        # tb1_image가 NumPy 배열이라고 가정
        if tb1_image is not None:
            try:
                # tb1_image가 NumPy 배열이며 RGB 형식인지 확인
                if isinstance(tb1_image, np.ndarray) and len(tb1_image.shape) == 3 and tb1_image.shape[2] == 3:
                    height, width, channel = tb1_image.shape
                    # OpenCV 배열을 QImage로 변환   
                    q_img = QImage(tb1_image.data, width, height, width * channel, QImage.Format_RGB888)
                    pixmap = QPixmap.fromImage(q_img)
                    self.monitoring_label.setPixmap(pixmap)
                    self.monitoring_label.setScaledContents(True)  # 이미지를 QLabel 크기에 맞게 조정
                    self.monitoring_label.setFixedSize(width, height)
                    print("tb1_image 업데이트")
                else:
                    self.monitoring_label.setText("올바르지 않은 이미지 형식")
            except Exception as e:
                self.node.get_logger().error(f"이미지 변환 오류: {e}")
                self.monitoring_label.setText("이미지 처리 오류")
        else:
            self.monitoring_label.setText("모니터링 화면: 이미지 없음")



    def update_display(self, map_data, tb1_position, tb2_position):
        """맵과 로봇 위치 업데이트"""
        if map_data is not None:
            pixmap = self.create_map_pixmap(map_data, tb1_position, tb2_position)
            self.map_label.setPixmap(pixmap)
        else:
            pixmap  = np.zeros((300, 300, 3), dtype=np.uint8)
            pixmap.fill(255)
            self.map_label.setPixmap(QPixmap.fromImage(QImage(pixmap, 800, 800, QImage.Format_RGB888)))

    def create_map_pixmap(self, map_data, tb1_position, tb2_position):
        try:
            width = map_data.info.width
            height = map_data.info.height
            resolution = map_data.info.resolution
            origin = map_data.info.origin.position

            grid_data = np.array(map_data.data, dtype=np.int8).reshape((height, width))
            normalized_data = np.zeros_like(grid_data, dtype=np.uint8)
            normalized_data[grid_data == -1] = 128  # Unknown: 회색
            normalized_data[grid_data == 0] = 255  # Free: 흰색
            normalized_data[grid_data > 0] = 0     # Occupied: 검은색

            image = QImage(normalized_data.data, width, height, QImage.Format_Grayscale8)
            pixmap = QPixmap.fromImage(image)

            painter = QPainter(pixmap)
            painter.setBrush(QColor(255, 0, 0))  # 빨간색

            if tb1_position:
                map_x = int((tb1_position.x - origin.x) / resolution)
                map_y = int((tb1_position.y - origin.y) / resolution)
                painter.drawEllipse(map_x - 5, map_y - 5, 10, 10)

            if tb2_position:
                map_x = int((tb2_position.x - origin.x) / resolution)
                map_y = int((tb2_position.y - origin.y) / resolution)
                painter.setBrush(QColor(0, 0, 255))  # 파란색
                painter.drawEllipse(map_x - 5, map_y - 5, 10, 10)

            painter.end()
            return pixmap
        except Exception as e:
            self.node.get_logger().error(f"맵 데이터 처리 중 오류: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)

    # ROS2 노드와 PyQt5 GUI 생성
    node = SystemNode()
    app = QApplication(sys.argv)
    gui = ControlTowerGUI(node)

    # ROS2 스핀을 별도의 쓰레드에서 실행
    def ros_spin():
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # PyQt5 GUI 실행
    gui.show()
    sys.exit(app.exec_())

    # ROS2 종료
    rclpy.shutdown()

if __name__ == "__main__":
    main()