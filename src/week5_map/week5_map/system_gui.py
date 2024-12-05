
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer

import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QFrame
from PyQt5.QtGui import QPixmap, QColor, QImage, QPainter
from PyQt5.QtCore import Qt, QTimer, QThread
from geometry_msgs.msg import PoseWithCovarianceStamped

class SystemNode(Node):
    def __init__(self):
        super().__init__('system_node')  # Node 이름 설정

        # ROS2 토픽 구독
        self.map_subscription = self.create_subscription(OccupancyGrid,'/map',
            self.map_callback,10)
        self.tb1_subscription = self.create_subscription(PoseWithCovarianceStamped,'/tb1/amcl_pose',
            self.tb1_callback, 10)
        self.tb2_subscription = self.create_subscription(PoseWithCovarianceStamped,'/tb2/amcl_pose',
            self.tb2_callback, 10)

        # 데이터 저장 및 락 설정
        self.map_data = None
        self.tb1_position = None
        self.tb2_position = None
        self.lock = threading.Lock()

    def map_callback(self, msg):
        """맵 데이터를 저장"""
        with self.lock:
            self.map_data = msg
            self.get_logger().info("Map data updated.")

    def get_map(self):
        """맵 데이터를 반환 (스레드 안전)"""
        with self.lock:
            return self.map_data

    def tb1_callback(self, msg):
        """tb1의 위치를 저장"""
        with self.lock:
            self.tb1_position = msg.pose.pose.position
        self.get_logger().info(f"tb1 위치: x={self.tb1_position.x:.2f}, y={self.tb1_position.y:.2f}")

    def tb2_callback(self, msg):
        """tb2의 위치를 저장"""
        with self.lock:
            self.tb2_position = msg.pose.pose.position
        self.get_logger().info(f"tb2 위치: x={self.tb2_position.x:.2f}, y={self.tb2_position.y:.2f}")

    def get_robot_positions(self):
        """로봇 위치를 반환 (스레드 안전)"""
        with self.lock:
            return self.tb1_position, self.tb2_position

class ControlTowerGUI(QWidget):
    def __init__(self, node):
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
        # 맵과 로봇 위치를 가져와 업데이트
        map_data = self.node.map_data  # SystemNode의 맵 데이터 가져오기
        tb1_position, tb2_position = self.node.get_robot_positions()  # 로봇 위치 가져오기

        # 맵 업데이트
        self.update_display(map_data, tb1_position, tb2_position)

    def update_display(self, map_data, tb1_position, tb2_position):
        """맵과 로봇 위치 업데이트"""
        if map_data is not None:
            pixmap = self.create_map_pixmap(map_data, tb1_position, tb2_position)
            self.map_label.setPixmap(pixmap)
        else:
            print("No map data")

    def create_map_pixmap(self, map_data, tb1_position, tb2_position):
        
        """맵과 로봇 위치를 결합하여 QPixmap 생성"""
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin = map_data.info.origin.position

        # OccupancyGrid 데이터를 numpy 배열로 변환
        grid_data = np.array(map_data.data, dtype=np.int8).reshape((height, width))
        width = map_data.info.width
        height = map_data.info.height
        grid_data = np.array(map_data.data, dtype=np.int8).reshape((height, width))

        # 맵 데이터를 이미지로 변환
        normalized_data = np.zeros_like(grid_data, dtype=np.uint8)
        normalized_data[grid_data == -1] = 128  # Unknown: 회색
        normalized_data[grid_data == 0] = 255  # Free: 흰색
        normalized_data[grid_data > 0] = 0     # Occupied: 검은색

        image = QImage(normalized_data.data, width, height, QImage.Format_Grayscale8)
        # numpy 배열을 QPixmap으로 변환
        pixmap = QPixmap.fromImage(image)

        # 로봇 위치 표시
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

def main(args=None):
    rclpy.init(args=args)

    # ROS2 노드와 PyQt5 GUI 생성
    node = SystemNode()
    app = QApplication(sys.argv)
    gui = ControlTowerGUI(node)

    # ROS2 스핀을 별도의 쓰레드에서 실행
    def ros_spin():
        rclpy.spin(node)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # PyQt5 GUI 실행
    gui.show()
    sys.exit(app.exec_())

    # ROS2 종료
    rclpy.shutdown()

if __name__ == "__main__":
    main()