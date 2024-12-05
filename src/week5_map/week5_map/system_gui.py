
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
from PyQt5.QtGui import QPixmap, QColor, QImage
from PyQt5.QtCore import Qt, QTimer, QThread

def map_to_pixmap(map_data):
        """OccupancyGrid 데이터를 QPixmap으로 변환"""
        width = map_data.info.width
        height = map_data.info.height

        # OccupancyGrid 데이터를 numpy 배열로 변환
        data = np.array(map_data.data, dtype=np.int8).reshape((height, width))

        # 값의 범위를 0-255로 정규화
        normalized_data = np.zeros_like(data, dtype=np.uint8)
        normalized_data[data == -1] = 128  # unknown은 중간값(회색)
        normalized_data[data >= 0] = (100 - data[data >= 0]) * 2.55  # 0: 흰색, 100: 검은색

        # numpy 배열을 QImage로 변환
        image = QImage(normalized_data.data, width, height, QImage.Format_Grayscale8)

        # QImage를 QPixmap으로 변환
        return QPixmap.fromImage(image)

class ControlTowerGUI(QWidget):
    def __init__(self,node: SystemNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("화재 관제 시스템")
        self.setGeometry(100, 100, 1000, 800)
        self.setStyleSheet("background-color: white;")
        
        # 화재 경고 상태를 추적하는 변수
        self.fire_alarm_active = False

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

        # 실시간 업데이트를 위한 쓰레드 실행
        self.start_threads()

    def create_map_section(self, layout):
        """지도 표시 부분 생성"""
        map_frame = QFrame()
        map_frame.setStyleSheet("background-color: #4e99b0; border-radius: 10px;")
        map_layout = QVBoxLayout()

        # 지도 이미지
        temp_image = QPixmap("map.png")
        map_label = QLabel()
        map_label.setPixmap(temp_image.scaled(300, 200, Qt.KeepAspectRatio))
        map_label.setAlignment(Qt.AlignCenter)
        map_layout.addWidget(map_label)

        map_frame.setLayout(map_layout)
        layout.addWidget(map_frame)

    def update_map(self):
        """ROS2 노드에서 맵 데이터를 가져와 GUI에 표시"""
        

        if map_data is not None:
            # OccupancyGrid 데이터를 QPixmap으로 변환
            pixmap = self.map_to_pixmap(map_data)
            self.map_label.setPixmap(pixmap)

    def create_fire_status_section(self, layout):
        """실시간 화재 상태 표시 부분 생성"""
        fire_status_frame = QFrame()
        fire_status_frame.setStyleSheet("background-color: #5cb85c; border-radius: 10px;")
        fire_status_layout = QVBoxLayout()

        self.fire_status_label = QLabel("화재 상태: 안전")
        self.fire_status_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
        self.fire_status_label.setAlignment(Qt.AlignCenter)
        fire_status_layout.addWidget(self.fire_status_label)

        fire_status_frame.setLayout(fire_status_layout)
        layout.addWidget(fire_status_frame)

    def create_monitoring_section(self, layout):
        """화면 모니터링 부분 생성"""
        monitoring_frame = QFrame()
        monitoring_frame.setStyleSheet("background-color: #d9534f; border-radius: 10px;")
        monitoring_layout = QVBoxLayout()

        self.monitoring_label = QLabel("모니터링 화면")
        self.monitoring_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
        self.monitoring_label.setAlignment(Qt.AlignCenter)
        monitoring_layout.addWidget(self.monitoring_label)

        monitoring_frame.setLayout(monitoring_layout)
        layout.addWidget(monitoring_frame)

    def create_robot_monitoring_section(self, layout):
        """로봇 실시간 모니터링 표시 부분 생성"""
        robot_monitoring_frame = QFrame()
        robot_monitoring_frame.setStyleSheet("background-color: #f0ad4e; border-radius: 10px;")
        robot_monitoring_layout = QVBoxLayout()

        self.robot_monitoring_label = QLabel("로봇 상태: 대기 중")
        self.robot_monitoring_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
        self.robot_monitoring_label.setAlignment(Qt.AlignCenter)
        robot_monitoring_layout.addWidget(self.robot_monitoring_label)

        robot_monitoring_frame.setLayout(robot_monitoring_layout)
        layout.addWidget(robot_monitoring_frame)

    def update_fire_status(self):
        """실시간 화재 상태를 업데이트하는 함수"""
        while True:
            self.fire_status_label.setText("화재 발생!")
            self.fire_status_label.setStyleSheet("font-size: 16px; background-color: #ffcccc; padding: 20px;")
            self.fire_alarm_active = True  # 화재 경고 활성화
            time.sleep(5)

            self.fire_status_label.setText("화재 상태: 안전")
            self.fire_status_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
            self.fire_alarm_active = False
            time.sleep(5)

    def update_monitoring(self):
        """실시간 화면 모니터링을 업데이트하는 함수"""
        while True:
            self.monitoring_label.setText("모니터링 중...")
            self.monitoring_label.setStyleSheet("font-size: 16px; background-color: #ffcc99; padding: 20px;")
            time.sleep(2)

    def update_robot_monitoring(self):
        """로봇 실시간 모니터링을 업데이트하는 함수"""
        while True:
            self.robot_monitoring_label.setText("로봇 상태: 작동 중")
            self.robot_monitoring_label.setStyleSheet("font-size: 16px; background-color: #e5f7f7; padding: 20px;")
            time.sleep(3)

    def blink_fire_monitoring(self):
        """화재 모니터링을 빨간 불빛처럼 깜박이게 하는 함수"""
        while True:
            if self.fire_alarm_active:
                self.monitoring_label.setStyleSheet("font-size: 16px; background-color: #ff3333; padding: 20px;")
                time.sleep(0.5)
                self.monitoring_label.setStyleSheet("font-size: 16px; background-color: white; padding: 20px;")
                time.sleep(0.5)
            else:
                time.sleep(1)

    def start_threads(self):
        """실시간 화재 상태와 모니터링을 위한 쓰레드 실행"""
        fire_thread = threading.Thread(target=self.update_fire_status, daemon=True)
        monitoring_thread = threading.Thread(target=self.update_monitoring, daemon=True)
        robot_monitoring_thread = threading.Thread(target=self.update_robot_monitoring, daemon=True)
        blink_thread = threading.Thread(target=self.blink_fire_monitoring, daemon=True)

        fire_thread.start()
        monitoring_thread.start()
        robot_monitoring_thread.start()
        blink_thread.start()

class SystemNode(Node):
    def __init__(self):
        rclpy.init()
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_data = None
        self.map_data_lock = threading.Lock()

        # Timer for periodic TF requests
        self.timer = self.create_timer(0.1, self.update_robot_position)  # 0.1초 간격
        self.robot_position = None  # 로봇 위치 저장
        
    def map_callback(self, msg):
        """맵 데이터를 저장"""
        with self.map_data_lock:
            self.map_data = msg

    def get_map(self):
        """맵 데이터를 안전하게 반환"""
        with self.map_data_lock:
            return self.map_data

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_position = (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
            self.get_logger().info(f"Robot Position: {self.robot_position}")
        except Exception as e:
            self.get_logger().info(f"Could not get transform: {e}")


if __name__ == "__main__":
    