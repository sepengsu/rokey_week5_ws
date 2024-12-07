import tkinter as tk
from tkinter import ttk
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FireControlCenter:
    def __init__(self,node):
        self.root= tk.Tk()
        self.root.title("화재 관제 시스템")
        self.root.geometry("1000x1000")
        self.root.configure(bg="white")  # 전체 배경 색상 설정
        style = ttk.Style()
        style.configure("Map.TLabelframe", background="#4e99b0", font=("Arial", 14), foreground="white")
        style.configure("FireStatus.TLabelframe", background="#5cb85c", font=("Arial", 14), foreground="white")
        style.configure("Monitoring.TLabelframe", background="#d9534f", font=("Arial", 14), foreground="white")
        style.configure("RobotMonitoring.TLabelframe", background="#f0ad4e", font=("Arial", 14), foreground="white")
        # 화재 경고 상태를 추적하는 변수
        self.fire_alarm_active = False

        # 지도 표시
        self.create_map_section()

        # 실시간 화재 상태 표시
        self.create_fire_status_section()

        # 화면 모니터링
        self.create_monitoring_section()

        # 로봇 실시간 모니터링
        self.create_robot_monitoring_section()

        # 실시간 화재 상태와 모니터링을 위한 쓰레드 실행
        self.start_threads()

        # GUI 메인 루프
        self.root.mainloop()


    def create_map_section(self):
        """지도 표시 부분을 생성"""
        map_frame = ttk.LabelFrame(self.root, text="map", width=400, height=300, padding=10)
        map_frame.grid(row=0, column=0, padx=20, pady=20)
        map_frame.configure(style="Map.TLabelframe")  # 스타일 적용
        self.map_label = tk.Label(map_frame, text="지도 표시 영역", width=40, height=10, bg="#ffffff", font=("Arial", 16))
        self.map_label.grid(row=0, column=0)

    def create_fire_status_section(self):
        """실시간 화재 상태 표시 부분을 생성"""
        fire_status_frame = ttk.LabelFrame(self.root, text="실시간 화재 상태", width=400, height=300, padding=10)
        fire_status_frame.grid(row=0, column=1, padx=20, pady=20)
        fire_status_frame.configure(style="FireStatus.TLabelframe")  # 스타일 적용
        self.fire_status_label = tk.Label(fire_status_frame, text="화재 상태: 안전", width=40, height=10, bg="#ffffff", font=("Arial", 16))
        self.fire_status_label.grid(row=0, column=0)

    def create_monitoring_section(self):
        """화면 모니터링 부분을 생성"""
        monitoring_frame = ttk.LabelFrame(self.root, text="화재 모니터링", width=400, height=300, padding=10)
        monitoring_frame.grid(row=1, column=0, padx=20, pady=20)
        monitoring_frame.configure(style="Monitoring.TLabelframe")  # 스타일 적용
        self.monitoring_label = tk.Label(monitoring_frame, text="모니터링 화면", width=40, height=10, bg="#ffffff", font=("Arial", 16))
        self.monitoring_label.grid(row=0, column=0)

    def create_robot_monitoring_section(self):
        """로봇 실시간 모니터링 표시 부분을 생성"""
        robot_monitoring_frame = ttk.LabelFrame(self.root, text="로봇 실시간 모니터링", width=400, height=300, padding=10)
        robot_monitoring_frame.grid(row=1, column=1, padx=20, pady=20)
        robot_monitoring_frame.configure(style="RobotMonitoring.TLabelframe")  # 스타일 적용
        self.robot_monitoring_label = tk.Label(robot_monitoring_frame, text="로봇 상태: 대기 중", width=40, height=10, bg="#ffffff", font=("Arial", 16))
        self.robot_monitoring_label.grid(row=0, column=0)

    def update_fire_status(self):
        """실시간 화재 상태를 업데이트하는 함수"""
        while True:
            # 여기서 실시간 화재 데이터 업데이트 로직 추가
            self.fire_status_label.config(text="화재 발생!", bg="#ffcccc")  # 화재 발생 시 배경 색상 변경
            self.fire_alarm_active = True  # 화재 경고 활성화
            time.sleep(5)  # 5초마다 상태 갱신

            # 화재 상태가 종료되면 경고 비활성화
            self.fire_status_label.config(text="화재 상태: 안전", bg="#ffffff")
            self.fire_alarm_active = False
            time.sleep(5)  # 5초마다 상태 갱신

    def update_monitoring(self):
        """실시간 화면 모니터링을 업데이트하는 함수"""
        while True:
            # 여기서 실시간 영상 업데이트 로직 추가 (예: OpenCV로 영상 표시)
            self.monitoring_label.config(text="모니터링 중...", bg="#ffcc99")  # 모니터링 중 상태 표시
            time.sleep(2)

    def update_robot_monitoring(self):
        """로봇 실시간 모니터링을 업데이트하는 함수"""
        while True:
            # 여기서 로봇의 상태 업데이트 로직 추가 (예: 센서 정보 또는 카메라 피드)
            self.robot_monitoring_label.config(text="로봇 상태: 작동 중", bg="#e5f7f7")  # 로봇 작동 중 상태 표시
            time.sleep(3)  # 3초마다 상태 갱신

    def blink_fire_monitoring(self):
        """화재 모니터링을 빨간 불빛처럼 깜박이게 하는 함수"""
        while True:
            if self.fire_alarm_active:  # 화재 경고가 활성화된 경우에만 깜박이게
                self.monitoring_label.config(bg="#ff3333")  # 빨간 배경
                time.sleep(0.5)  # 0.5초마다 색 변경
                self.monitoring_label.config(bg="#ffffff")  # 원래 배경색
                time.sleep(0.5)
            else:
                time.sleep(1)  # 화재 경고가 비활성화되면 깜박임을 멈춤

    def start_threads(self):
        """실시간 화재 상태와 화면 모니터링을 위한 쓰레드 실행"""
        fire_thread = threading.Thread(target=self.update_fire_status, daemon=True)
        monitoring_thread = threading.Thread(target=self.update_monitoring, daemon=True)
        robot_monitoring_thread = threading.Thread(target=self.update_robot_monitoring, daemon=True)
        blink_thread = threading.Thread(target=self.blink_fire_monitoring, daemon=True)

        fire_thread.start()
        monitoring_thread.start()
        robot_monitoring_thread.start()
        blink_thread.start()

class Node:
    pass

if __name__=="__main__":
    # FireControlCenter 인스턴스 생성
    node = Node()
    control_center = FireControlCenter(node)

 