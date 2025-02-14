import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tkinter import Tk, Canvas


class MapGuiNode(Node):
    def __init__(self):
        super().__init__('map_gui_node')

        # 캔버스 크기
        self.canvas_width = 500
        self.canvas_height = 500

        # ROS2 맵 구독
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # GUI 초기화
        self.root = Tk()
        self.root.title("Scaled Map GUI")
        self.canvas = Canvas(self.root, width=self.canvas_width, height=self.canvas_height, bg='white')
        self.canvas.pack()

    def map_callback(self, msg):
        self.get_logger().info("Received map data")
        
        # 맵 정보
        map_width = msg.info.width
        map_height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        # 실제 맵 크기 (미터 단위)
        map_real_width = map_width * resolution
        map_real_height = map_height * resolution

        # 캔버스와 맵의 스케일링 비율 계산
        scale_x = self.canvas_width / map_real_width
        scale_y = self.canvas_height / map_real_height
        scale = min(scale_x, scale_y)  # 비율 유지

        # 맵 데이터를 GUI 캔버스 좌표로 변환
        self.canvas.delete("all")  # 이전 데이터를 삭제
        for y in range(map_height):
            for x in range(map_width):
                # OccupancyGrid 값 (-1: 알 수 없음, 0: 비어 있음, 100: 장애물)
                value = data[y * map_width + x]
                if value == -1:
                    color = 'gray'  # 알 수 없음
                elif value == 0:
                    color = 'white'  # 비어 있음
                else:
                    color = 'black'  # 장애물

                # 픽셀 좌표 계산
                canvas_x = int((x * resolution - origin_x) * scale)
                canvas_y = int((map_height - y - 1) * resolution * scale)  # Y축 반전
                rect_size = int(resolution * scale)

                # GUI 캔버스에 사각형으로 그리기
                self.canvas.create_rectangle(
                    canvas_x, canvas_y,
                    canvas_x + rect_size, canvas_y + rect_size,
                    fill=color, outline=""
                )

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = MapGuiNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()