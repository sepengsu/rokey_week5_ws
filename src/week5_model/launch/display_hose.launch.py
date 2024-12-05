import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # `use_sim_time` 설정
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 패키지 경로와 Xacro 파일 경로
    pkg_path = os.path.join(get_package_share_directory("week5_model"))
    xacro_file = os.path.join(pkg_path, "urdf", "hose_robot.urdf.xacro")
    
    # Xacro 파일 처리 및 로봇 설명 생성
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # RViz2 설정 파일 경로
    rviz_config_file = os.path.join(pkg_path, "config", "hose_robot.rviz")

    # 파라미터 정의
    params = {"robot_description": robot_description, "use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            # `use_sim_time` 런치 아규먼트 선언
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true"
            ),
            # Robot State Publisher 노드
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            # RViz2 노드
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],  # RViz 설정 파일 경로
            ),
        ],
    )