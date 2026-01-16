import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 1. 패키지 경로 찾기
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    pkg_rhphumanoid_gazebo = FindPackageShare("rhphumanoid_gazebo")

    # 2. GUI 설정 파일(gui.config) 경로 설정
    # (주의: gui.config 파일이 rhphumanoid_gazebo/config 폴더에 있어야 합니다)
    gui_config_path = PathJoinSubstitution([
        pkg_rhphumanoid_gazebo, "config", "gui.config"
    ])

    # 3. Gazebo 실행 인수 설정
    # -r: 시뮬레이션 자동 실행
    # empty.sdf: 빈 월드 실행 (나중에 다른 월드로 교체 가능)
    # --gui-config: 우리가 만든 화면 설정 파일 불러오기
    gz_args = [
        "-r empty.sdf ",
        "--gui-config ", gui_config_path
    ]

    # 4. Gazebo 시뮬레이터 실행 (ros_gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    return LaunchDescription([
        gazebo
    ])
