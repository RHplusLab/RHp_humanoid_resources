from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    # Gazebo Harmonic 실행 (빈 월드)
    # 필요한 경우 "empty.sdf" 대신 다른 월드 파일을 지정할 수 있습니다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    return LaunchDescription([
        gazebo
    ])
