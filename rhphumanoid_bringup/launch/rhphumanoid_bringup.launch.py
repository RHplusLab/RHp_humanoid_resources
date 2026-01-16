from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 패키지 경로 설정
    description_pkg = FindPackageShare("rhphumanoid_description")
    bringup_pkg = FindPackageShare("rhphumanoid_bringup")
    gazebo_pkg = FindPackageShare("rhphumanoid_gazebo")

    # 2. 파일 경로 설정
    xacro_file = PathJoinSubstitution([description_pkg, "urdf", "rhphumanoid.urdf.xacro"])
    controller_file = PathJoinSubstitution([bringup_pkg, "config", "rhphumanoid_controllers.yaml"])

    # 3. Launch Arguments
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Start robot in Gazebo simulation if true, otherwise real hardware"
    )
    use_sim = LaunchConfiguration("use_sim")

    # 4. Robot Description 생성 (Xacro 실행)
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            xacro_file, " ",
            "use_gazebo:=", use_sim
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 5. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim}],
    )

    # 6. [Gazebo 전용] 시뮬레이터 실행 (ros_gz_sim)
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_pkg, "launch", "rhphumanoid_gz.launch.py"])
        ]),
        condition=IfCondition(use_sim)
    )

    # 7. [Gazebo 전용] 로봇 스폰 (ros_gz_sim create)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "rhphumanoid",
            "-z", "1.0",  # 로봇 시작 높이
        ],
        output="screen",
        condition=IfCondition(use_sim)
    )

    # 8. [Gazebo 전용] ROS-Gazebo Bridge (시간 동기화 필수)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(use_sim)
    )

    # 9. [실제 로봇 전용] Controller Manager 실행
    # (Gazebo에서는 플러그인이 매니저를 실행하므로 실행하지 않음)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_file],
        output="both",
        condition=UnlessCondition(use_sim)
    )

    # 10. 컨트롤러 Spawner 정의
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_controller", "--controller-manager", "/controller_manager"],
    )

    head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller", "--controller-manager", "/controller_manager"],
    )

    # 11. 실행 순서 제어 (Delay)

    # [실제 로봇] ros2_control_node 켜진 후 스폰
    delay_spawners_real = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                leg_controller_spawner,
                head_controller_spawner,
            ],
        ),
        condition=UnlessCondition(use_sim)
    )

    # [시뮬레이션] Gazebo 스폰 후 안전하게 대기 후 컨트롤러 실행 (Timer 사용)
    # Gazebo 내부 Controller Manager가 준비될 시간을 줍니다.
    delay_spawners_sim = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            leg_controller_spawner,
            head_controller_spawner,
        ],
        condition=IfCondition(use_sim)
    )

    return LaunchDescription([
        use_sim_arg,
        robot_state_publisher_node,
        include_gazebo,
        spawn_entity,
        bridge,
        control_node,
        delay_spawners_real,
        delay_spawners_sim,
    ])
