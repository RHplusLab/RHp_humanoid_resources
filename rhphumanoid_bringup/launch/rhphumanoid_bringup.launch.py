from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, IncludeLaunchDescription
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
    gazebo_pkg = FindPackageShare("rhphumanoid_gazebo") # Gazebo 패키지 추가

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

    # 4. Robot Description 생성
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            xacro_file, " ",
            "use_gazebo:=", use_sim
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 5. 노드 정의: Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim}],
    )

    # 6. Gazebo 실행 (use_sim=true 일 때만)
    # rhphumanoid_gazebo 패키지의 launch 파일을 포함시킵니다.
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gazebo_pkg,
                "launch",
                "rhphumanoid_gz.launch.py" # Gazebo 서버/클라이언트 켜는 파일 이름 확인 필요
            ])
        ]),
        condition=IfCondition(use_sim)
    )

    # 7. Gazebo 로봇 스폰 (use_sim=true 일 때만)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rhphumanoid"],
        output="screen",
        condition=IfCondition(use_sim)
    )

    # 8. 실제 로봇용 Controller Manager (use_sim=false 일 때만)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_file],
        output="both",
        condition=UnlessCondition(use_sim)
    )

    # 9. 컨트롤러 Spawner 정의
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

    # 10. 실행 순서 제어 (Delay)

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

    # [시뮬레이션] spawn_entity 완료 후 스폰 (Gazebo가 컨트롤러 매니저를 켬)
    delay_spawners_sim = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                leg_controller_spawner,
                head_controller_spawner,
            ],
        ),
        condition=IfCondition(use_sim)
    )

    return LaunchDescription([
        use_sim_arg,
        robot_state_publisher_node,
        include_gazebo,  # Gazebo 실행
        spawn_entity,    # 로봇 생성
        control_node,    # 실제 로봇 연결
        delay_spawners_real,
        delay_spawners_sim,
    ])
