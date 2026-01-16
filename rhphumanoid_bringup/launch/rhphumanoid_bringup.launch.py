from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 패키지 경로 설정
    description_pkg_share = FindPackageShare("rhphumanoid_description")
    bringup_pkg_share = FindPackageShare("rhphumanoid_bringup")

    # 2. 파일 경로 설정
    # URDF는 description 패키지에 있음
    xacro_file = PathJoinSubstitution([description_pkg_share, "urdf", "rhphumanoid.urdf.xacro"])

    # Controller Config는 bringup 패키지의 config 폴더로 이동했다고 가정
    controller_params_file = PathJoinSubstitution([bringup_pkg_share, "config", "rhphumanoid_controllers.yaml"])

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

    # 5. 노드 정의

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim}],
    )

    # Controller Manager (실제 로봇일 때만 실행)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="both",
        condition=UnlessCondition(use_sim)
    )

    # 6. 컨트롤러 Spawner 정의
    # (주의: controller.yaml에 정의한 이름과 일치해야 합니다)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 우리가 나눈 3개의 컨트롤러
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

    # 7. 실행 순서 제어 (Delay)
    # 실제 하드웨어 구동 시 controller_manager가 뜬 뒤에 spawner를 실행
    delay_spawners_for_real_robot = RegisterEventHandler(
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

    # 시뮬레이션일 경우 (Gazebo가 뜬 뒤 실행해야 하므로 Timer 사용)
    delay_spawners_for_sim = TimerAction(
        period=5.0, # Gazebo 로딩 시간에 맞춰 조절
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            leg_controller_spawner,
            head_controller_spawner
        ],
        condition=IfCondition(use_sim)
    )

    return LaunchDescription([
        use_sim_arg,
        robot_state_publisher_node,
        control_node,
        delay_spawners_for_real_robot,
        delay_spawners_for_sim,
    ])
