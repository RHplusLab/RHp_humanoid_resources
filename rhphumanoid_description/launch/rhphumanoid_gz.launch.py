import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    rhp_share = get_package_share_directory("rhphumanoid_description")  
    share_parent = os.path.dirname(rhp_share)                            

    # --- Args ---
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo GUI (gz sim).",
        )
    )
    gui = LaunchConfiguration("gui")


    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(
            [os.environ.get("GZ_SIM_RESOURCE_PATH", ""), share_parent]
        ).strip(os.pathsep),
    )


    set_gz_model_path = SetEnvironmentVariable(
        name="GZ_SIM_MODEL_PATH",
        value=os.pathsep.join(
            [os.environ.get("GZ_SIM_MODEL_PATH", ""), share_parent]
        ).strip(os.pathsep),
    )

    # --- Gazebo (gz sim) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", "--headless-rendering -s -r -v 3 empty.sdf")],
        condition=UnlessCondition(gui),
    )

    # --- Bridge clock ---
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # --- robot_description from xacro ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rhphumanoid_description"), "urdf", "rhphumanoid.urdf.xacro"]
            ),
            " ",
            "use_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            **robot_description,
            "use_sim_time": True,
            "publish_robot_description": True,
        }],
    )

    # --- Controllers yaml ---
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("rhphumanoid_description"), "config", "rhphumanoid_controllers.yaml"]
    )

    # --- Spawn entity in Gazebo from /robot_description ---
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "rhphumanoid",
            "-x", "0", "-y", "0", "-z", "1.0",
        ],
    )
    spawn_delayed = TimerAction(period=2.0, actions=[gz_spawn_entity])

    # --- Spawners ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
        output="screen",
    )

    nodes = [
        set_gz_resource_path,
        set_gz_model_path,

        gazebo,
        gazebo_headless,
        gazebo_bridge,

        node_robot_state_publisher,
        spawn_delayed,

        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
