from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  ld = LaunchDescription()

  rhphumanoid_description_path = FindPackageShare('rhphumanoid_description')
  rhphumanoid_urdf_path = PathJoinSubstitution([rhphumanoid_description_path, 'urdf', 'rhphumanoid.urdf.xacro'])
  rhphumanoid_description_content = ParameterValue(Command(['xacro ', rhphumanoid_urdf_path]), value_type=str)

  default_rviz_config_path = PathJoinSubstitution([rhphumanoid_description_path, 'rviz', 'rhphumanoid.rviz'])

  # Launch description 
  # joint_state_publisher 
#   ld.add_action(Node(
#     package='joint_state_publisher',
#     executable='joint_state_publisher',
#     parameters=[{'source_list': ['/present_joint_states']}],
#     remappings=[('/joint_states', '/present_joint_states')])
#   )

  # joint_state_publisher_gui
  ld.add_action(Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    parameters=[{'source_list': ['/present_joint_states']}],
    remappings=[('/joint_states', '/present_joint_states')])
  )

  # robot_state_publisher 
  ld.add_action(Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': rhphumanoid_description_content,}],
    remappings=[('/joint_states', '/present_joint_states'),],)
  )

  # Rviz 
  ld.add_action(Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', default_rviz_config_path],)
  )

  return ld
