cmake_minimum_required(VERSION 3.5)
project(rhphumanoid_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY doc launch meshes rviz urdf gazebo ros2_control config
	DESTINATION share/${PROJECT_NAME}
)
