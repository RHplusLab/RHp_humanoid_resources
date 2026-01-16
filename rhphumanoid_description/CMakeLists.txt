cmake_minimum_required(VERSION 3.8)
project(rhphumanoid_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 데이터 파일 설치
install(DIRECTORY
  urdf
  meshes
  rviz
  ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
