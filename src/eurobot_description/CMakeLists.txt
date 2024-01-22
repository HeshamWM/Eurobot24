cmake_minimum_required(VERSION 3.5)
project(eurobot_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Generate URDF from xacro
xacro_add_files(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/mon_robot.urdf.xacro
  INSTALL DESTINATION urdf
)

install(
  DIRECTORY
    urdf
    rviz
    #meshes -- Pour le jour où j'en mettrai
    config
    gazebo
  DESTINATION share/${PROJECT_NAME}
)

ament_package()