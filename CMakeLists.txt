cmake_minimum_required(VERSION 3.0.2)
project(sonar_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)

# Add Fusion library subdirectory
add_subdirectory(lib/Fusion/Fusion)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  lib/Fusion/Fusion
)

# Collect all cpp files in src directory
file(GLOB SOURCES "src/*.cpp")

add_executable(main ${SOURCES})
target_link_libraries(main
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Open3D_LIBRARIES}
  ${Eigen3_LIBRARIES}
  Fusion
)
