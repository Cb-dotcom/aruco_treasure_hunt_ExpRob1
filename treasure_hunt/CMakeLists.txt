cmake_minimum_required(VERSION 3.8)
project(treasure_hunt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(treasure_hunt_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/aruco_detector_component.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge image_transport rclcpp rclcpp_components sensor_msgs treasure_hunt_interfaces
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "treasure_hunt::ArucoDetectorComponent" EXECUTABLE aruco_detector)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
