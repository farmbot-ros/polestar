cmake_minimum_required(VERSION 3.15)
project(farmbot_polestar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
cmake_policy(SET CMP0074 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)


install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

FetchContent_Declare(
  concord
  GIT_REPOSITORY https://github.com/bresilla/concord.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(concord)


set(executables
  fix_n_bearing
  using_enu
  odom_n_path
  transform_pub
)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(farmbot_interfaces REQUIRED)

set(dependencies_ros
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  diagnostic_msgs
  diagnostic_updater
  farmbot_interfaces
)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})
find_package(fmt REQUIRED)

set(dependencies_sys
  fmt::fmt
  ${Eigen_LIBRARIES}
  concord
)

foreach(executable ${executables})
  add_executable(${executable} src/${executable}.cpp)
  ament_target_dependencies(${executable} ${dependencies_ros})
  target_link_libraries(${executable} ${dependencies_sys})
  # target_compile_options(${executable} PRIVATE -Wno-deprecated-declarations -Wunused-parameter)
endforeach()




install(TARGETS
  ${executables}
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
