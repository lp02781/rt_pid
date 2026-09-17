cmake_minimum_required(VERSION 3.16)
project(rt_pid)

# -Release by default: an unoptimised control loop wastes time it does not have.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# rt_pid_loop -- the real-time process. Note it links NO ROS libraries at all:
# that is the whole point of the design, and CMake enforces it here.
# ---------------------------------------------------------------------------
add_executable(rt_pid_loop src/rt_pid_loop.cpp)
target_include_directories(rt_pid_loop PRIVATE include)
target_compile_features(rt_pid_loop PRIVATE cxx_std_17)
# librt provides shm_open/shm_unlink; pthread is needed for the atomics runtime.
target_link_libraries(rt_pid_loop rt pthread)

# ---------------------------------------------------------------------------
# pid_node -- the ROS 2 process.
# ---------------------------------------------------------------------------
add_executable(pid_node src/pid_node.cpp)
target_include_directories(pid_node PRIVATE include)
target_compile_features(pid_node PRIVATE cxx_std_17)
target_link_libraries(pid_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  rt
)

install(TARGETS rt_pid_loop pid_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
