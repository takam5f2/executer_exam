cmake_minimum_required(VERSION 3.8)
project(executor_exam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_library(sample_node SHARED src/sample_node.cpp)
target_compile_definitions(sample_node PRIVATE "SAMPLE_NODE_BUILDING_LIBRARY")
ament_target_dependencies(sample_node
  rclcpp
  std_msgs
  rclcpp_components
)
rclcpp_components_register_nodes(sample_node  "executor_exam::SampleNode")

add_executable(timer_callback_multi_thread_node src/timer_callback_multi_thread.cpp)
add_executable(timer_callback_single_thread_node src/timer_callback_single_thread.cpp)

target_link_libraries(timer_callback_multi_thread_node sample_node)
target_link_libraries(timer_callback_single_thread_node sample_node)
ament_target_dependencies(timer_callback_multi_thread_node rclcpp std_msgs)
ament_target_dependencies(timer_callback_single_thread_node rclcpp std_msgs)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(std_msgs)


install(DIRECTORY
  include/executor_exam
  DESTINATION include
)

install(TARGETS
  sample_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install (TARGETS
  timer_callback_multi_thread_node
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  timer_callback_single_thread_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
