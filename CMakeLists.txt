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

function(add_library_dependencies library)
  target_compile_definitions(${library} PRIVATE "SAMPLE_NODE_BUILDING_DLL")
  ament_target_dependencies(${library}
    "rclcpp"
    "rclcpp_components"
    "std_msgs")
endfunction()

# library for node.
add_library(sample_node SHARED src/nodes/sample_node.cpp)
add_library(tiny_sample_node SHARED src/nodes/tiny_sample_node.cpp)
add_library(callback_group_based_sample_node SHARED src/nodes/callback_group_based_sample_node.cpp)
add_library(throw_message_node SHARED src/nodes/throw_message_node.cpp)
add_library(receive_message_node SHARED src/nodes/receive_message_node.cpp)

add_library_dependencies(sample_node)
add_library_dependencies(tiny_sample_node)
add_library_dependencies(callback_group_based_sample_node)
add_library_dependencies(throw_message_node)
add_library_dependencies(receive_message_node)

rclcpp_components_register_nodes(sample_node  "executor_exam::SampleNode")
rclcpp_components_register_nodes(tiny_sample_node  "executor_exam::TinySampleNode")
rclcpp_components_register_nodes(callback_group_based_sample_node  "executor_exam::CallbackGroupBasedSampleNode")
rclcpp_components_register_nodes(throw_message_node "executor_exam::ThrowMessageNode")
rclcpp_components_register_nodes(receive_message_node  "executor_exam::ReceiveMessageNode")

# executable files.
add_executable(timer_callback_multi_thread_exec src/timer_callback_multi_thread.cpp)
add_executable(timer_callback_single_thread_exec src/timer_callback_single_thread.cpp)
add_executable(multi_node_timer_callback_multi_thread_exec src/multi_node_timer_callback_multi_thread.cpp)
add_executable(reentrant_timer_callback_multi_thread_exec src/reentrant_timer_callback_multi_thread.cpp)
add_executable(throw_recv_multi_thread_exec src/throw_recv_multi_thread.cpp)

target_link_libraries(timer_callback_multi_thread_exec sample_node)
target_link_libraries(timer_callback_single_thread_exec sample_node)
target_link_libraries(multi_node_timer_callback_multi_thread_exec sample_node tiny_sample_node)
target_link_libraries(reentrant_timer_callback_multi_thread_exec callback_group_based_sample_node)
target_link_libraries(throw_recv_multi_thread_exec throw_message_node receive_message_node)

ament_target_dependencies(timer_callback_multi_thread_exec rclcpp std_msgs)
ament_target_dependencies(timer_callback_single_thread_exec rclcpp std_msgs)
ament_target_dependencies(multi_node_timer_callback_multi_thread_exec rclcpp std_msgs)
ament_target_dependencies(reentrant_timer_callback_multi_thread_exec rclcpp std_msgs)
ament_target_dependencies(throw_recv_multi_thread_exec rclcpp std_msgs)

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
install(TARGETS
  tiny_sample_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS
  callback_group_based_sample_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS
  throw_message_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS
  receive_message_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


install (TARGETS
  timer_callback_single_thread_exec
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  timer_callback_multi_thread_exec
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  multi_node_timer_callback_multi_thread_exec
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  reentrant_timer_callback_multi_thread_exec
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  throw_recv_multi_thread_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
