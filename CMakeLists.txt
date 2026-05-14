cmake_minimum_required(VERSION 3.8)
project(serl_franka_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/ZeroJacobian.msg"
  DEPENDENCIES std_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/joint_position_controller.cpp
  src/cartesian_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rosidl_typesupport_fastrtps_cpp
  controller_interface
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(fastrtps_cpp_typesupport_target
  ${PROJECT_NAME}_msgs "rosidl_typesupport_fastrtps_cpp")

target_link_libraries(${PROJECT_NAME}
  ${cpp_typesupport_target}
  ${fastrtps_cpp_typesupport_target}
  ${Franka_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(
  controller_interface serl_franka_controllers_plugin.xml)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "$ORIGIN/../lib"
  INSTALL_RPATH "$ORIGIN/../lib"
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rosidl_typesupport_fastrtps_cpp
  controller_interface
  franka_semantic_components
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \
  libserl_franka_controllers_msgs__rosidl_generator_c.so \
  libserl_franka_controllers__rosidl_generator_c.so \
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)")

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \
  libserl_franka_controllers_msgs__rosidl_typesupport_cpp.so \
  libserl_franka_controllers__rosidl_typesupport_cpp.so \
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)")

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \
  libserl_franka_controllers_msgs__rosidl_typesupport_c.so \
  libserl_franka_controllers__rosidl_typesupport_c.so \
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)")

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \
  libserl_franka_controllers_msgs__rosidl_typesupport_fastrtps_cpp.so \
  libserl_franka_controllers__rosidl_typesupport_fastrtps_cpp.so \
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)")

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \
  libserl_franka_controllers_msgs__rosidl_typesupport_fastrtps_c.so \
  libserl_franka_controllers__rosidl_typesupport_fastrtps_c.so \
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)")

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \
  libserl_franka_controllers_msgs__rosidl_typesupport_introspection_cpp.so \
  libserl_franka_controllers__rosidl_typesupport_introspection_cpp.so \
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)")

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \
  libserl_franka_controllers_msgs__rosidl_typesupport_introspection_c.so \
  libserl_franka_controllers__rosidl_typesupport_introspection_c.so \
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)")

ament_package()