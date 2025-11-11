cmake_minimum_required(VERSION 3.8)
project(robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 包含目录
include_directories(
  ${PROJECT_SOURCE_DIR}/control/Inc
  /workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware
  /workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware/unitree_legs
  /workspaces/RMCS/rmcs_ws/src/rmcs_core/librmcs
  /usr/include/libusb-1.0 
)

# 检测系统架构并选择对应的库
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
  set(UNITREE_LIB_NAME "UnitreeMotorSDK_Arm64")
  message(STATUS "Detected ARM64 architecture, using libUnitreeMotorSDK_Arm64.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
  set(UNITREE_LIB_NAME "UnitreeMotorSDK_Linux64")
  message(STATUS "Detected x86_64 architecture, using libUnitreeMotorSDK_Linux64.so")
else()
  message(WARNING "Unknown architecture ${CMAKE_SYSTEM_PROCESSOR}, defaulting to Linux64")
  set(UNITREE_LIB_NAME "UnitreeMotorSDK_Linux64")
endif()

# 创建硬件接口库
add_library(${PROJECT_NAME}_hardware SHARED
  control/Src/My_arm.cpp
)

target_include_directories(${PROJECT_NAME}_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/control/Inc>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# 链接宇树库
target_link_libraries(${PROJECT_NAME}_hardware
  -lusb-1.0
  /workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware/lib/lib${UNITREE_LIB_NAME}.so
)

# 设置 RPATH，让动态链接器能找到 Unitree SDK
set_target_properties(${PROJECT_NAME}_hardware PROPERTIES
  BUILD_RPATH "/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware/lib"
  INSTALL_RPATH "/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware/lib:/usr/local/lib:$ORIGIN"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# 插件描述文件
pluginlib_export_plugin_description_file(hardware_interface robot_arm_hardware.xml)

# 安装
install(TARGETS ${PROJECT_NAME}_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY control/Inc/
  DESTINATION include
)

# 安装目录
install(DIRECTORY 
  launch
  config
  urdf
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_hardware)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

ament_package()