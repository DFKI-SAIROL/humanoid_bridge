cmake_minimum_required(VERSION 3.15)
project(robot_bridge)


## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Choose build type
set(default_build_type "Release")

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bridge_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)


include_directories(include include/common)

# ==============================================================Unitree H1 and G1=============================================================

add_executable(G1_bridge
  src/bridge_core.cpp
  src/estop.cpp
  src/gates.cpp
  src/G1_bridge.cpp
  src/g1_node.cpp
  src/common/motor_crc_hg.cpp
)

ament_target_dependencies(G1_bridge
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  bridge_interface
  rosbag2_cpp
  rosidl_default_runtime
)


install(
  TARGETS G1_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



add_executable(H1_bridge
  src/bridge_core.cpp
  src/estop.cpp
  src/H1_bridge.cpp
  src/h1_node.cpp
  src/common/motor_crc.cpp
)

ament_target_dependencies(H1_bridge
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  bridge_interface
  rosbag2_cpp
  rosidl_default_runtime
)

ament_python_install_package(robot_bridge)


install(
  TARGETS H1_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Unitree Go2: unitree_go, 12 motors in the 20-slot frame (go2_frame.hpp).
add_executable(Go2_bridge
  src/bridge_core.cpp
  src/estop.cpp
  src/gates.cpp
  src/go2_frame.cpp
  src/Go2_bridge.cpp
  src/go2_node.cpp
  src/common/motor_crc.cpp
)

ament_target_dependencies(Go2_bridge
  unitree_go
  rclcpp
  std_msgs
  std_srvs
  bridge_interface
  rosidl_default_runtime
)

install(
  TARGETS Go2_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


add_executable(H1_2_bridge
  src/bridge_core.cpp
  src/estop.cpp
  src/H1_2_bridge.cpp
  src/h1_2_node.cpp
  src/common/motor_crc_hg.cpp
)

  
ament_target_dependencies(H1_2_bridge
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  bridge_interface
  rosbag2_cpp
  rosidl_default_runtime
)

install(
  TARGETS H1_2_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)




# set(UNITREE_SDK_PATH $ENV{HOME}/library/unitree)

# # Import Libraries for unitree_sdk2
# find_library(DDSC_LIB ddsc PATHS ${UNITREE_SDK_PATH}/lib NO_DEFAULT_PATH)
# add_library(ddsc SHARED IMPORTED GLOBAL)
# set_target_properties(ddsc PROPERTIES IMPORTED_LOCATION ${DDSC_LIB})
# set_target_properties(ddsc PROPERTIES IMPORTED_NO_SONAME TRUE)
# target_link_libraries(ddsc INTERFACE Threads::Threads)
# target_link_directories(ddsc INTERFACE ${UNITREE_SDK_PATH}/lib)
# target_include_directories(ddsc INTERFACE
#         $<BUILD_INTERFACE:${UNITREE_SDK_PATH}/include>
#         $<INSTALL_INTERFACE:include>)

# find_library(DDSCXX_LIB ddscxx PATHS ${UNITREE_SDK_PATH}/lib NO_DEFAULT_PATH)
# add_library(ddscxx SHARED IMPORTED GLOBAL)
# set_target_properties(ddscxx PROPERTIES IMPORTED_LOCATION ${DDSCXX_LIB})
# set_target_properties(ddscxx PROPERTIES IMPORTED_NO_SONAME TRUE)
# target_link_libraries(ddscxx INTERFACE Threads::Threads)
# target_link_directories(ddscxx INTERFACE ${UNITREE_SDK_PATH}/lib)
# target_include_directories(ddscxx INTERFACE
#         $<BUILD_INTERFACE:${UNITREE_SDK_PATH}/include>
#         $<BUILD_INTERFACE:${UNITREE_SDK_PATH}/include/ddscxx>
#         $<INSTALL_INTERFACE:include>
#         $<INSTALL_INTERFACE:include/ddscxx>)

# ## Import Unitree SDK2 library
# find_library(UNITREE_SDK_LIB unitree_sdk2 PATHS ${UNITREE_SDK_PATH}/lib NO_DEFAULT_PATH)

# if (NOT UNITREE_SDK_LIB)
#     message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
# else ()
#     message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
# endif ()

# message(STATUS "Importing: ${UNITREE_SDK_LIB}")

# add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
# set_target_properties(unitree_sdk2 PROPERTIES IMPORTED_LOCATION ${UNITREE_SDK_LIB})
# target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
# target_include_directories(unitree_sdk2 INTERFACE
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<INSTALL_INTERFACE:include>)

# add_executable(release_node src/release_node.cpp)
# target_link_libraries(release_node unitree_sdk2)
# target_include_directories(release_node PRIVATE ${UNITREE_SDK_PATH}/include)

# # target_include_directories(robot_bridge PRIVATE ${UNITREE_SDK_PATH}/include)
# install(TARGETS
#   release_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(bridge src/bridge.cpp src/common/motor_crc.cpp src/key_event_handler.cpp)
# target_link_libraries(bridge unitree_sdk2)
# target_include_directories(bridge PRIVATE ${UNITREE_SDK_PATH}/include)
# ament_target_dependencies(bridge
#   rclcpp
#   std_msgs
#   std_srvs
#   unitree_go
#   unitree_hg
#   unitree_api
#   bridge_interface
#   booster_interface
# )

# install(TARGETS
#   bridge
#   DESTINATION lib/${PROJECT_NAME}
# )

# ==============================================================Booster T1===================================================================================

# booster_mode.hpp stands in for the Booster SDK: T1_bridge builds on any host.
option(BUILD_BOOSTER_T1 "Build Booster T1 bridge" ON)

if(BUILD_BOOSTER_T1)
  find_package(booster_interface REQUIRED)

  add_executable(T1_bridge
    src/t1_node.cpp
    src/bridge_core.cpp
    src/estop.cpp
    src/gates.cpp
    src/T1_bridge.cpp
  )

  target_include_directories(T1_bridge
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_target_dependencies(T1_bridge
    rclcpp
    std_msgs
    std_srvs
    bridge_interface
    booster_interface
    rosidl_default_runtime
  )

  install(
    TARGETS T1_bridge
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ============================================================== AgiBot X2 =================================================================================

option(BUILD_AGIBOT_X2 "Build AgiBot X2 bridge" OFF)

if(BUILD_AGIBOT_X2)
  find_package(aimdk_msgs REQUIRED)

  add_executable(X2_bridge
    src/bridge_core.cpp
    src/estop.cpp
    src/X2_bridge.cpp
    src/x2_node.cpp
  )

  ament_target_dependencies(X2_bridge
    rclcpp
    std_msgs
    sensor_msgs
    std_srvs
    bridge_interface
    aimdk_msgs
    rosidl_default_runtime
  )

  install(
    TARGETS X2_bridge
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(
    FILES params/X2_config.yaml
    DESTINATION share/${PROJECT_NAME}/params
  )
  
endif()


# ==============================================================test node cmake==============================================================================
# OFF by default: test_node streams a 23-motor (T1) /robot_cmd at 500 Hz; started by mistake
# beside a bridge, it is an outside /robot_cmd publisher.
option(BUILD_TEST_NODE "Build and install test_node" OFF)

if(BUILD_TEST_NODE)
  add_executable(test_node src/test.cpp)
  target_include_directories(test_node PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_node
   rclcpp 
   std_msgs
   std_srvs
   unitree_go
   bridge_interface
   )

  install(TARGETS
    test_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Pure e-stop logic, no ROS. Run alone: build/robot_bridge/test_estop
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_estop tests/test_estop.cpp src/estop.cpp)
  target_include_directories(test_estop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_gates tests/test_gates.cpp src/gates.cpp)
  target_include_directories(test_gates PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # The Booster mode switch without the SDK; tools/check-booster-sdk.py checks the values against it.
  ament_add_gtest(test_booster tests/test_booster.cpp)
  target_include_directories(test_booster PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # The Go2 frame (header, level flag, stopped slots, CRC against Unitree's SDK) and its stop frames.
  ament_add_gtest(test_go2 tests/test_go2.cpp src/go2_frame.cpp src/estop.cpp src/common/motor_crc.cpp)
  target_include_directories(test_go2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/common)
  ament_target_dependencies(test_go2 rclcpp unitree_go)
  # The /lowcmd CRC against Unitree's SDK; run it in the build type the robot gets.
  ament_add_gtest(test_crc tests/test_crc.cpp src/common/motor_crc_hg.cpp)
  target_include_directories(test_crc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/common)
  ament_target_dependencies(test_crc rclcpp unitree_hg)
endif()

ament_package()
