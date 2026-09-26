cmake_minimum_required(VERSION 3.8)
project(aimdk_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MessageHeader.msg"
  "msg/DomainErrorState.msg"
  "msg/JointCommand.msg"
  "msg/JointCommandArray.msg"
  "msg/JointState.msg"
  "msg/JointStateArray.msg"
  "msg/CommonRequest.msg"
  "msg/CommonResponse.msg"
  "msg/RequestHeader.msg"
  "msg/ResponseHeader.msg"
  "msg/CommonState.msg"
  "msg/SystemStatus.msg"
  "srv/GetSystemState.srv"
  "srv/MigrateSystemState.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()