cmake_minimum_required(VERSION 3.8)
project(riskgraph_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RiskFactor.msg"
  "msg/RiskEvent.msg"
  "msg/RouteSegment.msg"
  "msg/Route.msg"
  "msg/RouteScore.msg"
  "msg/RouteScoreArray.msg"
  "msg/RouteExplanation.msg"
  "srv/ScoreRoutes.srv"
  "srv/QuerySegmentRisk.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
