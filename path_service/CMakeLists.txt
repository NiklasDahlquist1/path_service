cmake_minimum_required(VERSION 3.8)
project(path_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(path_planning REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)



INCLUDE_DIRECTORIES(

  include
  ${path_planning_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  yaml-cpp/include
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPath.srv"
  "srv/SetMap.srv"
  "srv/PositionStatus.srv"
  DEPENDENCIES
  builtin_interfaces
  rosidl_default_generators
  std_msgs
  nav_msgs
  geometry_msgs
 )


ament_export_dependencies(

 rosidl_default_runtime
)
# add_dependencies(${PROJECT_NAME} path_service_node)



ament_package()







add_executable(path_service_node 
  src/path_service_node.cpp
  src/simulation_2d.cpp
  )

ament_target_dependencies(path_service_node
   rclcpp
   std_msgs

   geometry_msgs
   nav_msgs
   path_planning
  #  cv_bridge
  #  image_transport
   OpenCV
 )
 rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
 target_link_libraries(path_service_node "${cpp_typesupport_target}" yaml-cpp)
# target_link_libraries(bt_generation_test_node yaml-cpp)

install(TARGETS
  path_service_node
  DESTINATION lib/${PROJECT_NAME})
  
  
install(DIRECTORY
#  launch
#  files
  DESTINATION share/${PROJECT_NAME})




