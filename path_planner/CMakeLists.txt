cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# ////////////////////////////////////////////////////////////////////

add_library(a_star_planner src/a_star_planner.cpp )

ament_target_dependencies(
  a_star_planner
  nav_msgs  )
target_include_directories(a_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${nav_msgs_INCLUDE_DIRS})
  # ${nlohmann_json_schema_validator_INCLUDE_DIRS})
target_compile_features(a_star_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++

ament_export_targets(a_star_planner HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/path_planner
  DESTINATION include
)
 install(TARGETS a_star_planner
 EXPORT a_star_planner
 LIBRARY DESTINATION lib
 ARCHIVE DESTINATION lib
 RUNTIME DESTINATION bin
 INCLUDES DESTINATION include/path_planner
 )


# ////////////////////////////////////////////////////////////////////////////////

add_executable(path_planner src/main.cpp src/path_planner.cpp)
ament_target_dependencies(path_planner rclcpp nav_msgs nav2_core geometry_msgs rclcpp_components)
target_link_libraries(path_planner
  a_star_planner
)

add_executable(path_planner_client src/path_planner_client.cpp)
ament_target_dependencies(path_planner_client rclcpp nav_msgs geometry_msgs)

add_executable(map_publisher src/map_publisher.cpp)
ament_target_dependencies(map_publisher rclcpp nav_msgs)

target_include_directories(path_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  path_planner
  map_publisher
  path_planner_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
