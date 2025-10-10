cmake_minimum_required(VERSION 3.8)
project(uuv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(uuv_planner src/uuv_planner.cpp)
target_include_directories(uuv_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(uuv_planner PUBLIC cxx_std_17)

ament_target_dependencies(uuv_planner
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  custom_msg
)

# Use PLAIN signature (no PRIVATE/INTERFACE) to avoid mixing modes
target_link_libraries(uuv_planner nlohmann_json::nlohmann_json)

install(TARGETS uuv_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
