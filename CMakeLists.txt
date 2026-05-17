cmake_minimum_required(VERSION 3.8)
project(octomap_2d_slicer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# ── Find dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(nav_msgs             REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(tf2_ros              REQUIRED)
find_package(tf2                  REQUIRED)
find_package(octomap_msgs         REQUIRED)
find_package(octomap              REQUIRED)

# ── Library (reusable node class) ──────────────────────────────────────────────
add_library(${PROJECT_NAME}_lib SHARED
  src/octomap_2d_slicer_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  octomap_msgs
  octomap
)

# ── Executable ─────────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL   # don't fail if the dirs are empty
)

ament_package()