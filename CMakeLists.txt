cmake_minimum_required(VERSION 3.8)
project(tum_lanelet2_osm_fusion)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(CURL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io registration)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(MAKE_DIRECTORY lib)

####################################
# file_in
####################################

add_library(file_in SHARED
  src/file_io/file_in.cpp
)

target_include_directories(file_in
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/file_io>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(file_in rclcpp lanelet2_extension CURL)

####################################
# file_out
####################################

add_library(file_out SHARED
  src/file_io/file_out.cpp
)

target_include_directories(file_out
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/file_io>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(file_out rclcpp lanelet2_extension)

####################################
# extract_network
####################################

add_library(extract_network SHARED
  src/extract_network/extract_network.cpp
)

target_include_directories(extract_network
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/extract_network>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(extract_network rclcpp lanelet2_extension)

####################################
# map_transformation
####################################

# Transformation with Umeyama/ICP
add_library(align SHARED
  src/map_transformation/align.cpp
)

target_include_directories(align
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/map_transformation>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(align rclcpp Eigen3 lanelet2_extension)
target_link_libraries(align ${PCL_LIBRARIES})

# Rubber-Sheeting
add_library(rubber_sheeting SHARED
  src/map_transformation/rubber_sheeting.cpp
)

target_include_directories(rubber_sheeting
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/map_transformation>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rubber_sheeting rclcpp Eigen3 lanelet2_extension)
target_link_libraries(rubber_sheeting ${PCL_LIBRARIES})

####################################
# conflation
####################################

# Map matching
add_library(matching SHARED
  src/conflation/matching.cpp
)

target_include_directories(matching
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/conflation>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(matching rclcpp Eigen3 lanelet2_extension)

# Map conflation
add_library(conflation SHARED
  src/conflation/conflation.cpp
)

target_include_directories(conflation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/conflation>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(conflation rclcpp Eigen3 lanelet2_extension)

####################################
# messages
####################################

add_library(messages SHARED
  src/messages/messages.cpp
)

target_include_directories(messages
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/messages>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(messages rclcpp geometry_msgs visualization_msgs lanelet2_extension)

####################################
# analysis
####################################

add_library(analysis SHARED
  src/analysis/analysis.cpp
)

target_include_directories(analysis
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion/analysis>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(analysis rclcpp Eigen3 lanelet2_extension)

####################################
# Main Executable
####################################

add_executable(lanelet2_osm
  src/lanelet2_osm.cpp
)

target_include_directories(lanelet2_osm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tum_lanelet2_osm_fusion>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(lanelet2_osm rclcpp geometry_msgs visualization_msgs lanelet2_extension)
target_link_libraries(lanelet2_osm file_in file_out extract_network align rubber_sheeting
  matching conflation messages analysis)

####################################
# Building
####################################

install(
  DIRECTORY lib/
  DESTINATION lib
)

install(TARGETS
  file_in
  file_out
  extract_network
  align
  rubber_sheeting
  matching
  conflation
  messages
  analysis
  DESTINATION lib
)

install(TARGETS
  lanelet2_osm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
