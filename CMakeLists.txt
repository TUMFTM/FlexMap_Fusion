cmake_minimum_required(VERSION 3.8)
project(flexmap_fusion)

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
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(CURL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(PCL 1.3 REQUIRED)

include_directories(${pcl_msgs_INCLUDE_DIRS})
include_directories(${message_filters_INCLUDE_DIRS})
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/file_io>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(file_in rclcpp CURL lanelet2_core lanelet2_io lanelet2_projection)
target_link_libraries(file_in ${PCL_LIBRARIES})

####################################
# file_out
####################################

add_library(file_out SHARED
  src/file_io/file_out.cpp
)

target_include_directories(file_out
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/file_io>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(file_out rclcpp lanelet2_core lanelet2_io lanelet2_projection)
target_link_libraries(file_out ${PCL_LIBRARIES})

####################################
# extract_network
####################################

add_library(extract_network SHARED
  src/extract_network/extract_network.cpp
)

target_include_directories(extract_network
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/extract_network>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(extract_network rclcpp lanelet2_core lanelet2_io lanelet2_projection)

####################################
# map_transformation
####################################

# Transformation with Umeyama/ICP
add_library(align SHARED
  src/map_transformation/align.cpp
)

target_include_directories(align
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/map_transformation>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(align rclcpp Eigen3 lanelet2_core lanelet2_io lanelet2_projection)

# Rubber-Sheeting
add_library(rubber_sheeting SHARED
  src/map_transformation/rubber_sheeting.cpp
)

target_include_directories(rubber_sheeting
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/map_transformation>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rubber_sheeting rclcpp geometry_msgs Eigen3 lanelet2_core lanelet2_io lanelet2_projection)
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/conflation>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(matching rclcpp Eigen3 lanelet2_core lanelet2_io lanelet2_projection)

# Map conflation
add_library(conflation SHARED
  src/conflation/conflation.cpp
)

target_include_directories(conflation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/conflation>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(conflation rclcpp Eigen3 lanelet2_core lanelet2_io lanelet2_projection)

####################################
# messages
####################################
# message conversion
add_library(message_conversion SHARED
  src/messages/message_conversion.cpp
)

target_include_directories(message_conversion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/messages>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(message_conversion rclcpp geometry_msgs visualization_msgs lanelet2_core lanelet2_io lanelet2_projection)

add_library(visualization SHARED
  src/messages/visualization.cpp
)

# visualization
target_include_directories(visualization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/messages>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(visualization rclcpp geometry_msgs visualization_msgs lanelet2_core lanelet2_io lanelet2_projection)

add_library(messages SHARED
  src/messages/messages.cpp
)

# messages
target_include_directories(messages
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/messages>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(messages message_conversion visualization ${PCL_LIBRARIES} ${message_filters_LIBRARIES} ${pcl_msgs_LIBRARIES})
ament_target_dependencies(messages rclcpp geometry_msgs visualization_msgs lanelet2_core lanelet2_io lanelet2_projection)

####################################
# analysis
####################################

add_library(analysis SHARED
  src/analysis/analysis.cpp
)

target_include_directories(analysis
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion/analysis>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(analysis rclcpp Eigen3 lanelet2_core lanelet2_io lanelet2_projection)

####################################
# Main Executables
####################################

# lanelet2_osm_fusion
add_executable(lanelet2_osm
  src/lanelet2_osm.cpp
)

target_include_directories(lanelet2_osm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(lanelet2_osm rclcpp geometry_msgs visualization_msgs lanelet2_core lanelet2_io lanelet2_projection)
target_link_libraries(lanelet2_osm file_in file_out extract_network align rubber_sheeting
  matching conflation messages analysis)

# kiss_icp_georef
add_executable(kiss_icp_georef
  src/kiss_icp_georef.cpp
)

target_include_directories(kiss_icp_georef
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexmap_fusion>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(kiss_icp_georef rclcpp geometry_msgs visualization_msgs lanelet2_core lanelet2_io lanelet2_projection)
target_link_libraries(kiss_icp_georef file_in file_out align rubber_sheeting messages analysis)

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
  message_conversion
  visualization
  messages
  analysis
  DESTINATION lib
)

install(TARGETS
  lanelet2_osm
  kiss_icp_georef
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
