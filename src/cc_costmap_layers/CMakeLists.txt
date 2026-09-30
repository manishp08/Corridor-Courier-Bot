cmake_minimum_required(VERSION 3.8)
project(cc_costmap_layers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ROS-independent core, also built as a C-API shared library for the 2D simulator.
add_library(cc_cost_model SHARED src/cost_model.cpp src/cost_model_capi.cpp)
target_include_directories(cc_cost_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_library(${PROJECT_NAME} SHARED src/perception_layer.cpp)
target_link_libraries(${PROJECT_NAME} cc_cost_model)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp nav2_costmap_2d pluginlib vision_msgs geometry_msgs tf2 tf2_geometry_msgs)

pluginlib_export_plugin_description_file(nav2_costmap_2d perception_layer.xml)

install(TARGETS cc_cost_model ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES perception_layer.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_cost_model test/test_cost_model.cpp)
  target_link_libraries(test_cost_model cc_cost_model)
  add_test(NAME test_cost_model COMMAND test_cost_model)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(cc_cost_model ${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp nav2_costmap_2d pluginlib vision_msgs geometry_msgs tf2 tf2_geometry_msgs)
ament_package()
