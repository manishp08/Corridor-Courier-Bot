cmake_minimum_required(VERSION 3.8)
project(cc_localization)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/covariance_relay.py scripts/localization_monitor.py scripts/make_maps.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
