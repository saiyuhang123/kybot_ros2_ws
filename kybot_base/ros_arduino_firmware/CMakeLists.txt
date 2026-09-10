cmake_minimum_required(VERSION 3.5)
project(ros_arduino_firmware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
