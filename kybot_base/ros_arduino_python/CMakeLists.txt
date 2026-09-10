cmake_minimum_required(VERSION 3.5)
project(ros_arduino_python)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

install(PROGRAMS
  src/${PROJECT_NAME}/arduino_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arduino_node
)

install(PROGRAMS
  src/${PROJECT_NAME}/save_image.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_image
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
