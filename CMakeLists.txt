cmake_minimum_required(VERSION 2.8.3)
project(ros_modbus_device)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ros_modbus_device.py
  scripts/ros_modbus_device
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   launch/ros_modbus_device.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

# if (CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(test/websocket/test_smoke.test)
# endif()
