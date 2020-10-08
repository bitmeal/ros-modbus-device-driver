cmake_minimum_required(VERSION 2.8.3)
project(ros_modbus_device_driver)

# find dependencies
find_package(catkin REQUIRED
  COMPONENTS
    rospy
    catkin_virtualenv
    std_msgs
    message_generation
)

# init python environment
catkin_python_setup()
catkin_generate_virtualenv()


# add message definitions
add_message_files(
  FILES
  Byte.msg
  Word.msg
  Dword.msg
  Lword.msg
)

# generate message bindings
generate_messages(
  DEPENDENCIES
  std_msgs
)

# init catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

# catkin virtualenv package
install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/modbus_device_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   launch/ros_modbus_device.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

# if (CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(sometestfile.test)
# endif()
