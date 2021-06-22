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

# get ROS python interpreter version based on ROS_PYTHON_VERSION or ROS_DISTRO envvar
if(DEFINED ENV{ROS_PYTHON_VERSION})
  set(PYTHON_VENV_INTERPRETER python$ENV{ROS_PYTHON_VERSION})
else()
  message(WARNING "environment variable ROS_PYTHON_VERSION not defined! testing for ROS_DISTRO...")
  # test ROS_DISTRO name
  if(DEFINED ENV{ROS_DISTRO})
    string(TOLOWER $ENV{ROS_DISTRO} ROS_DISTRO_NORM)
    if(${ROS_DISTRO_NORM} STRLESS noetic)
      set(PYTHON_VENV_INTERPRETER python2)
    else()
      set(PYTHON_VENV_INTERPRETER python3)
    endif()
    message(STATUS "assuming ${PYTHON_VENV_INTERPRETER} based on ROS distro $ENV{ROS_DISTRO}")
  else()
    message(FATAL_ERROR "non of environment varibales ROS_PYTHON_VERSION or ROS_DISTRO are defined! cannot identify python interpreter!")
  endif()
endif()
message(STATUS "using python interpreter ${PYTHON_VENV_INTERPRETER}")

# init python environment
catkin_python_setup()
catkin_generate_virtualenv(
  PYTHON_INTERPRETER ${PYTHON_VENV_INTERPRETER}
)


# add message definitions
add_message_files(
  FILES
  Byte.msg
  Word.msg
  DWord.msg
  LWord.msg
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
