cmake_minimum_required(VERSION 2.8.3)
project(u_devices)

find_package(catkin REQUIRED COMPONENTS  roscpp  rospy  std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

#############
## includes ##
#############

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


#############
##  Build  ##
#############

add_library(gpio_lib include/gpio_lib.h include/gpio_lib.c)
add_library(spi_lib include/spi_lib.h include/spi_lib.c ${catkin_LIBRARIES})

add_library(marsboard include/spi_lib.h include/gpio_lib.h include/marsboard.h include/gpio.c include/spi.c)
target_link_libraries(marsboard gpio_lib spi_lib ${catkin_LIBRARIES})

add_executable(joystick src/devices.h src/joystick.cpp)
target_link_libraries(joystick marsboard ${catkin_LIBRARIES})

add_executable(keypad src/devices.h src/keypad.cpp)
target_link_libraries(keypad marsboard ${catkin_LIBRARIES})

# add_executable(io include/marsboard.h src/test.c)
# target_link_libraries(io marsboard ${catkin_LIBRARIES})

#############
## Install ##
#############
