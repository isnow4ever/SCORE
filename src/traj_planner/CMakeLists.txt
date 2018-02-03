cmake_minimum_required(VERSION 2.8.3)
project(traj_planner)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH $(CMAKE_MODULE_PATH) "/home/lgl/master/SCARA/xCLib/src")
find_package(catkin REQUIRED roscpp std_msgs moveit_ros_planning_interface tf)

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED regex)
find_package(xC    REQUIRED)
find_package(xCDev REQUIRED)
find_package(Qt4   REQUIRED)

#############
##  catkin  #
#############
catkin_package()

#############
## includes #
#############
include(${QT_USE_FILE})
include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${XC_INCLUDE_DIRS})
include_directories(${XCDEV_INCLUDE_DIRS})

link_directories(${XC_LIBRARY_PATH})
link_directories(${XCBRIDGE_LIBRARY_PATH})
link_directories(${XCDEV_LIBRARY_PATH})

#############
##   GUI   ##
#############
#QT4_WRAP_UI(INTERFACE_UI_HDRS src/interface_dialog.ui)
#QT4_WRAP_CPP(INTERFACE_MOC_SRCS src/interface_dialog.h)

#QT4_WRAP_UI(COORDINATOR_UI_HDRS src/coordinator_dialog.ui)
#QT4_WRAP_CPP(COORDINATOR_MOC_SRCS src/coordinator_dialog.h)

#############
##  Build  ##
#############
#add_executable(interface src/interface.cpp src/spline.cpp src/constants.h)
#target_link_libraries(interface ${XC_LIBRARIES} ${XCDEV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(coordinator src/coordinator.cpp src/constants.h)
target_link_libraries(coordinator ${catkin_LIBRARIES} ${XC_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(calibration src/calibration.cpp src/constants.h)
#target_link_libraries(calibration ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#add_executable(recorder src/recorder.cpp src/constants.h)
#target_link_libraries(recorder ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(coordinator src/coordinator.cpp src/constants.h)
#target_link_libraries(coordinator ${catkin_LIBRARIES} ${XC_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(gui_interface src/interface_gui.cpp src/interface_dialog.cpp ${INTERFACE_UI_HDRS} ${INTERFACE_MOC_SRCS})
#target_link_libraries(gui_interface ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${QT_LIBRARIES} )

#add_executable(gui_coordinator src/coordinator_gui.cpp src/coordinator_dialog.cpp ${COORDINATOR_UI_HDRS} ${COORDINATOR_MOC_SRCS})
#target_link_libraries(gui_coordinator ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${QT_LIBRARIES} )

#add_executable(x50_simulator src/simulator.cpp)
#target_link_libraries(x50_simulator ${XCDEV_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(x50_controller src/controller.cpp)
#target_link_libraries(x50_controller ${XCDEV_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(x50_planner src/planner.cpp src/constants.h)
#target_link_libraries(x50_planner ${catkin_LIBRARIES})

#add_executable(scene_operator src/_scene_operator.cpp)
#target_link_libraries(scene_operator  ${catkin_LIBRARIES})

#add_executable(x50_pipline src/_pipline.cpp)
#target_link_libraries(x50_pipline  ${catkin_LIBRARIES})

#############
## Install ##
#############
#install(TARGETS interface
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#       )

#install(TARGETS coordinator
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#       )

#install(DIRECTORY include/ ${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h"
#        PATTERN ".svn" EXCLUDE
#       )

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
