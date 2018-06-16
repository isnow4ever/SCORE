cmake_minimum_required(VERSION 2.8.3)
project(rqt_scara_gui)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
  tf
)
find_package(Boost COMPONENTS system regex REQUIRED)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs
#  DEPENDS system_lib
)


set (rqt_scara_gui_SRCS
  src/rqt_scara_gui/my_plugin.cpp
  src/rqt_scara_gui/indicatorlamp.cpp
)

set(rqt_scara_gui_HDRS
  include/rqt_scara_gui/my_plugin.h
  include/rqt_scara_gui/indicatorlamp.h
)

set(rqt_scara_gui_UIS
  src/rqt_scara_gui/my_plugin.ui
)

set (scara_gui_SRCS
  src/rqt_scara_gui/indicatorlamp.cpp
  src/mypluginwidget.cpp
  #src/working_thread.cpp
)

set(scara_gui_HDRS
  include/rqt_scara_gui/indicatorlamp.h
  include/mypluginwidget.h
  #include/working_thread.h
)

set(scara_gui_UIS
  src/rqt_scara_gui/my_plugin.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_scara_gui_MOCS ${rqt_scara_gui_HDRS})
  qt5_wrap_ui(rqt_scara_gui_UIS_H ${rqt_scara_gui_UIS})
  qt5_wrap_cpp(scara_gui_MOCS ${scara_gui_HDRS})
  qt5_wrap_ui(scara_gui_UIS_H ${scara_gui_UIS})
else()
  qt4_wrap_cpp(rqt_scara_gui_MOCS ${rqt_scara_gui_HDRS})
  qt4_wrap_ui(rqt_scara_gui_UIS_H ${rqt_scara_gui_UIS})
  qt4_wrap_cpp(scara_gui_MOCS ${scara_gui_HDRS})
  qt4_wrap_ui(scara_gui_UIS_H ${scara_gui_UIS})
endif()
file(COPY include/rqt_scara_gui/indicatorlamp.h DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    ${rqt_scara_gui_SRCS}
    ${rqt_scara_gui_MOCS}
    ${rqt_scara_gui_UIS_H}
    )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

add_executable(test_node_pub
  src/test_node_pub.cpp
)
add_dependencies(test_node_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_node_pub
  ${catkin_LIBRARIES}
)

add_executable(test_node_sub
  src/test_node_sub.cpp
)
add_dependencies(test_node_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_node_sub
  ${catkin_LIBRARIES}
)

add_executable(scara_gui src/main.cpp ${scara_gui_SRCS}  ${scara_gui_MOCS} ${scara_gui_UIS_H})
link_directories(scara_gui include/)
target_link_libraries(scara_gui ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})