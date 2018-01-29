cmake_minimum_required(VERSION 2.8.3)
project(ipendant)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

find_package(Qt4 COMPONENTS QTCORE QTGUI QTDECLARATIVE)
include(${QT_USE_FILE})

find_package(Boost COMPONENTS system regex REQUIRED)
#find_package(Boost COMPONENTS regex REQUIRED)

#generate_messages(DEPENDENCIES std_msgs)

#############
##  catkin  #
#############

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS roscpp std_msgs)

#############
## includes #
#############

include_directories(
	${catkin_INCLUDE_DIRS}
	${QT_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        # "/home/isnow4ever/Qt4.8.5/mkspecs/linux-g++-64/" #x86-64
	# "/usr/local/qt4.8.5/mkspecs/linux-arm-gnueabi-g++/" #arm
#	"/usr/local/regex/include/"
	)

#############
## settings #
#############

set(SRCS
        src/interface/mainwindow.cpp
        src/interface/newfiledialog.cpp
        src/interface/notificationdialog.cpp
        src/interface/insertprogdialog.cpp
	src/interface/virtualkeyboard.cpp
        src/interface/commandHandler.cpp
        src/interface/indicatorlamp.cpp
        src/interface/working_thread.cpp
        src/interface/res.h
	src/interface/commandProcessor.cpp
	src/interface/project.cpp
        src/interface/projectHandler.cpp
        src/interface/playback.cpp
)

set(MOC_HDRS
        src/interface/mainwindow.h
        src/interface/newfiledialog.h
        src/interface/notificationdialog.h
        src/interface/insertprogdialog.h
	src/interface/virtualkeyboard.h
        src/interface/commandHandler.h
        src/interface/indicatorlamp.h        
        src/interface/working_thread.h
	src/interface/commandProcessor.h
	src/interface/project.h
        src/interface/projectHandler.h
        src/interface/playback.h
)

set(ipendant_UI
        src/interface/mainwindow.ui
        src/interface/newfiledialog.ui
        src/interface/notificationdialog.ui
        src/interface/insertprogdialog.ui
        src/interface/virtualkeyboard.ui
)

set(ipendant_RESOURCES
        src/interface/images.qrc
)

set(CMAKE_DEVEL_DIR ${CMAKE_CURRENT_BINARY_DIR}/devel/lib/ipendant/)


#############
##   GUI   ##
#############
QT4_WRAP_UI(ipendant_UI_HDRS ${ipendant_UI})
QT4_ADD_RESOURCES(ipendant_RESOURCES_RCC ${ipendant_RESOURCES})
QT4_WRAP_UI(startup_UI_HDRS src/startup/dialog.ui)

#############
##  Build  ##
#############

qt4_wrap_cpp(MOC_SRCS ${MOC_HDRS})
qt4_wrap_cpp(MOC_startup src/startup/dialog.h)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_NO_KEYWORDS)

add_executable(startup src/startup/startup.cpp src/startup/dialog.cpp ${MOC_startup} ${startup_UI_HDRS})
target_link_libraries(startup ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ipendant src/interface/main.cpp ${SRCS}  ${MOC_SRCS} ${ipendant_UI_HDRS} ${ipendant_RESOURCES} ${ipendant_RESOURCES_RCC})
link_directories(ipendant include/)
target_link_libraries(ipendant Xtst X11 ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

#file(COPY src/qml DESTINATION ${CMAKE_DEVEL_DIR})
file(COPY src/interface/indicatorlamp.h DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
