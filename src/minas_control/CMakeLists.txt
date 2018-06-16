cmake_minimum_required(VERSION 2.8.3)
project(minas_control)

find_package(catkin REQUIRED COMPONENTS controller_manager diagnostic_updater transmission_interface hardware_interface sensor_msgs joint_limits_interface trajectory_msgs ethercat_manager soem)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS diagnostic_updater trajectory_msgs sensor_msgs)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ## The following work around allows SOEM headers to include other SOEM headers.
  ## SOEM headers assume all headers are installed in a flat directory structure
  ## See https://github.com/smits/soem/issues/4 for more information.
  ${soem_INCLUDE_DIRS}/soem
)

## Declare a C++ library
add_library(minas_client src/minas_client.cpp)

## Declare a C++ executable for ROS_CONTROL
add_executable(main src/main.cpp src/minas_hardware_interface.cpp)

## Declare a C++ executable
add_executable(slaveinfo src/slaveinfo.cpp)
add_executable(simple_test src/simple_test.cpp)
add_executable(reset src/reset.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(main ${catkin_LIBRARIES} minas_client)
target_link_libraries(slaveinfo ${catkin_LIBRARIES})
target_link_libraries(simple_test ${catkin_LIBRARIES} minas_client)
target_link_libraries(reset ${catkin_LIBRARIES} minas_client)

function(set_sudo_binary tgt)
  get_target_property(loc ${tgt} "LOCATION")
  string(ASCII 27 ESCAPE)
  add_custom_target(sudo_${tgt}
    COMMAND sudo -n sh -c 'setcap cap_net_raw,cap_ipc_lock=+ep ${loc}' || (>&2 echo "${ESCAPE}[31m#\ type\ following\ command\ before\ execute\ reset...\ sudo\ setcap cap_net_raw,cap_ipc_lock=+ep\ ${loc}${ESCAPE}[0m")
    DEPENDS bin/${tgt})
  add_custom_target(sudo_${tgt}_all ALL DEPENDS sudo_${tgt})
endfunction()
set_sudo_binary(main)
set_sudo_binary(slaveinfo)
set_sudo_binary(simple_test)
set_sudo_binary(reset)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS slaveinfo simple_test reset minas_client main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# https://github.com/tork-a/minas/issues/20 / http://stackoverflow.com/questions/3352041/creating-binary-with-cmake-removes-runtime-path
set_property(TARGET slaveinfo PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_property(TARGET simple_test PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_property(TARGET reset PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_property(TARGET minas_client PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_property(TARGET main PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(status "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()

#############
## Release ##
#############

# debbuild
# catkin build minas_control --make-args debbuild
# ROS_PARALLEL_JOBS='' catkin_make --pkg minas_control -- debbuild
add_custom_target(debbuild_${PROJECT_NAME}
  COMMAND sudo -n true || (>&2 echo "debbuild need sudo access to make debs" && exit 1)
  COMMAND bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
  COMMAND MAKEFLAGS= fakeroot debian/rules binary
  COMMAND find debian obj-x86_64-linux-gnu -type f -not -name postinst -print0 | xargs -0 rm --
  COMMAND find debian obj-x86_64-linux-gnu -type d -empty -delete
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )

# make docs
# ROS_PARALLEL_JOBS='' catkin_make --pkg minas_control -- docbuild
add_custom_target(docbuild_${PROJECT_NAME}
  COMMAND cmake -E remove_directory output
  COMMAND cmake -E make_directory output
  COMMAND sphinx-build -b latex doc output
  COMMAND sed -i 's@\\\\chapter{[0-9. ]*@\\\\chapter{@' output/minas_control.tex
  COMMAND sed -i 's@\\\\section{[0-9. ]*@\\\\section{@' output/minas_control.tex
  COMMAND sed -i 's@\\\\subsection@\\\\subsubsection@' output/minas_control.tex
  COMMAND sed -i 's@\\\\section@\\\\subsection@' output/minas_control.tex
  COMMAND sed -i 's@\\\\chapter@\\\\section@' output/minas_control.tex
  # COMMAND sed -i 's@\\\\tableofcontents@\\\\chapter{unicub\\\\_gazebo} %\\\\tableofcontents@' output/minas_control.tex
  COMMAND sed -i 's@\\\\tableofcontents@\\\\setcounter{chapter}{1}\\\\setcounter{tocdepth}{3}\\\\tableofcontents@' output/minas_control.tex
  COMMAND sed -i 's@\\\\setbox0@\\\\iffalse@' output/minas_control.tex
  COMMAND sed -i 's@{\\\\box0}\\\\end{center}@{\\\\box0}\\\\end{center}\\\\fi@' output/minas_control.tex
  COMMAND sed -i 's@\\\\def\\\\PYGZsq{\\S*}@\\\\usepackage{upquote}\\\\def\\\\PYGZsq{\\\\textquotesingle}@' output/minas_control.tex
  COMMAND make -C output
  COMMAND cmake -E copy output/${PROJECT_NAME}.pdf output/${PROJECT_NAME}-${minas_control_VERSION}.pdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )
