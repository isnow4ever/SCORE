^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package minas_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2017-11-28)
------------------
* Add parameter home_encoder_offset (`#63 <https://github.com/tork-a/minas/issues/63>`_)
* Reduce sleep time in each client (`#51 <https://github.com/tork-a/minas/issues/51>`_)
* Add dependency on tinyxml (`#62 <https://github.com/tork-a/minas/issues/62>`_)
* Contributors: Ryosuke Tajima

1.0.6 (2017-09-12)
------------------

1.0.5 (2017-09-12)
------------------
* add .travis.yml using ros_build_farm (`#58 <https://github.com/tork-a/minas/issues/58>`_)
  * add rosdoc.yaml
  * package.xml, CMakeLists.txt: add depends to diagnostic_updater
  * Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

1.0.4 (2017-09-11)
------------------
* set license as GPLv2/CC-BY-SA,add LICENSE for meshes (`#55 <https://github.com/tork-a/minas/issues/55>`_)
  * set license as GPLv2/CC-BY-SA,add LICENSE for meshes
  * cleanup code
  * fix wrong license data, add LICENSE information to xacro files
  * cleanup package.xml
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

1.0.3 (2017-08-04)
------------------

1.0.2 (2017-05-12)
------------------
* add more ROS_INFO message while insitializeing
* use current readInputs and wait 10msec
* servoon/servooff : printPDSstatus once every seconds
* call reset before servooff in shutdown
* Improve servo ON time
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

1.0.1 (2017-05-12)
------------------
* fix comments on params
* On site fix 5/11 by Tajima
* add comment
* fix if condition
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

1.0.0 (2017-04-22)
------------------
* add test codes (`#39 <https://github.com/tork-a/minas/pull/39`_)
  * minas_control/src/main.cpp: continue running evern if mlockall failed, when we run with simulaiton
  * add depends to diagnostic_updator
* use rosparam to set control param (`#33 <https://github.com/tork-a/minas/pull/33`_)
  * update install process, 1.0.0
  * doc/index.rst : use two-backquote instaad of backquote to show code block
  * doc/index.rst : comment on how to run tra1_bringup.launch and set controller parameters
  * use getParamFromROS function, read and display parameters in simulation mode too
  * move custom controller parameters for tra1 to tra1_bringup/launch/tra1_bringup.lauch
  * minas_control.launch add eth arg to set ether device name
* Contributors: Tokyo Opensource Robotics Developer 534

0.5.2 (2017-04-01)
------------------
* CMakeLists.txt: set INSTALL_RPARH_USE_LINK_PATH for preserving RPATH, fix `#20 <https://github.com/tork-a/minas/issues/20>`_
* CMakeLists.txt: do not remove debian/postinst on debbuild target
* doc/index.rst: Fix some typos
* doc/index.rst: add all deb files to install
* Contributors: Tokyo Opensource Robotics Developer 7675, Tokyo Opensource Robotics Developer 534

0.5.1 (2017-03-26)
------------------
* add debian/postinst to set cap/ipc
* CMakeLists.txt add to install launch directory
* Add error text for MinasClient::readInputs()
  - This is for `#10 <https://github.com/tork-a/minas/issues/10>`_
* fix typo on dpkg-i ros-indigo-*.deb section
* Contributors: Tokyo Opensource Robotics Developer 7675, Tokyo Opensource Robotics Developer 534

0.5.0 (2017-03-06)
------------------

0.4.0 (2017-03-06)
------------------
* add doc for step 4 (realtime control)
* Contributors: Tokyo Opensource Robotics Developer 534

0.3.0 (2017-03-06)
------------------
* fix error message Failed to lock memory
* add documents for step 3 (support multiple joints)
* add 1/101 gear reduction
* change docbuild/debbuild to docbuild\_${PROJECT_NAME}/debbuild\_${PROJECT_NAME}, Close `#5 <https://github.com/tork-a/minas/issues/5>`_.
* add tra1_bringup
* add ros_control for minas robot
* doc: run debbuild --no-deps-> fix typo
* Contributors: Tokyo Opensource Robotics Developer 534

0.2.1 (2017-02-04)
------------------
* doc: segmentation fault no longer occour
* update doc for step 3/4
* minas_control: change LICENSE to GPL
* simple_client: support multiple clients
* Contributors: Tokyo Opensource Robotics Developer 534

0.2.0 (2017-02-04)
------------------
* fix readOutput
* update simple_test for both pp and csp mode
* fix setInterpolationTimePeriod, now argumetns is [usec]
* add PDO mapping 4 + position offset, use cyclic synchronous position(csp) mode
* setInterpolationTimePeriod: display more info
* add setInterpolationTimePeriod
* add getPDSOperation, getPDSControl, getPDSStatus
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.2 (2017-01-16)
------------------
* conf.py : add C++ API feature, but not enable due to bad layout
* index.rst: use xml.etree to get version number from package.xml
* doc/index.rst segfault was resolved
* doc/index.rst: resolved permission problem
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.1 (2017-01-15)
------------------
* CMakeLists:txt : debbuild : make sure that we can run sudo
* CMakeLists.txt : forget to install minas_client
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.0 (2017-01-14)
------------------
* add doc and inital index.rst
* CMakeLists.txt : add install include directory
* CMakeLists.txt : add debbuild/docbuild target
* CMakeLists.txt : run setcap to run seom apps without sudo
* update simple_test, set velocity profile
* add setTorqueForEmergencyStop, setOveerLoadLevel, setOverSpeedLevel, setMotorWorrkingRange, setProfileVelocity
* MinasClinet: add reset/servoOn/servoOff method
* minus_control.cpp: usetPDO Default maping 4
* CMakeLists.txt,src/reset.cpp: add reset.cpp
* add CMakeLists.txt package.xml include/minas_control/minas_client.h src/minas_client.cpp src/slaveinfo.cpp src/simple_test.cpp
* Contributors: Tokyo Opensource Robotics Developer 534
