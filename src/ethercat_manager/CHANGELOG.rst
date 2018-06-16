^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ethercat_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2017-11-28)
------------------
* fixed IF_MINAS (`#60 <https://github.com/tork-a/minas/issues/60>`_)
  * an operator "&" seems to be missed.
* Contributors: Cong Liu

1.0.6 (2017-09-12)
------------------

1.0.5 (2017-09-12)
------------------

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

1.0.1 (2017-05-12)
------------------

1.0.0 (2017-04-22)
------------------
* add test codes (`#39 <https://github.com/tork-a/minas/pull/39`_)
* check if slaves are MINAS or not (`#38 <https://github.com/tork-a/minas/pull/38`_)
  * getNumClients() returns only MINAS drivers
* Contributors: Tokyo Opensource Robotics Developer 534

0.5.2 (2017-04-01)
------------------

0.5.1 (2017-03-26)
------------------

0.5.0 (2017-03-06)
------------------

0.4.0 (2017-03-06)
------------------

0.3.0 (2017-03-06)
------------------
* change docbuild/debbuild to docbuild_${PROJECT_NAME}/debbuild_${PROJECT_NAME}
* use clock_nanosleep for cycleWorkder
* Contributors: Tokyo Opensource Robotics Developer 534

0.2.1 (2017-02-04)
------------------

0.2.0 (2017-02-04)
------------------
* return number of ether cat clients
* ec_SDOread fails only when ret is minus
* use hex to display Failed to read from
* add position offsset (60b0) to PDO mappoing 4
* read current SDO syncmode, cycle time
* add writeSDO(char, unsigned char)
* add readSDO
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.2 (2017-01-16)
------------------
* ethercat_manager use pre allcoated iomap\_ to avoid segfault soem/test/ also uses this strategy
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.1 (2017-01-15)
------------------

0.1.0 (2017-01-14)
------------------
* CMakeLists.txt : add debbuild target
* CMakeLists.txt : add install include directory
* add writeSDO method
* readInput/readOutput: add boundary check
* initSoem : set to PDO Default maping 4
* :~EtherCatManager(): move to INIT mode in destructor
* add CMakeLists.txt package.xml include/ethercat_manager/ethercat_manager.h src/ethercat_manager.cpp
* Contributors: Tokyo Opensource Robotics Developer 534
