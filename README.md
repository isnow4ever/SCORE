# SCORE
SCARA Controller based On ROS and EtherCAT


########################
##+ scara_interface
##
##  contains "kinematic controller", "interface to minas drivers", and "GUIs for debug".
##----------------------
##+ scara_description
##
##  contains scara STL files from Solidworks and URDFs.
##----------------------
##+ scara_moveit_config
##
##  generated from MoveIt! tools.
##----------------------
##+ minas_control
##
##  Panasonic minas drivers library.
##----------------------
##+ ethercat_manager
##
##  EtherCAT manager library based on SOEM.
##----------------------
##+ RT-core
##
##  based on Xenomai. (unfinished)
##----------------------
##+ scara_gui
##
##  display robot and driver states.
##----------------------
##+ iPendant
##
##  teach pendant program for articulated robots
##  
##----------------------
##+ u_devices
##
##  Hardware interface program for teach pendant based on Marsboard.
########################

partly refers to Yongzhuo Gao, Zhijiang Du, Xueshan Gao, Yanyu Su, Yu Mu, Li Ning Sun, Wei Dong, (2018) "Implementation of openarchitecture kinematic controller for articulated robots under ROS", Industrial Robot: An International Journal

2018.06.16 update: SCARA release 1.0
1. [scara_gui] SCARA gui in QtGui instead of rqt because segmentation fault often happens in rqt;
2. [scara_interface] [scara_gui] Driver States: Robot running state, Servo state, Auto/Manual state, Error state;
3. [scara_gui] Gui Buttons: ServoON, ServoOFF, Reset Buttons;
4. [scara_moveit_config] Respawn function in "soem init" and roslaunch files;
5. [iPendant] Cyclic Mode in Playback;
6. [iPendant] add robot joint pos limits;
7. [iPendant] add Auto/Manual state for Teach and Playback;
8. [iPendant] [scara_gui] fix tf frame id for cartesian states;
9. [iPendant] revise time interval between two playback commands;
10. [ethercat_manager] add slave lost error throw code;