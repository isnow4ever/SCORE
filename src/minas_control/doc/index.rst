MINAS ROS (Robot Operating System) driver
#########################################

Overview
========

The ``minas_control`` pacakge contains basic control tools for `MINAS-A5B`_ EtherCAT communication driver for indusdtrial robots.

Prerequisite
===============

This simulation software has been testd on the following environment: 

* Ubuntu Linux 14.04.3 "Trusty" 64bit

* `ROS Indigo Igloo <http://wiki.ros.org/indigo>`_

Install Common Components
----------------------------

First install a few components this package needs: `ROS`_ (robotics middleware)

  The following snippet shows a simple way to install `ROS Indigo` on Ubuntu linux 14.04 `Trusty`. For completeness, you're advised to see `ROS wiki <http://wiki.ros.org/indigo/Installation/Ubuntu>`_.

.. code-block:: bash

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" \
          > /etc/apt/sources.list.d/ros-latest.list'
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O -  | sudo apt-key add -
  sudo apt-get update && sudo apt-get install -y python-rosdep
  sudo rosdep init && rosdep update
  
  echo "### For ROS setting" >> ~/.bashrc
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
  source ~/.bashrc


Install From Deb
----------------

Obtain ``ethercat_manager``, ``minas_control`` and ``tra1-*`` deb files (here assumes``ros-indigo-ethercat-manager_1.0.0-0trusty_amd64.deb`` and ``ros-indigo-minas-control_1.0.0-0trusty_amd64.deb``). Plase it under current directory.

.. code-block:: bash

  sudo apt-get install -y gdebi
  sudo gdebi -n ros-indigo-ethercat-manager_1.0.0-0trusty_amd64.deb
  sudo gdebi -n ros-indigo-minus-control_1.0.0-0trusty_amd64.deb
  sudo gdebi -n ros-indigo-tra1-description_1.5.0-0trusty_amd64.deb
  sudo gdebi -n ros-indigo-tra1-bringup_1.0.0-0trusty_amd64.deb
  sudo gdebi -n ros-indigo-tra1-moveit-config_1.0.0-0trusty_amd64.deb

Running MINAS-A5B Controller
============================

To run MINAS-A5B controller, you can start with following roslaunch command.

.. code-block:: bash

  roslaunch tra1_bringup tra1_bringup.launch

Running ``rostopic`` command will show a list of input and output topics of this controller

.. code-block:: bash

  $ rostopic list
  /diagnostics
  /joint_states
  /position_trajectory_controller/command
  /position_trajectory_controller/follow_joint_trajectory/cancel
  /position_trajectory_controller/follow_joint_trajectory/feedback
  /position_trajectory_controller/follow_joint_trajectory/goal
  /position_trajectory_controller/follow_joint_trajectory/result
  /position_trajectory_controller/follow_joint_trajectory/status
  /position_trajectory_controller/state
  /rosout
  /rosout_agg
  /tf

rostopic named ``/joint_states`` will show the status of each joint including

-  the position of the joint (rad or m)
-  the velocity of the joint (rad/s or m/s)
-  the effort that is applied in the joint (Nm or N)

.. code-block:: bash

  $ rosmsg show sensor_msgs/JointState
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string[] name
  float64[] position
  float64[] velocity
  float64[] effort

To send commands to the controller, you can use ``/position_trajectory_controller/follow_joint_trajectory/goal`` of type ``control_msgs/JointTrajectoryActionGoal``.

.. code-block:: bash

  $ rosmsg show control_msgs/JointTrajectoryActionGoal
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  actionlib_msgs/GoalID goal_id
    time stamp
    string id
  control_msgs/JointTrajectoryGoal goal
    trajectory_msgs/JointTrajectory trajectory
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      string[] joint_names
      trajectory_msgs/JointTrajectoryPoint[] points
        float64[] positions
        float64[] velocities
        float64[] accelerations
        float64[] effort
        duration time_from_start


The ``tra1_bringup.launch`` assumes you have connected EtherCAT device to the ``eth0`` device of your machine. To run controller with custom settings, you can use ``eth`` argument.

.. code-block:: bash

  roslaunch tra1_bringup tra1_bringup.launch eth:=eth4

If you would like to run withtout hardware devices, you can run MINAS-A5B controller with simulation mode

.. code-block:: bash

  roslaunch tra1_bringup tra1_bringup.launch simulation:=true

To change control parameter, you can use following rosparams. These are relative to ``/main/joint1`` ... ``/main/joint6``.

- torque_for_emergency_stop : Set up the torque limit at emergency stop, When setup value is 0, the torque limit for normal operation is applied. Range is 0 - 500 (%). Default value is 100 (%).
- over_load_level : You can set up the over-load level. The overload level becomes 115[%] by setting up this to 0. Use this with 0 setup in normal operation. Set up other value only when you need to lower the over-load level. Range is 0 - 500 (%). Default value is 50 (%).
- over_speed_level : If the motor speed exceeds this setup value, Err26.0 Over-speed protection occurs. The over-speed level becomes 1.2 times of the motor max. speed by setting up this to 0. Range is 0 - 2000 (r/min). Default value is 120 (r/min).
- motor_working_range : You can set up the movable range of the motor against the position command input range. When the motor movement exceeds the setup value, software limit protection of Err34.0 will be triggered. Range is 0 - 1.0 (revolution). Default value is 0.0 (revolution).

For more detail, see 4-50 of the manual (https://industrial.panasonic.com/content/data/MT/PDF/manual/en/acs/minas-a5-2_manu_e.pdf)

- max_motor_speed : Set the maximum velocity of motor. The maximum value is limited by the 3910h(Maximum over-speed level) in internal processing.. It is tq and cst and restricts speed with the preset value of this object. Range is 0 - 4294967295 (rad/min). Default value is 120 (rad/min) (6080h / 00h)
- max_torque : Set the maximum torque of the motor. The maximum value is limited by the maximum torque which is calculated from 3904h(Mass of motor's movable section/ Motor inertia) and 3905h(Rated motor thrust / Rated motor torque). The maximum torque of the motor varies with the motor used. Range is 0 - 65535 (0.1%). Default value is 500 (50%). (6072h / 00h)

For more detail, see p.151 of the manual (https://industrial.panasonic.com/content/data/MT/PDF/refer/en/acs/SX-DSV02830_R1_00E.pdf)


These parameters are overwrited at ``tra1_bringup.launch``. If you wan to change these parameters, rewriete launch files.

To show contents of current ``tra1_bringup.launch`` file. You can use ``roscat tra1_bringup  tra1_bringup.launch`` command.


.. code-block:: bash

  <launch>
  
    <!-- GDB functionality -->
    <arg name="debug" default="false" />
    <arg name="simulation" default="false" />
    <arg name="eth" default="eth0" />
  
    <!-- Load robot description -->
    <param name="robot_description"
      command="$(find xacro)/xacro.py '$(find tra1_description)/urdf/tra1.xacro'" />
  
    <rosparam>
  main/joint1/torque_for_emergency_stop : 100  <!-- 100 % -->
  main/joint1/over_load_level           : 100  <!-- 100 % -->
  main/joint1/over_speed_level          : 3000 <!-- rad/min -->
  main/joint1/motor_working_range       : 0.1  <!-- 0.1 -->
  main/joint1/max_motor_speed           : 3000 <!-- rad/min -->
  main/joint1/max_torque                : 50   <!-- 100% -->
  main/joint2/torque_for_emergency_stop : 100  <!-- 100 % -->
  main/joint2/over_load_level           : 100  <!-- 100 % -->
  ...


Easiest way should be copy launch file to current directory, change parameters and run that file.

.. code-block:: bash

  $ roscp tra1_bringup tra1_bringup.launch my_tra1_bringup.launch
  $ emacs my_tra1_bringup.launch
  $ roslaunch my_tra1_bringup.launch



MINAS-A5B Control Tools
=======================

Before you start we  have to configure ``SI1`` and ``SI2`` input selection, Please change No. 4.01 from default setting ``818181h`` to ``010101h`` and No 4.02 from ``28282h`` to ``020202h`` using `PANATERM`_, see page 13 of the `Manual`_.

First you need to know the network adapter neme for the EtherCAT netwok, ``ifconfig`` will give you the list of network adpater of your computer, for example, at a following case, eth1 is your EtherCAT network and we'll use ``eth1`` here after, if you have different adapter name, please use that name when you run the application.

.. code-block:: bash

  $ ifconfig            
  eth0      Link encap:Ethernet  HWaddr 74:03:db:f7:9a:39
            inet addr:192.169.100.1  Bcast:192.168.100.255  Mask:255.255.255.0
            inet6 addr: fe80::7603:bdff:fe7f:9a39/64 Scope:Link
            UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
            RX packets:38503098 errors:0 dropped:337 overruns:0 frame:0
            TX packets:5419325 errors:0 dropped:0 overruns:0 carrier:0
            collisions:0 txqueuelen:1000
            RX bytes:4368155082 (4.3 GB)  TX bytes:1391012577 (1.3 GB)
  
  eth1      Link encap:Ethernet  HWaddr 68:f7:82:42:0f:bc
            inet6 addr: fe80::6af7:28ff:fe24:fbc/64 Scope:Link
            UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
            RX packets:2901790 errors:0 dropped:124 overruns:0 frame:0
            TX packets:4073359 errors:0 dropped:0 overruns:0 carrier:0
            collisions:0 txqueuelen:1000
            RX bytes:284659686 (284.6 MB)  TX bytes:516196518 (516.1 MB)
            Interrupt:20 Memory:f0600000-f0620000
  
  lo        Link encap:Local Loopback  
            inet addr:127.0.0.1  Mask:255.0.0.0
            inet6 addr: ::1/128 Scope:Host
            UP LOOPBACK RUNNING  MTU:65536  Metric:1
            RX packets:11730343164 errors:0 dropped:0 overruns:0 frame:0
            TX packets:11730343164 errors:0 dropped:0 overruns:0 carrier:0
            collisions:0 txqueuelen:0 
            RX bytes:186698529957677 (186.6 TB)  TX bytes:186698529957677 (186.6 TB)

slave_info
----------

Now let's run ``salveinfo`` to show current configuration of your EtherCAT network. Please change ``eth1`` to your settings.

.. code-block:: bash

  $ rosrun minas_control slaveinfo eth1
  SOEM (Simple Open EtherCAT Master)
  Slaveinfo
  Initializing etherCAT master
  wkc = 2
  SOEM found and configured 2 slaves
  len = 9
  len = 9
  len = 9
  len = 9
  RxPDO mapping object index 1 = 1603 ret=3
  TxPDO mapping object index 1 = 1a03 ret=6
  RxPDO mapping object index 2 = 1603 ret=3
  TxPDO mapping object index 2 = 1a03 ret=6
  SOEM IOMap size: 100
  
  Slave:1
   Name:MADHT1105B01
   Output size: 200bits
   Input size: 200bits
  State: 8
   Delay: 0[ns]
   Has DC: 1
   DCParentport:0
   Activeports:1.1.0.0
   Configured address: 1001
  
  Slave:2
   Name:MADHT1107B21
   Output size: 200bits
   Input size: 200bits
   State: 8
   Delay: 680[ns]
   Has DC: 1
   DCParentport:1
   Activeports:1.0.0.0
   Configured address: 1002
  PDO syncmode 00, cycle time 0 ns (min 17000), sync0 cycle time 0 ns, ret = 4
  PDO syncmode 00, cycle time 0 ns (min 17000), sync0 cycle time 0 ns, ret = 4
    
    Finished configuration successfully
    End program

simple_test
-----------

Then let's move to next step. The ``simple_test`` is the example program to control motors. '-h' or '--help' option will show the usages of this program.

.. code-block:: bash

  $ rosrun minas_control simple_test -h
  MINAS Simple Test using SOEM (Simple Open EtherCAT Master)
  Usage: simple_test [options]
    Available options
      -i, --interface     NIC interface name for EtherCAT network
      -p, --position_mode Sample program using Position Profile (pp) mode (Default)
      -c, --cycliec_mode  Sample program using cyclic synchronous position(csp) mode
      -h, --help          Print this message and exit

On default settings, ``simple_test`` will servo on, rotate about 360 degree and servo off. The ``simple_test`` program basically follow the instruction described in the manual, i.e Start up guide in p.3 and Motion of ``pp`` control mode in p. 107. Basic flow of the cpp program as follows.

.. code-block:: cpp

  minas_control::MinasInput input = client->readInputs();
  int32 current_position = input.position_actual_value;

  // set target position
  minas_control::MinasOutput output;
  output.target_position = (current_position > 0)?
              (current_position - 0x100000):(current_position + 0x100000);

  output.max_motor_speed = 120;  // rad/min
  output.target_torque = 500;    // 0% (unit 0.1%)
  output.max_torque    = 500;    // 50% (unit 0.1%)
  output.controlword   = 0x001f; // move to operation enabled +
                                 // new-set-point (bit4) +
                                 //  change set immediately (bit5)

  output.operation_mode = 0x01; // (pp) position profile mode

  // set profile velocity
  client->setProfileVelocity(0x20000000);

  // pp control model setup (see statusword(6041.h) 3) p.107)
  client->writeOutputs(output);
  while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)
    input = client->readInputs();
  }
  output.controlword   &= ~0x0010; // clear new-set-point (bit4)
  client->writeOutputs(output);

To run ``simple_test`` with pp mode, use ``-p`` option.

.. code-block:: bash

  $ rosrun minas_control simple_test -p -i eth1
  MINAS Simple Test using SOEM (Simple Open EtherCAT Master)
  Initializing etherCAT master
  wkc = 2
  SOEM found and configured 2 slaves
  len = 9
  len = 9
  len = 9
  len = 9
  RxPDO mapping object index 1 = 1603 ret=3
  TxPDO mapping object index 1 = 1a03 ret=6
  RxPDO mapping object index 2 = 1603 ret=3
  TxPDO mapping object index 2 = 1a03 ret=6
  SOEM IOMap size: 100
  
  Slave:1
   Name:MADHT1105B01
   Output size: 200bits
   Input size: 200bits
   State: 8
   Delay: 0[ns]
   Has DC: 1
   DCParentport:0
   Activeports:1.1.0.0
   Configured address: 1001
  
  Slave:2
   Name:MADHT1107B21
   Output size: 200bits
   Input size: 200bits
   State: 8
   Delay: 680[ns]
   Has DC: 1
   DCParentport:1
   Activeports:1.0.0.0
   Configured address: 1002
  PDO syncmode 00, cycle time 0 ns (min 17000), sync0 cycle time 0 ns,ret = 4
  PDO syncmode 00, cycle time 0 ns (min 17000), sync0 cycle time 0 ns,ret = 4
    overrun: 0.000596
    overrun: 0.000572
    overrun: 0.002370
  Set interpolation time period 4000 us (4000000/4)
    overrun: 0.005399
  1c32h: cycle time 0
  60c2h: interpolation time period value 25
  Statusword(6041h): 0a70
   Switch on disabled
   Internal limit active
   Following error
   Drive follows command value
    overrun: 0.007179
    overrun: 0.006475
    overrun: 0.000108
  Statusword(6041h): 0e37
   Operation enabled
   Internal limit active
   Following error
   Set-point acknowledge
   Target reached
    overrun: 0.000403
  target position = 000e912d
    overrun: 0.000011
    overrun: 0.000191
  Set interpolation time period 4000 us (4000000/4)
    overrun: 0.000659
  1c32h: cycle time 0
  60c2h: interpolation time period value 25
  Statusword(6041h): 0a70
   Switch on disabled
   Internal limit active
   Following error
   Drive follows command value
  Statusword(6041h): 0e31
   Ready to switch on
   Internal limit active
   Following error
   Set-point acknowledge
   Target reached
    overrun: 0.001740
    overrun: 0.004097
  target position = 000c2bba
    overrun: 0.003520
  err = 0000, ctrl 000f, status 0237, op_mode =  1, pos = fffe9196, vel = 00000cb2, tor = 00000017
  Tick 1488782766.167119670
  Input:
   603Fh 00000000 :Error code
   6041h 00000237 :Statusword
   6061h 00000001 :Modes of operation display
   6064h fffe9196 :Position actual value
   606Ch 00000cb2 :Velocity actual value
   6077h 00000017 :Torque actual value
   60B9h 00000000 :Touch probe status
   60BAh 00000000 :Touch probe pos1 pos value
   60FDh c0000000 :Digital inputs
  Output:
   6040h 0000000f :Controlword
   6060h 00000001 :Mode of operation
    overrun: 0.002877
   6071h 000001f4 :Target Torque
   6072h 000001f4 :Max Torque
   607Ah 000e912d :Target Position
   6080h 00000078 :Max motor speed
   60B8h 00000000 :Touch Probe function
   60FFh 00000000 :Target Velocity
   60B0h 00000000 :Position Offset
    overrun: 0.002274
  err = 0000, ctrl 000f, status 1237, op_mode =  1, pos = fffc2bb6, vel = fffffe0c, tor = 00000000
  Tick 1488782766.167119670
  Input:
   603Fh 00000000 :Error code
   6041h 00001237 :Statusword
   6061h 00000001 :Modes of operation display
   6064h fffc2bb6 :Position actual value
   606Ch fffffe0c :Velocity actual value
   6077h 00000000 :Torque actual value
   60B9h 00000000 :Touch probe status
   60BAh 00000000 :Touch probe pos1 pos value
   60FDh c0000000 :Digital inputs

You can see some erros in the first a few seconds, until the motors servo on, but that's expected behavior and you can ingreo for now.

If you run ``simple_test`` with ``-c`` option, it will servo on, rotate about 180 degree back and forth with sin curve and servo off. Basic flow of the cpp program as follows.

.. code-block:: cpp

  client->setInterpolationTimePeriod(4000);     // 4 msec

  minas_control::MinasInput input = client->readInputs();
  int32 current_position = input.position_actual_value;

  // set target position
  minas_control::MinasOutput output;
  output.target_position = current_position;

  output.max_motor_speed = 120;  // rad/min
  output.target_torque = 500;    // 0% (unit 0.1%)
  output.max_torque    = 500;    // 50% (unit 0.1%)
  output.controlword   = 0x001f; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5)

  output.operation_mode = 0x08; // (csp) cyclic synchronous position mode

  client->writeOutputs(output);

  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);

  while ( 1 ) {

    output.position_offset = 0x80000*sin(i/200.0);
    client->writeOutputs(output);

    // sleep for next tick
    timespecInc(tick, period);
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
  }

reset
-----

If you have somethig wrong, you can run reset command. If you still have issue, use `PANATERM`_ to clear alarms.

.. code-block:: bash

  $ rosrun minas_control reset eth0
  SOEM (Simple Open EtherCAT Master)
  Simple test
  Initializing etherCAT master
  wkc = 1
  SOEM found and configured 1 slaves
  RxPDO mapping object index 1 = 1603 ret=3
  TxPDO mapping object index 1 = 1a03 ret=6
  SOEM IOMap size: 46
  
  Slave:1
   Name:MADHT1105B01
   Output size: 168bits
   Input size: 200bits
   State: 8
   Delay: 0[ns]
   Has DC: 1
   DCParentport:0
   Activeports:1.0.0.0
   Configured address: 1001
  
  Finished configuration successfully
  End program

main (ROS controlelr program)
-----------------------------

The ``main`` executable is ROS based controller program.  '-h' or '--help' option will show the usages of this program.

.. code-block:: bash

  $ rosrun minas_control main -h
  Usage: main [options]
    Available options
      -i, --interface             NIC interface name for EtherCAT
      -l, --loopback              Use loopback interface for Controller (i.e. simulation mode)
      -p, --period                RT loop period in msec
      -s, --stats                 Publish statistics on the RT loop jitter on
                                  "node_name/realtime" in seconds
      -h, --help                  Print this message and exit

If you do not have MINAS-A5B hardwre, you can run with simulation mode

.. code-block:: bash

  $ rosrun minas_control main -l
  [ INFO] [1488677269.130094946]: Minas Hardware Interface in simulation mode

and check the realtime capability of the ros control program by listening ``/diagnostics`` ROS topic.

..

To run controllers with physical MINAS A-5 Hardware connecting at ``eth1`` EtherCAT network, you can ``main`` program as follows. Please change ``eth1`` to your settings.

.. code-block:: bash

  $ rosrun minas_control main -i eth1
  Initializing etherCAT master
  wkc = 2
  SOEM found and configured 2 slaves
  len = 9
  len = 9
  len = 9
  len = 9
  RxPDO mapping object index 1 = 1603 ret=3
  TxPDO mapping object index 1 = 1a03 ret=6
  RxPDO mapping object index 2 = 1603 ret=3
  TxPDO mapping object index 2 = 1a03 ret=6
  SOEM IOMap size: 100
  
  Slave:1
   Name:MADHT1105B01
   Output size: 200bits
   Input size: 200bits
   State: 8
   Delay: 0[ns]
   Has DC: 1
   DCParentport:0
   Activeports:1.1.0.0
   Configured address: 1001
  
  Slave:2
   Name:MADHT1107B21
   Output size: 200bits
   Input size: 200bits
   State: 8
   Delay: 680[ns]
   Has DC: 1
   DCParentport:1
   Activeports:1.0.0.0
   Configured address: 1002
  PDO syncmode 00, cycle time 0 ns (min 17000), sync0 cycle time 0 ns, ret = 4
  PDO syncmode 00, cycle time 0 ns (min 17000), sync0 cycle time 0 ns, ret = 4
  Finished configuration successfully
  [ERROR] [1488776588.629694406]: Minas Hardware Interface expecting 6 clients
    overrun: 0.000117
    overrun: 0.000442
    overrun: 0.000259
  Statusword(6041h): 0e33
   Switched on
   Internal limit active
   Following error
   Set-point acknowledge
   Target reached
  Statusword(6041h): 0a37
   Operation enabled
   Internal limit active
   Following error
   Set-point acknowledge
   Target reached
  [ WARN] [1488776588.870953939]: target position = 00000000
  [ WARN] [1488776588.871001884]: position offset = fffc2bb3
  [ERROR] [1488776588.871041451]: Could not find EtherCAT client
  [ERROR] [1488776588.871057483]: Minas Hardware Interface uses Dummy joint 3
  [ERROR] [1488776588.871073659]: Could not find EtherCAT client
  [ERROR] [1488776588.871084746]: Minas Hardware Interface uses Dummy joint 4
  [ERROR] [1488776588.871099793]: Could not find EtherCAT client
  [ERROR] [1488776588.871110595]: Minas Hardware Interface uses Dummy joint 5
  [ERROR] [1488776588.871122447]: Could not find EtherCAT client
  [ERROR] [1488776588.871132278]: Minas Hardware Interface uses Dummy joint 6


You can see some erros, specially if you do not set connect 6 motors on your EtherCAT network, but still the controlle software is able to run as they use loopback driver for these joints.

To check current realtime capabiliy of ROS control, you can run ``rostopic echo /diagnostics``.

.. code-block:: bash

  $ rostopic echo /diagnostics
  ---
  header: 
    seq: 200
    stamp: 
      secs: 1488776789
      nsecs:  50168139
    frame_id: ''
  status: 
    - 
      level: 0
      name: Realtime Control Loop
      message: Realtime loop used too much time in the last 30 seconds.
      hardware_id: ''
      values: 
        - 
          key: Max EtherCAT roundtrip (us)
          value: 4030.91
        - 
          key: Avg EtherCAT roundtrip (us)
          value: 13.41
        - 
          key: Max Controller Manager roundtrip (us)
          value: 383.95
        - 
          key: Avg Controller Manager roundtrip (us)
          value: 5.41
        - 
          key: Max Total Loop roundtrip (us)
          value: 5127.10
        - 
          key: Avg Total Loop roundtrip (us)
          value: 1000.01
        - 
          key: Max Loop Jitter (us)
          value: 1136.49
        - 
          key: Avg Loop Jitter (us)
          value: 71.25
        - 
          key: Control Loop Overruns
          value: 11
        - 
          key: Recent Control Loop Overruns
          value: 0
        - 
          key: Last Control Loop Overrun Cause
          value: ec: 1221.71us, cm: 2.58us
        - 
          key: Last Overrun Loop Time (us)
          value: 281.10
        - 
          key: Realtime Loop  Frequency
          value: 971.6667

.. API Documents
.. =============

.. .. toctree::
..    :maxdepth: 2

..    api_ethercat_manager
..    api_minas_control

Maintainer Tips
===============

Create DEB file
---------------

Following command will build DEB (binary installer file for Ubuntu with which you can install software by a simple run of ``gdebi`` command) files.

Before start please add following line to your ``/etc/ros/rosdep/sources.list.d/20-default.list`` file

.. code-block:: bash

  yaml file:///etc/ros/rosdep/ethercat_manager.yaml

and create ``ethercat_manager.yaml`` file that contains

.. code-block:: bash

  ethercat_manager:
    ubuntu:
      apt: ros-indigo-ethercat-manager
  minas_control:
    ubuntu:
      apt: ros-indigo-minas-control
  tra1_description:
    ubuntu:
      apt: ros-indigo-tra1-description
  tra1_moveit_config:
    ubuntu:
      apt: ros-indigo-tra1-movei-tconfig
  tra1_bringup:
    ubuntu:
      apt: ros-indigo-tra1-bringup

and run ``rosdep update``. Then create deb fiels as follows.

.. code-block:: bash

  catkin b ethercat_manager --no-deps --make-args debbuild_ethercat_manager
  dpkg -i ros-indigo-ethercat-manager_0.0.1-0trusty_amd64.deb
  catkin b minas_control --no-deps --make-args debbuild_minas_control
  dpkg -i ros-indigo-minas-control_0.0.1-0trusty_amd64.deb
  catkin b tra1_description --no-deps --make-args debbuild_tra1_description
  dpkg -i ros-indigo-tra1-description_0.0.1-0trusty_amd64.deb
  catkin b tra1_moveit_config --no-deps --make-args debbuild_tra1_moveit_config
  dpkg -i ros-indigo-tra1-moveit-config_0.0.1-0trusty_amd64.deb
  catkin b tra1_bringup --no-deps --make-args debbuild_tra1_bringup
  dpkg -i ros-indigo-tra1-bringup_0.0.1-0trusty_amd64.deb

To install DEB file from command line, please use ``gdebi``. Using ``apt-get`` may fail due to missing dependent deb package, and that breaks your local apt database (wich may fixed by ``sudo apt-get install -f install`` as reported on the `community site <http://askubuntu.com/questions/58202/how-to-automatically-fetch-missing-dependencies-when-installing-software-from-d>`_)

.. code-block:: bash

  sudo apt-get install gdebi
  gdebi -n ros-indigo-minas-control_0.0.1-0trusty_amd64.deb

Create documents
----------------

Following command will build pdf manual.

.. code-block:: bash

  catkin b minas_control --no-deps --make-args docbuild_minas_control

To build the manual you have to install following deb packages

.. code-block:: bash

  apt-get install python-bloom sphinx-common python-catkin-shpinx pdflatex \
                  texlive-latex-base  texlive-latex-recommended texlive-lang-cjk

Known Issues
------------

Trouble shooting
----------------

- If you could not initialize ethercat driver as follows,

  .. code-block:: bash

    $ reset eth1
    SOEM (Simple Open EtherCAT Master)
    Simple test
    Initializing etherCAT master
    Could not initialize ethercat driver
    terminate called after throwing an instance of 'ethercat::EtherCatError'
      what():  Could not initialize SOEM
    Aborted (Core dump)

Failed to lock memory. It is recommended to set permission to
executables, for example: sudo setcap cap_net_raw,cap_ipc_lock=+ep
main: Cannot allocate memory

  Please check if your binary have correctly set permissions by

  .. code-block:: bash

    $ getcap /opt/ros/indigo/lib/minas_control/reset
    /opt/ros/indigo/lib/minas_control/reset = cap_net_raw+ep

  If you can any ``capability``, please try

  .. code-block:: bash

    $ sudo setcap cap_net_raw+ep /opt/ros/indigo/lib/minas_control/reset


.. _MINAS-A5B:  https://industrial.panasonic.com/ww/products/motors-compressors/fa-motors/ac-servo-motors/minas-a5b

.. _ROS: http://ros.org/

.. _PANATERM: https://industrial.panasonic.com/jp/products/motors-compressors/fa-motors/ac-servo-motors/minas-a5-panaterm

.. _Manual: https://industrial.panasonic.com/content/data/MT/PDF/refer/jp/acs/SX-DSV02469_R4_00J.pdf
