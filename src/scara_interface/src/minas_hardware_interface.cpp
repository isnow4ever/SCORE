/****************************************************************************
# # minas_hardware_interface.cpp:  MINAS A5B EtherCAT Motor Controller      #
# Copyright (C) 2017, Tokyo Opensource Robotics Kyokai Association          #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/

#include <minas_control/minas_hardware_interface.h>
#include <getopt.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace minas_control
{

#define PULSE_PER_REVOLUTE ( (1048576 / (2 * M_PI) ) * 101 ) // 20 bit / 101 reduction
  //#define PULSE_PER_REVOLUTE ( ( 131072 / (2 * M_PI) ) * 101 )// 17 bit / 101 reduction

  EtherCATJointControlInterface::EtherCATJointControlInterface(ethercat::EtherCatManager* manager, int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd, int torque_for_emergency_stop, int over_load_level, int over_speed_level, double motor_working_range, int max_motor_speed, int max_torque, int home_encoder_offset) : JointControlInterface(slave_no, jnt_stat, jnt_cmd)
  {
    ROS_INFO("Initialize EtherCATJoint (%d)", slave_no);
    // EtherCAT
    int operation_mode = 0x08; // (csp) cyclic synchronous position mode

    client = new MinasClient(*manager, slave_no);

    ROS_INFO("Initialize EtherCATJoint (reset)");
    client->reset();

    // set paramete from PANATERM test program
    ROS_INFO("Initialize EtherCATJoint (TorqueForEmergencyStop %d)", torque_for_emergency_stop);
    client->setTrqueForEmergencyStop(torque_for_emergency_stop); // unit [%]
    ROS_INFO("Initialize EtherCATJoint (OverLoadLevel %d)", over_load_level);
    client->setOverLoadLevel(over_load_level);          // unit [%]
    ROS_INFO("Initialize EtherCATJoint (OverSpeedLevel %d)", over_speed_level);
    client->setOverSpeedLevel(over_speed_level);        // [r/min]
    ROS_INFO("Initialize EtherCATJoint (MotorWorkingRange %.1f)", motor_working_range);
    client->setMotorWorkingRange(motor_working_range);  // (unit 0.1, full range is 1)

    ROS_INFO("Initialize EtherCATJoint (InterpolationTimePeriod)");
    client->setInterpolationTimePeriod(4000);     // 4 msec

    // servo on
    ROS_INFO("Initialize EtherCATJoint (servoOn)");
    client->servoOn();

    // get current positoin
    ROS_INFO("Initialize EtherCATJoint (readInputs)");
    input = client->readInputs();
    int32 current_position = input.position_actual_value;

    ROS_INFO("Initialize EtherCATJoint (set target position)");
    // set target position
    memset(&output, 0x00, sizeof(minas_control::MinasOutput));
    if ( operation_mode == 0x01 )
      { // (pp) position profile mode
	output.target_position = (current_position > 0)?(current_position - 0x100000):(current_position + 0x100000);
      }
    else
      { // (csp) cyclic synchronous position mode
	//output.target_position = current_position;
	output.target_position = 0;
	output.position_offset = current_position;
      }
    output.max_motor_speed = max_motor_speed;  // rad/min
    output.target_torque = 0;    // 0% (unit 0.1%)
    output.max_torque    = max_torque;    // 50% (unit 0.1%)
    output.controlword   = 0x001f; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5)

    // change to cyclic synchronous position mode
    output.operation_mode = operation_mode;
    //output.operation_mode = 0x08; // (csp) cyclic synchronous position mode
    //output.operation_mode = 0x01; // (pp) position profile mode

    // set profile velocity
    ROS_INFO("Initialize EtherCATJoint (setProfileVelocity)");
    client->setProfileVelocity(0x20000000);

    ROS_INFO("Initialize EtherCATJoint (pp control model setup)");
    // pp control model setup (see statusword(6041.h) 3) p.107)
    client->writeOutputs(output);
    while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)
      input = client->readInputs();
    }
    ROS_INFO("Initialize EtherCATJoint (clear new set point)");
    output.controlword   &= ~0x0010; // clear new-set-point (bit4)
    client->writeOutputs(output);

    // set home_encoder_offset
    // encoder resolution is 17Bit(131072 per round)
    if (abs(home_encoder_offset) > 100000)
    {
      ROS_WARN("Invalid large home_encoder_offset value: %d", joint.home_encoder_offset_);
      ROS_WARN("Please check your home_encoder_offset parameter is correct.");
    }
    else
    {
      joint.home_encoder_offset_ = home_encoder_offset;
    }
    ROS_INFO("home_encoder_offset = %d", joint.home_encoder_offset_);

    ROS_WARN("target position = %08x", output.target_position);
    ROS_WARN("position offset = %08x", output.position_offset);
    joint.cmd_ = joint.pos_ = (current_position - joint.home_encoder_offset_) / PULSE_PER_REVOLUTE;
    joint.vel_ = joint.eff_ = 0;
    ROS_INFO("Initialize EtherCATJoint .. done");
  }

  EtherCATJointControlInterface::~EtherCATJointControlInterface()
  {
    ROS_INFO_STREAM_NAMED("minas", "~EtherCATJointControlInterface()");
    shutdown();
    delete(client);
  }

  void EtherCATJointControlInterface::shutdown()
  {
    ROS_INFO_STREAM_NAMED("minas", joint.name_ + " shutdown()");
    client->printPDSStatus(input);
    client->printPDSOperation(input);
    client->reset();
    client->servoOff();
  }

  void EtherCATJointControlInterface::read()
  {
    input = client->readInputs();
    output = client->readOutputs();
    joint.pos_ = int32_t(input.position_actual_value - joint.home_encoder_offset_) / PULSE_PER_REVOLUTE;
    joint.vel_ = int32_t(input.velocity_actual_value) / PULSE_PER_REVOLUTE;
    joint.eff_ = int32_t(input.torque_actual_value) / PULSE_PER_REVOLUTE;
  }

  void EtherCATJointControlInterface::write()
  {
    output.position_offset = uint32_t(joint.cmd_ * PULSE_PER_REVOLUTE + joint.home_encoder_offset_);
    client->writeOutputs(output);
  }

  //
  DummyJointControlInterface::DummyJointControlInterface(int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd) : JointControlInterface(slave_no, jnt_stat, jnt_cmd) {
    joint.cmd_ = joint.pos_ = joint.vel_ = joint.eff_ = joint.home_encoder_offset_ = 0;
  }

  void DummyJointControlInterface::read()
  {
    joint.pos_ = joint.cmd_;
    joint.vel_ = 0;
    joint.eff_ = 0;
  }

  void DummyJointControlInterface::write()
  {
  }

  //
  void MinasHardwareInterface::getParamFromROS(int joint_no, int &torque_for_emergency_stop, int &over_load_level, int &over_speed_level, double &motor_working_range, int &max_motor_speed, int &max_torque, int &home_encoder_offset)
  {
    std::string joint_name("~joint" + boost::lexical_cast<std::string>(joint_no));
    ros::param::param<int>(joint_name + "/torque_for_emergency_stop", torque_for_emergency_stop, 100); // 100%
    ros::param::param<int>(joint_name + "/over_load_level", over_load_level, 50); // 50%
    ros::param::param<int>(joint_name + "/over_speed_level", over_speed_level, 120); // r/min
    ros::param::param<double>(joint_name + "/motor_working_range", motor_working_range, 0.1); // 0.1
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/torque_for_emergency_stop : " << torque_for_emergency_stop);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/over_load_level           : " << over_load_level);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/over_speed_level          : " << over_speed_level);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/motor_working_range       : " << motor_working_range);
    ros::param::param<int>(joint_name + "/max_motor_speed", max_motor_speed, 120); // rad/min
    ros::param::param<int>(joint_name + "/max_torque", max_torque, 500); // 50% (unit 0.1%)
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/max_motor_speed           : " << max_motor_speed);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/max_torque                : " << max_torque);
    // Encoder offset at angle = 0
    ros::param::param<int>(joint_name + "/home_encoder_offset", home_encoder_offset, 0);
    ROS_INFO_STREAM_NAMED("minas", joint_name + "/home_encoder_offset       : " << home_encoder_offset);
  }

  MinasHardwareInterface::MinasHardwareInterface(std::string ifname, bool in_simulation)
    : manager(NULL)
  {
    /* start MinasClient */
    if (in_simulation) {
      ROS_INFO_STREAM_NAMED("minas","Minas Hardware Interface in simulation mode");
      for (int i = 1; i <= 6; i++ ) {
        int torque_for_emergency_stop, over_load_level, over_speed_level;
        double motor_working_range;
        int max_motor_speed, max_torque, home_encoder_offset;
        getParamFromROS(i, torque_for_emergency_stop, over_load_level, over_speed_level, motor_working_range, max_motor_speed, max_torque, home_encoder_offset);
	registerControl(new DummyJointControlInterface(i,
						       joint_state_interface,
						       joint_position_interface
						       ));
      }
    } else {

      manager = new ethercat::EtherCatManager(ifname);

      n_dof_ = manager->getNumClients();
      if ( n_dof_ != 6 ) {
	ROS_ERROR_STREAM_NAMED("minas", "Minas Hardware Interface expecting 6 clients");
      }
      int i;
      for (i = 1; i <= n_dof_; i++ )
	{
          int torque_for_emergency_stop, over_load_level, over_speed_level;
          double motor_working_range;
          int max_motor_speed, max_torque;
          int home_encoder_offset;
          getParamFromROS(i, torque_for_emergency_stop, over_load_level, over_speed_level, motor_working_range, max_motor_speed, max_torque, home_encoder_offset);
	  registerControl(new EtherCATJointControlInterface(manager, i,
							    joint_state_interface,
							    joint_position_interface,
                                                            torque_for_emergency_stop, over_load_level, over_speed_level, motor_working_range,
                                                            max_motor_speed, max_torque, home_encoder_offset
                                                            ));
	}
      for (; i <= 6; i++ )
	{
	  registerControl(new DummyJointControlInterface(i,
							 joint_state_interface,
							 joint_position_interface
							 ));
	  ROS_ERROR_STREAM_NAMED("minas", "Could not find EtherCAT client");
	  ROS_ERROR_STREAM_NAMED("minas", "Minas Hardware Interface uses Dummy joint " << i);
	}
    }

    registerInterface(&joint_state_interface);
    registerInterface(&joint_position_interface);
  }

  MinasHardwareInterface::~MinasHardwareInterface()
  {
    shutdown();
  }

  void MinasHardwareInterface::shutdown()
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->shutdown();
    }
    controls.clear();
    if ( manager != NULL ) {
      ROS_INFO_STREAM_NAMED("minas", "Delete manager");
      delete(manager);
    }
    manager = NULL;
  }

  void MinasHardwareInterface::registerControl(JointControlInterface* control)
  {
    controls.push_back(control);
  }

  bool MinasHardwareInterface::read(const ros::Time time, const ros::Duration period)
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->read();
    }
  }

  void MinasHardwareInterface::write(const ros::Time time, const ros::Duration period)
  {
    BOOST_FOREACH (JointControlInterface* control, controls) {
      control->write();
    }
  }

  inline ros::Time MinasHardwareInterface::getTime()
  {
    return ros::Time::now();
  }

  inline ros::Duration MinasHardwareInterface::getPeriod()
  {
    return ros::Duration(0.001);
  }

} // namespace

