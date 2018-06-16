/****************************************************************************
# minas_hardware_interface.h:  MINAS A5B EtherCAT Motor Controller          #
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

#ifndef MINAS_CONTROL_MINAS_HARDWARE_INTERFACE_
#define MINAS_CONTROL_MINAS_HARDWARE_INTERFACE_

// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// minas
#include <ethercat_manager/ethercat_manager.h>
#include <minas_control/minas_client.h>

namespace minas_control
{

struct JointData
{
  std::string name_;
  double cmd_;
  double pos_;
  double vel_;
  double eff_;
  int home_encoder_offset_;
};

class JointControlInterface
{
 public:
  JointControlInterface(int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd) {
    // create joint name
    std::stringstream ss;
    ss << "joint" << slave_no;
    joint.name_ = ss.str();

    // register joint
    //registerJoint(joint.name_, jnt_stat, jnt_cmd);
    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_, &joint.eff_);
    jnt_stat.registerHandle(state_handle);
    // // position handle
    hardware_interface::JointHandle pos_handle(jnt_stat.getHandle(joint.name_), &joint.cmd_);
    jnt_cmd.registerHandle(pos_handle);
  }
  ~JointControlInterface() {};
  virtual void read() = 0;
  virtual void write() = 0;
  virtual void shutdown() = 0;

  protected:
    JointData joint;
};

class EtherCATJointControlInterface : public JointControlInterface
{
 public:
  EtherCATJointControlInterface(ethercat::EtherCatManager* manager, int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd, int torque_for_emergency_stop, int over_load_level, int over_speed_level, double motor_working_range, int max_motor_speed, int max_torque, int home_encoder_offset);
  ~EtherCATJointControlInterface();
  void read();
  void write();
  void shutdown();

 private:
  minas_control::MinasClient* client;
  minas_control::MinasInput input;
  minas_control::MinasOutput output;
};

class DummyJointControlInterface : public JointControlInterface
{
 public:
  DummyJointControlInterface(int slave_no, hardware_interface::JointStateInterface& jnt_stat, hardware_interface::PositionJointInterface& jnt_cmd);
  ~DummyJointControlInterface() {};
  void read();
  void write();
  void shutdown() {};

 private:
};

class MinasHardwareInterface : public hardware_interface::RobotHW
{
private:
  //Node Handles
  ros::NodeHandle nh_; // no namespace

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface joint_position_interface;

  // Kinematic properties
  unsigned int n_dof_;

  typedef std::vector<JointControlInterface*> JointControlContainer;
  JointControlContainer controls;

  // Ethercat Manager
  ethercat::EtherCatManager* manager;

public:
  /**
   * \brief Constructor/Descructor
   */
  MinasHardwareInterface(std::string ifname, bool in_simulation = false);
  ~MinasHardwareInterface();
  void registerControl(JointControlInterface*);
  bool read(const ros::Time time, const ros::Duration period); 
  void write(const ros::Time time, const ros::Duration period);
  void shutdown();
  ros::Time getTime();
  ros::Duration getPeriod();

  void getParamFromROS(int joint_no, int &torque_for_emergency_stop, int &over_load_level, int &over_speed_level, double &motor_working_range, int &max_motor_speed, int &max_torque, int &home_encoder_offset);
};

} // namespace

#endif // MINAS_CONTROL_MINAS_HARDWARE_INTERFACE_
