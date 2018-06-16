/*
* License:The MIT License (MIT)
*
* Copyright (c) 2013,2014 Yanyu Su
* State Key Laboratory of Robotics and System, Harbin Institute of Technology
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/
#include <xc/core/core.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
//#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <std_msgs/String.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include "constants.h"

//-----------------------------------------------------------------------------
// StringHandller
//
//
//-----------------------------------------------------------------------------
class StringHandller
{
public:
  void callback ( const std_msgs::String::ConstPtr & message )
  {
    mCrtMsg = *message;
    mNewMsg = true;
  }

  StringHandller()
    : mCrtMsg()
    , mNewMsg ( false )
  {
  }

  const bool newMsg()
  {
    return mNewMsg;
  }

  const std_msgs::String & fetchMsg()
  {
    mNewMsg = false;
    return mCrtMsg;
  }

protected:
  std_msgs::String mCrtMsg;
  bool mNewMsg;
};

//-----------------------------------------------------------------------------
// JointStateHandller
//
//
//-----------------------------------------------------------------------------
class JointStateHandller
{
public:
  void callback ( const sensor_msgs::JointState::ConstPtr & message )
  {
    mCrtMsg = *message;
    mNewMsg = true;
  }

  JointStateHandller()
    : mCrtMsg()
    , mNewMsg ( false )
  {
  }

  const bool newMsg()
  {
    return mNewMsg;
  }

  const sensor_msgs::JointState & fetchMsg()
  {
    mNewMsg = false;
    return mCrtMsg;
  }

protected:
  sensor_msgs::JointState mCrtMsg;
  bool mNewMsg;
};

class CommandProcessor
{
public:
  virtual trajectory_msgs::JointTrajectory _ret GenerateTrajectory ( std::string _in command,
      sensor_msgs::JointState    _out s_CURRENT_JOINT_STATE,
      robot_state::RobotStatePtr _out s_CURRENT_STATE,
      sensor_msgs::JointState    _out s_COMMAND_REFERENCE_JOINT_STATE,
      robot_state::RobotStatePtr _out s_COMMAND_REFERENCE_STATE,
      std::string                _out s_CURRENT_ID ) = 0;

  virtual bool _ret NeedProcess ( std::string _in command ) = 0;

public:
  std::string regex;
};

class DMOVJ_Processor : public CommandProcessor
{
public:
  DMOVJ_Processor()
  {
    regex = COORDINATOR_V6_CMD;
  }

  trajectory_msgs::JointTrajectory _ret GenerateTrajectory ( std::string _in command,
      sensor_msgs::JointState    _out s_CURRENT_JOINT_STATE,
      robot_state::RobotStatePtr _out s_CURRENT_STATE,
      sensor_msgs::JointState    _out s_COMMAND_REFERENCE_JOINT_STATE,
      robot_state::RobotStatePtr _out s_COMMAND_REFERENCE_STATE,
      std::string                _out s_CURRENT_ID )
  {

  }

  bool _ret NeedProcess ( std::string _in command )
  {
    std::vector<std::string> result;
    boost::split ( result, command, boost::is_any_of ( " " ) );
    //
    std::string ID      = result[0];
    std::string COMMAND = result[1];
    std::string TARGET  = result[3];
    std::vector<double> VALUES;
    bool  ZERO = true;
    for ( size_t i = 3; i < result.size() - 2; ++i )
    {
      VALUES.push_back ( atof ( result[i].c_str() ) );
    }
    for ( size_t i = 0; i < VALUES.size() ; ++i )
    {
      if ( fabs ( VALUES[i] ) > 0.0000001 )
        ZERO = false;
    }
    std::string UNIT = result[result.size() - 2];
    std::string FRAME = result[result.size() - 1];
    if ( COMMAND != "DMOVJ" )
      return false;
    if ( ZERO )
      return false;
    return true;
  }

};


//-----------------------------------------------------------------------------
// main
//
//
//-----------------------------------------------------------------------------
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "x50_coordinator" );
  ros::NodeHandle NODE;
  //
  StringHandller                       COMMAND_HANDLLER;
  JointStateHandller                   JOINT_STATE_HANDLLER;
  boost::regex                         REGEX_V6_COMMAND ( COORDINATOR_V6_CMD );
  robot_model_loader::RobotModelLoader ROBOT_MODEL_LOADER ( "robot_description" );
  const double                         p_LOOP_TIME         = COORDINATOR_LOOP;
  const double                         p_PMAC_TIME         = PMAC_LOOP;
  robot_model::RobotModelPtr           p_ROBOT_MODEL       = ROBOT_MODEL_LOADER.getModel();
  const robot_state::JointModelGroup*  p_JOINT_MODEL_GROUP = p_ROBOT_MODEL->getJointModelGroup ( "manipulator" );
  const std::vector<std::string> &     p_JOINT_NAMES       = p_JOINT_MODEL_GROUP->getJointModelNames();
  const std::string                    p_EEF_NAME          = "tool";
  const std::string                    p_BASE_NAME         = p_ROBOT_MODEL->getModelFrame();
  const std::vector<double>            MAX_VEL ( 6, 15 / 180 * 3.1415926 );
  //
  ros::Subscriber subscriber_command =
    NODE.subscribe<std_msgs::String> (
      COORDINATOR_COMMAND, 1024, &StringHandller::callback,
      &COMMAND_HANDLLER );
  //
  ros::Subscriber subscriber_joint_states =
    NODE.subscribe<sensor_msgs::JointState> (
      INTERFACE_JOINT_STATES, 1024,  &JointStateHandller::callback,
      &JOINT_STATE_HANDLLER );
  //
  ros::Publisher  publisher_command =
    NODE.advertise<trajectory_msgs::JointTrajectory> (
      INTERFACE_JOINT_PATH_COMMAND, 1024 );
  //
  ros::Publisher publisher_status =
    NODE.advertise<std_msgs::String> (
      COORDINATOR_STATES, 1, true );
  //
  ROS_INFO ( "Model frame: %s", p_ROBOT_MODEL->getModelFrame().c_str() );
  for ( size_t i = 0; i < p_JOINT_NAMES.size() ; ++i )
    ROS_INFO ( "JOINT_NAMES[%ld] = %s", i, p_JOINT_NAMES[i].c_str() );
  ROS_INFO ( "EEF = %s", p_EEF_NAME.c_str() );
  ROS_INFO ( "BASE = %s", p_BASE_NAME.c_str() );
  ROS_INFO ( "Command Format: %s", COORDINATOR_V6_CMD );
  //
  sensor_msgs::JointState    s_CURRENT_JOINT_STATE;
  robot_state::RobotStatePtr s_CURRENT_STATE ( new robot_state::RobotState ( p_ROBOT_MODEL ) );
  sensor_msgs::JointState    s_COMMAND_REFERENCE_JOINT_STATE;
  robot_state::RobotStatePtr s_COMMAND_REFERENCE_STATE ( new robot_state::RobotState ( p_ROBOT_MODEL ) );
  std::string                s_CURRENT_ID = "undefined_id";
  while ( ros::ok() )
  {
    ros::spinOnce();
    if ( JOINT_STATE_HANDLLER.newMsg() )
    {
      s_CURRENT_JOINT_STATE = JOINT_STATE_HANDLLER.fetchMsg();
      s_CURRENT_STATE->setVariableValues ( s_CURRENT_JOINT_STATE );
      s_COMMAND_REFERENCE_JOINT_STATE = s_CURRENT_JOINT_STATE;
      s_COMMAND_REFERENCE_STATE->setVariableValues ( s_COMMAND_REFERENCE_JOINT_STATE );
      break;
    }
    ros::Duration ( 1.0 ).sleep ();
    ROS_INFO ( "waiting for joint state update" );
  }
  //
  while ( ros::ok() )
  {
    try
    {
      std::string status;
      status.reserve ( 512 );
      double start = ros::Time::now().toSec();
      // spin
      //----------
      ros::spinOnce();
      //
      if ( JOINT_STATE_HANDLLER.newMsg() )
      {
        s_CURRENT_JOINT_STATE = JOINT_STATE_HANDLLER.fetchMsg();
        s_CURRENT_STATE->setVariableValues ( s_CURRENT_JOINT_STATE );
      }
      //
      bool UpdateRef = false;
      if ( !COMMAND_HANDLLER.newMsg() )
      {
        UpdateRef = true;
      }
      else
      {
        std_msgs::String input = COMMAND_HANDLLER.fetchMsg();
        std::string command = input.data;
        ROS_INFO ( "command:%s", command.c_str() );
        if ( boost::regex_match ( command, REGEX_V6_COMMAND ) )
        {
          std::vector<std::string> result;
          boost::split ( result, command, boost::is_any_of ( " " ) );
          //
          std::string ID      = result[0];
          std::string COMMAND = result[1];
          std::string TARGET  = result[3];
          std::vector<double> VALUES;
          bool  ZERO = true;
          for ( size_t i = 3; i < result.size() - 2; ++i )
          {
            VALUES.push_back ( atof ( result[i].c_str() ) );
          }
          for ( size_t i = 0; i < VALUES.size() ; ++i )
          {
            if ( fabs ( VALUES[i] ) > 0.0000001 )
              ZERO = false;
          }
          std::string UNIT = result[result.size() - 2];
          std::string FRAME = result[result.size() - 1];

          s_CURRENT_ID = ID;

          if ( COMMAND == "DMOVJ" ) //------------------------------------------
          {
            if ( ZERO )
            {
              UpdateRef = true;
            }
            else
            {
              std::vector<double> values ( VALUES.size() );
              std::vector<double> joint_values ( p_JOINT_NAMES.size() );
              for ( size_t i = 0; i < VALUES.size() ; ++i )
              {
                values[i] = VALUES[i] / 180.0 * 3.1415926 ;
              }
              //
              for ( size_t i = 0; i < s_COMMAND_REFERENCE_JOINT_STATE.name.size() ; ++i )
              {
                bool mapped = false;
                for ( size_t j = 0; j < p_JOINT_NAMES.size() && !mapped; ++j )
                {
                  if ( s_COMMAND_REFERENCE_JOINT_STATE.name[i] == p_JOINT_NAMES[j] )
                  {
                    mapped = true;
                    joint_values[j] = s_COMMAND_REFERENCE_JOINT_STATE.position[i] + values[j] ;
                    ROS_DEBUG_STREAM ( "move " << p_JOINT_NAMES[j] << " from "
                                       << s_COMMAND_REFERENCE_JOINT_STATE.position[i] << " rad to "
                                       << joint_values[j] << " rad" );
                  }
                }
                assert ( mapped );
              }
              //
              s_COMMAND_REFERENCE_JOINT_STATE.header.stamp = ros::Time::now();
              s_COMMAND_REFERENCE_JOINT_STATE.name = p_JOINT_NAMES;
              s_COMMAND_REFERENCE_JOINT_STATE.position = joint_values;
              s_COMMAND_REFERENCE_STATE->setVariableValues ( s_COMMAND_REFERENCE_JOINT_STATE );
              //
              trajectory_msgs::JointTrajectory output_command;
              output_command.joint_names = p_JOINT_NAMES;
              output_command.points.resize ( 1 );
              output_command.points[0].time_from_start = ros::Duration ( p_PMAC_TIME );
              output_command.points[0].positions = joint_values;
              publisher_command.publish ( output_command );
            }
          }
          else if ( COMMAND == "DMOV" ) //--------------------------------------
          {
            if ( ZERO )
            {
              UpdateRef = true;
            }
            else
            {
              geometry_msgs::Twist twist;
              twist.linear.x = VALUES[0] / 1000.0;
              twist.linear.y = VALUES[1] / 1000.0;
              twist.linear.z = VALUES[2] / 1000.0;
              twist.angular.x = VALUES[3] / 180.0 * 3.1415926;
              twist.angular.y = VALUES[4] / 180.0 * 3.1415926;
              twist.angular.z = VALUES[5] / 180.0 * 3.1415926;
              robot_state::RobotStatePtr target_state ( new robot_state::RobotState ( *s_COMMAND_REFERENCE_STATE ) );
              bool found_ik = target_state->setFromDiffIK ( p_JOINT_MODEL_GROUP, twist, p_EEF_NAME, 1 );
              if ( !found_ik )
              {
                ROS_INFO ( "Did not find IK solution" );
              }
              else
              {
                std::vector<double> joint_values;
                target_state->copyJointGroupPositions ( p_JOINT_MODEL_GROUP, joint_values );
                for ( std::size_t i = 0; i < p_JOINT_NAMES.size(); ++i )
                {
                  ROS_INFO ( "Joint %s: %f", p_JOINT_NAMES[i].c_str(), joint_values[i] );
                }
                //
                s_COMMAND_REFERENCE_JOINT_STATE.name = p_JOINT_NAMES;
                s_COMMAND_REFERENCE_JOINT_STATE.position = joint_values;
                s_COMMAND_REFERENCE_STATE->setVariableValues ( s_COMMAND_REFERENCE_JOINT_STATE );
                trajectory_msgs::JointTrajectory output_command;
                output_command.header.stamp = ros::Time::now();
                output_command.joint_names = p_JOINT_NAMES;
                output_command.points.resize ( 1 );
                output_command.points[0].time_from_start = ros::Duration ( p_PMAC_TIME );
                output_command.points[0].positions = joint_values;
                publisher_command.publish ( output_command );
              }
            }
          }
          else if ( COMMAND == "DMOVL" ) //-------------------------------------
          {
            if ( ZERO )
            {
              UpdateRef = true;
            }
            else
            {
              Eigen::Vector3d direction;
              direction[0] = VALUES[0] / 1000.0;
              direction[1] = VALUES[1] / 1000.0;
              direction[2] = VALUES[2] / 1000.0;
              double distance = direction.norm();
              direction.normalize();
              bool global_reference_frame = true;
              double max_step = 0.2;
              double jump_threshold = 0.0;
              robot_state::RobotStatePtr start_state ( new robot_state::RobotState ( *s_COMMAND_REFERENCE_STATE ) );
              std::vector<robot_state::RobotStatePtr> traj;
              double result = start_state->computeCartesianPath ( p_JOINT_MODEL_GROUP, traj, start_state->getLinkModel ( p_EEF_NAME ),
                              direction, global_reference_frame, distance, max_step, jump_threshold );
              if ( result <= 1.0 )
              {
                ROS_INFO ( "Did not find IK solution" );
              }
              else
              {
                trajectory_msgs::JointTrajectory output_command;
                output_command.header.stamp = ros::Time::now();
                output_command.joint_names = p_JOINT_NAMES;
                output_command.points.resize ( traj.size () );
                for ( size_t i = 0; i < traj.size (); ++i )
                {
                  std::vector<double> joint_values;
                  traj[i]->copyJointGroupPositions ( p_JOINT_MODEL_GROUP, joint_values );

                  for ( std::size_t j = 0; j < p_JOINT_NAMES.size(); ++j )
                  {
                    ROS_INFO ( "Joint %s: %f", p_JOINT_NAMES[j].c_str(), joint_values[j] );
                  }
                  //
                  /*MAX_VEL[]*/

                  s_COMMAND_REFERENCE_JOINT_STATE.name = p_JOINT_NAMES;
                  s_COMMAND_REFERENCE_JOINT_STATE.position = joint_values;
                  s_COMMAND_REFERENCE_STATE->setVariableValues ( s_COMMAND_REFERENCE_JOINT_STATE );
                  output_command.points[i].time_from_start = ros::Duration ( p_PMAC_TIME ) * i;
                  output_command.points[i].positions = joint_values;
                }
                publisher_command.publish ( output_command );
              }
            }
          }
          else if ( COMMAND == "MOVL" ) //--------------------------------------
          {
            if ( ZERO )
            {
              UpdateRef = true;
            }
            else
            {
              std::vector<double> values = VALUES;
              values[0] = VALUES[0] / 1000.0;
              values[1] = VALUES[1] / 1000.0;
              values[2] = VALUES[2] / 1000.0;
              values[3] = VALUES[3] / 180.0 * 3.1415926;
              values[4] = VALUES[4] / 180.0 * 3.1415926;
              values[5] = VALUES[5] / 180.0 * 3.1415926;
              Eigen::Affine3d target;
              target =  Eigen::Translation3d ( values[0], values[1], values[2] ) *
                        Eigen::AngleAxisd ( values[3], Eigen::Vector3d::UnitX() ) *
                        Eigen::AngleAxisd ( values[4], Eigen::Vector3d::UnitY() ) *
                        Eigen::AngleAxisd ( values[5], Eigen::Vector3d::UnitZ() );
              robot_state::RobotStatePtr current_state ( new robot_state::RobotState ( *s_COMMAND_REFERENCE_STATE ) );
              std::vector<robot_state::RobotStatePtr> traj;
              bool global_reference_frame = true;
              double max_step = 0.2;
              double jump_threshold = 0.0;
              double result = current_state->computeCartesianPath ( p_JOINT_MODEL_GROUP, traj, current_state->getLinkModel ( p_EEF_NAME ),
                              target, global_reference_frame,  max_step, jump_threshold );
              if ( result <= 1.0 )
              {
                ROS_INFO ( "Did not find IK solution" );
              }
              else
              {
                trajectory_msgs::JointTrajectory output_command;
                output_command.header.stamp = ros::Time::now();
                output_command.joint_names = p_JOINT_NAMES;
                output_command.points.resize ( traj.size () );
                for ( size_t i = 0; i < traj.size (); ++i )
                {
                  std::vector<double> joint_values;
                  traj[i]->copyJointGroupPositions ( p_JOINT_MODEL_GROUP, joint_values );
                  for ( std::size_t j = 0; j < p_JOINT_NAMES.size(); ++j )
                  {
                    ROS_INFO ( "Joint %s: %f", p_JOINT_NAMES[j].c_str(), joint_values[j] );
                  }
                  //
                  s_COMMAND_REFERENCE_JOINT_STATE.name = p_JOINT_NAMES;
                  s_COMMAND_REFERENCE_JOINT_STATE.position = joint_values;
                  s_COMMAND_REFERENCE_STATE->setVariableValues ( s_COMMAND_REFERENCE_JOINT_STATE );
                  output_command.points[i].time_from_start = ros::Duration ( p_PMAC_TIME ) * i;
                  output_command.points[i].positions = joint_values;
                }
                publisher_command.publish ( output_command );
              }
            }
          }
          else if ( COMMAND == "MOVWP" ) //-------------------------------------
          {
            if ( ZERO )
            {
              UpdateRef = true;
            }
            else
            {
              std::vector<double> values = VALUES;
              values[0] = VALUES[0] / 1000.0;
              values[1] = VALUES[1] / 1000.0;
              values[2] = VALUES[2] / 1000.0;
              values[3] = VALUES[3] / 180.0 * 3.1415926;
              values[4] = VALUES[4] / 180.0 * 3.1415926;
              values[5] = VALUES[5] / 180.0 * 3.1415926;

              EigenSTL::vector_Affine3d targets;
              Eigen::Affine3d target;
              target =  Eigen::Translation3d ( values[0], values[1], values[2] ) *
                        Eigen::AngleAxisd ( values[3], Eigen::Vector3d::UnitX() ) *
                        Eigen::AngleAxisd ( values[4], Eigen::Vector3d::UnitY() ) *
                        Eigen::AngleAxisd ( values[5], Eigen::Vector3d::UnitZ() );


              robot_state::RobotStatePtr current_state ( new robot_state::RobotState ( *s_COMMAND_REFERENCE_STATE ) );
              std::vector<robot_state::RobotStatePtr> traj;
              bool global_reference_frame = true;
              double max_step = 0.2;
              double jump_threshold = 0.0;
              double result = current_state->computeCartesianPath ( p_JOINT_MODEL_GROUP, traj, current_state->getLinkModel ( p_EEF_NAME ),
                              targets, global_reference_frame,  max_step, jump_threshold );
              if ( result <= 1.0 )
              {
                ROS_INFO ( "Did not find IK solution" );
              }
              else
              {
                trajectory_msgs::JointTrajectory output_command;
                output_command.header.stamp = ros::Time::now();
                output_command.joint_names = p_JOINT_NAMES;
                output_command.points.resize ( traj.size () );
                for ( size_t i = 0; i < traj.size (); ++i )
                {
                  std::vector<double> joint_values;
                  traj[i]->copyJointGroupPositions ( p_JOINT_MODEL_GROUP, joint_values );
                  for ( std::size_t j = 0; j < p_JOINT_NAMES.size(); ++j )
                  {
                    ROS_INFO ( "Joint %s: %f", p_JOINT_NAMES[j].c_str(), joint_values[j] );
                  }
                  //
                  s_COMMAND_REFERENCE_JOINT_STATE.name = p_JOINT_NAMES;
                  s_COMMAND_REFERENCE_JOINT_STATE.position = joint_values;
                  s_COMMAND_REFERENCE_STATE->setVariableValues ( s_COMMAND_REFERENCE_JOINT_STATE );
                  output_command.points[i].time_from_start = ros::Duration ( p_PMAC_TIME ) * i;
                  output_command.points[i].positions = joint_values;
                }
                publisher_command.publish ( output_command );
              }
            }
          }
          else
          {
            // other command
          }
        } // good command
      } // new command
      //----------
      if ( UpdateRef )
      {
        s_COMMAND_REFERENCE_JOINT_STATE = s_CURRENT_JOINT_STATE;
        s_COMMAND_REFERENCE_STATE->setVariableValues ( s_COMMAND_REFERENCE_JOINT_STATE );
      }
      //
      // control time
      //----------
      double  sleep_time = p_LOOP_TIME - ( ros::Time::now().toSec() - start );
      if ( sleep_time > 0 )
      {
        ros::Duration ( sleep_time ).sleep();
      }
      else
      {
        ROS_WARN ( "control loop overtime: %f s", -sleep_time );
      }
      //
      //ROS_INFO ( "%s", status.c_str() );
    }
    catch ( xc::Result err )
    {
      ROS_ERROR ( "recognized error occured: %s", err.message() );
    }
  }
  ros::shutdown();
  return 0;
}
