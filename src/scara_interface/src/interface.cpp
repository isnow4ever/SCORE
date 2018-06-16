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

// override XC_PRINT functions with ROS functions
#define XC_DEBUG(...)   ROS_DEBUG(__VA_ARGS__)
#define XC_INFO(...)    ROS_INFO(__VA_ARGS__)
#define XC_WARN(...)    ROS_WARN(__VA_ARGS__)
#define XC_ERROR(...)   ROS_ERROR(__VA_ARGS__)
//
#include <cmath>
#include <Eigen/Eigen>
//
#include <boost/algorithm/string.hpp>
//
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/String.h>
//
#include <xc/core/core.h>
#include <xcdev/PMAC/protocol.h>
//
#include "constants.h"
#include "spline.h"

const int MODE_INIT             = 0;
const int MODE_DIRECT           = 1;
const int MODE_SPLINE           = 2;
const int MODE_STOP             = 3;

const int ERROR_UNHANDLEABLE_INPUT                  = 1;
const int ERROR_INVALID_TRAJECTORY                  = 2;
const int ERROR_INVALID_FEEDBACK                    = 3;

bool stop_flag = 0;
class JointTrajectoryHandller
{
 public:
  void callback ( const trajectory_msgs::JointTrajectory::ConstPtr & message )
  {
    mCrtMsg = *message;
    mNewMsg = true;
  }

  JointTrajectoryHandller()
    : mCrtMsg()
    , mNewMsg ( false )
  {
  }

  const bool newMsg()
  {
    return mNewMsg;
  }

  void clear()
  {
    mNewMsg = false;
  }

  const trajectory_msgs::JointTrajectory & fetchMsg()
  {
    mNewMsg = false;
    return mCrtMsg;
  }

 protected:
  trajectory_msgs::JointTrajectory mCrtMsg;
  bool mNewMsg;
};

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

  void clear()
  {
    mNewMsg = false;
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
class PTTrajectory
{
 public:

  PTTrajectory ( std::vector< std::string> _in pmacNames, double _in timestep )
    : mAxis ( pmacNames ),
      mData ( pmacNames.size() ),
      mTimeStep ( timestep )
  {
  }

  PTTrajectory ( std::vector< std::string> _in pmacNames, size_t _in steps, double _in timestep )
    : mAxis ( pmacNames ),
      mData ( pmacNames.size() ),
      mTimeStep ( timestep )
  {
    Resize ( steps );
  }

  PTTrajectory ( PTTrajectory _in v )
    : mAxis ( v.mAxis ),
      mData ( v.mData ),
      mTimeStep ( v.mTimeStep )
  {

  }

  virtual ~PTTrajectory()
  {

  }

  inline void Reset ( std::vector< std::string> _in pmacNames, size_t _in steps, double _in timestep )
  {
    mAxis = pmacNames;
    mData.resize ( mAxis.size() );
    mTimeStep = timestep;
    Resize ( steps );
  }

  inline size_t _ret NumOfAxis() const
  {
    return mAxis.size();
  }

  inline size_t _ret NumOfPoints()
  {
    return mData[0].size();
  }

  inline void clear()
  {
    for ( size_t i = 0; i < mData.size(); ++i )
    {
      mData[i].clear();
    }
  }

  inline void Resize ( size_t _in steps )
  {
    for ( size_t i = 0; i < mData.size(); ++i )
    {
      mData[i].resize ( steps );
    }
  }

  inline std::vector<double> _ret GetPoint ( size_t _in step )
  {
    std::vector<double> ret ( NumOfAxis() );

    for ( size_t i = 0; i < NumOfAxis(); ++i )
      ret[i] = mData[i][step];

    return ret;
  }

  inline void Set ( size_t _in axis, size_t _in step, double _in v )
  {
    mData[axis][step] = v;
  }

  inline double _ret Get ( size_t _in axis, size_t _in step ) const
  {
    return mData[axis][step];
  }

  inline void SetStep ( size_t _in step, std::vector<double> _in v )
  {
    for ( size_t i = 0; i < NumOfAxis(); ++i )
      mData[i][step] = v[i];

    return;
  }

  inline void PushBack ( std::vector<double> _in v )
  {
    for ( size_t i = 0; i < NumOfAxis(); ++i )
      mData[i].push_back ( v[i] );

    return;
  }

  inline std::vector<double> _ret PopFront()
  {
    std::vector<double> ret ( NumOfAxis() );

    for ( size_t i = 0; i < NumOfAxis(); ++i )
    {
      ret[i] = mData[i].front();
      mData[i].pop_front();
    }

    return ret;
  }

  inline std::string _ret PopFrontString ( double _in ratio = 180.0 / 3.1415926 )
  {
    std::string ret;
    ret.reserve ( 128 );
    for ( size_t i = 0; i < NumOfAxis(); ++i )
    {
      xc::StringAppend ( ret, "%s%lf", mAxis[i].c_str(), mData[i].front() * ratio );
      mData[i].pop_front();
    }
    return ret;
  }

  inline void Append ( PTTrajectory _in v )
  {
    for ( size_t i = 0; i < NumOfAxis(); ++i )
    {
      mData[i].insert ( mData[i].end(),
                        v.mData[i].begin(),
                        v.mData[i].end() );
    }
  }

 public:
  std::vector< std::deque<double> >  mData;
  std::vector< std::string>          mAxis;
  double                             mTimeStep;
};

class TrajectoryProcessor
{
 public:
  TrajectoryProcessor ( double _in step )
    : mAxisROS()
    , mAxisPMAC()
    , mAxisFdb()
    , mTimeStep ( step )
  {

  }

  TrajectoryProcessor ( TrajectoryProcessor _in v )
    : mAxisROS ( v.mAxisROS )
    , mAxisPMAC ( v.mAxisPMAC )
    , mAxisFdb ( v.mAxisFdb )
    , mTimeStep ( v.mTimeStep )
  {

  }

  virtual ~TrajectoryProcessor()
  {

  }

  void AddAxis ( std::string _in joint, std::string _in pmac, std::string _in fdb )
  {
    mAxisROS.push_back ( joint );
    mAxisPMAC.push_back ( pmac );
    mAxisFdb.push_back ( fdb );
  }

  size_t _ret NumOfAxis()
  {
    return mAxisROS.size();
  }

  std::vector<std::string> _rd JOINTS() const
  {
    return mAxisROS;
  }

  std::vector<std::string> _rd PMAC() const
  {
    return mAxisPMAC;
  }

  PTTrajectory _ret MakeTrajectory()
  {
    return PTTrajectory ( mAxisPMAC, mTimeStep );
  }

  std::string _ret MakeFeedbackCommand()
  {
    std::string ret;
    for ( size_t i = 0; i < NumOfAxis(); ++i )
    {
      xc::StringAppend ( ret, "%s", mAxisFdb[i].c_str() );
    }

    // return
    return ret;
  }

  xc::Result _ret GenerateMap ( trajectory_msgs::JointTrajectory _in trajecotry, std::vector<size_t> _out ros2trj )
  {
    // check size
    if ( trajecotry.joint_names.size() != NumOfAxis() )
    {
      return xc::Result ( ERROR_INVALID_TRAJECTORY, "invalid trajecotry of disagreed joints" );
    }
    // check map
    ros2trj.resize ( NumOfAxis() );
    for ( size_t i = 0; i < trajecotry.joint_names.size(); ++i )
    {
      bool maped = false;
      for ( size_t j = 0; j < NumOfAxis(); ++j )
      {
        if ( trajecotry.joint_names[i] == mAxisROS[j] )
        {
          ros2trj[i] = j;
          maped = true;
        }
      }
      if ( !maped )
      {
        return xc::Result ( ERROR_INVALID_TRAJECTORY, "invalid trajecotry of disagreed joints" );
      }
    }
    // check points size
    if ( trajecotry.points.size() <= 0 )
    {
      return xc::Result ( ERROR_INVALID_TRAJECTORY, "invalid trajecotry of empty data" );
    }

    // return
    return xc::Result::Success();
  }

  xc::Result _ret TranslateDirect ( trajectory_msgs::JointTrajectory _in trajectory, PTTrajectory _out outputPOS )
  {
    // check
    std::<size_t> ros2trj;
    xc::Result res = GenerateMap ( trajectory, ros2trj );
    if ( res.is_failed () )
    {
      return res;
    }

    // copy and check time step
    outputPOS.Reset ( mAxisPMAC, trajectory.points.size(), mTimeStep );
    for ( size_t p = 0; p < trajectory.points.size(); ++p )
    {
      trajectory_msgs::JointTrajectoryPoint pnt = trajectory.points[p];

      if ( fabs ( pnt.time_from_start.toSec() - ( p + 1) *mTimeStep ) > mTimeStep / 10.0 )
        return xc::Result ( ERROR_INVALID_TRAJECTORY, "invalid trajecotry of disagreed step time" );

      for ( size_t i = 0; i < NumOfAxis(); ++i )
      {
        outputPOS.Set ( ros2trj[i], p, pnt.positions[i] );
      }
    }

    // return
    return xc::Result::Success ();
  }

  xc::Result _ret TranslateSpline ( trajectory_msgs::JointTrajectory _in trajectory, PTTrajectory _out outputPOS )
  {
    // check
    std::vector<size_t> ros2trj;
    xc::Result res = GenerateMap ( trajectory, ros2trj );
    if ( res.is_failed () )
    {
      return res;
    }

    // trajectory has to contain at least 2 points
    if ( trajectory.points.size() <= 1 )
    {
      return xc::Result ( ERROR_INVALID_TRAJECTORY, "invalid trajecotry of disagreed step time" );
    }

    // get input
    double inputT[trajectory.points.size()];
    double inputPOS[NumOfAxis()][trajectory.points.size()];
    for ( size_t p = 0; p < trajectory.points.size(); ++p )
    {
      trajectory_msgs::JointTrajectoryPoint pnt = trajectory.points[p];
      inputT[p] = pnt.time_from_start.toSec();

      for ( size_t i = 0; i < NumOfAxis(); ++i )
      {
        inputPOS[ros2trj[i]][p] = pnt.positions[i];
      }
    }

    // if the input contains more than one point, uses spline
    size_t outputLength = ceil ( ( inputT[trajectory.points.size() - 1] - inputT[0] ) / mTimeStep ) + 1;
    double outputT[outputLength];
    for ( size_t i = 0; i < outputLength - 1; ++i )
      outputT[i] =  inputT[0] + mTimeStep * i;
    outputT[outputLength - 1] = inputT[trajectory.points.size() - 1];
    outputPOS.Reset ( mAxisPMAC, outputLength, mTimeStep );
    double b[trajectory.points.size()];
    double c[trajectory.points.size()];
    double d[trajectory.points.size()];
    for ( size_t i = 0; i < NumOfAxis(); ++i )
    {
      int result = 0;
      spline ( trajectory.points.size(), 0, 0, 0, 0, inputT, inputPOS[i], b, c, d, &result );
      if ( 0 != result )
      {
        return xc::Result ( ERROR_INVALID_TRAJECTORY, "trajecotry interpolation with spline method failed" );
      }
      //
      int last = 0;
      for ( size_t j = 0; j < outputLength; ++j )
      {
        double p = seval ( trajectory.points.size(), outputT[j], inputT, inputPOS[i], b, c, d, &last );
        outputPOS.Set ( i, j, p );
      }
    }

    // return
    return xc::Result::Success ();
  }

 protected:
  std::vector<std::string> mAxisROS;
  std::vector<std::string> mAxisPMAC;
  std::vector<std::string> mAxisFdb;
  double                   mTimeStep;
};

//------------------------------------------------------------------------------
// main
//
//
//------------------------------------------------------------------------------
int main ( int argc, char **argv )
{
  //--- init ROS, do nothing, if a excpetion is thrown -------------------------
  printf ( "initializing ROS\n" );
  ros::init ( argc, argv, "x50_interface" );
  ros::NodeHandle NODE;
  printf ( "initializing ROS succeed\n" );
  //----------------------------------------------------------------------------

  while ( ros::ok() )
  {
    ROS_INFO ( "main-loop begins" );
    //---perfactly OK if a excpetion is thrown----------------------------------
    //
    //
    ROS_INFO ( "setting parameters" );
    const double            p_PMAC_TIME              = PMAC_LOOP;
    const double            p_LOOP_TIME              = INTERFACE_LOOP;
    const double            p_FEEDBACK_TIME          = STATE_LOOP;
    //
    std::string             p_ROBOT_IP               = "192.6.94.5";  //"127.0.0.1";
    std::string             p_ROBOT_PORT             = "1025";        //"1025";
    //
    TrajectoryProcessor     TRAJ_PROCESSOR ( p_PMAC_TIME );
    TRAJ_PROCESSOR.AddAxis ( "joint_1", "X", "P1" );
    TRAJ_PROCESSOR.AddAxis ( "joint_2", "Y", "P2" );
    TRAJ_PROCESSOR.AddAxis ( "joint_3", "Z", "P3" );
    TRAJ_PROCESSOR.AddAxis ( "joint_4", "A", "P4" );
    TRAJ_PROCESSOR.AddAxis ( "joint_5", "B", "P5" );
    TRAJ_PROCESSOR.AddAxis ( "joint_6", "C", "P6" );
    //
    const double            p_MinimalInputTime     = 1.0;
    const size_t            p_MinimalStartSize     = 5;
    const size_t            p_MinimalSendSize      = 7;
    //
    const size_t            p_RotaryBufferLength   = 80;
    const size_t            p_RotaryBufferSize     = TRAJ_PROCESSOR.NumOfAxis() * p_RotaryBufferLength;
    //
    if ( !NODE.getParam ( "robot_ip", p_ROBOT_IP ) )
    {
      ROS_WARN ( "can not read parameter robot_ip, use default value: %s", p_ROBOT_IP.c_str() );
    }
    //
    if ( !NODE.getParam ( "robot_port", p_ROBOT_PORT ) )
    {
      ROS_WARN ( "can not read parameter robot_port, use default value: %s", p_ROBOT_PORT.c_str() );
    }
    ROS_INFO ( "setting parameters succeed" );
    //--------------------------------------------------------------------------
    ROS_INFO ( "initializing subscriber and publisher" );
    JointTrajectoryHandller TRAJECTORY_HANDLLER;
    StringHandller          COMMAND_HANDLLER;
    ros::Subscriber subscriber_joint_path =
      NODE.subscribe<trajectory_msgs::JointTrajectory> (
        INTERFACE_JOINT_PATH_COMMAND, 1024, &JointTrajectoryHandller::callback,
        &TRAJECTORY_HANDLLER );
    ros::Subscriber subscriber_command =
      NODE.subscribe<std_msgs::String> (
        INTERFACE_COMMAND, 1024, &StringHandller::callback,
        &COMMAND_HANDLLER );
    ros::Publisher  pFeedbackStates =
      NODE.advertise<control_msgs::FollowJointTrajectoryFeedback> (
        INTERFACE_FEEDBACK_STATES, 1024 );
    ros::Publisher  pJointStates =
      NODE.advertise<sensor_msgs::JointState> (
        INTERFACE_JOINT_STATES, 1024 );
    ros::Publisher  pInterfaceStates =
      NODE.advertise<std_msgs::String> (
        INTERFACE_STATES, 1024 );
    ROS_INFO ( "initializing subscriber and publisher succeed" );
    //--------------------------------------------------------------------------
    ROS_INFO ( "connecting to PMAC" );
    xc::Dev::PMAC::ConnectorPtr  PMAC_CONNECTOR = xc::Dev::PMAC::Connector::MakeInstace();
    //

    while ( !PMAC_CONNECTOR->Connect ( p_ROBOT_IP, p_ROBOT_PORT ) )
    {
      ROS_INFO ( "failed to connect to PMAC." );
      for ( size_t i = 3; i > 0; --i )
      {
        ROS_INFO ( "retry in %lu seconds.", i );
        sleep ( 1 );
      }
    }
    ROS_INFO ( "connecting to PMAC succeed" );
    //--------------------------------------------------------------------------
    ROS_INFO ( "creating PMAC operators" );
    xc::Dev::PMAC::OnlineOperator       PMAC_OPERATOR ( PMAC_CONNECTOR );
    xc::Dev::PMAC::RotaryBufferOperator PMAC_BUFFER ( PMAC_CONNECTOR );
    ROS_INFO ( "creating PMAC operators succeed" );
    if(stop_flag)
    {
        PMAC_OPERATOR.SendLine("#1k");
        ROS_INFO("motors killed...");
        stop_flag = 0;
    }
    //
    //
    //---perfactly OK if a excpetion is thrown----------------------------------

    //---excpetions is thrown---------------------------------------------------
    //

    try
    {
      //------------------------------------------------------------------------
      ROS_INFO ( "initializing PMAC" );
      PMAC_OPERATOR.Flush();
      PMAC_OPERATOR.SendLine ( "?" );
      PMAC_OPERATOR.ReadReady();
      ROS_INFO ( "initializing PMAC succeed" );
      //------------------------------------------------------------------------
      int                     s_MODE                   = MODE_INIT;
      //------------------------------------------------------------------------------
      ROS_INFO ( "starting control-loop" );

      //
      PTTrajectory            s_TRAJECTORY             = TRAJ_PROCESSOR.MakeTrajectory();
      double                  s_LAST_INPUT_TIME        = 0.0;
      //
      double                  s_LAST_FEEDBACK_TIME     = 0.0;
      std::vector<double>     s_LAST_FEEDBACK ( TRAJ_PROCESSOR.NumOfAxis(), 0 );
      //
      size_t                  s_REMAINING_COMMAND_SIZE = 0;
      double                  s_EXPECTED_FINISH_TIME   = ros::Time::now ().toSec ();
      bool                    s_MOVING                 = false;
      //
      ROS_INFO ( "control loop started" );
      std::string strActions;
      std::string strStatus;
      strActions.reserve ( 256 );
      strStatus.reserve ( 256 );
      while ( ros::ok() && MODE_STOP != s_MODE )
      {
        strActions.clear ();
        double start = ros::Time::now().toSec();
        //---excpetions is thrown-----------------------------------------------
        try
        {
          bool communicated = false;
          //---spin-------------------------------------------------------------
          ros::spinOnce();
          //---read input of command and determine state------------------------

          if ( COMMAND_HANDLLER.newMsg() )
          {
 //              ROS_INFO("%d\n", s_MODE);
            std_msgs::String input_ = COMMAND_HANDLLER.fetchMsg();
            std::string input = input_.data;
            boost::to_upper ( input );
            std::cout << "get input " <<input << std::endl;
            xc::StringAppend ( strActions, "%s\t", input.c_str () );
            if ( MODE_DIRECT == s_MODE || MODE_SPLINE == s_MODE )
            {
              if ( input == INTERFACE_CMD_STOP )
              {
                s_MODE = MODE_STOP;
                stop_flag = 1;
              }
              else if ( input == INTERFACE_CMD_CLEAR )
              {
                s_TRAJECTORY.clear();
              }
            }
            else if ( MODE_INIT == s_MODE )
            {
              if ( input == INTERFACE_CMD_STOP )
              {
                s_TRAJECTORY.clear();
                TRAJECTORY_HANDLLER.clear ();
                s_MODE = MODE_STOP;
                stop_flag = 1;
              }
              else if ( input == INTERFACE_CMD_DIRECT || input == INTERFACE_CMD_SPLINE )
              {
                s_TRAJECTORY.clear();
                TRAJECTORY_HANDLLER.clear ();
                //--------------------------------------------------------------
                ROS_INFO ( "initializing rotary buffer" );
                // PMAC_BUFFER.WriteBuffer ( "CLEAR" ); // ****** TEST ******
                xc::Dev::PMAC::Array response;
                response = PMAC_OPERATOR.GetResponse ( "?" );
                ROS_INFO("%s", response.ToString().c_str());

                PMAC_OPERATOR.GetResponse ( "CLOSE ALL" );
                PMAC_OPERATOR.GetResponse ( "END GATHER" );
                PMAC_OPERATOR.GetResponse ( "DELETE GATHER" );
                PMAC_OPERATOR.GetResponse ( "DELETE ROT" );
 //                PMAC_OPERATOR.GetResponse ( "#1o0" );
                PMAC_OPERATOR.GetResponse ( "#1J/#2J/#3J/#4J/#5J/#6J/" );
                ROS_INFO ( "motor enabled" );
                PMAC_BUFFER.Create ( 1, p_RotaryBufferSize );
                //PMAC_CONNECTOR->Close(); // ****** TEST ******
                PMAC_BUFFER.Locate ( 0 );
                PMAC_BUFFER.Open();
                PMAC_BUFFER.WriteBuffer ( "CLEAR" );
                PMAC_BUFFER.WriteBuffer ( "SPLINE1 TM 50" );
                PMAC_BUFFER.Close();
                ROS_INFO ( "initializing rotary buffer succeed" );
                //--------------------------------------------------------------
                ROS_INFO ( "running PMAC program" );
                PMAC_OPERATOR.GetResponse ( "R" );
                ROS_INFO ( "running PMAC program succeed" );
                //--------------------------------------------------------------
                if ( input == INTERFACE_CMD_SPLINE )
                  s_MODE = MODE_SPLINE;
                else
                  s_MODE = MODE_DIRECT;
              }
            }
          }
          //---read input of trajectory and maintain PMAC trajectory----------
          if ( TRAJECTORY_HANDLLER.newMsg() )
          {
            xc::StringAppend ( strActions, "new_trajectory\t" );
            trajectory_msgs::JointTrajectory input = TRAJECTORY_HANDLLER.fetchMsg();
            if ( ( MODE_DIRECT == s_MODE || MODE_SPLINE == s_MODE ) )
            {
              PTTrajectory output = TRAJ_PROCESSOR.MakeTrajectory();
              if ( MODE_DIRECT == s_MODE )
              {
                xc::Result ec = TRAJ_PROCESSOR.TranslateDirect ( input, output );
                if ( ec.is_failed() )
                  throw ec;
              }
              else if ( MODE_SPLINE == s_MODE )
              {
                xc::Result ec = TRAJ_PROCESSOR.TranslateSpline ( input, output );
                if ( ec.is_failed() )
                  throw ec;
                if ( s_TRAJECTORY.NumOfPoints () > 0 )
                  throw xc::Result ( ERROR_UNHANDLEABLE_INPUT,
                                     "invalid trajectory for auto model: platform is moving" );
                if ( output.NumOfPoints() < p_MinimalStartSize )
                  throw xc::Result ( ERROR_UNHANDLEABLE_INPUT,
                                     "invalid trajectory for auto model: not enough points" );
              }
              //----------------------------------------------------------------
              if ( ros::Time::now().toSec() - s_LAST_INPUT_TIME > p_MinimalInputTime )
              {
                ROS_WARN ( "clear trajectory" );
                s_TRAJECTORY.clear();
              }
              s_TRAJECTORY.Append ( output );
              s_LAST_INPUT_TIME =  ros::Time::now().toSec();
            }
            else
            {
              s_TRAJECTORY.clear ();
            }
          }
          //---feedback---------------------------------------------------------
          if ( MODE_STOP != s_MODE )
          {
            if ( ros::Time::now().toSec() + p_LOOP_TIME - s_LAST_FEEDBACK_TIME - p_FEEDBACK_TIME >= 0 )
            {
              xc::StringAppend ( strActions, "feedback\t");

              communicated = true;
              // read state from pmac
              std::vector<double> feedback ( TRAJ_PROCESSOR.NumOfAxis() ), velocity ( TRAJ_PROCESSOR.NumOfAxis() );
              std::string feedback_command = TRAJ_PROCESSOR.MakeFeedbackCommand();
              xc::Dev::PMAC::Array feedback_array = PMAC_OPERATOR.GetResponse ( feedback_command );
              xc::Dev::PMAC::Reply feedback_reply ( feedback_array );
              for ( size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis(); ++i )
              {
                try
                {
                  feedback[i] = feedback_reply.AsDouble ( i ) / 180.0 * 3.1415926;
                  velocity[i] = ( feedback[i] - s_LAST_FEEDBACK[i] ) / ( ros::Time::now().toSec() - s_LAST_FEEDBACK_TIME );
                }
                catch ( ... )
                {
                  throw xc::Result ( ERROR_INVALID_FEEDBACK, "can not retrieve feedback data" );
                }
              }
              // publish joint state
              sensor_msgs::JointState jointState;
              jointState.header.stamp = ros::Time::now();
              jointState.name = TRAJ_PROCESSOR.JOINTS();
              jointState.position = feedback;
              jointState.velocity = velocity;
              pJointStates.publish ( jointState );
              // publish feedback
              control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_feedback;
              follow_joint_trajectory_feedback.header.stamp = ros::Time::now();
              follow_joint_trajectory_feedback.joint_names = TRAJ_PROCESSOR.JOINTS();
              follow_joint_trajectory_feedback.actual.positions = feedback;
              pFeedbackStates.publish ( follow_joint_trajectory_feedback );
              //
              s_LAST_FEEDBACK_TIME = ros::Time::now().toSec();
              s_LAST_FEEDBACK = feedback;
            }
          }
          else
          {
            s_LAST_FEEDBACK_TIME = 0.0;
            std::fill ( s_LAST_FEEDBACK.begin (), s_LAST_FEEDBACK.end (), 0.0 );
          }
          //---send trajectory to pmac------------------------------------------
          if ( MODE_DIRECT == s_MODE  ||  MODE_SPLINE == s_MODE )
          {
            // wait for at least MINIMAL_START_SIZE points
            if ( !s_MOVING && s_TRAJECTORY.NumOfPoints() >= p_MinimalStartSize )
              s_MOVING = true;
            // send trajectory
            if ( s_MOVING  && !communicated )
            {
              communicated = true;
              // check rotary buffer and send command---------------------------
              if ( s_EXPECTED_FINISH_TIME - ros::Time::now().toSec() < p_PMAC_TIME * p_MinimalSendSize )
              {
                xc::StringAppend ( strActions, "send_trajectory\t" );
                PMAC_BUFFER.Open();
                xc::Dev::PMAC::Reply rp = PMAC_BUFFER.PR();
                s_REMAINING_COMMAND_SIZE = rp.AsLong ( 0 );
                if ( s_TRAJECTORY.NumOfPoints() > 0 )
                {
                  size_t SendSize = s_TRAJECTORY.NumOfPoints() > 10 ? 10 : s_TRAJECTORY.NumOfPoints();
                  std::vector<xc::String> lines;
                  lines.reserve ( SendSize );
                  //ROS_INFO ( "SendSize = %ld", SendSize );
                  for ( size_t i = 0; i < SendSize; ++i )
                  {
                    std::string aline = s_TRAJECTORY.PopFrontString ( 180.0 / 3.1415926 );
                    lines.push_back ( aline );
                    //ROS_INFO ( "%s", aline.c_str() );
                    //PMAC_BUFFER.WriteBuffer ( aline );
                  }

                  PMAC_BUFFER.WriteBuffer ( lines );
                  s_REMAINING_COMMAND_SIZE = s_REMAINING_COMMAND_SIZE + SendSize;
                }
                s_EXPECTED_FINISH_TIME = static_cast<double> ( s_REMAINING_COMMAND_SIZE ) * p_PMAC_TIME +  ros::Time::now().toSec();
                PMAC_BUFFER.Close();
                //
                if ( s_REMAINING_COMMAND_SIZE == 0 )
                {
                  s_MOVING = false;
                  ROS_WARN ( "->  stoped  <-");
                }
              }
              else
              {
                s_REMAINING_COMMAND_SIZE = ( s_EXPECTED_FINISH_TIME - ros::Time::now().toSec() ) / p_PMAC_TIME;
              }
            }
          }
          else
          {
            s_MOVING = false;
            s_REMAINING_COMMAND_SIZE = 0.0;
            s_EXPECTED_FINISH_TIME = 0.0;
          }
          //---send status------------------------------------------------------
          strStatus.clear();
          if ( MODE_DIRECT == s_MODE )
            xc::StringAppend ( strStatus, INTERFACE_STATE_DIRECT INTERFACE_STATE_SPARATER );
          else if ( MODE_SPLINE == s_MODE )
            xc::StringAppend ( strStatus, INTERFACE_STATE_SPLINE INTERFACE_STATE_SPARATER );
          else if ( MODE_INIT == s_MODE )
            xc::StringAppend ( strStatus, "INIT" INTERFACE_STATE_SPARATER );
          else if ( MODE_STOP == s_MODE )
            xc::StringAppend ( strStatus, "STOP" INTERFACE_STATE_SPARATER );
          //
          if ( s_MOVING )
            xc::StringAppend ( strStatus, INTERFACE_STATE_MOVING INTERFACE_STATE_SPARATER );
          else
            xc::StringAppend ( strStatus, INTERFACE_STATE_STABLE INTERFACE_STATE_SPARATER );
          //
          xc::StringAppend ( strStatus, "%4ld %8.3lf\t", s_REMAINING_COMMAND_SIZE, s_EXPECTED_FINISH_TIME );
          ROS_DEBUG ( "%s" INTERFACE_STATE_SPARATER "%s", strStatus.c_str (), strActions.c_str () );
          //
          std_msgs::String output;
          output.data = strStatus;
          pInterfaceStates.publish ( output );
          //---control time-----------------------------------------------------
          double  sleep_time = p_LOOP_TIME - ( ros::Time::now().toSec() - start );
          if ( sleep_time > 0 )
            ros::Duration ( sleep_time ).sleep();
          else
            ROS_WARN ( "control loop overtime: %f s", -sleep_time );
          //
          //
          //---excpetions is thrown---------------------------------------------
        }
        catch ( xc::Result err )
        {
          if ( ERROR_UNHANDLEABLE_INPUT == err.code() )
          {
            ROS_ERROR ( "recognized error occured: %s. "
                        "input is ignored.",
                        err.message() );
          }
          else if ( ERROR_INVALID_TRAJECTORY == err.code() )
          {
            ROS_ERROR ( "recognized error occured: %s. "
                        "PMAC program is stoped.",
                        err.message() );
            s_MODE = MODE_INIT;
          }
          else if ( ERROR_INVALID_FEEDBACK == err.code() )
          {
            ROS_ERROR ( "recognized error occured: %s. "
                        "please TURN OFF the hardware IMMEDIATELY" ,
                        err.message() );
          }
          else
          {
            ROS_ERROR ( "recognized error occured: %s. "
                        "please TURN OFF the hardware IMMEDIATELY" ,
                        err.message() );
          }
        }
      } // control-loop
      ROS_INFO ( "control-loop endded" );
      PMAC_OPERATOR.GetResponse ( "CLOSE ALL" );
      PMAC_OPERATOR.GetResponse ( "A" );
      PMAC_OPERATOR.GetResponse ( "DELETE ROT" );
      PMAC_OPERATOR.GetResponse ( "DELETE GATHER" );
      PMAC_CONNECTOR->Close();
      //
    }
    catch ( xc::exception ec )
    {
      ROS_ERROR ( "a unhandleable boost exception ourcced. emergency stop.\n"
                  "------------------------------NOTE------------------------------\n"
                  "There is NO tested method to recover the robot from this except-\n"
                  "ion right now.\n "
                  "xc:exception %s",
                  ec.Message() );
    }
    catch ( std::exception ec )
    {
      ROS_ERROR ( "a unhandleable std exception ourcced. emergency stop.\n"
                  "------------------------------NOTE------------------------------\n"
                  "There is NO tested method to recover the robot from this except-\n"
                  "ion right now.\n "
                  "std::exception %s",
                  ec.what() );
    }
    catch ( boost::system::error_code ec )
    {
      ROS_ERROR ( "a unhandleable boost exception ourcced. emergency stop.\n"
                  "------------------------------NOTE------------------------------\n"
                  "There is NO tested method to recover the robot from this except-\n"
                  "ion right now.\n "
                  "boost::system::error_code %s:%s",
                  ec.category().name(),
                  ec.message().c_str() );
    }
    catch ( ... )
    {
      ROS_ERROR ( "a unhandleable unkown exception ourcced. emergency stop.\n"
                  "------------------------------NOTE------------------------------\n"
                  "There is NO tested method to recover the robot from this except-\n"
                  "ion right now.\n " );
    }
    //
    //
    //---excpetions is thrown---------------------------------------------------

    //---perfactly OK if a excpetion is thrown----------------------------------
    //
    //
    //
    //
    subscriber_joint_path.shutdown();
    subscriber_command.shutdown();
    pFeedbackStates.shutdown();
    pJointStates.shutdown();
    ROS_INFO ( "shutting down subscriber and publisher succeed" );
    if ( ros::ok() )
    {
      for ( size_t i = 1; i > 0; --i )
      {
        ROS_INFO ( "restart main-loop in %lu seconds.", i );
        sleep ( 1 );
      }
    }
    //
    //
    //
    //---perfactly OK if a excpetion is thrown----------------------------------
    ROS_INFO ( "main-loop ends" );

  } // main loop
  ros::waitForShutdown();
  return 0;
}
