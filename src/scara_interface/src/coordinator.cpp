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
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include "constants.h"
#include <xc/math/math.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//------------------------------------------------------------------------------
class StringHandller
{
public:
    void callback ( const std_msgs::String::ConstPtr & message )
    {
        mCrtMsg.push_back(*message);
    }

    StringHandller()
        : mCrtMsg()
    {
    }

    const bool newMsg()
    {
        return mCrtMsg.size() > 0;
    }

    const std_msgs::String fetchMsg()
    {
        std_msgs::String ret = mCrtMsg.front();
        mCrtMsg.pop_front();
        return ret;
    }

    void clear()
    {
        mCrtMsg.clear();
    }

    size_t size() const
    {
        return mCrtMsg.size();
    }

public:
    std::list<std_msgs::String>  mCrtMsg;
};

//------------------------------------------------------------------------------
class StringHandllerOnce
{
public:
    void callback ( const std_msgs::String::ConstPtr & message )
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    StringHandllerOnce()
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
struct Parameter
{
    double                         loop_time;   // loop_time
    double                         pmac_time;   // pmac_time
    robot_model::RobotModelPtr     robot_model; // robot_mode
    robot_state::JointModelGroup*  joint_group; // joint model group
    std::vector<std::string>       joints;      // joints names
    std::string                    eef;         // eef name
    std::string                    base;        // base name
    std::vector<double>            max_vel;   // max vel of joints
    Eigen::VectorXd                max_joint_disp;
    std::string                     childframe_name;
    std::string                     parentframe_name;
    tf::Transform                   userFrameTF;
    tf::StampedTransform            T2B_transform;
    tf::StampedTransform            T2U_transform;
};

//------------------------------------------------------------------------------
struct State
{
    std::string                commandData;
    double                     commandTime;
    std::string                interfaceState;
    sensor_msgs::JointState    jointState; // current (real) joint state
    robot_state::RobotStatePtr robotState; // current (real) state
    std::string                commandID;  // current processing command ID
    long long                  commandIDNumber;
    std::string                ExecuteResult; // current error

    bool                       newframe;
};

//------------------------------------------------------------------------------
struct Actor
{
    sensor_msgs::JointState     ref_joint; // reference joint state for commands
    robot_state::RobotStatePtr  ref_state; // reference state for commands
    ros::Publisher              pIntferfaceJointPath;
    ros::Publisher              pCoordinateStates;
    ros::Publisher              pIntferfaceCommand;
};

//------------------------------------------------------------------------------
class CommandProcessor
{
public:
    CommandProcessor ( std::string _in x ) : regex ( x ) {  }
    CommandProcessor ( CommandProcessor _in x ) : regex ( x.regex ) {  }
    virtual ~CommandProcessor() {  }

    virtual std::string _ret Process ( std::string _in command,
                                       Parameter  _in  parameter,
                                       State      _in  state,
                                       Actor      _out actor ) = 0;

    virtual bool _ret CanProcess ( std::string _in command )
    {

        bool result = boost::regex_match ( command, regex ) ;
        //if ( result )
        //  ROS_INFO ( "%s == %s", command.c_str (), regex.expression () );
        return result;
    }

public:
    boost::regex regex;
};

//------------------------------------------------------------------------------
/*
class Empty_Processor : public CommandProcessor
{
public:
  Empty_Processor()
    : CommandProcessor ( COORDINATOR_CMD_NUMBER ( DMOV, 6 ) )
  {  }

  void Process ( std::string _in command,
                 Parameter  _in  params,
                 State      _in  state,
                 Actor      _out actor )
  {
    // 1. explain command
    std::vector<std::string> result;
    boost::split ( result, command, boost::is_any_of ( " " ) );
    std::string ID      = result[0];
    std::string COMMAND = result[1];
    std::string TARGET  = result[3];
    std::vector<double> VALUES;
    for ( size_t i = 3; i < result.size() - 2; ++i )
    {
      VALUES.push_back ( atof ( result[i].c_str() ) );
    }
    std::string UNIT = result[result.size() - 2];
    std::string FRAME = result[result.size() - 1];

    // 2. update actor.cur_ID if required
    actor.cur_ID = ID;

    // 3. update state.ref if required, else goto 4
    actor.ref_joint = state.jointState;
    actor.ref_state->setVariableValues ( actor.ref_joint );

    // 4. generate command
    std::vector<double> joint_values ( 6, 0.0 );

    // 5. update state.ref if required
    actor.ref_joint.header.stamp = ros::Time::now();
    actor.ref_joint.name = params.joints;
    actor.ref_joint.position = joint_values;
    actor.ref_state->setVariableValues ( actor.ref_joint );

    // 6. publish
    trajectory_msgs::JointTrajectory output_command;
    output_command.joint_names = params.joints;
    output_command.points.resize ( 1 );
    output_command.points[0].time_from_start = ros::Duration ( params.pmac_time );
    output_command.points[0].positions = joint_values;
    actor.publisher_command.publish ( output_command );

    // 7. set actor.cur_error to success
    actor.cur_status = "error";
  }
};
*/

//------------------------------------------------------------------------------
class STOP_Processor : public CommandProcessor
{
public:
    STOP_Processor()
        : CommandProcessor ( COORDINATOR_CMD_DIRACTE ( STOP) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command

        // 3. update state.ref if required, else goto 4
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        // 4. generate command


        // 6. update state.ref if required

        // 7. publish
        std_msgs::String output;
        output.data = INTERFACE_CMD_STOP;
        actor.pIntferfaceCommand.publish(output);
        std::cout << "send" << output.data << std::endl;

        // 8. set actor.cur_error to success
        return "succeed";
    }
};

//------------------------------------------------------------------------------
class DIRECT_Processor : public CommandProcessor
{
public:
    DIRECT_Processor()
        : CommandProcessor ( COORDINATOR_CMD_DIRACTE ( DIRECT) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command

        // 3. update state.ref if required, else goto 4
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        // 4. generate command


        // 6. update state.ref if required

        // 7. publish
        std_msgs::String output;
        output.data = INTERFACE_CMD_DIRECT;
        actor.pIntferfaceCommand.publish(output);

        // 8. set actor.cur_error to success
        return "succeed";
    }
};


//------------------------------------------------------------------------------
class DMOVJ_Processor : public CommandProcessor
{
public:
    DMOVJ_Processor()
        : CommandProcessor ( COORDINATOR_CMD_NUMBER ( DMOVJ, 6 ) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command
        std::vector<std::string> result;
        boost::split ( result, command, boost::is_any_of ( " " ), boost::algorithm::token_compress_on );
        std::string ID      = result[0];
        std::string COMMAND = result[1];
        std::string TARGET  = result[2];
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

        // 3. update state.ref if required, else goto 4
        if ( ZERO )
        {
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues ( actor.ref_joint );
            return "zero input";
        }

        // 4. generate command
        if( (VALUES.size())%6  !=0 )
            return "wrong number of points";
        std::vector<double> values ( VALUES.size() );

        for ( size_t i = 0; i < VALUES.size() ; ++i )
        {
            values[i] = VALUES[i] / 180.0 * 3.1415926 + actor.ref_joint.position[i];
        }
        std::vector<double> joint_values = values;

        // 5. check over speed
        std::vector<double> last_joint_values;
        actor.ref_state->copyJointGroupPositions ( params.joint_group, last_joint_values );
        for ( size_t i = 0; i < values.size() ; ++i )
        {
            if ( fabs (( joint_values[i] - last_joint_values[i]) / params.pmac_time ) > params.max_vel[i] )
            {
                return "over speed";
            }
            std::cout << values[i] << std::endl;
        }

        // 6. update state.ref if required
        actor.ref_joint.header.stamp = ros::Time::now();
        actor.ref_joint.name = params.joints;
        actor.ref_joint.position = joint_values;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        // 7. publish
        trajectory_msgs::JointTrajectory output_command;
        output_command.joint_names = params.joints;
        output_command.points.resize ( 1 );
        output_command.points[0].time_from_start = ros::Duration ( params.pmac_time );
        output_command.points[0].positions = joint_values;
        actor.pIntferfaceJointPath.publish ( output_command );

        // 8. set actor.cur_error to success
        return "succeed";
    }
};

class DMOV_Processor : public CommandProcessor
{
public:
    DMOV_Processor()
        : CommandProcessor ( COORDINATOR_CMD_NUMBER ( DMOV, 6 ) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command
        std::vector<std::string> result;
        boost::split ( result, command, boost::is_any_of ( " " ), boost::algorithm::token_compress_on );
        std::string ID      = result[0];
        std::string COMMAND = result[1];
        std::string TARGET  = result[2];
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

        // 3. update state.ref if required, else goto 4
        if ( ZERO )
        {
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues ( actor.ref_joint );

            // add
            std::cout << "DMOV" << std::endl;
            //

            return "zero input";
        }

        // 4. generate command
        if( (VALUES.size())%6  !=0 )
            return "wrong number of points";
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        geometry_msgs::Twist in_twist;
        in_twist.linear.x = VALUES[0] / 1000.0;
        in_twist.linear.y = VALUES[1] / 1000.0;
        in_twist.linear.z = VALUES[2] / 1000.0;
        in_twist.angular.x = VALUES[3] / 180.0 * 3.1415926;
        in_twist.angular.y = VALUES[4] / 180.0 * 3.1415926;
        in_twist.angular.z = VALUES[5] / 180.0 * 3.1415926;

        tf::Vector3 twist_rot(in_twist.angular.x,
                              in_twist.angular.y,
                              in_twist.angular.z);
        tf::Vector3 twist_vel(in_twist.linear.x,
                              in_twist.linear.y,
                              in_twist.linear.z);
        tf::Vector3 out_rot1 = params.T2B_transform.getBasis() * twist_rot;
        tf::Vector3 out_vel1 = params.T2B_transform.getBasis() * twist_vel;
        tf::Vector3 out_rot3 = params.T2U_transform.getBasis() * twist_rot;
        tf::Vector3 out_vel3 = params.T2U_transform.getBasis() * twist_vel;

//----------------------------------------------------New Add-----------------------------------------------------------------------------------------------------------
        geometry_msgs::Twist twist;
        int m_frame = 0;
        if(FRAME == "{b}")
            m_frame = 1;
        else if(FRAME == "{t}")
            m_frame = 2;
        else if(FRAME == "{u}")
            m_frame = 3;
        else
            m_frame = 0;

        switch (m_frame) {
        case 1:
            twist.linear.x = out_vel1.x();
            twist.linear.y = out_vel1.y();
            twist.linear.z = out_vel1.z();
            twist.angular.x = out_rot1.x();
            twist.angular.y = out_rot1.y();
            twist.angular.z = out_rot1.z();
            break;
        case 2:
            twist = in_twist;
            break;
        case 3:
            if(state.newframe)
            {
                twist.linear.x = out_vel3.x();
                twist.linear.y = out_vel3.y();
                twist.linear.z = out_vel3.z();
                twist.angular.x = out_rot3.x();
                twist.angular.y = out_rot3.y();
                twist.angular.z = out_rot3.z();
            }
            else {
                ROS_WARN("No User Frame Yet! Using Tool Frame!");
                twist = in_twist;
            }
            break;
        default:
            ROS_WARN("Unknown Frame! Using Tool Frame!");
            twist = in_twist;
            break;
        }
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        //Eigen::MatrixXd jcob = actor.ref_state->getJacobian ( params.joint_group );
        //std::cout << "jcob = " << std::endl << jcob << std::endl;
        //std::cout << "max_delta0 = " << std::endl << params.max_joint_disp / 3.1415926 * 180.0 << std::endl;
        //std::cout << "max_delta = " << std::endl << jcob * params.max_joint_disp << std::endl;

        robot_state::RobotStatePtr target_state ( new robot_state::RobotState ( *actor.ref_state ) );
        for (int i = 0; i< 100; ++i)
        {
            bool found_ik = target_state->setFromDiffIK ( params.joint_group, twist, params.eef, 0.010 );
            if ( !found_ik )
            {
                return "did not find IK solution";
            }
        }
        std::vector<double> joint_values;
        target_state->copyJointGroupPositions ( params.joint_group, joint_values );

        // add
        for(int i = 0 ; i < state.jointState.position.size(); ++i)
        {
            std::cout << state.jointState.position[i] << " ";
        }
        std::cout << ",";
        for(int i = 0 ; i < joint_values.size(); ++i)
        {
            std::cout << joint_values[i] << " ";
        }
        std::cout << std::endl;
        //

//        //
//        Eigen::Affine3d tr = actor.ref_state->getFrameTransform(params.eef );
//        Eigen::Affine3d tt = target_state->getFrameTransform ( params.eef );
//        ROS_INFO_STREAM (  "twist:" << twist.linear.x << " "<< twist.linear.y << " "<< twist.linear.z <<
//        " "<< twist.angular.x << " "<< twist.angular.y << " "<< twist.angular.z);
//        ROS_INFO_STREAM ( "reference pose " << tr.translation().transpose() << ", " << xc::math::irpy ( tr, 1 ).transpose() );
//        ROS_INFO_STREAM ( "target    pose " << tt.translation().transpose() << ", " << xc::math::irpy ( tt, 1 ).transpose() );
//        //

        // 5. check over speed
        std::vector<double> last_joint_values;
        actor.ref_state->copyJointGroupPositions ( params.joint_group, last_joint_values );
        for ( size_t i = 0; i < VALUES.size() ; ++i )
        {
            double v = fabs ( joint_values[i] - last_joint_values[i] ) / params.pmac_time ;
            if ( v > params.max_vel[i] )
            {
                ROS_WARN ( "over speed: v[%ld] = %f", i, v / 3.1415926 * 180.0 );
                return "over speed";
            }
        }

        // 6. update state.ref if required
        actor.ref_joint.header.stamp = ros::Time::now();
        actor.ref_joint.name = params.joints;
        actor.ref_joint.position = joint_values;
        actor.ref_state->setVariableValues ( actor.ref_joint );




        //
        //Eigen::Affine3d tr2 = actor.ref_state->getFrameTransform(params.eef );
        //ROS_INFO_STREAM ( "reference pose " << tr2.translation().transpose() << ", " << xc::math::irpy ( tr2, 1 ).transpose() );
        //

        //
        // 7. publish
        trajectory_msgs::JointTrajectory output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.joints;
        output_command.points.resize ( 1 );
        output_command.points[0].time_from_start = ros::Duration ( params.pmac_time );
        output_command.points[0].positions = joint_values;
        actor.pIntferfaceJointPath.publish ( output_command );

        // 7. set actor.cur_error to success
        return "succeed";
    }
};

class MOVJ_LSPB_Processor : public CommandProcessor
{
public:
    MOVJ_LSPB_Processor()
        : CommandProcessor ( COORDINATOR_CMD_NUMBER_TIME ( MOVJ_LSPB, 6 ) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command
        std::vector<std::string> result;
        boost::split ( result, command, boost::is_any_of ( " " ), boost::algorithm::token_compress_on );
        std::string ID      = result[0];
        std::string COMMAND = result[1];
        std::string TARGET  = result[2];
        std::vector<double> VALUES;
        for ( size_t i = 3; i < result.size() - 3; ++i )
        {
            VALUES.push_back ( atof ( result[i].c_str() ) );
        }
        std::string TIME = result[result.size() - 3];
        std::string UNIT = result[result.size() - 2];
        std::string FRAME = result[result.size() - 1];

        // 3. update state.ref
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        // 4. generate commands
        if( (VALUES.size())%6  !=0 )
            return "wrong number of points";

        std::vector<double> values = VALUES;
        values[0] = VALUES[0] / 180.0 * 3.1415926;
        values[1] = VALUES[1] / 180.0 * 3.1415926;
        values[2] = VALUES[2] / 180.0 * 3.1415926;
        values[3] = VALUES[3] / 180.0 * 3.1415926;
        values[4] = VALUES[4] / 180.0 * 3.1415926;
        values[5] = VALUES[5] / 180.0 * 3.1415926;

        double time = atof(TIME.substr(1,TIME.size()-1).c_str());
        if(time <= 0.0)
            return "time error";

        xc::math::colvecX indexes = xc::math::lspb ( 0.0, 1.0, time, params.pmac_time );
        xc::math::matrix6X joints_in;
        joints_in.resize(6,2);
        joints_in << actor.ref_joint.position[0] , values[0],
                actor.ref_joint.position[1] , values[1],
                actor.ref_joint.position[2] , values[2],
                actor.ref_joint.position[3] , values[3],
                actor.ref_joint.position[4] , values[4],
                actor.ref_joint.position[5] , values[5];
        xc::math::matrix6X joints_out = xc::math::mcspline<6> ( joints_in, indexes );

        // add
        std::cout << "MOVJ_LSPB" << std::endl;
        //

        trajectory_msgs::JointTrajectory output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.joints;
        output_command.points.resize ( joints_out.cols () );
        std::vector<double> last_joint_values = actor.ref_joint.position;
        for ( size_t i = 0; i < joints_out.cols (); ++i )
        {
            std::vector<double> joint_values;
            joint_values.resize(joints_out.rows());
            for ( size_t j = 0; j < joints_out.rows() ; ++j )
            {
                joint_values[j] = joints_out(j,i);
            }

            // 5. check over speed
            for ( size_t j = 0; j < joint_values.size() ; ++j )
            {
                double v = fabs ( joint_values[j] - last_joint_values[j] ) / params.pmac_time ;
                if ( v > params.max_vel[j] )
                {
                    ROS_WARN ( "over speed: v[%ld] = %f (%f)",
                               j,
                               v / 3.1415926 * 180.0, params.max_vel[j] / 3.1415926 * 180.0);
                    return "over speed";
                }
            }
            last_joint_values = joint_values;

            output_command.points[i].time_from_start = ros::Duration ( ( i + 1 ) * params.pmac_time );
            output_command.points[i].positions = joint_values;

            // 6. update state.ref if required
            actor.ref_joint.header.stamp = ros::Time::now();
            actor.ref_joint.name = params.joints;
            actor.ref_joint.position = joint_values;
            actor.ref_state->setVariableValues ( actor.ref_joint );

            // add
            for(int i = 0 ; i < state.jointState.position.size(); ++i)
            {
                std::cout << std::setprecision(12) << state.jointState.position[i] << " ";
            }
            std::cout << ",";
            for(int i = 0 ; i < joint_values.size(); ++i)
            {
                std::cout << joint_values[i] << " ";
            }
            std::cout << std::endl;
            //
        }

        // 5. publish
        actor.pIntferfaceJointPath.publish ( output_command );

        // 7. set actor.cur_error to success
        return "succeed";
    }
};

class MOVL_RPY_LSPB_Processor : public CommandProcessor
{
public:
    MOVL_RPY_LSPB_Processor()
        : CommandProcessor ( COORDINATOR_CMD_NUMBER_TIME ( MOVL_RPY_LSPB, 6 ) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command
        std::vector<std::string> result;
        boost::split ( result, command, boost::is_any_of ( " " ), boost::algorithm::token_compress_on );
        std::string ID      = result[0];
        std::string COMMAND = result[1];
        std::string TARGET  = result[2];
        std::vector<double> VALUES;
        for ( size_t i = 3; i < result.size() - 3; ++i )
        {
            VALUES.push_back ( atof ( result[i].c_str() ) );
        }
        std::string TIME = result[result.size() - 3];
        std::string UNIT = result[result.size() - 2];
        std::string FRAME = result[result.size() - 1];

        // 3. update state.ref
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        // 4. generate commands
        if( (VALUES.size())%6  !=0 )
            return "wrong number of points";

        std::vector<double> values = VALUES;
        values[0] = VALUES[0] / 1000.0;
        values[1] = VALUES[1] / 1000.0;
        values[2] = VALUES[2] / 1000.0;
        values[3] = VALUES[3] / 180.0 * 3.1415926;
        values[4] = VALUES[4] / 180.0 * 3.1415926;
        values[5] = VALUES[5] / 180.0 * 3.1415926;

        double time = atof(TIME.substr(1,TIME.size()-1).c_str());
        if(time <= 0.0)
            return "time error";

        xc::math::Affine3 T1 = actor.ref_state->getFrameTransform ( params.eef );
        xc::math::colvecX indexes = xc::math::lspb ( 0.0, 1.0, time, params.pmac_time );
        xc::math::Affine3 T2 = xc::math::translation ( values[0], values[1], values[2] ) * xc::math::rpy ( values[3], values[4], values[5] );
        xc::math::VectorAffine3 traj_cs = xc::math::interpolate_linear_transform ( T1, T2, indexes );

        trajectory_msgs::JointTrajectory output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.joints;
        output_command.points.resize ( traj_cs.size () ); // we do not use the first data
        robot_state::RobotStatePtr trajectory_state ( new robot_state::RobotState ( *actor.ref_state ) );
        std::vector<double> last_joint_values;
        actor.ref_state->copyJointGroupPositions ( params.joint_group, last_joint_values );
        for ( size_t i = 0; i < traj_cs.size (); ++i )
        {
            Eigen::Affine3d pose = traj_cs[i];
            std::vector<double> joint_values;
            bool found_ik = trajectory_state->setFromIK ( params.joint_group, pose, params.eef, 5 );
            if ( !found_ik )
            {
                std::cout << "cannot find IK solution for pose" << std::endl
                          << pose.matrix() << std::endl;
                return "did not find IK solution";
            }
            trajectory_state->copyJointGroupPositions ( params.joint_group, joint_values );

            // 5. check over speed
            for ( size_t j = 0; j < joint_values.size() ; ++j )
            {
                double v = fabs ( joint_values[j] - last_joint_values[j] ) / params.pmac_time ;
                if ( v > params.max_vel[j] )
                {
                    ROS_WARN ( "over speed: v[%ld] = %f (%f)",
                               j,
                               v / 3.1415926 * 180.0, params.max_vel[j] / 3.1415926 * 180.0);
                    return "over speed";
                }
            }
            last_joint_values = joint_values;

            output_command.points[i].time_from_start = ros::Duration ( ( i + 1 ) * params.pmac_time );
            output_command.points[i].positions = joint_values;

            // 6. update state.ref if required
            actor.ref_joint.header.stamp = ros::Time::now();
            actor.ref_joint.name = params.joints;
            actor.ref_joint.position = joint_values;
            actor.ref_state->setVariableValues ( actor.ref_joint );
        }

        // 5. publish
        actor.pIntferfaceJointPath.publish ( output_command );

        // 7. set actor.cur_error to success
        return "succeed";
    }
};

class MOVJ_SPLINE_LSPB_Processor : public CommandProcessor
{
public:
    MOVJ_SPLINE_LSPB_Processor()
        : CommandProcessor ( COORDINATOR_CMD_MULTI_NUMBER_TIME ( MOVJ_SPLINE_LSPB ) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command
        std::vector<std::string> result;
        boost::split ( result, command, boost::is_any_of ( " " ), boost::algorithm::token_compress_on );
        std::string ID      = result[0];
        std::string COMMAND = result[1];
        std::string TARGET  = result[2];
        std::vector<double> VALUES;
        for ( size_t i = 3; i < result.size() - 3; ++i )
        {
            VALUES.push_back ( atof ( result[i].c_str() ) );
        }
        std::string TIME = result[result.size() - 3];
        std::string UNIT = result[result.size() - 2];
        std::string FRAME = result[result.size() - 1];

        // 3. update state.ref
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        // 4. generate commands
        if( (VALUES.size())%6  !=0 )
            return "wrong number of points";

        xc::math::matrix6X joints_in;
        joints_in.resize(6,(VALUES.size() )/6);
        for(size_t i = 0; i < (VALUES.size() )/6; ++i)
        {
            joints_in(0,i) = VALUES[0+i*6] / 180.0 * 3.1415926;
            joints_in(1,i) = VALUES[1+i*6] / 180.0 * 3.1415926;
            joints_in(2,i) = VALUES[2+i*6] / 180.0 * 3.1415926;
            joints_in(3,i) = VALUES[3+i*6] / 180.0 * 3.1415926;
            joints_in(4,i) = VALUES[4+i*6] / 180.0 * 3.1415926;
            joints_in(5,i) = VALUES[5+i*6] / 180.0 * 3.1415926;
        }
        double time = atof(TIME.substr(1,TIME.size()-1).c_str());
        if(time <= 0.0)
            return "time error";

        xc::math::colvecX indexes = xc::math::lspb ( 0.0, 1.0, time, params.pmac_time );
        xc::math::matrix6X joints_out = xc::math::spline<6> ( joints_in, indexes );

        // add
        std::cout << "MOVJ_LSPB" << std::endl;
        //

        trajectory_msgs::JointTrajectory output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.joints;
        output_command.points.resize ( joints_out.cols () );
        std::vector<double> last_joint_values = actor.ref_joint.position;
        for ( size_t i = 0; i < joints_out.cols (); ++i )
        {
            std::vector<double> joint_values;
            joint_values.resize(joints_out.rows());
            for ( size_t j = 0; j < joints_out.rows() ; ++j )
            {
                joint_values[j] = joints_out(j,i);
            }

            // 5. check over speed
            for ( size_t j = 0; j < joint_values.size() ; ++j )
            {
                double v = fabs ( joint_values[j] - last_joint_values[j] ) / params.pmac_time ;
                if ( v > params.max_vel[j] )
                {
                    ROS_WARN ( "over speed: v[%ld] = %f (%f)",
                               j,
                               v / 3.1415926 * 180.0, params.max_vel[j] / 3.1415926 * 180.0);
                    return "over speed";
                }
            }
            last_joint_values = joint_values;

            output_command.points[i].time_from_start = ros::Duration ( ( i + 1 ) * params.pmac_time );
            output_command.points[i].positions = joint_values;

            // 6. update state.ref if required
            actor.ref_joint.header.stamp = ros::Time::now();
            actor.ref_joint.name = params.joints;
            actor.ref_joint.position = joint_values;
            actor.ref_state->setVariableValues ( actor.ref_joint );

            // add
            for(int i = 0 ; i < state.jointState.position.size(); ++i)
            {
                std::cout << state.jointState.position[i] << " ";
            }
            std::cout << ",";
            for(int i = 0 ; i < joint_values.size(); ++i)
            {
                std::cout << joint_values[i] << " ";
            }
            std::cout << std::endl;
            //
        }

        // 5. publish
        actor.pIntferfaceJointPath.publish ( output_command );

        // 7. set actor.cur_error to success
        return "succeed";
    }
};

class MOVJ_MCHP_LSPB_Processor : public CommandProcessor
{
public:
    MOVJ_MCHP_LSPB_Processor()
        : CommandProcessor ( COORDINATOR_CMD_MULTI_NUMBER_TIME ( MOVJ_MCHP_LSPB ) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command
        std::vector<std::string> result;
        boost::split ( result, command, boost::is_any_of ( " " ), boost::algorithm::token_compress_on );
        std::string ID      = result[0];
        std::string COMMAND = result[1];
        std::string TARGET  = result[2];
        std::vector<double> VALUES;
        for ( size_t i = 3; i < result.size() - 3; ++i )
        {
            VALUES.push_back ( atof ( result[i].c_str() ) );
        }
        std::string TIME = result[result.size() - 3];
        std::string UNIT = result[result.size() - 2];
        std::string FRAME = result[result.size() - 1];

        // 3. update state.ref
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        // 4. generate commands
        if( (VALUES.size())%6  !=0 )
            return "wrong number of points";

        xc::math::matrix6X joints_in;
        joints_in.resize(6,(VALUES.size() )/6);
        for(size_t i = 0; i < (VALUES.size() )/6; ++i)
        {
            joints_in(0,i) = VALUES[0+i*6] / 180.0 * 3.1415926;
            joints_in(1,i) = VALUES[1+i*6] / 180.0 * 3.1415926;
            joints_in(2,i) = VALUES[2+i*6] / 180.0 * 3.1415926;
            joints_in(3,i) = VALUES[3+i*6] / 180.0 * 3.1415926;
            joints_in(4,i) = VALUES[4+i*6] / 180.0 * 3.1415926;
            joints_in(5,i) = VALUES[5+i*6] / 180.0 * 3.1415926;
        }
        double time = atof(TIME.substr(1,TIME.size()-1).c_str());
        if(time <= 0.0)
            return "time error";

        xc::math::colvecX indexes = xc::math::lspb ( 0.0, 1.0, time, params.pmac_time );
        xc::math::matrix6X joints_out = xc::math::mcspline<6> ( joints_in, indexes );

        trajectory_msgs::JointTrajectory output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.joints;
        output_command.points.resize ( joints_out.cols () );
        std::vector<double> last_joint_values = actor.ref_joint.position;
        for ( size_t i = 0; i < joints_out.cols (); ++i )
        {
            std::vector<double> joint_values;
            joint_values.resize(joints_out.rows());
            for ( size_t j = 0; j < joints_out.rows() ; ++j )
            {
                joint_values[j] = joints_out(j,i);
            }

            // 5. check over speed
            for ( size_t j = 0; j < joint_values.size() ; ++j )
            {
                double v = fabs ( joint_values[j] - last_joint_values[j] ) / params.pmac_time ;
                if ( v > params.max_vel[j] )
                {
                    ROS_WARN ( "over speed: v[%ld] = %f (%f)",
                               j,
                               v / 3.1415926 * 180.0, params.max_vel[j] / 3.1415926 * 180.0);
                    return "over speed";
                }
            }
            last_joint_values = joint_values;

            output_command.points[i].time_from_start = ros::Duration ( ( i + 1 ) * params.pmac_time );
            output_command.points[i].positions = joint_values;

            // 6. update state.ref if required
            actor.ref_joint.header.stamp = ros::Time::now();
            actor.ref_joint.name = params.joints;
            actor.ref_joint.position = joint_values;
            actor.ref_state->setVariableValues ( actor.ref_joint );
        }

        // 5. publish
        actor.pIntferfaceJointPath.publish ( output_command );

        // 7. set actor.cur_error to success
        return "succeed";
    }
};

//---------------NEW ADD-------------------
class SET_Processor : public CommandProcessor
{
public:
    SET_Processor()
        : CommandProcessor ( COORDINATOR_CMD_IO ( SET) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command

        // 3. update state.ref if required, else goto 4

        // 4. generate command


        // 6. update state.ref if required

        // 7. publish

        // 8. set actor.cur_error to success
        return "succeed";
    }
};

class RESET_Processor : public CommandProcessor
{
public:
    RESET_Processor()
        : CommandProcessor ( COORDINATOR_CMD_IO ( RESET) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command

        // 3. update state.ref if required, else goto 4

        // 4. generate command


        // 6. update state.ref if required

        // 7. publish

        // 8. set actor.cur_error to success
        return "succeed";
    }
};

class WAIT_Processor : public CommandProcessor
{
public:
    WAIT_Processor()
        : CommandProcessor ( COORDINATOR_CMD_WAIT ( WAIT) )
    {  }

    std::string _ret Process ( std::string _in command,
                               Parameter  _in  params,
                               State      _in  state,
                               Actor      _out actor )
    {
        // 1. explain command

        // 3. update state.ref if required, else goto 4

        // 4. generate command


        // 6. update state.ref if required

        // 7. publish

        // 8. set actor.cur_error to success
        return "succeed";
    }
};
//-----------------------------------------

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
    StringHandllerOnce                   INTERFACE_STATE_HANDLLER;
//----------------------------------------------------------------------------
    StringHandllerOnce                   USER_FRAME_HANDLLER;
//----------------------------------------------------------------------------
    JointStateHandller                   JOINT_STATE_HANDLLER;
    robot_model_loader::RobotModelLoader ROBOT_MODEL_LOADER ( "robot_description" );
    boost::regex                         REGEX_COMMAND ( COORDINATOR_CMD );
    Parameter                            params;
    params.loop_time    = COORDINATOR_LOOP;
    params.pmac_time    = PMAC_LOOP;
    params.robot_model  = ROBOT_MODEL_LOADER.getModel();
    params.joint_group  = params.robot_model->getJointModelGroup ( "manipulator" );
    params.joints       = params.joint_group->getJointModelNames();
    params.eef          = "tool";
    params.base         = params.robot_model->getModelFrame();
    params.max_vel.resize ( 6 );
    params.max_vel[0] = 60.0 / 180 * 3.1415926 ;
    params.max_vel[1] = 60.0 / 180 * 3.1415926 ;
    params.max_vel[2] = 60.0 / 180 * 3.1415926 ;
    params.max_vel[3] = 60.0 / 180 * 3.1415926 ;
    params.max_vel[4] = 60.0 / 180 * 3.1415926 ;
    params.max_vel[5] = 60.0 / 180 * 3.1415926 ;
    if ( !NODE.getParam ( "/robot_description_planning/joint_limits/joint_1/max_velocity", params.max_vel[0] ) )
    {
        ROS_WARN ( "can not read parameter ~/joint_1/max_velocity, use default value: %f", params.max_vel[0] );
    }
    if ( !NODE.getParam ( "/robot_description_planning/joint_limits/joint_2/max_velocity", params.max_vel[1] ) )
    {
        ROS_WARN ( "can not read parameter ~/joint_2/max_velocity, use default value: %f", params.max_vel[1] );
    }
    if ( !NODE.getParam ( "/robot_description_planning/joint_limits/joint_3/max_velocity", params.max_vel[2] ) )
    {
        ROS_WARN ( "can not read parameter ~/joint_3/max_velocity, use default value: %f", params.max_vel[2] );
    }
    if ( !NODE.getParam ( "/robot_description_planning/joint_limits/joint_4/max_velocity", params.max_vel[3] ) )
    {
        ROS_WARN ( "can not read parameter ~/joint_4/max_velocity, use default value: %f", params.max_vel[3] );
    }
    if ( !NODE.getParam ( "/robot_description_planning/joint_limits/joint_5/max_velocity", params.max_vel[4] ) )
    {
        ROS_WARN ( "can not read parameter ~/joint_5/max_velocity, use default value: %f", params.max_vel[4] );
    }
    if ( !NODE.getParam ( "/robot_description_planning/joint_limits/joint_6/max_velocity", params.max_vel[5] ) )
    {
        ROS_WARN ( "can not read parameter ~/joint_6/max_velocity, use default value: %f", params.max_vel[5] );
    }
    params.max_joint_disp.resize ( 6, 1 );
    for ( size_t i = 0; i < params.max_vel.size (); ++i )
    {
        ROS_INFO ( "max velocity of joint [%ld] = %f deg", i + 1, params.max_vel[i] / 3.1415926 * 180.0 );
        params.max_joint_disp ( i ) = params.max_vel[i] * params.pmac_time;
    }
    params.childframe_name="";
    params.parentframe_name="";
    params.userFrameTF.setIdentity();
    params.T2B_transform.setIdentity();
    params.T2U_transform.setIdentity();


    Actor actor;
    actor.ref_state.reset ( new robot_state::RobotState ( params.robot_model ) );
//----------------------------------------------------New Add-----------------------------------------------------------------------------------------------------------
    ros::Subscriber subscriber_user_frame =
            NODE.subscribe<std_msgs::String> (
                COORDINATOR_USER_FRAME, 50, &StringHandllerOnce::callback,
                &USER_FRAME_HANDLLER );
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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
    ros::Subscriber subscriber_interface_states =
            NODE.subscribe<std_msgs::String> (
                INTERFACE_STATES , 1024,  &StringHandllerOnce::callback,
                &INTERFACE_STATE_HANDLLER );
    //
    actor.pIntferfaceJointPath =
            NODE.advertise<trajectory_msgs::JointTrajectory> (
                INTERFACE_JOINT_PATH_COMMAND, 1024, false );
    //
    actor.pIntferfaceCommand =
            NODE.advertise<std_msgs::String> (
                INTERFACE_COMMAND, 1, false );
    //
    actor.pCoordinateStates =
            NODE.advertise<std_msgs::String> (
                COORDINATOR_STATES, 1, true );
    //
    ros::Publisher pTest =
            NODE.advertise<std_msgs::Float64MultiArray> (
                "/my_test", 1, true );
    //    

    tf::TransformBroadcaster    tf_broadcaster;
    tf::TransformListener       tf_listener;

    STOP_Processor  processor_STOP;
    DIRECT_Processor  processor_DIRECT;
    DMOVJ_Processor processor_DMOVJ;
    DMOV_Processor  processor_DMOV;
    MOVL_RPY_LSPB_Processor processor_MOVL_RPY_LSPB;
    MOVJ_LSPB_Processor processor_MOVJ_LSPB;
    MOVJ_SPLINE_LSPB_Processor processor_MOVJ_SPLINE_LSPB;
    MOVJ_MCHP_LSPB_Processor processor_MCHP_LSPB;
    //---------------NEW ADD-------------------
    SET_Processor processor_SET;
    RESET_Processor processor_RESET;
    WAIT_Processor processor_WAIT;
    //-----------------------------------------

    //
    State state;
    state.commandID = "<undefined_id>";
    state.ExecuteResult = "";
    state.commandData = "";
    state.commandTime = 0.0;
    state.interfaceState = "not_available";
    state.robotState.reset ( new robot_state::RobotState ( params.robot_model ) );
    while ( ros::ok() )
    {
        ros::spinOnce();
        if ( JOINT_STATE_HANDLLER.newMsg() )
        {
            state.jointState = JOINT_STATE_HANDLLER.fetchMsg();
            state.robotState->setVariableValues ( state.jointState );
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues ( actor.ref_joint );
            break;
        }
        ros::Duration ( 1.0 ).sleep ();
        ROS_INFO ( "waiting for joint state update" );
    }
    state.newframe = false;
    //
    ROS_INFO ( "main loop started" );
    bool auto_ref = true;

    /*
    robot_state::RobotStatePtr trajectory_state ( new robot_state::RobotState ( *actor.ref_state ) );
    Eigen::Affine3d pose;
    pose.matrix ()<<  0.946858, -0.0162789,  -0.321239,  -0.358339,
                      0.0390909,   0.997139,  0.0646909,    1.05381,
                       0.319267, -0.0738107,   0.944786,    1.43538,
                              0,          0,          0,          1;

    std::cout << pose.matrix () << std::endl;
    bool found_ik = trajectory_state->setFromIK ( params.joint_group, pose);
    if(found_ik)
        std::cout << "found" << std::endl;
    else
        std::cout << "not found" << std::endl;
    std::vector<double> joint_values;
    trajectory_state->copyJointGroupPositions ( params.joint_group, joint_values );
    for(size_t i = 0; i < joint_values.size(); ++i)
    {
        std::cout << joint_values[i] / 3.1415926 * 180.0 << " ";
    }
    std::cout << std::endl;
    return 0;
    */

    while ( ros::ok() )
    {
        try
        {
            bool newcommand = false;
            double start = ros::Time::now().toSec();

            // spin
            ros::spinOnce();

            // get feedback
            if ( JOINT_STATE_HANDLLER.newMsg() )
            {
                state.jointState = JOINT_STATE_HANDLLER.fetchMsg();
                state.robotState->setVariableValues ( state.jointState );
            }

            // check if interface is ready
            bool interface_stable = false;
            bool interface_direct = false;
            if ( INTERFACE_STATE_HANDLLER.newMsg() )
            {
                std_msgs::String input = INTERFACE_STATE_HANDLLER.fetchMsg();
                state.interfaceState = input.data;
                std::vector<std::string> result;
                boost::split ( result, state.interfaceState, boost::is_any_of ( "\t" ) );
                if ( result.size () < 2 )
                    interface_stable = false;
                if ( result[1] == INTERFACE_STATE_STABLE )
                    interface_stable = true;
                if ( result[0] == INTERFACE_STATE_DIRECT )
                    interface_direct = true;
            }

//----------------------------------------------------New Add-----------------------------------------------------------------------------------------------------------
            // get user frame
            if ( USER_FRAME_HANDLLER.newMsg() )
            {
                //ROS_INFO("1");
                // 1. explain info
                std_msgs::String user_frame_info = USER_FRAME_HANDLLER.fetchMsg();
                std::vector<std::string> result;
                boost::split ( result, user_frame_info.data, boost::is_any_of ( " " ), boost::algorithm::token_compress_on );
                params.childframe_name      = result[0];
                params.parentframe_name     = result[1];
                std::vector<double> VALUES;
                for ( size_t i = 2; i < 8; ++i )
                {
                    VALUES.push_back ( atof ( result[i].c_str() ) );
                }
                //ROS_INFO("2");
                // 2. generate frame
                params.userFrameTF.setOrigin( tf::Vector3( VALUES[0], VALUES[1], VALUES[2] ) );
                tf::Quaternion q;
                q = tf::createQuaternionFromRPY( VALUES[3], VALUES[4], VALUES[5] );
                q = q.normalize();
                params.userFrameTF.setRotation(q);
                state.newframe = true;
            }

            // 3. broadcast user frame
            if(state.newframe)
            {
                //ROS_INFO("3");
                tf_broadcaster.sendTransform(tf::StampedTransform(params.userFrameTF, ros::Time::now(), "/root", "user"));
            }

            //ROS_INFO("4");
            tf::StampedTransform T2B_transform, T2U_transform;
            // 4. Lookup transform
            try {
                tf_listener.lookupTransform("/tool", "/root", ros::Time(0), T2B_transform);
                params.T2B_transform = T2B_transform;
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
//                ros::Duration(1.0).sleep();
            }
            if(state.newframe)
            {
                try {
                    tf_listener.lookupTransform("/tool", "user", ros::Time(0), T2U_transform);
                    params.T2U_transform = T2U_transform;
                }
                catch (tf::TransformException ex) {
                    ROS_ERROR("%s",ex.what());
    //                ros::Duration(1.0).sleep();
                }
            }

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

            // get command
            if ( COMMAND_HANDLLER.newMsg() )
            {
                newcommand = true;
                std_msgs::String input = COMMAND_HANDLLER.fetchMsg();
                std::string inputdata = input.data;
                boost::algorithm::replace_all(inputdata,"\n"," ");
                boost::algorithm::replace_all(inputdata,";"," ");
                state.commandData = inputdata;
                state.commandTime = ros::Time::now ().toSec ();
                ROS_DEBUG ( "command: %s", state.commandData.c_str() );
                if ( boost::regex_match ( state.commandData, REGEX_COMMAND ) )
                {
                    //
                    std::vector<std::string> result;
                    boost::split ( result, state.commandData, boost::is_any_of ( " " ) );
                    state.commandID = result[0];
                    state.commandID = state.commandID.substr(1,state.commandID.size()-2);
                    long long cmdIDNUmber = atoll(state.commandID.c_str());
                    if( cmdIDNUmber - state.commandIDNumber != 1)
                    {
                        ROS_WARN("missing command");
                    }
                    state.commandIDNumber = cmdIDNUmber;
                    //
                    bool processed = false;
                    // update reference
                    if( auto_ref )
                    {
                        actor.ref_joint = state.jointState;
                        actor.ref_state->setVariableValues ( actor.ref_joint );
                    }
                    auto_ref = true;


                    if ( processor_STOP.CanProcess ( state.commandData ) )
                    {
                        processed = true;
                        state.ExecuteResult = processor_STOP.Process ( state.commandData, params, state, actor );
                    }
                    else if( processor_DIRECT.CanProcess ( state.commandData ) )
                    {
                        processed = true;
                        state.ExecuteResult = processor_DIRECT.Process ( state.commandData, params, state, actor );
                    }

                    if ( interface_direct )
                    {
                        if ( processor_DMOVJ.CanProcess ( state.commandData ) )
                        {
                            processed = true;
                            state.ExecuteResult = processor_DMOVJ.Process ( state.commandData, params, state, actor );
                            auto_ref = false;
                        }
                        else if ( processor_DMOV.CanProcess ( state.commandData ) )
                        {
                            processed = true;
                            state.ExecuteResult = processor_DMOV.Process ( state.commandData, params, state, actor );
                            auto_ref = false;
                        }
                    }
                    //
                    if ( interface_direct && interface_stable )
                    {
                        if ( processor_MOVL_RPY_LSPB.CanProcess ( state.commandData ) )
                        {
                            processed = true;
                            state.ExecuteResult = processor_MOVL_RPY_LSPB.Process ( state.commandData, params, state, actor );
                        }
                        else if( processor_MOVJ_LSPB.CanProcess ( state.commandData ) )
                        {
                            processed = true;
                            state.ExecuteResult = processor_MOVJ_LSPB.Process ( state.commandData, params, state, actor );
                        }
                        else if( processor_MOVJ_SPLINE_LSPB.CanProcess ( state.commandData ) )
                        {
                            processed = true;
                            state.ExecuteResult = processor_MOVJ_SPLINE_LSPB.Process ( state.commandData, params, state, actor );
                        }

                        //---------------NEW ADD-------------------
                        else if( processor_SET.CanProcess( state.commandData))
                        {
                            processed = true;
                        }
                        else if( processor_RESET.CanProcess( state.commandData))
                        {
                            processed = true;
                        }
                        else if( processor_WAIT.CanProcess( state.commandData))
                        {
                            processed = true;
                        }
                        //-----------------------------------------

                        if( processor_MCHP_LSPB.CanProcess ( state.commandData ) )
                        {
                            processed = true;
                            state.ExecuteResult = processor_MCHP_LSPB.Process ( state.commandData, params, state, actor );
                        }
                    }
                    // update reference
                    if( auto_ref )
                    {
                        actor.ref_joint = state.jointState;
                        actor.ref_state->setVariableValues ( actor.ref_joint );
                    }
                    //
                    if ( !processed )
                        ROS_WARN ( "unsupported command: %s", state.commandData.c_str() );
                }
                else
                {
                    ROS_WARN ( "unrecognized command: %s", state.commandData.c_str() );
                }
            }

            // state output
            std_msgs::String stateoutput;
            std::string stateout;
            if (newcommand)
            {
                xc::StringAppend ( stateout, "new_command " );
            }
            else
            {
                if ( interface_direct )
                {
                    if ( interface_stable )
                    {
                        xc::StringAppend ( stateout, "stable " );
                    }
                    else
                    {
                        xc::StringAppend ( stateout, "moving " );
                    }
                }
                else
                {
                    xc::StringAppend ( stateout, "wrong_model " );
                }
            }

            Eigen::MatrixXd jcob = state.robotState->getJacobian ( params.joint_group );
            Eigen::MatrixXd manipulability = jcob * jcob.transpose();
            xc::StringAppend ( stateout, "%s ", state.commandID.c_str() );
            xc::StringAppend ( stateout, "%s ", state.ExecuteResult.c_str() );
            xc::StringAppend ( stateout, "%f ", manipulability.norm() );
            xc::StringAppend ( stateout, "%d ", COMMAND_HANDLLER.size() );
            ROS_DEBUG ( "%s", stateout.c_str () );

            stateoutput.data = stateout;
            actor.pCoordinateStates.publish ( stateoutput );

            std_msgs::Float64MultiArray tst;

            // current pos
            Eigen::Affine3d t = state.robotState->getFrameTransform ( params.eef );
            xc::math::colvec3 rpy = xc::math::irpy ( t, 1 );
            xc::math::colvec3 trans = t.translation();
            ROS_DEBUG_STREAM ( "current pose " << trans[0]*1000 << " "
                               << trans[1]*1000 << " "
                               << trans[2]*1000 << " , "
                               << rpy[0]*180/3.1415926 << " "
                               << rpy[1]*180/3.1415926 << " "
                               << rpy[2]*180/3.1415926 << std::endl);
            /*
            tst.data.resize(6);
            tst.data[0]= trans[0];
            tst.data[1]= trans[1];
            tst.data[2]= trans[2];
            tst.data[3]= rpy[0]*180.0/3.1415926;
            tst.data[4]= rpy[1]*180.0/3.1415926;
            tst.data[5]= rpy[2]*180.0/3.1415926;
            */

            // ref pos
            /*
            //ROS_INFO_STREAM ( "current pose " << t.translation().transpose() * 1000.0 << ", " << xc::math::irpy ( t ).transpose() * 180.0 / 3.1415926 );
            //ROS_INFO_STREAM ( "referent pose " << tr.translation().transpose() << ", " << xc::math::irpy ( tr, 1 ).transpose() );
            //ROS_INFO_STREAM ( "current pose " << t.translation().transpose() << ", " << xc::math::irpy ( t, 2 ).transpose() );
            Eigen::Affine3d tr = actor.ref_state->getFrameTransform(params.eef );
            xc::math::colvec3 rpyr = xc::math::irpy ( tr, 1 );
            xc::math::colvec3 transr = tr.translation();
            tst.data[6]= transr[0];
            tst.data[7]= transr[1];
            tst.data[8]= transr[2];
            tst.data[9]=  rpyr[0]*180.0/3.1415926;
            tst.data[10]= rpyr[1]*180.0/3.1415926;
            tst.data[11]= rpyr[2]*180.0/3.1415926;
            */

            // joint position
            tst.data.resize(6);
            tst.data[0]= actor.ref_joint.position[0];
            tst.data[1]= actor.ref_joint.position[1];
            tst.data[2]= actor.ref_joint.position[2];
            tst.data[3]= actor.ref_joint.position[3];
            tst.data[4]= actor.ref_joint.position[4];
            tst.data[5]= actor.ref_joint.position[5];
            pTest.publish(tst);

            // time control
            double  sleep_time = params.loop_time - ( ros::Time::now().toSec() - start );
            if ( sleep_time > 0 )
            {
                ros::Duration ( sleep_time ).sleep();
            }
            else
            {
                ROS_WARN("loop time out.");
            }
        }
        catch ( xc::Result err )
        {
            ROS_ERROR ( "recognized error occured: %s", err.message() );
        }
    }
    ros::shutdown();
    return 0;
}
