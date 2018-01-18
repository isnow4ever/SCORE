/*************************************************************************
	> File Name: ethercat_interface.h
	> Author: Likun Wang
	> Mail: likunwang@hit.edu.cn
	> Created Time: 2018年01月10日 星期三 23时30分17秒
 ************************************************************************/

#ifndef _ETHERCAT_INTERFACE_H
#define _ETHERCAT_INTERFACE_H

#include <stdio.h>
#include <iostream>
#include <getopt.h>
#include <time.h>
#include <cmath>
#include <Eigen/Eigen>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/String.h>
#include  <deque>
#include <vector>

#include <ethercat_manager/ethercat_manager.h>
#include <minas_control/minas_client.h>
#include <minas_control/minas_hardware_interface.h>
#include <soem/osal.h>

#include "constants.h"
//#include "controller.h"
#include <xc/core/core.h>
#include <xcdev/PMAC/protocol.h>


int AxisOne = 1;
int AxisTwo = 2;
int AxisThree = 3;
int AxisFour = 4;

static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

int INPUT_FINISH =  0;
double MinimalInputTime  = 0.01;
/*****************************************************
* function: timespecInc
* functioning: Inpsect system time (fork from package Minas control )
* return: none
*****************************************************/
void timespecInc(struct timespec &tick, int nsec)
{
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
    }
}


/*****************************************************
* Struct: motorType
* functioning:
* return:
*****************************************************/
typedef struct motorType
{
    double torqueForEmergencyStop;
    double overloadLevel;
    double overSpeedLevel;
    double motorWorkingRange;
    int interpolationTimePeriod;

    uint16 target_torque;
    uint32 max_motor_speed;
    uint16 max_torque;
    uint16 controlword;
    uint8  operation_mode;
    uint32_t profileVel;
}motorPara;

/*****************************************************
* function: motorParaInit
* functioning: setting the four scara motors parameters
* return: none
*****************************************************/
void motorParaInit(struct motorType *motorParaAxisOne,
                                struct motorType *motorParaAxisTwo,
                                struct motorType *motorParaAxisThree,
                                struct motorType *motorParaAxisFour)
{
    /* Axis One Unit: deg*/
    motorParaAxisOne->torqueForEmergencyStop            = 100;
    motorParaAxisOne->overloadLevel                              = 50;
    motorParaAxisOne->overSpeedLevel                           = 120;
    motorParaAxisOne->motorWorkingRange                   = 0.1;
    motorParaAxisOne->interpolationTimePeriod             = 4000;  //4msec
    motorParaAxisOne->max_motor_speed                      = 120;
    motorParaAxisOne->target_torque                             = 500;
    motorParaAxisOne->max_torque                               = 500;
    motorParaAxisOne->controlword                               = 0x001f;
    motorParaAxisOne->operation_mode                         = 0x08;
    motorParaAxisOne->profileVel                                    = 0x20000000;

    /* Axis Two Unit: deg*/
    motorParaAxisTwo->torqueForEmergencyStop            = 100;
    motorParaAxisTwo->overloadLevel                              = 50;
    motorParaAxisTwo->overSpeedLevel                           = 120;
    motorParaAxisTwo->motorWorkingRange                   = 0.1;
    motorParaAxisTwo->interpolationTimePeriod             = 4000;  //4msec
    motorParaAxisTwo->max_motor_speed                      = 120;
    motorParaAxisTwo->target_torque                             = 500;
    motorParaAxisTwo->controlword                               = 0x001f;
    motorParaAxisTwo->operation_mode                         = 0x08;
    motorParaAxisTwo->profileVel                                    = 0x20000000;
    motorParaAxisTwo->max_torque                               = 500;

    /* Axis Three Unit: mm*/
    motorParaAxisThree->torqueForEmergencyStop            = 100;
    motorParaAxisThree->overloadLevel                              = 50;
    motorParaAxisThree->overSpeedLevel                           = 120;
    motorParaAxisThree->motorWorkingRange                   = 0.1;
    motorParaAxisThree->interpolationTimePeriod             = 4000;  //4msec
    motorParaAxisThree->max_motor_speed                      = 120;
    motorParaAxisThree->target_torque                             = 500;
    motorParaAxisThree->controlword                               = 0x001f;
    motorParaAxisThree->operation_mode                         = 0x08;
    motorParaAxisThree->profileVel                                    = 0x20000000;
    motorParaAxisThree->max_torque                               = 500;

    /* Axis Four Unit: deg*/
    motorParaAxisFour->torqueForEmergencyStop            = 100;
    motorParaAxisFour->overloadLevel                              = 50;
    motorParaAxisFour->overSpeedLevel                           = 120;
    motorParaAxisFour->motorWorkingRange                   = 0.1;
    motorParaAxisFour->interpolationTimePeriod             = 4000;  //4msec
    motorParaAxisFour->max_motor_speed                      = 120;
    motorParaAxisFour->target_torque                             = 500;
    motorParaAxisFour->controlword                               = 0x001f;
    motorParaAxisFour->operation_mode                         = 0x08;
    motorParaAxisFour->profileVel                                    = 0x20000000;
    motorParaAxisFour->max_torque                               = 500;
}


/*****************************************************
* class: scaraEthtercatManager
* functioning: initialize four minas drivers for Ethercat communication
* return:
*****************************************************/
class scaraEthercatManager
{
    public:
        ~scaraEthercatManager()
        {

        }
        scaraEthercatManager(minas_control::MinasClient& client,
                                            minas_control::MinasInput& input,
                                            minas_control::MinasOutput& output, struct motorType *motorParaAxis)
        :client_(client),
        input_(input),
        output_(output)
        {
            ROS_INFO("Ethercat Initializing");
            client_.reset();
            client_.setTrqueForEmergencyStop(motorParaAxis->torqueForEmergencyStop);  //100%
            client_.setOverLoadLevel(motorParaAxis->overloadLevel );          // 50%
            client_.setOverSpeedLevel(motorParaAxis->overSpeedLevel );        // r/min
            client_.setMotorWorkingRange(motorParaAxis->motorWorkingRange);     // 0.1
            client_.setInterpolationTimePeriod(motorParaAxis->interpolationTimePeriod) ;     // 4 msec
            // set profile velocity
            client_.setProfileVelocity(motorParaAxis->profileVel );
            // servo on
            client_.servoOn();
            printf("Motor powering on \n");
            // get current positoin
            input_ = client_.readInputs();
            int32 current_position = input_.position_actual_value;

            // set target position
            printf("Initialing position \n");
            output_.target_position = current_position;
            output_.max_motor_speed = motorParaAxis->max_motor_speed;  // rad/min
            output_.target_torque = motorParaAxis->target_torque;    // 0% (unit 0.1%)
            output_.max_torque    = motorParaAxis->max_torque;    // 50% (unit 0.1%)
            output_.controlword   = motorParaAxis->controlword; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5) p77
            output_.operation_mode = motorParaAxis->operation_mode;


            // pp control model setup (see statusword(6041.h) 3) p.79)
            client_.writeOutputs(output_);
        }

         /*****************************************************
        * Function: sendCmdToMinas
        * functioning: sending position to Minas driver
        * return: none
        *****************************************************/
        void sendCmdToMinas(const int32 &position)
        {
            this->output_.target_position = position;
            this->client_.writeOutputs(output_);
        }
    private:
        minas_control::MinasClient& client_;
        minas_control::MinasInput& input_;
        minas_control::MinasOutput& output_;
};

/*****************************************************
* class: JointTrajectoryHandller
* functioning: handle string type, forked from Yanyu Su
* return:
*****************************************************/
class JointTrajectoryHandller
{
public:
  void callback ( const trajectory_msgs::JointTrajectory::ConstPtr & message)
  {
    mCrtMsg = *message;
    mNewMsg = true;
  }

  ~JointTrajectoryHandller()
  {

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

/*****************************************************
* class: StringHandller
* functioning: handle string type, forked from Yanyu Su
* return:
*****************************************************/
class StringHandller
{
public:
    /*****************************************************
    * Function: callback
    * functioning: if new message arrives, the mNewMsg will be set to be true
    * return: none
    *****************************************************/
  void callback ( const std_msgs::String::ConstPtr & message )
  {
    mCrtMsg = *message;
    mNewMsg = true;
  }

  ~StringHandller()
  {
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
        std::vector <std::deque<double> >      recData;          // received data
        std::vector <std::string>               nameAxis;          // number of axis
        double                                  numTimeStep;      // interplot duration

        PTTrajectory ( std::vector< std::string> _in minasNames, double _in timestep )
            :   nameAxis ( minasNames ),
                recData ( minasNames.size() ),
                numTimeStep ( timestep)
        {
        }

        PTTrajectory ( std::vector< std::string> _in minasNames, double _in steps, double _in timestep )
            : nameAxis ( minasNames ),
            recData ( minasNames.size() ),
            numTimeStep ( timestep )
        {
        }

        PTTrajectory ( PTTrajectory _in var )
        :  nameAxis ( var.nameAxis ),
            recData ( var.recData ),
            numTimeStep ( var.numTimeStep )
        {

        }
        virtual ~PTTrajectory()
        {
        }
        /*****************************************************
        * Function: resize
        * resize the data
        * return resize data
        *****************************************************/
        inline void resize(size_t _in steps)
        {
            for(size_t i = 0; i < recData.size(); ++i)
            {
                recData[i].resize(steps);
            }
        }
        /*****************************************************
        * Function: setData
        * Set data from the income variables
        * return recData
        *****************************************************/
        inline void setData (size_t _in axis, size_t _in steps, double _in var)
        {
            recData[axis][steps] = var;
        }
        /*****************************************************
        * Function: setDataSize
        * Set data size  [axis] [steps]
        * return recData
        *****************************************************/
        inline double _ret setDataSize(size_t _in axis, size_t _in steps) const
        {
            return recData[axis][steps];
        }
        /*****************************************************
        * Function: numOfAxis
        * return num of Axis
        * return size_t
        *****************************************************/
        inline size_t numOfAxis()
        {
            return nameAxis.size();
        }
        /*****************************************************
        * Function: setStep
        * Set the No. step to the received data from varaibles
        * return void
        *****************************************************/
        inline void setStep (size_t _in axis, size_t _in step, std::vector <double>_in var)
        {
            for (size_t i = 0; i< numOfAxis(); ++i)
            {
                recData[i][step] = var[i];
            }
        }
        /*****************************************************
        * Function: pushback
        * Add new element to the vecor container
        * return void
        *****************************************************/
        inline void pushback (std::vector <double> _in var)
        {
            for(size_t i =0; i < numOfAxis(); ++i )
            {
                recData[i].push_back(var[i]);
            }
        }
        /*****************************************************
        * Function: popFront
        * Pop the first element in the vector
        * return popped element
        *****************************************************/
        inline std::vector<double> _ret popFront()
        {
            std::vector <double> ret (numOfAxis());
            for (size_t i = 0; i<numOfAxis(); ++i)
            {
                ret[i] = recData[i].front();
                recData[i].pop_front();
            }
            return ret;
        }
        /*****************************************************
        * Function: popFrontString
        * Pop the first element in the vector
        * return received data
        *****************************************************/
        inline std::string _ret popFrontString (double _in ratio = 180.0/3.1415926)
        {
            std::string ret;
            ret.reserve(128);   //allocate memory
            for (size_t i =0; i< numOfAxis(); ++i)
            {
                xc::StringAppend ( ret, "%s%lf", nameAxis[i].c_str(), recData[i].front() * ratio );
                recData[i].pop_front();
            }
            return ret;
        }
        /*****************************************************
        * Function: appendInData
        * Add new data to the tail of the buffer
        * return void
        *****************************************************/
        inline void appendInData ( PTTrajectory _in var )
        {
            for ( size_t i = 0; i < numOfAxis(); ++i )
            {
                recData[i].insert ( recData[i].end(),
                        var.recData[i].begin(),
                        var.recData[i].end() );
            }
        }


};






#endif
