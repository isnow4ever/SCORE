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
#include <deque>
#include <vector>

#include <ethercat_manager/ethercat_manager.h>
#include <minas_control/minas_client.h>
#include <minas_control/minas_hardware_interface.h>
#include <soem/osal.h>

#include "constants.h"
//#include "controller.h"
#include <xc/core/core.h>
#include <xcdev/PMAC/protocol.h>


/*
    int rtos[4] = {0,0,0,0};
    int so[4] = {0};
    int oe[4] = {0};
    int f[4] = {0};
    int ve[4] = {0};
    int qs[4] = {0};
    int sod[4] = {0};
    int w[4] = {0};
    int r[4] = {0};
    int rm[4] = {0};
    int ila[4] = {0};
    int drive_follow_com[4] = {0};
    int follow_error[4] = {0};

    int NOT[4] = {0};
    int POT[4] = {0};
    int HM_SWITCH[4] = {0};
    int SI_MON1[4] = {0};
    int SI_MON2[4] = {0};
    int SI_MON3[4] = {0};
    int SI_MON4[4] = {0};
    int SI_MON5[4] = {0};
    int INP[4] = {0};
*/
int AxisOne = 1;
int AxisTwo = 2;
int AxisThree = 3;
int AxisFour = 4;

static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

int INPUT_FINISH =  0;
double MinimalInputTime  = 0.01;     // s 10ms

const int MODE_INIT             = 0;
const int MODE_DIRECT           = 1;
const int MODE_SPLINE           = 2;
const int MODE_STOP             = 3;



double torqueForEmergencyStopVec[4] = {100, 100, 100, 100};
double overloadLevelVec[4] = {50, 50, 50, 50};
double overSpeedLevelVec[4] = {00, 00 ,00, 00};
double motorWorkingRangeVec[4] = {0.1, 0.1, 0.1, 0.1};
int interpolationTimePeriodVec[4] = {2000, 2000, 2000, 2000};

uint16 target_torqueVec[4] = {500, 500, 500, 500};
uint32 max_motor_speedVec[4] = {120, 120, 120, 120};
uint16 max_torqueVec[4] = {500, 500, 500, 500};
uint16 controlwordVec[4] = {0x001f, 0x001f, 0x001f, 0x001f};
//uint8  operation_mode;
uint32_t profileVelVec[4] = {0x20000000, 0x20000000, 0x20000000, 0x20000000};


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


typedef struct controlPara
{
    double positon;
    double velocity;
    double effort;
};

typedef struct scaraControlPara
{
    struct controlPara paraAxisOne;
    struct controlPara paraAxisTwo;
    struct controlPara paraAxisThree;
    struct controlPara paraAxisFour;
};



/*****************************************************
* function: motorParaInitOne
* functioning: setting the four scara motors parameters
* return: none
*****************************************************/
void motorParaInitOne(struct motorType *motorParaAxisOne)
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
}



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



class scaraEthercatManagerModify
{
 public:
    ~scaraEthercatManagerModify()
    {
        ROS_INFO_STREAM_NAMED("minas", "~EtherCATJointControlInterface()");
        shutdown();
        delete(client);
    }
    scaraEthercatManagerModify(ethercat::EtherCatManager* manager, int slave_no, struct motorType *motorParaAxis)
    {
        ROS_INFO("Initialize EtherCATJoint (%d)", slave_no);
        int operation_mode = 0x08; // (csp) cyclic synchronous position mode
        client = new minas_control::MinasClient(*manager, slave_no);
        ROS_INFO("Initialize EtherCATJoint (reset)");
        client->reset();
        client->setTrqueForEmergencyStop(100);  //100%
        client->setOverLoadLevel(50 );          // 50%
        client->setOverSpeedLevel(120 );        // r/min
        client->setMotorWorkingRange(0.1);     // 0.1
        client->setInterpolationTimePeriod(4000) ;     // 4 msec
        // servo on
        client->servoOn();

            // get current positoin

        input = client->readInputs();
        int32 current_position = input.position_actual_value;
        memset(&output, 0x00, sizeof(minas_control::MinasOutput));
            // set target position

        output.target_position = 0;
        output.position_offset = current_position;

        //output.target_position = current_position;

        output.max_motor_speed = 120;  // rad/min

        output.target_torque = 500;    // 0% (unit 0.1%)

        output.max_torque    = 500;    // 50% (unit 0.1%)
        output.controlword   = 0x001f; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5) p77
        output.operation_mode = 0x08;
        client->setProfileVelocity(0x20000000 );
        client->writeOutputs(output);
        while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)  p79
            input = client->readInputs();
        }

        output.controlword   &= ~0x0010; // clear new-set-point (bit4)
        client->writeOutputs(output);


        //usleep(50000);
        //client->servoOff();

    }

  void shutdown()
  {
    //ROS_INFO_STREAM_NAMED("minas", joint.name_ + " shutdown()");
    client->printPDSStatus(input);
    client->printPDSOperation(input);
    client->reset();
    client->servoOff();
  }
 private:
  minas_control::MinasClient* client;
  minas_control::MinasInput input;
  minas_control::MinasOutput output;

};


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
            printf("waiting for servo on....\n");
            // servo on
            //client_.servoOn();
            //usleep(100);
            /*
            // set profile velocity
            client_.setProfileVelocity(motorParaAxis->profileVel );

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
            */

            // pp control model setup (see statusword(6041.h) 3) p.79)
            //client_.writeOutputs(output_);
        }

        double jointPos()
        {
            minas_control::MinasInput inputData;
            inputData = client_.readInputs();   //int 32
            int counts = (int)inputData.position_actual_value;
            double jointPosition = 2 * 3.1415926 * double(counts) / double(0x100000);
            return jointPosition;

        }

        void closeServo(minas_control::MinasClient& client)
        {
            client.servoOff();
        }

         /*****************************************************
        * Function: sendCmdToMinas
        * functioning: sending position to Minas driver
        * return: none
        *****************************************************/
        void sendCmdToMinas(const double &offset)
        {
            double temp = 0x100000 * offset/(2.0 *3.1415926);
            int32 offsetSend = (int32) temp;
            output_.position_offset = offsetSend;
            client_.writeOutputs(output_);
        }
    private:
        minas_control::MinasClient& client_;
        minas_control::MinasInput& input_;
        minas_control::MinasOutput& output_;
};



class trajectoryReceiving
{
    public:

        ~trajectoryReceiving()
        {
        }

        trajectoryReceiving()
            : trajectoryData()
            , newDataFlag ( false )
        {

        }

        void callback ( const trajectory_msgs::JointTrajectory::ConstPtr & message)
        {
            trajectoryData = *message;
            newDataFlag = true;
            size_t numAxis;
            numAxis = trajectoryData.joint_names.size();
            if(numAxis != 6)
            {
                printf("expect six Axises\n");
            }
            size_t numTrajPoints;
            numTrajPoints = trajectoryData.points.size();
            scaraRobotPara.paraAxisOne.positon = trajectoryData.points[0].positions[0];
            scaraRobotPara.paraAxisTwo.positon = trajectoryData.points[0].positions[1];
            scaraRobotPara.paraAxisThree.positon = trajectoryData.points[0].positions[2];
            scaraRobotPara.paraAxisFour.positon = trajectoryData.points[0].positions[3];

        }
        const bool newflag()
        {
            return newDataFlag;
        }
        void trajectoryClear()
        {
            newDataFlag = false;
        }

        const trajectory_msgs::JointTrajectory & fetchTraj()
        {
            newDataFlag = false;
            return trajectoryData;
        }
    public:
        trajectory_msgs::JointTrajectory trajectoryData;
        bool newDataFlag;
        struct scaraControlPara scaraRobotPara;
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
