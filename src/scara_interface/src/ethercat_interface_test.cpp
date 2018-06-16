/*************************************************************************
	> File Name: ethercat_interface_test.cpp
	> Author: Likun Wang
	> Mail: likunwang@hit.edu.cn
	> Created Time: 2018年01月10日 星期三 23时30分17秒
 ************************************************************************/
//#include <sstream>
#include "ethercat_interface.h"

int main(int argc, char *argv[])
{
    // initializing ROS
    printf("initializing ROS\n");
    ros::init(argc, argv, "ethercat_interface");
    ros::NodeHandle nh;
    printf("initializing ROS succeed\n");
    ROS_INFO ( "initializing subscriber and publisher" );
    JointTrajectoryHandller TRAJECTORY_HANDLLER;
    StringHandller          COMMAND_HANDLLER;
    StringHandller          SERVO_COMMAND_HANDLLER;
    StringHandller          ALARM_COMMAND_HANDLLER;
    ros::Subscriber subscriber_joint_path;    //只订阅了发的数据
    ros::Subscriber subscriber_command;
    ros::Subscriber subscriber_servo_command;
    ros::Subscriber subscriber_alarm_command;

    ros::Publisher  pJointStates;
    ros::Publisher  driveStates;

    std_msgs::String statusWord;
    sensor_msgs::JointState jointState;
    ROS_INFO ( "initializing subscriber and publisher succeed" );

    ROS_INFO ( "Main loop on the go" );
    int inputFlag = 1;
    int stopFlag  = 0;
    int clearAllData = 1;
    int servoStatus = 0;

    subscriber_joint_path = nh.subscribe<trajectory_msgs::JointTrajectory> (INTERFACE_JOINT_PATH_COMMAND, 1024, &JointTrajectoryHandller::callback, &TRAJECTORY_HANDLLER );
    subscriber_command = nh.subscribe<std_msgs::String> (INTERFACE_COMMAND, 1024, &StringHandller::callback, &COMMAND_HANDLLER );
    subscriber_servo_command = nh.subscribe<std_msgs::String>(INTERFACE_SERVO_COMMAND, 1024, &StringHandller::callback, &SERVO_COMMAND_HANDLLER);   //   LG
    subscriber_alarm_command = nh.subscribe<std_msgs::String>(INTERFACE_ALARM_CLEAR, 1024, &StringHandller::callback, &ALARM_COMMAND_HANDLLER);        //   LG

    pJointStates = nh.advertise<sensor_msgs::JointState> (INTERFACE_JOINT_STATES, 1024 );
    driveStates = nh.advertise<std_msgs::String> (INTERFACE_DRIVE_STATE, 1024 );

    int operation_mode = 0x08;
    std::string ifname;
	ifname = "eth0";
	std::cout << ifname << std::endl;
    double period = 4e+6;
    struct timespec tick;

    // start slaveinfo
    ethercat::EtherCatManager manager(ifname);
    int iDof = manager.getNumClients();
    printf("found %d ethercat clients \n", iDof);
    minas_control::MinasClient* clients[iDof];
    minas_control::MinasInput input[iDof];
    int32 current_position[iDof];
    minas_control::MinasOutput output[iDof];
    // 定义四个clients
    for (int j=0; j< iDof; j++)
    {
        clients[j] = new minas_control::MinasClient(manager, j+1);          //
    }

    //定义四个双向队列 一个数据读取需要 1ns
    std::deque < double > cmdAxisPosOne;
    std::deque < double > cmdAxisPosTwo;
    std::deque < double > cmdAxisPosThree;
    std::deque < double > cmdAxisPosFour;

    printf("initial ethercat framework....\n");

    clock_gettime(CLOCK_REALTIME, &tick);
    timespecInc(tick, period);
    int s_MODE = MODE_INIT;
    while(ros::ok)
    {
        for (int j =0; j< iDof; j++)
        {
            input[j] = clients[j]->readInputs();
            printf("Input:\n");
            printf(" 603Fh %08x :Error code\n", input[j].error_code);
            printf(" 6041h %08x :Statusword\n", input[j].statusword);
            printf(" 6061h %08x :Modes of operation display\n", input[j].operation_mode);
            printf(" 6064h %08x :Position actual value\n", input[j].position_actual_value);
            printf(" 606Ch %08x :Velocity actual value\n", input[j].velocity_actual_value);
            printf(" 6077h %08x :Torque actual value\n", input[j].torque_actual_value);
            printf(" 60B9h %08x :Touch probe status\n", input[j].touch_probe_status);
            printf(" 60BAh %08x :Touch probe pos1 pos value\n", input[j].touch_probe_posl_pos_value);
            printf(" 60FDh %08x :Digital inputs\n", input[j].digital_inputs);

            int32 temp_SW = input[j].statusword;
            int32 temp_DI = input[j].digital_inputs;

            //string Result;
            //std::ostringstream convert;
            //convert << temp_SW;//put the float WristRightR into stream
            //statusWord[j].data = convert.str(); //get string from stream

            /*
            rtos[j] = int (temp_SW && 0x0001);
            so[j] = int ( (temp_SW && 0x0003) >> 1);
            oe[j] = int ( (temp_SW && 0x0007) >> 2) ;
            f[j] = int ((temp_SW && 0x000F) >> 3);
            ve[j] = int ((temp_SW && 0x001F) >>4);
            qs[j] = int ((temp_SW && 0x003F) >> 5);
            sod[j] = int ((temp_SW && 0x007F) >> 6);
            w[j] = int ((temp_SW && 0x00FF) >> 7);
            r[j] = int ((temp_SW && 0x01FF) >> 8);
            rm[j] = int ( (temp_SW && 0x03FF) >> 9);
            //int oms = int ((temp_SW && 0x07FF) >> 10);

            ila[j] = int ((temp_SW && 0x0FFF) >>11);
            drive_follow_com[j] = int ((temp_SW && 0x1FFF) >> 12);
            follow_error[j] = int ((temp_SW && 0x3FFF) >> 13);

            NOT[j] = int (temp_DI && 0x00000001);
            POT[j] = int ((temp_DI && 0x00000003) >> 1);
            HM_SWITCH[j] = int ((temp_DI && 0x00000007) >>2);
            SI_MON1[j] = int ( (temp_DI && 0x000FFFFF) >> 19);
            SI_MON2[j] = int ((temp_DI && 0x001FFFFF) >> 20);
            SI_MON3[j] = int ((temp_DI && 0x003FFFFF) >> 21);
            SI_MON4.at(j) = int ((temp_DI && 0x007FFFFF) >> 22);
            SI_MON5.at(j) = int ((temp_DI && 0x00FFFFFF) >>23);
            INP[j]          = int ((temp_DI && 0x01FFFFFF) >> 24);
            */
            std::ostringstream joint;
            joint << "joint" << j;
            jointState.name[j] = joint.str();
            int counts = (int)input[j].position_actual_value;
            double jointPosition = 2 * 3.1415926 * double(counts) / double(0x100000);
            jointState.position[j] = jointPosition;
        }
        /*
        bool robot_running = drive_follow_com[0] && drive_follow_com[1] && drive_follow_com[2] && drive_follow_com[3];
        bool error_state      = f[0] && f[1] && f[2] && f[3];
        bool servo_state      = so[0] && so[1] && so[2] && so[3];
        bool auto_state        = SI_MON5[0] && SI_MON5[1] && SI_MON5[2] && SI_MON5[3];

        std::ostringstream convertStatusWord;
        convertStatusWord << robot_running << " " << error_state << " " << servo_state << " " << auto_state;
        statusWord.data = convertStatusWord.str();


        // publish
        jointState.header.stamp = ros::Time::now();
        driveStates.publish(statusWord);
        pJointStates.publish(jointState);
        //
        */
        int follow_error = 1;   // for test purpose
        ros::spinOnce();
        // read ros topic and publish
        if ( COMMAND_HANDLLER.newMsg() )
        {
            std_msgs::String inputCMD = COMMAND_HANDLLER.fetchMsg();
            std::string inputCMDData = inputCMD.data;
            boost::to_upper ( inputCMDData );
            std::cout << "get traj command input " <<inputCMDData << std::endl;
            if(inputCMDData == INTERFACE_CMD_STOP)
            {
                //TRAJECTORY_HANDLLER.clear();
                s_MODE = MODE_STOP;
                for(int ii = 0; ii < iDof; ii++)
                {
                    clients[ii]-> printPDSStatus(input[ii]);
                    clients[ii]->printPDSOperation(input[ii]);
                    clients[ii]->servoOff();
                }
            }
            if(follow_error)
            {
                printf("mission hasn't be accomplished \n");
                printf("command setting is invalid \n");
            }
            else
            {
                 if(inputCMDData == INTERFACE_CMD_DIRECT)
                {
                    if( s_MODE == MODE_SPLINE || s_MODE == MODE_STOP || s_MODE == MODE_INIT)
                    {
                        cmdAxisPosOne.clear();
                        cmdAxisPosTwo.clear();
                        cmdAxisPosThree.clear();
                        cmdAxisPosFour.clear();
                    }
                    s_MODE = MODE_DIRECT;
                }
                else
                {
                    if( s_MODE == MODE_DIRECT || s_MODE == MODE_STOP || s_MODE == MODE_INIT)
                    {
                        cmdAxisPosOne.clear();
                        cmdAxisPosTwo.clear();
                        cmdAxisPosThree.clear();
                        cmdAxisPosFour.clear();
                    }
                    s_MODE = MODE_SPLINE;
                }
            }

        }

        if(s_MODE == MODE_DIRECT)   // send direct joint position
        {
                //ros::spinOnce();
            if(TRAJECTORY_HANDLLER.newMsg() )
            {
                trajectory_msgs::JointTrajectory inputPoint = TRAJECTORY_HANDLLER.fetchMsg();
                size_t numAxis;
                size_t numTrajPoints;
                numAxis = inputPoint.joint_names.size();
                numTrajPoints = inputPoint.points.size();
                if(numAxis != 4)
                {
                    printf("expect four Axises\n");
                }
                else if(numTrajPoints != 1 )
                {
                    printf("expect data transition step by step\n");
                }
                else
                {
                    for(int i =0; i< numTrajPoints; i++)
                    {
                        cmdAxisPosOne.push_back(inputPoint.points[i].positions[0]);
                        cmdAxisPosTwo.push_back(inputPoint.points[i].positions[1]);
                        cmdAxisPosThree.push_back(inputPoint.points[i].positions[2]);
                        cmdAxisPosFour.push_back(inputPoint.points[i].positions[3]);
                    }
                }
            }
        }

        if(s_MODE == MODE_SPLINE)   // send many joint position
        {
                //ros::spinOnce();
            if(TRAJECTORY_HANDLLER.newMsg() )
            {
                trajectory_msgs::JointTrajectory inputPoint = TRAJECTORY_HANDLLER.fetchMsg();
                size_t numAxis;
                size_t numTrajPoints;
                numAxis = inputPoint.joint_names.size();
                numTrajPoints = inputPoint.points.size();
                if(numAxis != 4)
                {
                    printf("expect four Axises\n");
                }
                else if(numTrajPoints <2 )
                {
                    printf("expect data transition more than two points\n");
                }
                else
                {
                    for(int i =0; i< numTrajPoints; i++)
                    {
                        cmdAxisPosOne.push_back(inputPoint.points[i].positions[0]);
                        cmdAxisPosTwo.push_back(inputPoint.points[i].positions[1]);
                        cmdAxisPosThree.push_back(inputPoint.points[i].positions[2]);
                        cmdAxisPosFour.push_back(inputPoint.points[i].positions[3]);
                    }
                }
            }
        }
         if ( ALARM_COMMAND_HANDLLER.newMsg() )
        {
            std_msgs::String inputCMD = ALARM_COMMAND_HANDLLER.fetchMsg();
            std::string inputCMDData = inputCMD.data;
            //boost::to_upper ( inputCMDData );
            std::cout << "get servo command input " <<inputCMDData << std::endl;
            if(inputCMDData == "1")
            {
                for(int j =0 ; j< iDof; j++)
                {
                    clients[j]->reset();
                    printf("some error need reboot system \n");
                }
            }
        }
        if ( SERVO_COMMAND_HANDLLER.newMsg() )
        {
            std_msgs::String inputCMD = SERVO_COMMAND_HANDLLER.fetchMsg();
            std::string inputCMDData = inputCMD.data;
            //boost::to_upper ( inputCMDData );
            std::cout << "get servo command input " <<inputCMDData << std::endl;
            if(inputCMDData == "1")
            {
                if(servoStatus == 0)
                {
                    for(int j =0 ; j< iDof; j++)
                    {
                        clients[j]->reset();
                        clients[j]->setTrqueForEmergencyStop(torqueForEmergencyStopVec[j] ); // 100%
                        clients[j]->setOverLoadLevel(overloadLevelVec[j] );          // 50%
                        clients[j]->setOverSpeedLevel(overSpeedLevelVec[j] );        // r/min
                        clients[j]->setMotorWorkingRange(motorWorkingRangeVec[j] );     // 0.1
                        clients[j]->setInterpolationTimePeriod(interpolationTimePeriodVec[j] );     // 4 msec
                        clients[j]->servoOn();
                        input[j] = clients[j]->readInputs();
                        current_position[j] = input[j].position_actual_value;

                        memset(&output[j], 0x00, sizeof(minas_control::MinasOutput));
                        output[j].target_position = current_position[j];
                        output[j].max_motor_speed = max_motor_speedVec[j];  // rad/min
                        output[j].target_torque = target_torqueVec[j];    // 0% (unit 0.1%)
                        output[j].max_torque    = max_torqueVec[j];    // 50% (unit 0.1%)
                        output[j].controlword   = controlwordVec[j]; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5) p77
                        output[j].operation_mode = operation_mode;
                        clients[j]->setProfileVelocity(profileVelVec[j]);
                        clients[j]->writeOutputs(output[j]);

                        while ( ! (input[j].statusword & 0x1000) )
                        {// bit12 (set-point-acknowledge)  p79
                            input[j] = clients[j]->readInputs();
                        }
                        output[j].controlword   &= ~0x0010; // clear new-set-point (bit4)
                        clients[j]->writeOutputs(output[j]);
                        printf("target position = %08x\n", output[j].target_position);
                    }
                }
                else
                {
                    printf("servo has been turned on \n");
                }
                servoStatus = 1;
            }
            if(inputCMDData == "0")
            {
                if(servoStatus == 1)
                {
                    for(int ii =0; ii < iDof; ii++)
                    {
                        clients[ii]-> printPDSStatus(input[ii]);
                        clients[ii]->printPDSOperation(input[ii]);
                        clients[ii]->servoOff();
                        //delete clients[ii];
                    }
                }
                servoStatus = 0;
            }
        }

        if(servoStatus)
        {
            if(s_MODE == MODE_SPLINE && cmdAxisPosOne.size() > 0 && cmdAxisPosTwo.size() > 0 &&  cmdAxisPosThree.size() > 0 && cmdAxisPosFour.size() > 0)
            {
                output[0] = clients[0]->readOutputs();
                printf("sending command....\n");

                double temp1 = 0x100000 * cmdAxisPosOne.at(0) / (2.0 * 3.141596);
                output[0].position_offset =  int32(temp1);
                std::cout <<  output[0].position_offset  << std::endl;
                clients[0]->writeOutputs(output[0]);
                cmdAxisPosOne.pop_front();

                double temp2 = 0x100000 * cmdAxisPosTwo.at(0) / (2.0 * 3.141596);
                output[1].position_offset =  int32(temp2);
                std::cout <<  output[1].position_offset  << std::endl;
                clients[1]->writeOutputs(output[1]);
                cmdAxisPosTwo.pop_front();

                double temp3 = 0x100000 * cmdAxisPosThree.at(0) / (2.0 * 3.141596);
                output[2].position_offset =  int32(temp3);
                std::cout <<  output[2].position_offset  << std::endl;
                clients[2]->writeOutputs(output[2]);
                cmdAxisPosThree.pop_front();

                double temp4 = 0x100000 * cmdAxisPosFour.at(1) / (2.0 * 3.141596);
                output[3].position_offset =  int32(temp4);
                std::cout <<  output[3].position_offset  << std::endl;
                clients[3]->writeOutputs(output[3]);
                cmdAxisPosFour.pop_front();
            }

            if(s_MODE == MODE_DIRECT && cmdAxisPosOne.size() > 50 && cmdAxisPosTwo.size() > 50 &&  cmdAxisPosThree.size() > 50 && cmdAxisPosFour.size() > 50)
            {
                output[0] = clients[0]->readOutputs();
                printf("sending command....\n");

                double temp1 = 0x100000 * cmdAxisPosOne.at(0) / (2.0 * 3.141596);
                output[0].position_offset =  int32(temp1);
                std::cout <<  output[0].position_offset  << std::endl;
                clients[0]->writeOutputs(output[0]);
                cmdAxisPosOne.pop_front();

                double temp2 = 0x100000 * cmdAxisPosTwo.at(0) / (2.0 * 3.141596);
                output[1].position_offset =  int32(temp2);
                std::cout <<  output[1].position_offset  << std::endl;
                clients[1]->writeOutputs(output[1]);
                cmdAxisPosTwo.pop_front();

                double temp3 = 0x100000 * cmdAxisPosThree.at(0) / (2.0 * 3.141596);
                output[2].position_offset =  int32(temp3);
                std::cout <<  output[2].position_offset  << std::endl;
                clients[2]->writeOutputs(output[2]);
                cmdAxisPosThree.pop_front();

                double temp4 = 0x100000 * cmdAxisPosFour.at(1) / (2.0 * 3.141596);
                output[3].position_offset =  int32(temp4);
                std::cout <<  output[3].position_offset  << std::endl;
                clients[3]->writeOutputs(output[3]);
                cmdAxisPosFour.pop_front();
            }
        }
        // send command
        /*
        for (int i = 0; i <= (2*200*3.14); i++ )
        {
            for(int k = 0; k < iDof; k++)
            {
                output[k] = clients[k]->readOutputs();
                printf("sending command....\n");
                output[k].position_offset =  0x80000*sin(i/200.0);
                std::cout <<  output[k].position_offset  << std::endl;
                clients[k]->writeOutputs(output[k]);
            }

            timespecInc(tick, period);
            struct timespec before;
            clock_gettime(CLOCK_REALTIME, &before);
            double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -  (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
            if (overrun_time > 0.0)
            {
                fprintf(stderr, "  overrun: %f", overrun_time);
            }

            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
        }
        */
        //usleep(1000000);
        timespecInc(tick, period);
        struct timespec before;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }

    for(int ii =0; ii < iDof; ii++)
    {
         if(servoStatus == 1)
         {
            clients[ii]-> printPDSStatus(input[ii]);
            clients[ii]->printPDSOperation(input[ii]);
            clients[ii]->servoOff();
            delete clients[ii];
        }
        else
        {
            printf("servo has been turned off \n");
        }
    }
    printf("End program\n");

    return 0;
}




