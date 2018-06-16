/*************************************************************************
	> File Name: ethercat_interface_test.cpp
	> Author: Likun Wang
	> Mail: likunwang@hit.edu.cn
	> Created Time: 2018年01月10日 星期三 23时30分17秒
 ************************************************************************/
//#include <sstream>
#include "ethercat_interface.h"
#include <QString>

int main(int argc, char *argv[])
{    
    // initializing ROS
    printf("initializing ROS\n");
    ros::init(argc, argv, "ethercat_interface");
    ros::NodeHandle                     nh;
    printf("initializing ROS succeed\n");
    ROS_INFO ( "initializing subscriber and publisher" );
    JointTrajectoryHandller             TRAJECTORY_HANDLLER;
    StringHandller                      COMMAND_HANDLLER;
    StringHandller                      SERVO_COMMAND_HANDLLER;
    StringHandller                      ALARM_COMMAND_HANDLLER;
    ros::Subscriber                     subscriber_joint_path;    //只订阅了发的数据
    ros::Subscriber                     subscriber_command;
    ros::Subscriber                     subscriber_servo_command;
    ros::Subscriber                     subscriber_alarm_command;
    ros::Publisher                      pJointStates;
    ros::Publisher                      driveStates;
    ros::Publisher                      pInterfaceStates;


    std::vector<std::string> j_names ;
    std::string j_name1 = "joint1";
    std::string j_name2 = "joint2";
    std::string j_name3 = "joint3";
    std::string j_name4 = "joint4";
    j_names.push_back(j_name1);
    j_names.push_back(j_name2);
    j_names.push_back(j_name3);
    j_names.push_back(j_name4);



    sensor_msgs::JointState             jointState;
    std::vector<double>                  jointPos(4);
    ROS_INFO ( "initializing subscriber and publisher succeed" );
    ROS_INFO ( "Main loop on the go" );
    
    double                  period                          = 1e+6;
    struct timespec         tick;

    //int iDof = 4;

    std::deque < double >    cmdAxisPosOne;
    std::deque < double >    cmdAxisPosTwo;
    std::deque < double >    cmdAxisPosThree;
    std::deque < double >    cmdAxisPosFour;


    int ct = 1;
    int tt = 1;
    double                  directAxisOne;
    double                  directAxisTwo;
    double                  directAxisThree;
    double                  directAxisFour;
    double                  reference_point[4];    
  
    std::vector <int>       fault(4,0);//driver fault
    std::vector <int>       oe(4,0);//operation enable
    std::vector <int>       so(4,0);//switched on
    std::vector <int>       drive_follows(4,0);//drive follows command value
    int                     SI_MON4;
    int                     SI_MON5;


    int                     servoStatus                     = 0;
    int                     servoReady                      = 0;
    int                     error_state                     = 0;
    int                     auto_state                      = 0;
    int                     robot_run_state                 = 0;
    int                     direct_accept_times             = 0; 
    int                     direct_not_accept_times         = 0;
    int                     direct_runflag                  = 0; 
    int                     direct_stopflag                 = 1;
    int                     spline_runflag                  = 0; 
    int                     spline_stopflag                 = 1;
    int                     inputFlag                       = 1;
    int                     stopFlag                        = 0;
    int                     clearAllData                    = 1;
    std::string             strStatus;

    std::vector<double>      jointPosition(4);

    double reduction[4] = {50,50, 2*2*3.14159/0.02, 20};

    subscriber_joint_path = nh.subscribe<trajectory_msgs::JointTrajectory> (INTERFACE_JOINT_PATH_COMMAND, 1024, &JointTrajectoryHandller::callback, &TRAJECTORY_HANDLLER );
    subscriber_command = nh.subscribe<std_msgs::String> (INTERFACE_COMMAND, 1024, &StringHandller::callback, &COMMAND_HANDLLER );
    subscriber_servo_command = nh.subscribe<std_msgs::String>(INTERFACE_SERVO_COMMAND, 1024, &StringHandller::callback, &SERVO_COMMAND_HANDLLER);   //   LG
    subscriber_alarm_command = nh.subscribe<std_msgs::String>(INTERFACE_ALARM_CLEAR, 1024, &StringHandller::callback, &ALARM_COMMAND_HANDLLER);        //   LG

    pJointStates = nh.advertise<sensor_msgs::JointState> (INTERFACE_JOINT_STATES, 1024 );
    driveStates = nh.advertise<std_msgs::String> (INTERFACE_DRIVE_STATE, 1024 );
    pInterfaceStates = nh.advertise<std_msgs::String> (INTERFACE_STATES, 1024 );
    
/*****   
    for(int q = 1 ; q < 10 ; q++)
    {
        jointState.header.stamp = ros::Time::now();
        jointState.name = j_names;
        jointState.position = jointPos;
        pJointStates.publish(jointState);
        usleep(100000);
    }
  ******/ 
    ROS_INFO ( "initializing subscriber and publisher succeed" );

    ROS_INFO ( "Main loop on the go" );
  //  printf("initialled!....\n");

    //new add by mxx 2018.3.12
    //驱动器初始化，上伺服
    int operation_mode = 0x08;
    std::string ifname;
	ifname = "eth0";
	std::cout << ifname << std::endl;

    // start slaveinfo     
    retry: try
    {
        ethercat::EtherCatManager manager(ifname);
        int iDof = manager.getNumClients();
        ROS_INFO("found %d ethercat clients \n", iDof);
        minas_control::MinasClient* clients[iDof];
        minas_control::MinasInput input[iDof];
        int32 current_position[iDof];
        minas_control::MinasOutput output[iDof];
        // 定义四个clients
    //   for (int j=0; j< iDof; j++)
    //   {
    //       clients[j] = new minas_control::MinasClient(manager, j+1);          //
    //   }
        printf("initial ethercat framework....\n");
        //clock_gettime(CLOCK_REALTIME, &tick);
        //timespecInc(tick, period);

        clock_gettime(CLOCK_REALTIME, &tick);
        timespecInc(tick, period);
        int s_MODE = MODE_INIT;

        for(int q = 1 ; q < 10 ; q++)
        {
            jointState.header.stamp = ros::Time::now();
            jointState.name = j_names;
            jointState.position = jointPos;
            pJointStates.publish(jointState);
            usleep(100000);
        }

        while(ros::ok)
        { 
            ros::spinOnce();
            //**********************read ros topic and publish********************************************//
            if ( COMMAND_HANDLLER.newMsg() )
            {
                printf("ROS BEGIN!\n"); 
                std_msgs::String inputCMD = COMMAND_HANDLLER.fetchMsg();
                std::string inputCMDData = inputCMD.data;
                boost::to_upper ( inputCMDData );
                // std::cout << "get traj command input " <<inputCMDData << std::endl;
                if(inputCMDData == INTERFACE_CMD_START)
                {
                    //TRAJECTORY_HANDLLER.clear();
                    for(int j =0 ; j< iDof; j++)
                        {
                            clients[j] = new minas_control::MinasClient(manager, j+1);
                            clients[j]->reset();
                            clients[j]->setTrqueForEmergencyStop(torqueForEmergencyStopVec[j] ); // 100%
                            clients[j]->setOverLoadLevel(overloadLevelVec[j] );          // 50%
                            clients[j]->setOverSpeedLevel(overSpeedLevelVec[j] );        // r/min
                            clients[j]->setMotorWorkingRange(motorWorkingRangeVec[j] );     // 0.1
                            clients[j]->setInterpolationTimePeriod(interpolationTimePeriodVec[j] );     // 4 msec
                            clients[j]->servoOn();
                            input[j] = clients[j]->readInputs();
                        
                            int temp_position = (int)input[j].position_actual_value;
                            double current_jointPosition = double(temp_position) / double(0x20000);
                            reference_point[j] = current_jointPosition;
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
                            printf("Axis %d servo on!\n", j+1 );
                            usleep(100000);
                        }
                    printf("Servo on!\n");
                    servoStatus = 1;
                }
                if(inputCMDData == INTERFACE_CMD_STOP)
                {
                    //TRAJECTORY_HANDLLER.clear();
                    s_MODE = MODE_STOP;
                    //servoStatus                 = 0;
                    robot_run_state             = 0;
                    auto_state                  = 0;
                    direct_runflag              = 0;
                    direct_stopflag             = 1;
                    spline_runflag              = 0;
                    spline_stopflag             = 1;
                    cmdAxisPosOne.clear();
                    cmdAxisPosTwo.clear();
                    cmdAxisPosThree.clear();
                    cmdAxisPosFour.clear();
                    for(int ii = 0; ii < iDof; ii++)
                    {
                        clients[ii]-> printPDSStatus(input[ii]);
                        clients[ii]->printPDSOperation(input[ii]);
                        clients[ii]->servoOff();
                    }
                }
                
                if(inputCMDData == INTERFACE_CMD_DIRECT)
                {
                    if( s_MODE == MODE_SPLINE || s_MODE == MODE_STOP || s_MODE == MODE_INIT)
                    {
                        cmdAxisPosOne.clear();
                        cmdAxisPosTwo.clear();
                        cmdAxisPosThree.clear();
                        cmdAxisPosFour.clear();
                    }
                    // printf("mode direct!\n");
                    s_MODE = MODE_DIRECT;
                }
                if(inputCMDData == INTERFACE_CMD_SPLINE)
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
            //**********************states from drivers*******************************************//
            if(servoStatus)
            {
                for (int j =0; j< iDof; j++)
                {
                //ROS_INFO("read inputs");
                input[j] = clients[j]->readInputs();
                // printf("Input:\n");
                // printf(" 603Fh %08x :Error code\n", input[j].error_code);
                // printf(" 6041h %08x :Statusword\n", input[j].statusword);
                // printf(" 6061h %08x :Modes of operation display\n", input[j].operation_mode);
                // printf(" 6064h %08x :Position actual value\n", input[j].position_actual_value);
                // printf(" 606Ch %08x :Velocity actual value\n", input[j].velocity_actual_value);
                // printf(" 6077h %08x :Torque actual value\n", input[j].torque_actual_value);
                // printf(" 60B9h %08x :Touch probe status\n", input[j].touch_probe_status);
                // printf(" 60BAh %08x :Touch probe pos1 pos value\n", input[j].touch_probe_posl_pos_value);
                // printf(" 60FDh %08x :Digital inputs\n", input[j].digital_inputs);
                
                //手动自动模块
                int32 temp_ERR = input[j].error_code;
                int32 temp_SW = input[j].statusword;
                int32 temp_DI = input[j].digital_inputs;
                fault.at(j) = int ((temp_SW & 0x000F) >> 3);
                oe.at(j) = int ((temp_SW & 0x0007) >> 2);
                so.at(j) = int ((temp_SW & 0x0003) >> 1);
                drive_follows.at(j) = int ((temp_SW & 0x1fff) >> 12);
                if(j == 0)
                {
                    SI_MON4 = int ((temp_DI & 0x007FFFFF) >> 22);
                    SI_MON5 = int ((temp_DI & 0x00FFFFFF) >> 23);
                }
                //printf(" 60FDh %08x :Digital inputs\n", input[j].digital_inputs);

                int counts = (int)input[j].position_actual_value;
                // std::cout << j+1 << " axis counts is "<< counts <<std::endl;
                jointPosition[j] = double(counts) / double(0x20000);
                jointState.position[j] = jointPosition[j]/reduction[j];
                // std::cout << j+1 << " axis is "<< jointPosition[j] <<std::endl;
                }
                //jointState.position[2] = jointState.position[2] - 0.04; // offset
            }
            
            jointState.name =              j_names;
            robot_run_state = drive_follows.at(0) & drive_follows.at(1) & drive_follows.at(2) & drive_follows.at(3);
            //printf("robot_run_state = %d \n", robot_run_state);
            error_state = fault.at(0) & fault.at(1) & fault.at(2) & fault.at(3);
            //printf("error_state = %d \n", error_state);
            servoReady = so.at(0) & so.at(1) & so.at(2) & so.at(3);
            //printf("servoReady = %d \n", servoReady);
            servoStatus = oe.at(0) & oe.at(1) & oe.at(2) & oe.at(3);            
            if (SI_MON4 == 0 && SI_MON5 == 1)
            {
                //ROS_INFO("AUTO mode");
                auto_state = 1;
            }
            else
            {
                //ROS_INFO("MANUAL mode");
                auto_state = 0;
            }

            //revise robot_run_state
            robot_run_state = spline_runflag | direct_runflag;

            //*********************publish driver states******************************************//
            std_msgs::String outing;
            std::string output_driverStates;
            xc::StringAppend ( output_driverStates, "%d ", robot_run_state );
            xc::StringAppend ( output_driverStates, "%d ", servoStatus );
            xc::StringAppend ( output_driverStates, "%d ", error_state );
            xc::StringAppend ( output_driverStates, "%d ", auto_state );

            outing.data = output_driverStates;
            //ROS_INFO("driver states: %s.", outing.data.c_str());
            driveStates.publish ( outing );

            //*********************alarm clear command from gui******************************************//
            // if ( ALARM_COMMAND_HANDLLER.newMsg() )
            // {
            //     std_msgs::String inputCMD = ALARM_COMMAND_HANDLLER.fetchMsg();
            //     std::string inputCMDData = inputCMD.data;
            //     //boost::to_upper ( inputCMDData );
            //     //std::cout << "get servo command input " <<inputCMDData << std::endl;
            //     if(inputCMDData == "1")
            //     {
            //         for(int j =0 ; j< iDof; j++)
            //         {
            //             clients[j]->reset();
            //             // printf("some error need reboot system \n");
            //         }
            //     }
            // }
            //*******************************************************************************************//


            //**********************output for drivers***************************************************//
            if(servoStatus)
            {
                    
                if(s_MODE == MODE_DIRECT)   // send direct joint position
                {
                        //ros::spinOnce();
                    if(TRAJECTORY_HANDLLER.newMsg() )
                    {
                        // printf("I heard new message 1 hahaha!!!!!\n");
                        direct_not_accept_times         =           0;
                        direct_accept_times++;
                        if( spline_runflag == 0 && spline_stopflag == 1)
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
                                        // cmdAxisPosOne.push_back(inputPoint.points[i].positions[0]);
                                        // cmdAxisPosTwo.push_back(inputPoint.points[i].positions[1]);
                                        // cmdAxisPosThree.push_back(inputPoint.points[i].positions[2]);
                                        // cmdAxisPosFour.push_back(inputPoint.points[i].positions[3]);
                                        directAxisOne = inputPoint.points[i].positions[0];
                                        directAxisTwo = inputPoint.points[i].positions[1];
                                        directAxisThree = inputPoint.points[i].positions[2];
                                        directAxisFour = inputPoint.points[i].positions[3];
                                        // printf("points added!!\n");
                                    }
                                }
                                // printf("Receive %d Direct point !\n", cmdAxisPosOne.size());
                                    
                                    // printf("accept %d\n", direct_accept_times);
                                // direct_runflag                  =           0;
                                // direct_stopflag                 =           1;
                            }
                            else
                            {
                                // printf("A spline is running, delete new direct!\n");
                                TRAJECTORY_HANDLLER.clear();
                                s_MODE = MODE_SPLINE;
                            }
                    }
                    else
                    {
                        direct_not_accept_times++;
                        // printf("not accept %d\n", direct_not_accept_times);
                        if(direct_not_accept_times == 200)
                        {
                            direct_stopflag                 =           1;
                            direct_runflag                  =           0;
                            direct_not_accept_times         =           0;
                            direct_accept_times             =           0;
                            // cmdAxisPosOne.clear();
                            // cmdAxisPosTwo.clear();
                            // cmdAxisPosThree.clear();
                            // cmdAxisPosFour.clear();
                            // printf("points cleared!!\n");
                        }
                    }
                    if(direct_accept_times > 10)
                    {
                        direct_runflag  =   1;
                        direct_stopflag =   0;
                        tt = 1;
                    }

                }
                        
                    // if(cmdAxisPosOne.size() > 50 && cmdAxisPosTwo.size() > 50 &&  cmdAxisPosThree.size() > 50 && cmdAxisPosFour.size() > 50)
                    // {
                    //     direct_runflag  =   1;
                    //     direct_stopflag =   0;
                    // }
                    /**********
                    else
                    {
                        direct_runflag  =   0;
                        direct_stopflag =   1;
                        //printf("points number < 50, stop!\n");
                    }
                    **********/
                    // if(cmdAxisPosOne.size() == 0 || cmdAxisPosTwo.size() == 0 || cmdAxisPosThree.size() == 0 || cmdAxisPosFour.size() == 0)
                    // {
                    //     direct_runflag  =   0;
                    //     direct_stopflag =   1;
                    // }
                    
                
                
                if(s_MODE == MODE_SPLINE)   
                {
                    if(TRAJECTORY_HANDLLER.newMsg() )
                    {   
                    
                        if(spline_runflag == 0 && spline_stopflag == 1)
                        {
                            trajectory_msgs::JointTrajectory inputPoint = TRAJECTORY_HANDLLER.fetchMsg();
                            size_t numAxis;
                            size_t numTrajPoints;
                            numAxis = inputPoint.joint_names.size();
                            numTrajPoints = inputPoint.points.size();
                            if(numAxis != 4)
                            {
                                // printf("expect four Axises\n");
                            }
                            else if(numTrajPoints <2 )
                            {
                                // printf("expect data transition more than two points\n");
                            }
                            else
                            {
                                for(int p =0; p< numTrajPoints; p++)
                                    {
                                        cmdAxisPosOne.push_back(inputPoint.points[p].positions[0]);
                                        cmdAxisPosTwo.push_back(inputPoint.points[p].positions[1]);
                                        cmdAxisPosThree.push_back(inputPoint.points[p].positions[2]);
                                        cmdAxisPosFour.push_back(inputPoint.points[p].positions[3]);
                                    }
                            }
                        }
                        else
                        {
                            // printf("A spline is running, delete new spline!\n");
                            TRAJECTORY_HANDLLER.clear();
                        }                        
                            
                    }

                }
            
                if(s_MODE == MODE_SPLINE && cmdAxisPosOne.size() > 0 && cmdAxisPosTwo.size() > 0 &&  cmdAxisPosThree.size() > 0 && cmdAxisPosFour.size() > 0)
                { 
                        // output[0] = clients[0]->readOutputs();
                        //printf("sending command....\n");
                        spline_runflag  = 1;
                        spline_stopflag = 0;
                        double temp1 = cmdAxisPosOne.at(0);
                        //std::cout << temp1 << std::endl;
                        output[0].position_offset = 0x20000 * (50 * temp1 - reference_point[0]);
                        // std::cout <<  output[0].position_offset  << std::endl;
                        clients[0]->writeOutputs(output[0]);
                        cmdAxisPosOne.pop_front();

                        double temp2 = cmdAxisPosTwo.at(0);
                        output[1].position_offset = 0x20000 * (50 * temp2 - reference_point[1]);
                        //std::cout <<  output[1].position_offset  << std::endl;
                        clients[1]->writeOutputs(output[1]);
                        cmdAxisPosTwo.pop_front();

                        double temp3 = cmdAxisPosThree.at(0);
                        output[2].position_offset = 0x20000 * (2*2*3.14159/0.02 * temp3 - reference_point[2]);
                        //std::cout <<  output[2].position_offset  << std::endl;
                        clients[2]->writeOutputs(output[2]);
                        cmdAxisPosThree.pop_front();

                        double temp4 = cmdAxisPosFour.at(0);
                        output[3].position_offset = 0x20000 * (20 * temp4 - reference_point[3]);
                    // std::cout <<  output[3].position_offset  << std::endl;
                        clients[3]->writeOutputs(output[3]);
                        cmdAxisPosFour.pop_front();
                /*************           
                        printf("sending command....\n");
                        spline_runflag  = 1;
                        spline_stopflag = 0;
                        jointPos.clear();

                        double temp1 =cmdAxisPosOne.at(0);
                        jointPos.push_back(temp1);
                        cmdAxisPosOne.pop_front();

                        double temp2 =  cmdAxisPosTwo.at(0);
                        jointPos.push_back(temp2);
                        cmdAxisPosTwo.pop_front();

                        double temp3 =  cmdAxisPosThree.at(0);
                        jointPos.push_back(temp3);
                        cmdAxisPosThree.pop_front();

                        double temp4 = cmdAxisPosFour.at(0) ;
                        jointPos.push_back(temp4);
                        cmdAxisPosFour.pop_front();
                        //printf("pos is %f,%f,%f,%f\n",temp1,temp2,temp3,temp4);
                        
                    ******************************/ 
                        if(cmdAxisPosOne.size() == 0 || cmdAxisPosTwo.size() == 0 || cmdAxisPosThree.size() == 0 || cmdAxisPosFour.size() == 0)
                        {
                            cmdAxisPosOne.clear();
                            cmdAxisPosTwo.clear();
                            cmdAxisPosThree.clear();
                            cmdAxisPosFour.clear();
                            spline_runflag  = 0;
                            spline_stopflag = 1;
                        }

                }

                if(s_MODE == MODE_DIRECT && direct_runflag && !direct_stopflag)
                {
                    // printf("I am ready to run 2 hahaha!!!!!\n");
                    // printf("sending command....\n");

                    double temp1 = 2*directAxisOne / 3.1415926 * 180 / 0.2;
                    // printf("%f....\n",temp1);
                    output[0].position_offset =  ((jointPosition[0] - reference_point[0]) + 50 * temp1 * 0.5 * 0.00069813 ) * 0x20000;
                    // std::cout <<  output[0].position_offset  << std::endl;
                    clients[0]->writeOutputs(output[0]);

                    double temp2 = 2*directAxisTwo / 3.1415926 * 180 / 0.2;
                    output[1].position_offset =  ((jointPosition[1] - reference_point[1]) + 50 * temp2 * 0.5 * 0.00069813 ) * 0x20000;
                    // std::cout <<  output[1].position_offset  << std::endl;
                    clients[1]->writeOutputs(output[1]);
                    if(directAxisThree != 0)
                    {
                        double temp3 = 2*directAxisThree *1000 / 0.2;
                        output[2].position_offset =  ((jointPosition[2] - reference_point[2]) + 2 * 5 * temp3 * 0.00069813 ) * 0x20000;
                        // std::cout <<  output[2].position_offset  << std::endl;
                        clients[2]->writeOutputs(output[2]);
                    }
                    if(directAxisFour != 0)
                    {
                        double temp4 = 2*directAxisFour / 3.1415926 * 180 / 0.2;
                        output[3].position_offset =  ((jointPosition[3] - reference_point[3]) + 20 * 4.5 * temp4 * 0.00069813 ) * 0x20000;
                        clients[3]->writeOutputs(output[3]);

                        output[2].position_offset =  ((jointPosition[2] - reference_point[2]) + 2 * 4.5 * temp4 * 0.00069813 ) * 0x20000;
                        // std::cout <<  output[2].position_offset  << std::endl;
                        clients[2]->writeOutputs(output[2]);
                    }

                    //output[3].position_offset =  ((jointPosition[3] - reference_point[3]) + 20*18*0.00069813 ) * 0x20000;
                    //output[2].position_offset =  ((jointPosition[2] - reference_point[2]) + 2*20*0.00069813 ) * 0x20000;
                    //output[2].position_offset = 10 * (0.000069813* tt ) * 0x20000;
                    //output[1].position_offset =  ((jointPosition[1] - reference_point[1]) - 50*2*0.00069813 ) * 0x20000;
                    //output[0].position_offset =  ((jointPosition[0] - reference_point[0]) + 50*2*0.00069813 ) * 0x20000;
                    // if(ct%100 == 0 )
                    // {
                    //     std::cout << jointPosition[3] - reference_point[3] << std::endl;
                    //     std::cout << jointPosition[3] - reference_point[3] - 0.00069813 << std::endl;
                    // //     ct = 1;
                    // }
                    // ct ++;
                    // std::cout << jointPosition[3] - reference_point[3] << std::endl;
                    // std::cout << jointPosition[3] - reference_point[3] + 0.000069813 << std::endl;
                    // // std::cout <<  output[3].position_offset  << std::endl;
                    //clients[3]->writeOutputs(output[3]);

                    //clients[2]->writeOutputs(output[2]);
                    //clients[1]->writeOutputs(output[1]);
                // clients[0]->writeOutputs(output[0]);
                        // double temp2 =  cmdAxisPosTwo.at(0);
                        // jointPos.push_back(temp2);
                        // cmdAxisPosTwo.pop_front();

                        // double temp3 =  cmdAxisPosThree.at(0);
                        // jointPos.push_back(temp3);
                        // cmdAxisPosThree.pop_front();

                        // double temp4 = cmdAxisPosFour.at(0) ;
                        // jointPos.push_back(temp4);
                        // cmdAxisPosFour.pop_front();

                        //printf("pos is %f,%f,%f,%f\n",temp1,temp2,temp3,temp4);
                    // publish
                    
                }
            }

            //**********************publish joint_states************************************************//
            jointState.header.stamp = ros::Time::now();
            pJointStates.publish(jointState);
            //*******************************************************************************************//

            //**********************publish interface_states: mode + runflag*****************************//
            strStatus.clear();
            if ( MODE_DIRECT == s_MODE )
                xc::StringAppend ( strStatus, INTERFACE_STATE_DIRECT INTERFACE_STATE_SPARATER );
            else if ( MODE_SPLINE == s_MODE )
                xc::StringAppend ( strStatus, INTERFACE_STATE_DIRECT INTERFACE_STATE_SPARATER );
            else if ( MODE_INIT == s_MODE )
                xc::StringAppend ( strStatus, "INIT" INTERFACE_STATE_SPARATER );
            else if ( MODE_STOP == s_MODE )
                xc::StringAppend ( strStatus, "STOP" INTERFACE_STATE_SPARATER );
            //
            if ((direct_runflag == 1 && direct_stopflag == 0) || (spline_runflag == 1 && spline_stopflag == 0))
                xc::StringAppend ( strStatus, INTERFACE_STATE_MOVING INTERFACE_STATE_SPARATER );
            else
                xc::StringAppend ( strStatus, INTERFACE_STATE_STABLE INTERFACE_STATE_SPARATER );

            std_msgs::String output;
            output.data = strStatus;
            pInterfaceStates.publish ( output );
            //*******************************************************************************************//
            
            //**********************检测超速from one******************************************************//
            timespecInc(tick, period);
            struct timespec before;
            clock_gettime(CLOCK_REALTIME, &before);
            double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -  (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
            if (overrun_time > 0.0)
            {
                // fprintf(stderr, "  overrun: %f/n", overrun_time);
            }
            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
            //******************************************************************************************
            // tt++;
            // printf("%f\n",ros::Time::now().toSec());
        }


        //结束程序
        // for(int ii =0; ii < iDof; ii++)
        // {
        //      if(servoStatus == 1)
        //      {
        //         clients[ii]-> printPDSStatus(input[ii]);
        //         clients[ii]->printPDSOperation(input[ii]);
        //         clients[ii]->servoOff();
        //         delete clients[ii];
        //     }
        //     else
        //     {
        //         printf("servo has been turned off \n");
        //     }
        // }
        // printf("End program\n");
    }
    catch(std::exception & e)
    {
        ROS_WARN("%s", e.what());
        usleep(2000000);
        if(ros::ok())
            goto retry;
    }

    

    return 0;
}




