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
    
    int                     servoStatus                 = 0;
    double                  period                      = 4e+6;
    struct timespec         tick;

    int iDof = 4;

    std::deque < double >    cmdAxisPosOne;
    std::deque < double >    cmdAxisPosTwo;
    std::deque < double >    cmdAxisPosThree;
    std::deque < double >    cmdAxisPosFour;

    double                  directAxisOne;
    double                  directAxisTwo;
    double                  directAxisThree;
    double                  directAxisFour;
    
    int                     direct_accept_times             = 0; 
    int                     direct_not_accept_times         = 0;
    int                     direct_runflag                  = 0; 
    int                     direct_stopflag                 = 1;
    int                     spline_runflag                  = 0; 
    int                     spline_stopflag                 = 1;

    std::string             strStatus;

    subscriber_joint_path = nh.subscribe<trajectory_msgs::JointTrajectory> (INTERFACE_JOINT_PATH_COMMAND, 1024, &JointTrajectoryHandller::callback, &TRAJECTORY_HANDLLER );
    subscriber_command = nh.subscribe<std_msgs::String> (INTERFACE_COMMAND, 1024, &StringHandller::callback, &COMMAND_HANDLLER );
    subscriber_servo_command = nh.subscribe<std_msgs::String>(INTERFACE_SERVO_COMMAND, 1024, &StringHandller::callback, &SERVO_COMMAND_HANDLLER);   //   LG
    subscriber_alarm_command = nh.subscribe<std_msgs::String>(INTERFACE_ALARM_CLEAR, 1024, &StringHandller::callback, &ALARM_COMMAND_HANDLLER);        //   LG

    pJointStates = nh.advertise<sensor_msgs::JointState> (INTERFACE_JOINT_STATES, 1024 );
    driveStates = nh.advertise<std_msgs::String> (INTERFACE_DRIVE_STATE, 1024 );
    pInterfaceStates = nh.advertise<std_msgs::String> (INTERFACE_STATES, 1024 );
    
    
    
    for(int q = 1 ; q < 10 ; q++)
    {
        jointState.header.stamp = ros::Time::now();
        jointState.name = j_names;
        jointState.position = jointPos;
        pJointStates.publish(jointState);
        usleep(100000);
    }


    
    printf("initialled!....\n");

    clock_gettime(CLOCK_REALTIME, &tick);
    timespecInc(tick, period);
    int s_MODE = MODE_INIT;
    while(ros::ok)
    {
        
        ros::spinOnce();
        // read ros topic and publish
        if ( COMMAND_HANDLLER.newMsg() )
        {
            printf("ROS BEGIN!\n"); 
            std_msgs::String inputCMD = COMMAND_HANDLLER.fetchMsg();
            std::string inputCMDData = inputCMD.data;
            boost::to_upper ( inputCMDData );
            std::cout << "get traj command input " <<inputCMDData << std::endl;
            if(inputCMDData == INTERFACE_CMD_START)
            {
                //TRAJECTORY_HANDLLER.clear();
                printf("Servo on!\n");
                servoStatus                 = 1;
            }
            if(inputCMDData == INTERFACE_CMD_STOP)
            {
                //TRAJECTORY_HANDLLER.clear();
                s_MODE = MODE_STOP;
                servoStatus                 = 0;
                direct_runflag              = 0;
                direct_stopflag             = 1;
                spline_runflag              = 0;
                spline_stopflag             = 1;
                cmdAxisPosOne.clear();
                cmdAxisPosTwo.clear();
                cmdAxisPosThree.clear();
                cmdAxisPosFour.clear();
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
                    printf("mode direct!\n");
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
                    
                        printf("Axis %d servo on!\n", j+1 );
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
                        printf("Axis %d servo off!", ii+1 );
                    }
                }
                servoStatus = 0;
            }
        }
        //printf("%d",servoStatus);
    if(servoStatus)
        {
            
        if(s_MODE == MODE_DIRECT)   // send direct joint position
        {
                //ros::spinOnce();
            if(TRAJECTORY_HANDLLER.newMsg() )
            {
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
                                printf("points added!!\n");
                            }
                        }
                        // printf("Receive %d Direct point !\n", cmdAxisPosOne.size());
                            
                            // printf("accept %d\n", direct_accept_times);
                        // direct_runflag                  =           0;
                        // direct_stopflag                 =           1;
                    }
                    else
                    {
                        printf("A spline is running, delete new direct!\n");
                        TRAJECTORY_HANDLLER.clear();
                        s_MODE = MODE_SPLINE;
                    }
            }
            else
            {
                direct_not_accept_times++;
                // printf("not accept %d\n", direct_not_accept_times);
                if(direct_not_accept_times == 50)
                {
                    direct_stopflag                 =           1;
                    direct_runflag                  =           0;
                    direct_not_accept_times         =           0;
                    direct_accept_times             =           0;
                    cmdAxisPosOne.clear();
                    cmdAxisPosTwo.clear();
                    cmdAxisPosThree.clear();
                    cmdAxisPosFour.clear();
                    printf("points cleared!!\n");
                }
            }
            if(direct_accept_times > 10)
            {
                direct_runflag  =   1;
                direct_stopflag =   0;
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
                                printf("expect four Axises\n");
                            }
                            else if(numTrajPoints <2 )
                            {
                                printf("expect data transition more than two points\n");
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
                            printf("A spline is running, delete new spline!\n");
                            TRAJECTORY_HANDLLER.clear();
                    }
                    
                }

            }
       
            if(s_MODE == MODE_SPLINE && cmdAxisPosOne.size() > 0 && cmdAxisPosTwo.size() > 0 &&  cmdAxisPosThree.size() > 0 && cmdAxisPosFour.size() > 0)
            {
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
                    printf("sending command....\n");

                    jointPos[0]  = jointPos[0] + directAxisOne;
                    jointPos[1]  = jointPos[1] + directAxisTwo;
                    jointPos[2]  = jointPos[2] + directAxisThree;
                    jointPos[3]  = jointPos[3] + directAxisFour;

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


        jointState.header.stamp = ros::Time::now();
        jointState.position = jointPos;
        pJointStates.publish(jointState);
        

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
          //
        
          //
        std_msgs::String output;
        output.data = strStatus;
        pInterfaceStates.publish ( output );

        timespecInc(tick, period);
        struct timespec before;
        clock_gettime(CLOCK_REALTIME, &before);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }

    for(int ii =0; ii < iDof; ii++)
    {
         if(servoStatus == 1)
         {
            std::cout<< "a";
        }
        else
        {
            printf("servo has been turned off \n");
        }
    }
    printf("End program\n");

    return 0;
}




