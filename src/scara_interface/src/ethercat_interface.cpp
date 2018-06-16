/*************************************************************************
	> File Name: ethercat_interface.cpp
	> Author: Likun Wang
	> Mail: likunwang@hit.edu.cn
	> Created Time: 2018年01月10日 星期三 23时30分10秒
 ************************************************************************/
#include "ethercat_interface.h"

int main(int argc, char *argv[])
{
    // initializing ROS
    printf("initializing ROS\n");
    ros::init(argc, argv, "ethercat_interface");
    ros::NodeHandle nh;
    printf("initializing ROS succeed\n");

    printf("operation mode initializing \n");
	int operation_mode = 0x08; // (pp) position profile mode
	//printf("operation_model = %x \n", operation_mode);

    std::string ifname;
    ifname = "eth0";		// ifconfig
    std::cout << ifname << std::endl;
    //printf("ifname = %s\n", ifname);
    printf("EhterCat initializing \n");
  	ethercat::EtherCatManager manager(ifname);
    printf("EhterCat initializing succeed\n");
	int iDof = manager.getNumClients();
    printf("EhterCat initializing succeed, found  %d  clients \n", iDof);
    double LAST_INPUT_TIME = 0.0;


    /************************************************************/
    ROS_INFO ( "initializing subscriber and publisher" );
    JointTrajectoryHandller TRAJECTORY_HANDLLER;
    StringHandller          COMMAND_HANDLLER;
    ros::Subscriber subscriber_joint_path;    //只订阅了发的数据
    ros::Subscriber subscriber_command;
    //ros::Publisher  pFeedbackStates = NODE.advertise<control_msgs::FollowJointTrajectoryFeedback> (INTERFACE_FEEDBACK_STATES, 1024 );
    //ros::Publisher  pJointStates = NODE.advertise<sensor_msgs::JointState> (INTERFACE_JOINT_STATES, 1024 );
    //ros::Publisher  pInterfaceStates =NODE.advertise<std_msgs::String> (INTERFACE_STATES, 1024 );
    ROS_INFO ( "initializing subscriber and publisher succeed" );
    /************************************************************/

    ROS_INFO ( "Set motor parameters" );
    struct motorType *motorParaAxisOne;
    struct motorType *motorParaAxisTwo;
    struct motorType *motorParaAxisThree;
    struct motorType *motorParaAxisFour;
    // 读取电机参数
    motorParaInit(motorParaAxisOne, motorParaAxisTwo, motorParaAxisThree, motorParaAxisFour);

    // 初始化驱动器 电机参数
    // 报错机制已经设置
    minas_control::MinasClient clientAxisOne(manager, AxisOne);
    minas_control::MinasInput inputAxisOne;
    minas_control::MinasOutput outputAxisOne;
    scaraEthercatManager scaraclientOne(clientAxisOne, inputAxisOne, outputAxisOne, motorParaAxisOne);

    minas_control::MinasClient clientAxisTwo(manager, AxisTwo);
    minas_control::MinasInput inputAxisTwo;
    minas_control::MinasOutput outputAxisTwo;
    scaraEthercatManager scaraclientTwo(clientAxisTwo, inputAxisTwo, outputAxisTwo, motorParaAxisTwo);

    minas_control::MinasClient clientAxisThree(manager, AxisThree);
    minas_control::MinasInput inputAxisThree;
    minas_control::MinasOutput outputAxisThree;
    scaraEthercatManager scaraclientThree(clientAxisThree, inputAxisThree, outputAxisThree, motorParaAxisThree);

    minas_control::MinasClient clientAxisFour(manager, AxisThree);
    minas_control::MinasInput inputAxisFour;
    minas_control::MinasOutput outputAxisFour;
    scaraEthercatManager scaraclientFour(clientAxisFour, inputAxisFour, outputAxisFour, motorParaAxisFour);
    ROS_INFO ( "Setting motor parameters succeeds" );

    usleep(100);  // after serve on

    /************************************************************/
    ROS_INFO ( "Main loop on the go" );
    int inputFlag = 1;
    int stopFlag  = 0;
    int clearAllData = 1;
    //std::deque< trajectory_msgs::JointTrajectory > inputTra;
    subscriber_joint_path = nh.subscribe<trajectory_msgs::JointTrajectory> (INTERFACE_JOINT_PATH_COMMAND, 1024, &JointTrajectoryHandller::callback, &TRAJECTORY_HANDLLER );
    subscriber_command = nh.subscribe<std_msgs::String> (INTERFACE_COMMAND, 1024, &StringHandller::callback, &COMMAND_HANDLLER );
    ros::Publisher  pJointStates = nh.advertise<sensor_msgs::JointState> (INTERFACE_JOINT_STATES, 1024 );
    //trajectory_msgs::JointTrajectory inputTra;
    //std_msgs::String inputCMD;
    //定义四个双向队列 一个数据读取需要 1ns
    std::deque < double > cmdAxisPosOne;
    std::deque < double > cmdAxisPosTwo;
    std::deque < double > cmdAxisPosThree;
    std::deque < double > cmdAxisPosFour;


      // 4ms in nanoseconds
    double period = 4e+6;
    // get curren ttime
    struct timespec before, after;
    struct timespec sleepTime;
    //clock_gettime(CLOCK_REALTIME, &tick);
    //timespecInc(tick, period);
    double jointAngleOne;
    double jointAngleTwo;
    double jointAngleThree;
    double jointAngleFour;
    try
    {
        int s_MODE = MODE_INIT;
        while(ros::ok() )
        {
            clock_gettime(CLOCK_REALTIME, &before);


            ros::spinOnce(); // 很重要 要不得不到数据 测设 在 begin tutorial
            //input data
            //while (inputFlag != INPUT_FINISH)
            // read current position
            jointAngleOne = scaraclientOne.jointPos();
            jointAngleTwo = scaraclientTwo.jointPos();
            jointAngleThree = scaraclientThree.jointPos();
            jointAngleFour = scaraclientFour.jointPos();


            std::vector<double> feedback ( 4), velocity ( 4);

                feedback[0] = jointAngleOne;
                feedback[1] = jointAngleTwo;
                feedback[2] = jointAngleThree;
                feedback[3] = jointAngleFour;
                  //velocity[i] = ( feedback[i] - s_LAST_FEEDBACK[i] ) / ( ros::Time::now().toSec() - s_LAST_FEEDBACK_TIME );

              // publish joint state
              sensor_msgs::JointState jointState;
              jointState.header.stamp = ros::Time::now();
              //jointState.name = TRAJ_PROCESSOR.JOINTS();
              jointState.position = feedback;
              jointState.velocity = velocity;
              pJointStates.publish ( jointState );



            if ( COMMAND_HANDLLER.newMsg() )
            {
                std_msgs::String inputCMD = COMMAND_HANDLLER.fetchMsg();
                std::string inputCMDData = inputCMD.data;
                boost::to_upper ( inputCMDData );
                std::cout << "get input " <<inputCMDData << std::endl;
                if ( MODE_DIRECT == s_MODE || MODE_SPLINE == s_MODE )
                {
                    if (inputCMDData == INTERFACE_CMD_STOP)
                    {
                        //
                        s_MODE = MODE_STOP;
                        stopFlag = 1;

                    }
                    else if(inputCMDData == INTERFACE_CMD_CLEAR)
                    {
                         TRAJECTORY_HANDLLER.clear();    // clear treaj data
                    }
                }
                else if(MODE_INIT == s_MODE)
                {
                    if(inputCMDData == INTERFACE_CMD_STOP)
                    {
                        TRAJECTORY_HANDLLER.clear();
                        //
                        s_MODE = MODE_STOP;
                        stopFlag = 1;
                    }
                    else if(inputCMDData == INTERFACE_CMD_DIRECT || inputCMDData == INTERFACE_CMD_SPLINE)
                    {
                        if(inputCMDData == INTERFACE_CMD_DIRECT)
                        {
                            s_MODE = MODE_DIRECT;
                        }
                        else
                        {
                             s_MODE = MODE_SPLINE;
                        }
                    }
                }
            }
            /*************************************************************/
            if(s_MODE == MODE_DIRECT)
            {
                //ros::spinOnce();
                if(TRAJECTORY_HANDLLER.newMsg() )
                {
                    trajectory_msgs::JointTrajectory inputPoint = TRAJECTORY_HANDLLER.fetchMsg();
                    size_t numAxis;
                    size_t numTrajPoints;
                    numAxis = inputPoint.joint_names.size();
                    numTrajPoints = inputPoint.points.size();
                    if(numAxis != 6)
                    {
                        printf("expect six Axises\n");
                    }
                    else if(numTrajPoints != 1 )
                    {
                        printf("expect data transition step by step\n");
                    }
                    else
                    {
                        cmdAxisPosOne.push_back(inputPoint.points[0].positions[0]);
                        cmdAxisPosTwo.push_back(inputPoint.points[0].positions[1]);
                        cmdAxisPosThree.push_back(inputPoint.points[0].positions[2]);
                        cmdAxisPosFour.push_back(inputPoint.points[0].positions[3]);
                    }
                }

                if(cmdAxisPosOne.size() > 50 && cmdAxisPosTwo.size() > 50 &&  cmdAxisPosThree.size() > 50 && cmdAxisPosFour.size() > 50)
                {
                        scaraclientOne.sendCmdToMinas(cmdAxisPosOne.at(0));
                        scaraclientTwo.sendCmdToMinas(cmdAxisPosTwo.at(0));
                        scaraclientThree.sendCmdToMinas(cmdAxisPosThree.at(0));
                        scaraclientFour.sendCmdToMinas(cmdAxisPosFour.at(0));

                        cmdAxisPosOne.pop_front();
                        cmdAxisPosTwo.pop_front();
                        cmdAxisPosThree.pop_front();
                        cmdAxisPosFour.pop_front();
                }
                clock_gettime(CLOCK_REALTIME, &after);
                struct timespec sleepTime;
                sleepTime.tv_sec = after.tv_sec - before.tv_sec;
                sleepTime.tv_nsec = 4000000 - after.tv_nsec + before.tv_nsec;  // 4ms
                if(nanosleep(&sleepTime, NULL) != 0)
                    perror("nanosleep error....\n");
                //double overrun_time = (double(after.tv_sec) * NSEC_PER_SECOND + double(after.tv_nsec)) -  (double(before.tv_sec) * NSEC_PER_SECOND + double(before.tv_nsec));


            }
            else if( s_MODE == MODE_SPLINE )
            {
                // clear all the previous data
                if(clearAllData)
                {
                    cmdAxisPosOne.clear();
                    cmdAxisPosTwo.clear();
                    cmdAxisPosThree.clear();
                    cmdAxisPosFour.clear();
                }
                 while( inputFlag != INPUT_FINISH )
                 {
                    ros::spinOnce();
                    if(TRAJECTORY_HANDLLER.newMsg())   // 必须有个读取数据的flag 要不最后一个数据一直是一样的
                    {
                        trajectory_msgs::JointTrajectory inputTra = TRAJECTORY_HANDLLER.fetchMsg();
                        size_t numAxis;
                        size_t numTrajPoints;
                        numAxis = inputTra.joint_names.size();
                        numTrajPoints = inputTra.points.size();
                        if(numAxis != 6)
                        {
                            printf("expect six Axises\n");
                        }
                        else if(numTrajPoints != 1 )
                        {
                              printf("expect data transition step by step\n");
                        }
                        else
                        {
                            // 安装定义 解析 其实发很多数据也可以
                            cmdAxisPosOne.push_back(inputTra.points[0].positions[0]);
                            cmdAxisPosTwo.push_back(inputTra.points[0].positions[1]);
                            cmdAxisPosThree.push_back(inputTra.points[0].positions[2]);
                            cmdAxisPosFour.push_back(inputTra.points[0].positions[3]);
                        }
                        LAST_INPUT_TIME = ros::Time::now().toSec() ;
                        INPUT_FINISH = 0;
                    }
                    if(ros::Time::now().toSec()  - LAST_INPUT_TIME> MinimalInputTime)    // 10ms
                    {
                        INPUT_FINISH = 1;   // receive data finish, jump out of loop
                        // log in again until INPUT_FINISH = 0
                        clock_gettime(CLOCK_REALTIME, &before);
                    }
                 }


                if(cmdAxisPosOne.size() > 0 && cmdAxisPosTwo.size() > 0 &&  cmdAxisPosThree.size() > 0 && cmdAxisPosFour.size() > 0 )
                {
                        // 下发命令 永远下发第一个值
                    scaraclientOne.sendCmdToMinas(cmdAxisPosOne.at(0));
                    scaraclientTwo.sendCmdToMinas(cmdAxisPosTwo.at(0));
                    scaraclientThree.sendCmdToMinas(cmdAxisPosThree.at(0));
                    scaraclientFour.sendCmdToMinas(cmdAxisPosFour.at(0));
                        // 踢出第一个值 将下个命令设为第一个值
                    cmdAxisPosOne.pop_front();
                    cmdAxisPosTwo.pop_front();
                    cmdAxisPosThree.pop_front();
                    cmdAxisPosFour.pop_front();
                        /*差一个实时的时间补偿 比如 usleep 或者 nanosleep*/
                    clearAllData = 0;
                }
                else
                {
                    clearAllData = 1;
                    INPUT_FINISH = 0;   // waiting for receiving
                }
                clock_gettime(CLOCK_REALTIME, &after);
                sleepTime.tv_sec = after.tv_sec - before.tv_sec;
                sleepTime.tv_nsec = 4000000 - after.tv_nsec + before.tv_nsec;
                if(nanosleep(&sleepTime, NULL) != 0)
                    perror("nanosleep error....\n");
            }

            else if( s_MODE == MODE_STOP )
            {

            }
            else
            {
                printf("Waiting for the command and data....\n");
            }
            //clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
        }

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
    subscriber_joint_path.shutdown();
    subscriber_command.shutdown();
    //ros::waitForShutdown();
    //close servo
    scaraclientOne.closeServo(clientAxisOne);
    scaraclientTwo.closeServo(clientAxisTwo);
    scaraclientThree.closeServo(clientAxisThree);
    scaraclientFour.closeServo(clientAxisFour);

  printf("End program\n");

	return 0;

}
