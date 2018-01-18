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
    ros::Subscriber subscriber_joint_path;

    //ros::Subscriber subscriber_command = NODE.subscribe<std_msgs::String> (INTERFACE_COMMAND, 1024, &StringHandller::callback, &COMMAND_HANDLLER );
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

    motorParaInit(motorParaAxisOne, motorParaAxisTwo, motorParaAxisThree, motorParaAxisFour);

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

    /************************************************************/
    ROS_INFO ( "Main loop on the go" );
    int inputFlag = 1;
    //std::deque< trajectory_msgs::JointTrajectory > inputTra;
    while(ros::ok())
    {
        //input data
        while (inputFlag != INPUT_FINISH)
        {
            subscriber_joint_path = nh.subscribe<trajectory_msgs::JointTrajectory> (INTERFACE_JOINT_PATH_COMMAND, 1024, &JointTrajectoryHandller::callback, &TRAJECTORY_HANDLLER );
            if(TRAJECTORY_HANDLLER.newMsg())
            {
                trajectory_msgs::JointTrajectory input = TRAJECTORY_HANDLLER.fetchMsg();
                LAST_INPUT_TIME = ros::Time::now().toSec() ;
                INPUT_FINISH = 0;

            }
            if(ros::Time::now().toSec()  - LAST_INPUT_TIME> MinimalInputTime)    // SEC OR NANOSEC  ????
            {
                INPUT_FINISH = 1;   // receive data finish, jump out of loop
            }
        }

        scaraclientOne.sendCmdToMinas(0);
        scaraclientTwo.sendCmdToMinas(0);
        scaraclientThree.sendCmdToMinas(0);
        scaraclientFour.sendCmdToMinas(0);
    }
	return 0;

}
