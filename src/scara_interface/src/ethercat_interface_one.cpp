/*************************************************************************
	> File Name: ethercat_interface_test.cpp
	> Author: Likun Wang
	> Mail: likunwang@hit.edu.cn
	> Created Time: 2018年01月10日 星期三 23时30分17秒
 ************************************************************************/

#include "ethercat_interface.h"

int main(int argc, char *argv[])
{
    //int aaaa[4];
    //int aaaa[4] = {0,0,0,0};
    std::vector <int > fault(4, 0);
    std::vector <int> so(4,0);
    int SI_MON4;
    int SI_MON5;
    /*
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

    std::vector<std::string> j_names ;
    std::string j_name1 = "joint1";
    std::string j_name2 = "joint2";
    std::string j_name3 = "joint3";
    std::string j_name4 = "joint4";
    j_names.push_back(j_name1);
    j_names.push_back(j_name2);
    j_names.push_back(j_name3);
    j_names.push_back(j_name4);
    std::vector<double> jointPosition ;
    double reduction[4] = {50,50, 2*2*3.14159/0.02, 20};


    ROS_INFO ( "Main loop on the go" );
    int inputFlag = 1;
    int stopFlag  = 0;
    int clearAllData = 1;
    ////
    int testnum = 4;
    /////
    subscriber_joint_path = nh.subscribe<trajectory_msgs::JointTrajectory> (INTERFACE_JOINT_PATH_COMMAND, 1024, &JointTrajectoryHandller::callback, &TRAJECTORY_HANDLLER );
    subscriber_command = nh.subscribe<std_msgs::String> (INTERFACE_COMMAND, 1024, &StringHandller::callback, &COMMAND_HANDLLER );
    subscriber_servo_command = nh.subscribe<std_msgs::String>(INTERFACE_SERVO_COMMAND, 1024, &StringHandller::callback, &SERVO_COMMAND_HANDLLER);   //   LG
    subscriber_alarm_command = nh.subscribe<std_msgs::String>(INTERFACE_ALARM_CLEAR, 1024, &StringHandller::callback, &ALARM_COMMAND_HANDLLER);        //   LG

    pJointStates = nh.advertise<sensor_msgs::JointState> (INTERFACE_JOINT_STATES, 1024 );
    driveStates = nh.advertise<std_msgs::String> (INTERFACE_DRIVE_STATE, 1024 );

    int operation_mode = 0x08;  //csp
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

    printf("initial ethercat framework....\n");
    for(int j =0; j< testnum; j++)
    {
        clients[j] = new minas_control::MinasClient(manager, j+1);
        memset(&output[j], 0x00, sizeof(minas_control::MinasOutput));
        printf("reset begin....%d\n", j);
        output[j].operation_mode = operation_mode;
        output[j].controlword = 0x0080;
        clients[j]->writeOutputs(output[j]);
        //clients[j]->reset();
        clients[j]->setTrqueForEmergencyStop(torqueForEmergencyStopVec[j] ); // 100%
        clients[j]->setOverLoadLevel(overloadLevelVec[j] );          // 50%
        clients[j]->setOverSpeedLevel(overSpeedLevelVec[j] );        // r/min
        clients[j]->setMotorWorkingRange(motorWorkingRangeVec[j] );     // 0.1
        clients[j]->setInterpolationTimePeriod(interpolationTimePeriodVec[j] );     // 4 msec
        printf("reset done....%d\n", j);
        clients[j]->servoOn();
        printf("Axis %d servoOn done....%d\n", j + 1);
        input[j] = clients[j]->readInputs();
        current_position[j] = input[j].position_actual_value;

        
        output[j].target_position = current_position[j];
        output[j].max_motor_speed = max_motor_speedVec[j];  // rad/min
        output[j].target_torque = target_torqueVec[j];    // 0% (unit 0.1%)
        output[j].max_torque    = max_torqueVec[j];    // 50% (unit 0.1%)
        output[j].controlword   = controlwordVec[j]; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5) p77
        output[j].operation_mode = operation_mode;
        clients[j]->setProfileVelocity(profileVelVec[j]);
        output[j].controlword   &= ~0x0010; // clear new-set-point (bit4)
        clients[j]->writeOutputs(output[j]);
    /*
        if(j == 2)
        {
            output[j].digital_outputs = 0x00000000;
        }
        */
        clients[j]->writeOutputs(output[j]);

        while ( ! (input[j].statusword & 0x1000) )
        {// bit12 (set-point-acknowledge)  p79
            input[j] = clients[j]->readInputs();
        }
        
        printf("target position = %08x\n", output[j].target_position);
        usleep(100000);
    }
    
    std::vector<int > ro;
    while(ros::ok)
    {
        clock_gettime(CLOCK_REALTIME, &tick);
        timespecInc(tick, period);
        /*
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


        }*/
        

        for (int i = 0; i <= (2*3.14*20000); i++ )
        {
            //input[0] = clients[0]->readInputs();
            //printf(" 60FDh %08x :Digital inputs\n", input[0].digital_inputs);

        //if(i%10 == 0)
        
            for (int j =0; j< testnum; j++)
            {
                input[j] = clients[j]->readInputs();
                int32 temp_SW = input[j].statusword;
                int32 temp_DI = input[j].digital_inputs;

                //aaaa[j] = j;
                //ro.push_back( int ( (temp_SW && 0x0003) >> 1) );
                printf("the loop count is %d \n", i);
                fault.at(j)=int ((temp_SW & 0x000F) >> 3);
                so.at(j) = int ( (temp_SW & 0x0003) >> 1);
                if(j == 0)
                {
                    SI_MON4 = int ((temp_DI & 0x007FFFFF) >> 22);
                    SI_MON5 = int ((temp_DI & 0x00FFFFFF) >>23);
                }
                /*f[j] = int ((temp_SW && 0x000F) >> 3);
                ve[j] = int ((temp_SW && 0x001F) >>4);
                qs[j] = int ((temp_SW && 0x003F) >> 5);
                w[j] = int ((temp_SW && 0x00FF) >> 7);
                sod[j] = int ((temp_SW && 0x007F) >> 6);
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
                SI_MON4[j] = int ((temp_DI && 0x007FFFFF) >> 22);
                SI_MON5[j] = int ((temp_DI && 0x00FFFFFF) >>23);
                INP[j]          = int ((temp_DI && 0x01FFFFFF) >> 24);*/

                

                int counts = (int)input[j].position_actual_value;
                printf("counts is %d \n", counts);
                double temp1 = ( (double) counts / int(0x20000) ) / reduction[j] ;// /3.14159265 *180;
                //  printf("temp is %f \n", temp1);
                // double temp2= (temp1/(2 * 3.1415926 * 50)) * 360;
                jointPosition.push_back (temp1);
                //printf("Input:\n");
                //printf(" 603Fh %08x :Error code\n", input[j].error_code);
                //printf(" 6041h %08x :Statusword\n", input[j].statusword);
                //printf(" 6061h %08x :Modes of operation display\n", input[j].operation_mode);
                //printf(" 6064h %08x :Position actual value\n", input[j].position_actual_value);
                //printf(" 606Ch %08x :Velocity actual value\n", input[j].velocity_actual_value);
                //printf(" 6077h %08x :Torque actual value\n", input[j].torque_actual_value);
                //printf(" 60B9h %08x :Touch probe status\n", input[j].touch_probe_status);
                //printf(" 60BAh %08x :Touch probe pos1 pos value\n", input[j].touch_probe_posl_pos_value);
                printf(" 60FDh %08x :Digital inputs\n", input[j].digital_inputs);

            }
        //}
            jointState.header.stamp = ros::Time::now();
            jointState.name = j_names;
            jointState.position = jointPosition;
            pJointStates.publish(jointState);

            jointPosition.clear();
            
            int faultSent = fault.at(0) & fault.at(1) & fault.at(2) & fault.at(3);
            printf("judge = %d \n", faultSent);
            int swithedOn = so.at(0) & so.at(1) & so.at(2) & so.at(3);
            printf("swithedOn = %d \n", swithedOn);
            printf("SI_MON4 = %d \n", SI_MON4);
            printf("SI_MON5 = %d \n", SI_MON5);


            for(int k = 0; k <4; k++)
            {
                input[k] = clients[k]->readInputs();
                output[k] = clients[k]->readOutputs();

                //printf("sending command....\n");
                output[k].position_offset =  20*0.00069813*0x20000*(i);//reduction * circles * 2pi* 0x20000
                //std::cout <<  output[k].position_offset  << std::endl;
                //output[k].position_offset = 0x200000*sin(i/200.0);
                clients[k]->writeOutputs(output[k]);
            }
            timespecInc(tick, period);
            //printf("checking overrun....\n");
            struct timespec before;
            clock_gettime(CLOCK_REALTIME, &before);
            double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -  (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
            if (overrun_time > 0.0)
            {
                fprintf(stderr, "  overrun: %f", overrun_time);
            }

            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);   //4ms
        }
        //usleep(4000);
   }
    for(int ii =0; ii < 1; ii++)
    {
        /*
        if(ii == 2)
        {
            output[ii].digital_outputs = 0x00000001;
            clients[ii]->writeOutputs(output[ii]);
        }
        */
        clients[ii]-> printPDSStatus(input[ii]);
        clients[ii]->printPDSOperation(input[ii]);
        clients[ii]->servoOff();
        delete clients[ii];

    }
    printf("End program\n");

    return 0;
}




