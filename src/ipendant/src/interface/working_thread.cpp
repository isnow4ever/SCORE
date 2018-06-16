/*
* License:The MIT License (MIT)
*
* Copyright (c) 2013,2014 Yongzhuo Gao
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

#include "working_thread.h"

class JointStateHandler
{
  public:
    void callback(const sensor_msgs::JointState::ConstPtr &message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    JointStateHandler() : mCrtMsg(), mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const sensor_msgs::JointState &fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

    sensor_msgs::JointState mCrtMsg;

  protected:
    bool mNewMsg;
};

class StringMsgsHandler
{
  public:
    void callback(const std_msgs::String::ConstPtr &message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    StringMsgsHandler() : mCrtMsg(), mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const std_msgs::String &fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

    std_msgs::String mCrtMsg;

  protected:
    bool mNewMsg;
};

//Class ROS_Communication
ROS_Communication::ROS_Communication()
    : project(new Project),
      project_handler(new ProjectHandler),
      states(new State),
      params(new Parameter),
      controller_states(new Controller_State),
      interpreters(new Interpreter)
{
    //params init
    params->id = 0;
    states->imode = STANDBY;
    states->jogmode = 0;
    states->joystick_ctrl = FIRST_THREE;
    states->enable_value = 1;
    prev_enable = states->enable_value;
    states->playback_mode = 0;
    states->new_userframe = 0;
    states->new_userframe_is_ready = 0;
    for (int i = 0; i < 6; i++)
    {
        states->calibration_disabled_state.push_back(true);
        states->cartesian_disabled_state.push_back(true);
        states->joint_disabled_state.push_back(true);

        states->joint_workspace_state.push_back(0);
        states->cartesian_workspace_state.push_back(0);

        params->joint_state.push_back(0);
        params->cartesian_state.push_back(0);
        params->cartesian_state_in_U.push_back(0);
        params->value_for_joint.push_back(0);
        params->value_for_cartesian.push_back(0);
        prev_value.push_back(0.00);
    }
    for (int j = 0; j < 128; j++)
        states->iostate[j] = -1;

    states->time_modified = 0;

    //joint_limit
    params->joint_limit_min_position.push_back(LIMIT_MIN_POSITION_JOINT_1);
    params->joint_limit_min_position.push_back(LIMIT_MIN_POSITION_JOINT_2);
    params->joint_limit_min_position.push_back(LIMIT_MIN_POSITION_JOINT_3);
    params->joint_limit_min_position.push_back(LIMIT_MIN_POSITION_JOINT_4);
    params->joint_limit_min_position.push_back(LIMIT_MIN_POSITION_JOINT_5);
    params->joint_limit_min_position.push_back(LIMIT_MIN_POSITION_JOINT_6);
    params->joint_limit_min_position.push_back(LIMIT_MIN_POSITION_JOINT_2_AND_3);

    params->joint_limit_max_position.push_back(LIMIT_MAX_POSITION_JOINT_1);
    params->joint_limit_max_position.push_back(LIMIT_MAX_POSITION_JOINT_2);
    params->joint_limit_max_position.push_back(LIMIT_MAX_POSITION_JOINT_3);
    params->joint_limit_max_position.push_back(LIMIT_MAX_POSITION_JOINT_4);
    params->joint_limit_max_position.push_back(LIMIT_MAX_POSITION_JOINT_5);
    params->joint_limit_max_position.push_back(LIMIT_MAX_POSITION_JOINT_6);
    params->joint_limit_max_position.push_back(LIMIT_MAX_POSITION_JOINT_2_AND_3);

    controller_states->robot_state = false;
}

ROS_Communication::~ROS_Communication()
{
    delete playback;
    delete extraCmd;
}

void ROS_Communication::run()
{
    ros::NodeHandle n;

    playback = new Playback;
    QObject::connect(playback, SIGNAL(project_states(int)), this, SIGNAL(project_states(int)));
    QObject::connect(playback, SIGNAL(updateGUI(int)), this, SIGNAL(updateGUI(int)));
    extraCmd = new ExtraCommand;

    // StatesIndicator Initialize.
    StatesIndicator *comState = new StatesIndicator;
    connect(comState, SIGNAL(indicatorON()), this, SIGNAL(comON()));
    connect(comState, SIGNAL(indicatorOFF()), this, SIGNAL(comOFF()));

    StatesIndicator *enableState = new StatesIndicator;
    connect(enableState, SIGNAL(indicatorOFF()), this, SIGNAL(manualEnableON()));
    connect(enableState, SIGNAL(indicatorON()), this, SIGNAL(manualEnableOFF()));

    StatesIndicator *robotState = new StatesIndicator;
    connect(robotState, SIGNAL(indicatorON()), this, SIGNAL(isMovingState()));
    connect(robotState, SIGNAL(indicatorOFF()), this, SIGNAL(isStableState()));

    StatesIndicator *autoMode = new StatesIndicator;
    connect(autoMode, SIGNAL(indicatorON()), this, SIGNAL(autoModeON()));
    connect(autoMode, SIGNAL(indicatorOFF()), this, SIGNAL(autoModeOFF()));
    autoMode->getState(controller_states->auto_state);

    // Subscribers Initialize.
    JointStateHandler joint_state_handler;
    StringMsgsHandler controller_state_handler;
    StringMsgsHandler joystick_handler;
    StringMsgsHandler keypad_handler;
    StringMsgsHandler io_state_handler;
    // StringMsgsHandler time_modify_handler;
    StringMsgsHandler driver_state_handler;

    ros::Subscriber coordinator_states_sub = n.subscribe("coordinator_states", 2, &StringMsgsHandler::callback, &controller_state_handler);
    ros::Subscriber joint_states_sub = n.subscribe("joint_states", 2, &JointStateHandler::callback, &joint_state_handler);
    ros::Subscriber joystick_sub = n.subscribe("u_pendant/devices/joystick", 50, &StringMsgsHandler::callback, &joystick_handler);
    ros::Subscriber keypad_sub = n.subscribe("u_pendant/devices/keypad", 5, &StringMsgsHandler::callback, &keypad_handler);
    ros::Subscriber io_states_sub = n.subscribe("io_states", 2, &StringMsgsHandler::callback, &io_state_handler);
    //ros::Subscriber time_modify_sub = n.subscribe("mtime", 1000, &StringMsgsHandler::callback, &time_modify_handler);
    ros::Subscriber time_modify_sub = n.subscribe("/driver_state", 1000, &StringMsgsHandler::callback, &driver_state_handler);

    tf::TransformListener listener;

    Actor actors;
    actors.command_pub = n.advertise<std_msgs::String>("coordinator_command", 100);

    actors.userFrame_pub = n.advertise<std_msgs::String>("coordinator_user_frame", 1);

    actors.u_devices_pub = n.advertise<std_msgs::String>("u_pendant/devices/control", 1);

    actors.connection_feedback = n.advertise<std_msgs::String>("/tp_com/rx", 1);

    /*****************************************************************************
    ** Parameters Initialize
    *****************************************************************************/
    ros::Rate loop_rate(COM_LOOP_RATE);

    /*****************************************************************************
    ** Cycle Begins
    *****************************************************************************/

    while (ros::ok())
    {
        /*********************
            ** Com Check
            **********************/
        do
        {
            comState->getState(ros::master::check());
        } while (!ros::master::check());

        std_msgs::String tp_com;
        std::stringstream ssss;
        ssss << "1";
        tp_com.data = ssss.str();
        actors.connection_feedback.publish(tp_com);

        /*********************
            ** Subscribe
            **********************/

        ros::spinOnce();
        /*
        // 0. Handle time modify Msgs.
        if(!states->time_modified)
        {
            if(time_modify_handler.newMsg())
            {
                string setdate = "sudo date -s \"" + time_modify_handler.fetchMsg().data + "\"";
                system(setdate.c_str());
                ROS_INFO("time has been modified: [%s]", time_modify_handler.fetchMsg().data.c_str());
                states->time_modified = 1;

                //    time_t now;
                //    tm *timenow;

                //    time(&now);
                //    timenow = localtime(&now);
                //    QString t;
                //    t.append("\"");
                //    t.append(QString::number(timenow->tm_year + 1900) + "-");
                //    t.append(QString::number(timenow->tm_mon + 1) + "-");
                //    t.append(QString::number(timenow->tm_mday) + " ");
                //    t.append(QString::number(timenow->tm_hour) + ":");
                //    t.append(QString::number(timenow->tm_min) + ":");
                //    t.append(QString::number(timenow->tm_sec) + "\"");

                //    ROS_INFO("recent time is : %s", asctime(timenow));
                //    ROS_INFO("recent time is : %s", t.toStdString().c_str());
            }
            else
            {
                ROS_WARN("No time info.");
            }
        }
*/
        // 1.Handle Cartesian States Msgs.
        tf::StampedTransform transform;
        try
        {
            //            listener.waitForTransform("/root", "/tool",
            //                                      ros::Time (0), ros::Duration(10.0));
            listener.lookupTransform("/base_link", "/link3",
             ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        params->cartesian_state[0] = transform.getOrigin().x() * 1000;
        params->cartesian_state[1] = transform.getOrigin().y() * 1000;
        params->cartesian_state[2] = transform.getOrigin().z() * 1000;
        transform.getBasis().getRPY(params->roll, params->pitch, params->yaw);
        params->cartesian_state[3] = params->roll * 180 / PI;
        params->cartesian_state[4] = 0;//params->pitch * 180 / PI;
        params->cartesian_state[5] = 0;//params->yaw * 180 / PI;

        if (states->new_userframe)
        {
            tf::StampedTransform transform_in_U;
            try
            {
                states->new_userframe_is_ready = listener.waitForTransform("root", "tool",
                                                                           ros::Time(0), ros::Duration(10.0));
                listener.lookupTransform("user", "tool",
                                         ros::Time(0), transform_in_U);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }
            if (states->new_userframe_is_ready)
            {
                params->cartesian_state_in_U[0] = transform_in_U.getOrigin().x() * 1000;
                params->cartesian_state_in_U[1] = transform_in_U.getOrigin().y() * 1000;
                params->cartesian_state_in_U[2] = transform_in_U.getOrigin().z() * 1000;
                transform_in_U.getBasis().getRPY(params->roll, params->pitch, params->yaw);
                params->cartesian_state_in_U[3] = params->roll * 180 / PI;
                params->cartesian_state_in_U[4] = 0;//params->pitch * 180 / PI;
                params->cartesian_state_in_U[5] = 0;//params->yaw * 180 / PI;
            }
        }

        // 2.  Handle Joint States Msgs.
        if (joint_state_handler.newMsg())
        {
            params->joint_state[0] = (joint_state_handler.mCrtMsg.position[0]) * 180 / PI;
            params->joint_state[1] = (joint_state_handler.mCrtMsg.position[1]) * 180 / PI;
            params->joint_state[2] = (joint_state_handler.mCrtMsg.position[2]) * 1000;
            params->joint_state[3] = (joint_state_handler.mCrtMsg.position[3]) * 180 / PI;
            
            params->joint_state[4] = 0;
            params->joint_state[5] = 0;
        }

        // 3. Handle Controller States Msgs.
        if (driver_state_handler.newMsg())
        {
            std::string msg = driver_state_handler.fetchMsg().data;
            std::vector<std::string> result;
            boost::algorithm::split(result, msg, boost::algorithm::is_any_of(" "), boost::algorithm::token_compress_on);
  
            ROS_INFO("driver states: %s.", msg.c_str());
            controller_states->robot_state = QString::fromStdString(result.at(0)).toInt();
            controller_states->servo_state = QString::fromStdString(result.at(1)).toInt();
            controller_states->error_state = QString::fromStdString(result.at(2)).toInt();
            controller_states->auto_state = QString::fromStdString(result.at(3)).toInt();
        }
        if (controller_state_handler.newMsg())
        {
            boost::regex reg(STATE_FORMAT);
            boost::smatch mat;
            std::string cmd = controller_state_handler.fetchMsg().data;
            bool r = boost::regex_match(cmd, mat, reg);
            if (r)
            {
                controller_states->_robot_state = mat[1];
                controller_states->current_id = mat[2];
                controller_states->action_result = mat[3];

                // if (controller_states->_robot_state == "moving")
                //     controller_states->robot_state = true;
                // else
                //     controller_states->robot_state = false;
            }
            else
            {
                ROS_WARN("unknown robot state!");
            }
        }

        // 4. Handle Joystick Signals Msgs.
        if (joystick_handler.newMsg())
        {
            std::string msg = joystick_handler.fetchMsg().data;
            std::vector<std::string> result;
            boost::algorithm::split(result, msg, boost::algorithm::is_any_of(" "), boost::algorithm::token_compress_on);

            params->value_for_cartesian[0] = QString::fromStdString(result.at(0)).toDouble();
            params->value_for_cartesian[1] = QString::fromStdString(result.at(1)).toDouble();
            params->value_for_cartesian[2] = QString::fromStdString(result.at(2)).toDouble();
            params->value_for_cartesian[3] = QString::fromStdString(result.at(2)).toDouble();
            params->value_for_cartesian[4] = 0;//QString::fromStdString(result.at(1)).toDouble();
            params->value_for_cartesian[5] = 0;//QString::fromStdString(result.at(2)).toDouble();

            params->value_for_joint[0] = QString::fromStdString(result.at(2)).toDouble();
            params->value_for_joint[1] = QString::fromStdString(result.at(1)).toDouble();
            params->value_for_joint[2] = QString::fromStdString(result.at(0)).toDouble();
            params->value_for_joint[3] = QString::fromStdString(result.at(2)).toDouble();
            params->value_for_joint[4] = 0;//QString::fromStdString(result.at(1)).toDouble();
            params->value_for_joint[5] = 0;//QString::fromStdString(result.at(2)).toDouble();

            states->enable_value = atoi(result[3].c_str());
        }
        /*
        // 5. Handle Keypad Signals Msgs.
        if(keypad_handler.newMsg())
        {
            std::string input = keypad_handler.fetchMsg().data;
            char key[2];
            strcpy(key, input.c_str());
            switch (key[0]) {
            case 'A': ROS_DEBUG("D");
                break;
            case 'B': ROS_DEBUG("B");
                break;
            case 'C': ROS_DEBUG("C");
                break;
            case 'D': ROS_DEBUG("A");
                break;
            case '1':
                ROS_DEBUG("3");
                keyfunc(3);
                break;
            case '2':
                ROS_DEBUG("2");
                keyfunc(2);
                break;
            case '3':
                ROS_DEBUG("4");
                keyfunc(4);
                break;
            case '4':
                ROS_DEBUG("1");
                keyfunc(1);
                break;
            case 'O':
                ROS_DEBUG("Ok");
                joystickChange();
                break;
            case 'N': playbackStepControl(3); //next
                break;
            case 'P': playbackStepControl(2); //prev
                break;
            case 'G': playbackStepControl(1); //step
                break;
            case 'S': playbackStepControl(0); //stop
                break;
            case 'R': playbackStepControl(4);//send to GUI
                break;
            default: ROS_WARN("Unknown Key!");
                break;
            }
            key[0] = '0';                   //clear key values and return to loop
        }
*/
        // 6. Handle I/O States Msgs.
        if (io_state_handler.newMsg())
        {
            if (io_state_handler.fetchMsg().data.length() != 32)
            {
                ROS_WARN("Wrong io_states info.");
                for (int j = 0; j < 128; j++) // new add
                    states->iostate[j] = -1;
            }
            else
            {
                std::string io_states_hex = io_state_handler.fetchMsg().data;
                for (int i = 0; i < 32; i++)
                {
                    QString io_states_bin = io_decode(io_states_hex.at(i));
                    for (int j = 0; j < 4; j++)
                    {
                        if (io_states_bin.toStdString().at(j) == '0')
                        {
                            states->iostate[i * 4 + j] = 0;
                        }
                        else
                        {
                            states->iostate[i * 4 + j] = 1;
                        }
                    }
                }
            }
        }

        // 7. Send Robot States to GUI.
        update_robot_states();

        /*********************
            ** Publish
            **********************/
        std_msgs::String msg;
        std::stringstream ss;

        //Finite State Machine.
        switch (states->imode)
        {

        // 1. STANDBY MODE
        case STANDBY:
            break;

        // 2. MANUAL MODE
        case MANUAL:
            //            if(states->enable_value != prev_enable)
            //            {
            //                if(states->enable_value)
            //                {
            //                    ss << "<" << params->id << "> " << "STOP";
            //                    msg.data = ss.str();
            //                    ROS_INFO("%s", msg.data.c_str());
            //                    actors.command_pub.publish(msg);
            //                    prev_enable = states->enable_value;
            //                    break;
            //                }
            //                else if(states->enable_value == 0)
            //                {
            //                    ss << "<" << params->id << "> " << "DIRECT";
            //                    msg.data = ss.str();
            //                    ROS_INFO("%s", msg.data.c_str());
            //                    actors.command_pub.publish(msg);
            //                    prev_enable = states->enable_value;
            //                    break;
            //                }
            //            }

            enableState->getState(states->enable_value); //Enable state check
            get_axis_states(*params, *states);
            get_jogmode_states(*params, *states);
            get_joystickCtrl_states(*params, *states);

            ss << fixed << setprecision(2);

            // 2.1 JOINT FRAME
            if (states->frame == JOINT) // Frame: Joint
            {
                /*		for(int i = 0; i < 6; i++)
                {
                    if(fabs(joint_value[i] - prev_value.at(i)) > 2.0)
                    {
                        joint_value[i] = prev_value.at(i)
                                + 0.5 * (joint_value[i] - prev_value.at(i));
                    }
                    else if((fabs(joint_value[i] - prev_value.at(i)) <= 2.0)&&(fabs(joint_value[i] - prev_value.at(i)) > 0.5))
                    {
                        joint_value[i] = prev_value.at(i)
                                + 0.5 * (joint_value[i] - prev_value.at(i)) / fabs(joint_value[i] - prev_value.at(i));
                    }
		    prev_value.at(i) = joint_value[i];
                }   */
                if (states->enable_value == 0)
                {
                    ss << "<" << params->id << "> " << params->method << " {" << params->target << "} ";
                    ss << joint_value[0] / 20.0 << " ";
                    ss << joint_value[1] / 20.0 << " ";
                    ss << joint_value[2] / 20.0 << " ";
                    ss << joint_value[3] / 20.0 << " ";
                    ss << joint_value[4] / 20.0 << " ";
                    ss << joint_value[5] / 20.0 << " ";
                    ss << "(mm,deg,s) {" << params->frame << "}";

                    msg.data = ss.str();
                    ROS_INFO("%s", msg.data.c_str());
                    actors.command_pub.publish(msg);
                }
            }
            // 2.2 TOOL FRAME
            else //Frame: Tool, Base, User
            {
                /*              for(int i = 0; i < 6; i++)
                {
                    if(fabs(cartesian_value[i] - prev_value.at(i)) > 2.0)
                    {
                        cartesian_value[i] = prev_value.at(i)
                                + 0.5 * (cartesian_value[i] - prev_value.at(i));
                    }
                    else if((fabs(cartesian_value[i] - prev_value.at(i)) <= 2.0)&&(fabs(cartesian_value[i] - prev_value.at(i)) > 0.5))
                    {
                        cartesian_value[i] = prev_value.at(i)
                                + 0.5 * (cartesian_value[i] - prev_value.at(i)) / fabs(cartesian_value[i] - prev_value.at(i));
                    }
		    prev_value.at(i) = cartesian_value[i];
                }*/
                if (states->enable_value == 0)
                {
                    ss << "<" << params->id << "> " << params->method << " {" << params->target << "} ";
                    ss << cartesian_value[0] / 5.0 << " ";
                    ss << cartesian_value[1] / 5.0 << " ";
                    ss << cartesian_value[2] / 5.0 << " ";
                    ss << -cartesian_value[4] / 20.0 << " ";
                    ss << -cartesian_value[5] / 20.0 << " ";
                    ss << -cartesian_value[3] / 20.0 << " ";
                    ss << "(mm,deg,s) {" << params->frame << "}";

                    msg.data = ss.str();
                    ROS_INFO("%s", msg.data.c_str());
                    actors.command_pub.publish(msg);
                }
            }
            break;

        // 3. PLAYBACK MODE
        case PLAYBACK:
            //  TRIAL&STEP&CYCLE
            playback->process(project, project_handler, states, controller_states, params, interpreters, actors);
            break;

        // 4. CALIBRATION MODE
        case CALIBRATION:
            ss << "<" << params->id << "> "
               << "Calibrate {JOINTS} ";
            ss << states->calibration_disabled_state[0] << " ";
            ss << states->calibration_disabled_state[0] << " ";
            ss << states->calibration_disabled_state[0] << " ";
            ss << states->calibration_disabled_state[0] << " ";
            ss << states->calibration_disabled_state[0] << " ";
            ss << states->calibration_disabled_state[0] << " ";
            ss << "(mm,deg,s) {J}";

            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            actors.command_pub.publish(msg);

            states->imode = STANDBY;

            break;

        // 5. EXTRACMD MODE
        case EXTRACMD:
            if (extraCmd->newCmd())
            {
                ss << "<" << params->id << "> ";
                ss << extraCmd->fetchCmd();

                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str());
                actors.command_pub.publish(msg);
                states->imode = STANDBY;
            }
            break;

        // 6. USER_FRAME_BR MODE
        case USER_FRAME_BR:
            if (extraCmd->newCmd())
            {
                ss << extraCmd->fetchCmd();

                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str());
                actors.userFrame_pub.publish(msg);
                states->imode = STANDBY;
            }
            break;

        // 6. USER_FRAME_BR MODE
        case DEVICE_CONTROL:
            if (extraCmd->newCmd())
            {
                ss << extraCmd->fetchCmd();

                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str());
                actors.u_devices_pub.publish(msg);
                states->imode = STANDBY;
            }
            break;

        default:
            ROS_WARN("Wrong imode!");
            states->imode = STANDBY;
            break;
        }

        /*********************
            ** Indicators and Warnings
            **********************/

        //Robot moving state notification
        robotState->getState(controller_states->robot_state);
        autoMode->getState(controller_states->auto_state);

        //Robot beyond workspace notification
        workspaceNotification(*params, *states);

        if (params->id == 99999)
            params->id = 0;
        else
            params->id++;

        loop_rate.sleep();
    }
}

void ROS_Communication::get_axis_states(Parameter &params, State &states)
{
    for (int i = 0; i < 4; i++)
    {
        if (states.joint_disabled_state[i] == true)
        {
            if (states.joint_workspace_state[i] == 0)
            {
                joint_value[i] = double(params.value_for_joint[i]);
            }
            else if (states.joint_workspace_state[i] == 1)
            {
                joint_value[i] = (params.value_for_joint[i] < 0) ? double(params.value_for_joint[i]) : 0;
            }
            else if (states.joint_workspace_state[i] == -1)
            {
                joint_value[i] = (params.value_for_joint[i] > 0) ? double(params.value_for_joint[i]) : 0;
            }
        }
        else
            joint_value[i] = 0;
    }

    for (int i = 0; i < 4; i++)
    {
        if (states.cartesian_disabled_state[i] == true)
        {
            if (states.cartesian_workspace_state[i] == 0)
            {
                cartesian_value[i] = double(params.value_for_cartesian[i]);
            }
            else if (states.cartesian_workspace_state[i] == 1)
            {
                cartesian_value[i] = (params.value_for_cartesian[i] < 0) ? double(params.value_for_cartesian[i]) : 0;
            }
            else if (states.cartesian_workspace_state[i] == -1)
            {
                cartesian_value[i] = (params.value_for_cartesian[i] > 0) ? double(params.value_for_cartesian[i]) : 0;
            }
        }
        else
            cartesian_value[i] = 0;
    }
}

void ROS_Communication::get_jogmode_states(Parameter &params, State &states)
{
    if (states.jogmode == LOW_SPEED)
    {
        for (int i = 0; i < 6; i++)
        {
            if (states.joint_disabled_state[i] == true)
            {
                if (params.value_for_joint[i] > 0)
                {
                    joint_value[i] = 1.00;
                }
                else if (params.value_for_joint[i] < 0)
                {
                    joint_value[i] = -1.00;
                }
            }
            else
                joint_value[i] = 0;
            if (states.cartesian_disabled_state[i] == true)
            {
                if (params.value_for_cartesian[i] > 0)
                {
                    cartesian_value[i] = 1.00;
                }
                else if (params.value_for_cartesian[i] < 0)
                {
                    cartesian_value[i] = -1.00;
                }
            }
            else
                cartesian_value[i] = 0;
        }
    }
    else if (states.jogmode == MEDIUM_SPEED)
    {
        for (int i = 0; i < 6; i++)
        {
            if (states.joint_disabled_state[i] == true)
            {
                if (params.value_for_joint[i] > 0)
                {
                    joint_value[i] = 4.00;
                }
                else if (params.value_for_joint[i] < 0)
                {
                    joint_value[i] = -4.00;
                }
            }
            else
                joint_value[i] = 0;

            if (states.cartesian_disabled_state[i] == true)
            {
                if (params.value_for_cartesian[i] > 0)
                {
                    cartesian_value[i] = 4.00;
                }
                else if (params.value_for_cartesian[i] < 0)
                {
                    cartesian_value[i] = -4.00;
                }
            }
            else
                cartesian_value[i] = 0;
        }
    }
    else if (states.jogmode == HIGH_SPEED)
    {
        for (int i = 0; i < 6; i++)
        {
            if (states.joint_disabled_state[i] == true)
            {
                if (params.value_for_joint[i] > 0)
                {
                    joint_value[i] = 8.00;
                }
                else if (params.value_for_joint[i] < 0)
                {
                    joint_value[i] = -8.00;
                }
            }
            else
                joint_value[i] = 0;

            if (states.cartesian_disabled_state[i] == true)
            {
                if (params.value_for_cartesian[i] > 0)
                {
                    cartesian_value[i] = 8.00;
                }
                else if (params.value_for_cartesian[i] < 0)
                {
                    cartesian_value[i] = -8.00;
                }
            }
            else
                cartesian_value[i] = 0;
        }
    }
}

void ROS_Communication::get_joystickCtrl_states(Parameter &params, State &states)
{
    if (states.joystick_ctrl == FIRST_THREE)
    {
        joint_value[3] = 0;
        joint_value[4] = 0;
        joint_value[5] = 0;
        cartesian_value[3] = 0;
        cartesian_value[4] = 0;
        cartesian_value[5] = 0;
    }
    else
    {
        joint_value[0] = 0;
        joint_value[1] = 0;
        joint_value[2] = 0;
        cartesian_value[0] = 0;
        cartesian_value[1] = 0;
        cartesian_value[2] = 0;
    }
}

void ROS_Communication::workspaceNotification(Parameter &params, State &states)
{
    for (int i = 0; i < 4; i++)
    {
        if (params.joint_state[i] < params.joint_limit_min_position[i])
        {
            states.joint_workspace_state[i] = -1;
            beyond_workspace();
        }
        else if (params.joint_state[i] > params.joint_limit_max_position[i])
        {
            states.joint_workspace_state[i] = 1;
            beyond_workspace();
        }
        else
        {
            states.joint_workspace_state[i] = 0;
        }
    }
    /*
    if ((params.joint_state[1] + params.joint_state[2]) < params.joint_limit_min_position[6])
    {
        states.joint_workspace_state[1] = -1;
        states.joint_workspace_state[2] = -1;
        beyond_workspace();
    }
    else if ((params.joint_state[1] + params.joint_state[2]) > params.joint_limit_max_position[6])
    {
        states.joint_workspace_state[1] = 1;
        states.joint_workspace_state[2] = 1;
        beyond_workspace();
    }
    */
}

QString ROS_Communication::io_decode(const char ch)
{
    switch (ch)
    {
    case '0':
        return "0000";
    case '1':
        return "0001";
    case '2':
        return "0010";
    case '3':
        return "0011";
    case '4':
        return "0100";
    case '5':
        return "0101";
    case '6':
        return "0110";
    case '7':
        return "0111";
    case '8':
        return "1000";
    case '9':
        return "1001";
    case 'A':
    case 'a':
        return "1010";
    case 'B':
    case 'b':
        return "1011";
    case 'C':
    case 'c':
        return "1100";
    case 'D':
    case 'd':
        return "1101";
    case 'E':
    case 'e':
        return "1110";
    case 'F':
    case 'f':
        return "1111";
    default:
        break;
    }
    return "0000";
}

StatesIndicator::StatesIndicator()
    : mState(false)
{
    _mState = mState;
}

void StatesIndicator::getState(bool currentState)
{
    _mState = mState;
    if (currentState == 1)
        mState = 1;
    else
        mState = 0;

    if (mState != _mState)
    {
        if (mState == true)
            indicatorON();
        else
            indicatorOFF();
    }
}
