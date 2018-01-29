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

#include "playback.h"

#define new_cmd_delay 400000

bool wait_command_state = 0;

Playback::Playback() : stop_signal(0)
{
    count.childProjDepth = 0;
}

Playback::~Playback()
{ }

void Playback::setPlaybackProperties( int const mode, int const lineF, int const lineT, int const cycles)
{
    playback_mode = mode;
    lineFrom = lineF;
    lineTo = lineT;
    cycle = cycles;
    count.cycle = cycles;
    count.lineNo = lineF;
    count.line = "";
}

void Playback::clearPlaybackProperties()
{
    playback_mode = -1;
    lineFrom = -1;
    lineTo = -1;
    cycle = -1;
    count.cycle = -1;
    count.lineNo = -1;
    count.line = "";
    count.childProjDepth = 0;
}

void Playback::process(Project          *project,
                       ProjectHandler   *project_handler,
                       State            *states,
                       Controller_State *controller_states,
                       Parameter        *params,
                       Interpreter  *interpreters,
                       Actor            &actors)
{

    // 1. check stop operation
    if(stop_signal)
    {
        return;
    }

    // 2. publish
    std_msgs::String msg;
    std::stringstream ss;

    if(newCmd()) {
        ss << "<" << params->id << "> ";
        ss << fetchCmd();

        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        actors.command_pub.publish(msg);

        count.m_id = params->id;
        count.line = QString::fromStdString(fetchCmd());
        count.first_pub = 0;

        usleep(new_cmd_delay);
    }

    QString command;
    // 3. check previous cmd: if done, continue to 4, else return
    command = count.line;
    if(count.first_pub)
    {
        goto process;
    }
    else if(interpreters->processor_MOVJ_LSPB.CanContinue(count.m_id,
                                               command,
                                               controller_states,
                                               states) != 0)
    {
        prev_cmd_states = interpreters->processor_MOVJ_LSPB.CanContinue(count.m_id,
                                                                  command,
                                                                  controller_states,
                                                                  states);
        goto check_flag;
    }
    else if (interpreters->processor_MOVL_RPY_LSPB.CanContinue(count.m_id,
                                                    command,
                                                    controller_states,
                                                    states) != 0)
    {
        prev_cmd_states = interpreters->processor_MOVL_RPY_LSPB.CanContinue(count.m_id,
                                                                      command,
                                                                      controller_states,
                                                                      states);
        goto check_flag;
    }

    else if (interpreters->processor_MOVJ_SPLINE_LSPB.CanContinue(count.m_id,
                                                    command,
                                                    controller_states,
                                                    states) != 0)
    {
        prev_cmd_states = interpreters->processor_MOVJ_SPLINE_LSPB.CanContinue(count.m_id,
                                                                         command,
                                                                         controller_states,
                                                                         states);
        goto check_flag;
    }

    else if (interpreters->processor_SET.CanContinue(count.m_id,
                                                    command,
                                                    controller_states,
                                                    states) != 0)
    {
        prev_cmd_states = interpreters->processor_SET.CanContinue(count.m_id,
                                                            command,
                                                            controller_states,
                                                            states);
        goto check_flag;
    }

    else if (interpreters->processor_RESET.CanContinue(count.m_id,
                                                    command,
                                                    controller_states,
                                                    states) != 0)
    {
        prev_cmd_states = interpreters->processor_RESET.CanContinue(count.m_id,
                                                              command,
                                                              controller_states,
                                                              states);
        goto check_flag;
    }

    else if (interpreters->processor_WAIT.CanContinue(count.m_id,
                                                    command,
                                                    controller_states,
                                                    states) != 0)
    {
        prev_cmd_states = interpreters->processor_WAIT.CanContinue(count.m_id,
                                                             command,
                                                             controller_states,
                                                             states);
        goto check_flag;
    }
    else
    {
        prev_cmd_states = 0;
    }

check_flag: switch (prev_cmd_states) {
    case 1://match
        ROS_DEBUG("match");
        break;
    case 2://over speed
        updateGUI(0);
        states->imode = STANDBY;
    default://not succeed
        ROS_DEBUG("not succeed");
        return;
    }

    // 4. check playback mode

    checkmode:

    switch (playback_mode) {
    case TRIAL:
        if(!count.first_pub)
        {
            if(count.lineNo == lineTo)
            {
                ROS_INFO("The TRIAL is done");
                project_states(3);//Notify users of the TRIAL done.
                states->imode = STANDBY;
                playback_mode = 0;
                return;
            }
            else
            {
                count.lineNo++;
                updateGUI(1);
            }
        }
        break;
    case STEP:
        if(!count.first_pub)
        {
            if(!stepmatch)
            {
                stepmatch = 1;
                ROS_INFO("The Single command is done");
                project_states(1);//Notify users of the Single command done.
            }
            return;
        }
        break;
    case CYCLE:
        if(!count.first_pub)
        {
            if(count.lineNo == project->getLineList().count() -1)
            {
                if(count.childProjDepth != 0)
                {
                    count.childProjDepth = count.childProjDepth - 1;
                    QString f_name = params->fatherProjects.at(count.childProjDepth).name;
                    int f_flag = params->fatherProjects.at(count.childProjDepth).lineFlag;
                    int f_cycle = params->fatherProjects.at(count.childProjDepth).cycleFlag;
                    int f_lineTo = params->fatherProjects.at(count.childProjDepth).lineTo;
                    int f_mode = params->fatherProjects.at(count.childProjDepth).mode;
                    params->fatherProjects.pop_back();

                    project->setProjectName(f_name);
                    project_handler->loadProject(project, interpreters);
                    ROS_DEBUG("%s", f_name.toStdString().c_str());
                    if(f_mode == STEP)
                    {
                        count.lineNo = f_flag;
                        playback_mode = STEP;
                        stepmatch = 1;
                        count.first_pub = 0;
                        updateGUI(4);
                        return;
                    }
                    else
                    {
                        if((project->lineList.length() - 1 == f_flag)&&(f_cycle == 0))
                        {
                            count.lineNo = f_flag;
                            count.cycle = f_cycle;
                            updateGUI(4);
                            goto prog_done;
                        }
                        else
                        {
                            playbackFromTo(project, f_flag + 1, f_lineTo, f_mode);
                            count.lineNo = f_flag + 1;
                            count.cycle = f_cycle;
                            updateGUI(4);
                            goto cycle_minus;
                        }
                    }
                }
                else
                {
                    if(count.cycle > 0)
                    {
                        cycle_minus:
                        count.cycle--;
                        updateGUI(2);//cycle minus
                        count.lineNo = 0;
                    }
                    else
                    {
                        prog_done:
                        ROS_INFO("The whole prog is done");
                        project_states(2);//Notify users of the program done.
                        states->imode = STANDBY;
                        playback_mode = 0;
                        return;
                    }
                }
            }
            else
            {
                count.lineNo++;
                updateGUI(1);
            }
        }
        break;
    default:
        break;
    }


    // 5. process
    process:
    command = project->getLineList().at(count.lineNo);
    if(interpreters->processor_MOVJ_LSPB.CanProcess(command))
    {
        callback(interpreters->processor_MOVJ_LSPB.Process(command, params, actors).toStdString());
    }
    else if(interpreters->processor_MOVL_RPY_LSPB.CanProcess(command))
    {
        callback(interpreters->processor_MOVL_RPY_LSPB.Process(command, params, actors).toStdString());
    }
    else if(interpreters->processor_MOVJ_SPLINE_LSPB.CanProcess(command))
    {
        callback(interpreters->processor_MOVJ_SPLINE_LSPB.Process(command, params, actors).toStdString());
    }
    else if(interpreters->processor_SET.CanProcess(command))
    {
        callback(interpreters->processor_SET.Process(command, params, actors).toStdString());
    }
    else if(interpreters->processor_RESET.CanProcess(command))
    {
        callback(interpreters->processor_RESET.Process(command, params, actors).toStdString());
    }
    else if(interpreters->processor_WAIT.CanProcess(command))
    {
        callback(interpreters->processor_WAIT.Process(command, params, actors).toStdString());
    }
    else if(interpreters->processor_PROG.CanProcess(command))
    {
        // 1. Save original project name and lineFlag
        FatherProject fProj;
        fProj.name = project->getProjectName();
        fProj.lineFlag = count.lineNo;
        fProj.cycleFlag = count.cycle;
        fProj.lineTo = lineTo;
        fProj.mode = playback_mode;
        params->fatherProjects.push_back(fProj);
        ROS_DEBUG("%s, %d, %d", fProj.name.toStdString().c_str(), fProj.lineFlag, fProj.lineTo);

        // 2. Load child project
        QString name = interpreters->processor_PROG.Process(command, params, actors);
        project->setProjectName(name);
        project_handler->loadProject(project, interpreters);
        count.lineNo = 0;

        // 3. Update GUI
        updateGUI(4);

        // 4. Playback child project
        playbackStart(project, 1);

        // 5. update reference
        count.childProjDepth++;
    }
}

void Playback::playbackStart(Project *project, int cycle)
{
    setPlaybackProperties(CYCLE, 0, project->getLineList().count(), cycle-1);
    count.first_pub = 1;
}

void Playback::playbackStep(Project *project, int lineNo)
{
    setPlaybackProperties(STEP, lineNo, lineNo, 0);
    stepmatch = 0;
    count.first_pub = 1;
}

void Playback::playbackFromTo(Project *project, int from, int to, int mode)
{
    setPlaybackProperties(mode, from, to, 0);
    count.first_pub = 1;
}

void Playback::playbackStop(State *states)
{
    if(states->imode == PLAYBACK)
    {
        stop_signal = 1;
        ROS_INFO("The prog is stopped");
        states->imode = STANDBY;
        playback_mode = 0;
        project_states(0);
        clearPlaybackProperties();
        stop_signal = 0;
    }
}

void Playback::playbackNext(Project *project)
{
    if(count.lineNo == project->getLineList().count() - 1)
    {
        ROS_WARN("This is the last line!");
        return;
    }
    playbackStep(project, count.lineNo + 1);
    updateGUI(1); //hightlight next
}

void Playback::playbackPrev(Project *project)
{
    if(count.lineNo == 0)
    {
        ROS_WARN("This is the first line!");
        return;
    }
    playbackStep(project, count.lineNo - 1);
    updateGUI(3); //hightlight prev
}

Count Playback::getCurrentInfo() const
{
    return count;
}
