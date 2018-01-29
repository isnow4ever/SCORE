#include "commandProcessor.h"

//CommandProcessor
CommandProcessor::CommandProcessor ( QString const & ex )
{
    regex.setPattern(ex);
}
CommandProcessor::~CommandProcessor() {  }

bool const CommandProcessor::CanProcess ( QString const & command )
{
    QStringList rr = command.split(" ");
    if((rr.at(0) == "MOVJ_SPLINE_LSPB")||(rr.at(0) == "MOVJ_LSPB")||(rr.at(0) == "MOVL_RPY_LSPB"))
    {
	if((rr.length() - 5) % 6)
	{
	    ROS_ERROR("Wrong point number!");
	    return 0;
	}
    }
    bool result = regex.exactMatch(command);
    return result;
}

//MOVJ_LSPB_Processor
MOVJ_LSPB_Processor::MOVJ_LSPB_Processor()
    : CommandProcessor ( MOTION_CMD(MOVJ_LSPB) )
{  }

Command const MOVJ_LSPB_Processor::Load (QString const & command)
{
    Command cc;
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return cc;
    }

    QStringList result = command.split(" ");
    cc.type = MOTION;
    cc.pointsNum = 1;
    cc.method = "MOVJ_LSPB";
    cc.target = regex.cap(1);
    for(int i = 0; i < 6; i++)
        cc.motion_params.push_back(result.at(i+2).toDouble());
    cc.duration = regex.cap(3).toDouble();
    cc.units = "mm,deg,s";
    cc.frame = regex.cap(7);

    return cc;
}

QString const MOVJ_LSPB_Processor::Process ( QString const & command, Parameter const *params, Actor & actors )
{
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return 0;
    }

    // 1.explain command.
    Command cc;
    cc = Load(command);

    // 2.generate command.
    QString pub_cmd;
    pub_cmd.append(cc.method);
    pub_cmd.append(" {" + cc.target + "} ");
    for(int j = 0; j < 6; j++)
        pub_cmd.append(QString::number(cc.motion_params[j]) + " ");
    pub_cmd.append("[" + QString::number(cc.duration) + "] ");
    pub_cmd.append("(" + cc.units + ") ");
    pub_cmd.append("{" + cc.frame + "}");

    // 3.publish command.

    return pub_cmd;
}

int const MOVJ_LSPB_Processor::CanContinue(int const m_id, QString const & command, Controller_State const *controller_states, State const *states)
{
    Command cc;
    cc = Load(command);
    if(cc.method != "MOVJ_LSPB")
        return 0;

    if(m_id == atoi(controller_states->current_id.c_str()))
    {
        if((controller_states->_robot_state == "stable")&&(controller_states->action_result == "succeed"))
        {
            return 1;
        }
        else if(controller_states->action_result == "speed")
        {
            return 2;
        }
    }
    return 0;
}

//MOVL_RPY_LSPB_Processor
MOVL_RPY_LSPB_Processor::MOVL_RPY_LSPB_Processor()
    : CommandProcessor ( MOTION_CMD(MOVL_RPY_LSPB) )
{  }

Command const MOVL_RPY_LSPB_Processor::Load ( QString const & command)
{
    Command cc;
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return cc;
    }

    QStringList result = command.split(" ");
    cc.type = MOTION;
    cc.pointsNum = 1;
    cc.method = "MOVL_RPY_LSPB";
    cc.target = regex.cap(1);
    for(int i = 0; i < 6; i++)
        cc.motion_params.push_back(result.at(i+2).toDouble());
    cc.duration = regex.cap(3).toDouble();
    cc.units = "mm,deg,s";
    cc.frame = regex.cap(7);

    return cc;
}

QString const MOVL_RPY_LSPB_Processor::Process ( QString const & command, Parameter const *params, Actor & actors )
{
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return 0;
    }

    // 1.explain command.
    Command cc;
    cc = Load(command);

    // 2.generate command.
    QString pub_cmd;
    pub_cmd.append(cc.method);
    pub_cmd.append(" {" + cc.target + "} ");
    for(int j = 0; j < 6; j++)
        pub_cmd.append(QString::number(cc.motion_params[j]) + " ");
    pub_cmd.append("[" + QString::number(cc.duration) + "] ");
    pub_cmd.append("(" + cc.units + ") ");
    pub_cmd.append("{" + cc.frame + "}");

    // 3.publish command.

    return pub_cmd;
}

int const MOVL_RPY_LSPB_Processor::CanContinue(int const m_id, QString const & command, Controller_State const *controller_states, State const *states)
{
    Command cc;
    cc = Load(command);
    if(cc.method != "MOVL_RPY_LSPB")
        return 0;
        
    if(m_id == atoi(controller_states->current_id.c_str()))
    {
        if((controller_states->_robot_state == "stable")&&(controller_states->action_result == "succeed"))
        {
            return 1;
        }
        else if(controller_states->action_result == "speed")
        {
            return 2;
        }
    }
    return 0;
}

//MOVJ_SPLINE_LSPB_Processor
MOVJ_SPLINE_LSPB_Processor::MOVJ_SPLINE_LSPB_Processor()
    : CommandProcessor ( MOTION_CMD(MOVJ_SPLINE_LSPB) )
{  }

Command const MOVJ_SPLINE_LSPB_Processor::Load ( QString const & command)
{
    Command cc;
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return cc;
    }

    QStringList result = command.split(" ");
    cc.type = MOTION;
    if((result.length() -5) % 6)
    {
	ROS_ERROR("Wrong point number!");
	return cc;
    }
    cc.pointsNum = (result.length() - 5) / 6;
    cc.method = "MOVJ_SPLINE_LSPB";
    cc.target = regex.cap(1);
    for(int i = 0; i < cc.pointsNum * 6; i++)
        cc.motion_params.push_back(result.at(i+2).toDouble());
    cc.duration = regex.cap(3).toDouble();
    cc.units = "mm,deg,s";
    cc.frame = regex.cap(7);

    return cc;
}

QString const MOVJ_SPLINE_LSPB_Processor::Process ( QString const & command, Parameter const *params, Actor & actors )
{
    // 1.explain command.
    Command cc;
    cc = Load(command);

    // 2.generate command.
    QString pub_cmd;
    pub_cmd.append(cc.method);
    pub_cmd.append(" {" + cc.target + "} ");
    for(int j = 0; j < cc.pointsNum * 6; j++)
        pub_cmd.append(QString::number(cc.motion_params[j]) + " ");
    pub_cmd.append("[" + QString::number(cc.duration) + "] ");
    pub_cmd.append("(" + cc.units + ") ");
    pub_cmd.append("{" + cc.frame + "}");

    // 3.publish command.

    return pub_cmd;
}

int const MOVJ_SPLINE_LSPB_Processor::CanContinue(int const m_id, QString const & command, Controller_State const *controller_states, State const *states)
{
    Command cc;
    cc = Load(command);
    if(cc.method != "MOVJ_SPLINE_LSPB")
        return 0;
        
    if(m_id == atoi(controller_states->current_id.c_str()))
    {
        if((controller_states->_robot_state == "stable")&&(controller_states->action_result == "succeed"))
        {
            return 1;
        }
        else if(controller_states->action_result == "speed")
        {
            return 2;
        }
    }
    return 0;
}

//SET_Processor
SET_Processor::SET_Processor()
    : CommandProcessor(IO_CMD(SET))
{ }

Command const SET_Processor::Load(QString const & command)
{
    Command cc;
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return cc;
    }

    cc.type = IO;
    cc.method = "SET";
    cc.target = regex.cap(1);
    cc.io_result = regex.cap(2);

    return cc;
}

QString const SET_Processor::Process ( QString const & command, Parameter const *params, Actor & actors )
{
    // 1.explain command.
    Command cc;
    cc = Load(command);

    // 2.generate command.
    QString pub_cmd;
    pub_cmd.append(cc.method);
    pub_cmd.append(" " + cc.target + " ");
    pub_cmd.append(cc.io_result);

    // 3.publish command.

    return pub_cmd;
}

int const SET_Processor::CanContinue(int const m_id, QString const & command, Controller_State const *controller_states, State const *states)
{
    Command cc;
    cc = Load(command);
    if(cc.method != "SET")
        return 0;
    else
        return 1;
}

//RESET_Processor
RESET_Processor::RESET_Processor()
    : CommandProcessor(IO_CMD(RESET))
{ }

Command const RESET_Processor::Load(QString const & command)
{
    Command cc;
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return cc;
    }

    cc.type = IO;
    cc.method = "RESET";
    cc.target = regex.cap(1);

    return cc;
}

QString const RESET_Processor::Process ( QString const & command, Parameter const *params, Actor & actors )
{
    // 1.explain command.
    Command cc;
    cc = Load(command);

    // 2.generate command.
    QString pub_cmd;
    pub_cmd.append(cc.method);
    pub_cmd.append(" " + cc.target);

    // 3.publish command.

    return pub_cmd;
}

int const RESET_Processor::CanContinue(int const m_id, QString const & command, Controller_State const *controller_states, State const *states)
{
    Command cc;
    cc = Load(command);
    if(cc.method != "RESET")
        return 0;
    else
        return 1;
}

//WAIT_Processor
WAIT_Processor::WAIT_Processor()
    : CommandProcessor(WAIT_CMD)
{ }

Command const WAIT_Processor::Load(QString const & command)
{
    Command cc;
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return cc;
    }

    cc.type = WAIT;
    cc.method = "WAIT";
    cc.target = regex.cap(1);
    cc.io_result = regex.cap(2);// new add

    return cc;
}

QString const WAIT_Processor::Process ( QString const & command, Parameter const *params, Actor & actors )
{
    // 1.explain command.
    Command cc;
    cc = Load(command);

    // 2.generate command.
    QString pub_cmd;
    pub_cmd.append(cc.method);
    pub_cmd.append(" " + cc.target);
    pub_cmd.append(" " + cc.io_result);// new add

    // 3.publish command.

    return pub_cmd;
}

int const WAIT_Processor::CanContinue(int const m_id, QString const & command, Controller_State const *controller_states, State const *states)
{
    if(1) // (m_id == atoi(controller_states->current_id.c_str()))
    {
        QStringList commandline = command.split(" ");
        if(commandline.at(0) == "WAIT")
        {
            if(states->iostate[commandline.at(1).toInt()] == commandline.at(2).toInt())
            {
                return 1;
            }
        }
    }
    return 0;
}

//PROG_Processor
PROG_Processor::PROG_Processor()
    : CommandProcessor(PROG_CMD)
{ }

Command const PROG_Processor::Load(QString const & command)
{
    Command cc;
    if(regex.indexIn(command) == -1)
    {
        ROS_ERROR("Cannot process this command!");
        return cc;
    }

    cc.type = PROG;
    cc.method = "PROG";
    cc.target = regex.cap(1);

    return cc;
}

QString const PROG_Processor::Process ( QString const & command, Parameter const *params, Actor & actors )
{
    // 1.explain command.
    Command cc;
    cc = Load(command);

    // 2.generate command.
    if(!cc.target.endsWith(".txt"))
        cc.target = cc.target + ".txt";
    return cc.target;
}

int const PROG_Processor::CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states )
{
    ROS_ERROR("This function is no use!");
    return -1;
}
