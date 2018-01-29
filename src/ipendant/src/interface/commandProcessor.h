#ifndef COMMANDPROCESSOR_H
#define COMMANDPROCESSOR_H

#include "res.h"
#include "project.h"

class CommandProcessor
{
public:
    CommandProcessor ( QString const & ex );
    virtual ~CommandProcessor();

    virtual QString const Process ( QString const & command, Parameter const *params, Actor & actors ) = 0;

    virtual bool const CanProcess ( QString const & command );

    virtual Command const Load ( QString const & command ) = 0;

    virtual int const CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states ) = 0;

protected:
    QRegExp regex;
};

class MOVJ_LSPB_Processor : public CommandProcessor
{
public:
    MOVJ_LSPB_Processor();

    Command const Load (QString const & command);

    QString const Process ( QString const & command, Parameter const *params, Actor & actors );

    int const CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states );
};

class MOVL_RPY_LSPB_Processor : public CommandProcessor
{
public:
    MOVL_RPY_LSPB_Processor();

    Command const Load ( QString const & command);

    QString const Process ( QString const & command, Parameter const *params, Actor & actors ) ;

    int const CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states );
};

class MOVJ_SPLINE_LSPB_Processor : public CommandProcessor
{
public:
    MOVJ_SPLINE_LSPB_Processor();

    Command const Load ( QString const & command);

    QString const Process ( QString const & command, Parameter const *params, Actor & actors ) ;

    int const CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states );
};

class SET_Processor : public CommandProcessor
{
public:
    SET_Processor();

    Command const Load(QString const & command);

    QString const Process ( QString const & command, Parameter const *params, Actor & actors ) ;

    int const CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states );
};

class RESET_Processor : public CommandProcessor
{
public:
    RESET_Processor();

    Command const Load(QString const & command);

    QString const Process ( QString const & command, Parameter const *params, Actor & actors ) ;

    int const CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states );
};

class WAIT_Processor : public CommandProcessor
{
public:
    WAIT_Processor();

    Command const Load(QString const & command);

    QString const Process ( QString const & command, Parameter const *params, Actor & actors ) ;

    int const CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states );
};

class PROG_Processor : public CommandProcessor
{
public:
    PROG_Processor();

    Command const Load(QString const & command);

    QString const Process ( QString const & command, Parameter const *params, Actor & actors ) ;

    int const CanContinue ( int const m_id, QString const & command, Controller_State const *controller_states, State const *states );
};

struct Interpreter
{
    //Command Interpreter
    MOVL_RPY_LSPB_Processor                 processor_MOVL_RPY_LSPB;
    MOVJ_LSPB_Processor                          processor_MOVJ_LSPB;
    MOVJ_SPLINE_LSPB_Processor            processor_MOVJ_SPLINE_LSPB;
    SET_Processor                                       processor_SET;
    RESET_Processor                                   processor_RESET;
    WAIT_Processor                                    processor_WAIT;
    PROG_Processor                                   processor_PROG;
};


#endif // COMMANDPROCESSOR_H
