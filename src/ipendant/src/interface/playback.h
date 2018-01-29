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

#ifndef PLAYBACK_H
#define PLAYBACK_H

#include "res.h"
#include "commandProcessor.h"
#include "commandHandler.h"
#include "project.h"
#include "projectHandler.h"

struct Count
{
    int cycle;
    QString line;
    int lineNo;
    int m_id;
    bool first_pub;
    int childProjDepth;
};

class Playback : public CommandHandler
{
    Q_OBJECT
public:
    Playback();
    ~Playback();

    void setPlaybackProperties( int const mode, int const lineF, int const lineT, int const cycles);

    void clearPlaybackProperties();

    void process(Project *project,
                 ProjectHandler *project_handler,
                 State *states,
                 Controller_State *controller_states,
                 Parameter *params,
                 Interpreter *interpreters,
                 Actor & actors);

    void playbackStart(Project *project, int);
    void playbackFromTo(Project *project, int from, int to, int mode);
    void playbackStep(Project *project, int);
    void playbackStop(State *states);
    void playbackPrev(Project *project);
    void playbackNext(Project *project);

    Count getCurrentInfo() const;

Q_SIGNALS:
    void updateGUI(int);
    void project_states(int);
    void cycle_minus();

protected:
    bool stop_signal;
    int prev_cmd_states;
    bool stepmatch;

    int playback_mode;
    int lineFrom;
    int lineTo;
    int cycle;
    Count count;
};



#endif//PLAYBACK_H
