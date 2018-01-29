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

#include "projectHandler.h"

ProjectHandler::ProjectHandler()
{
    if (m_dir.cd(SAVE_DIR) == 0) {
        m_dir.mkdir(SAVE_DIR);
        m_dir.cd(SAVE_DIR);
    }
}

// Project load functions
void ProjectHandler::loadBasicInfo(const QStringList &basicInfo)
{
    //loadBasicInfo Function
}

void ProjectHandler::loadPoints(const QStringList &points)
{
    //loadPoints Function
}

void ProjectHandler::loadFileText(Project *project)
{
    // Open file.
    if(project->getProjectName().size() == 0)
    {
        ROS_WARN("No Project is Select.");
        return;
    }

    if (!project->getProjectName().endsWith(".txt")) {
        project->setProjectName(project->getProjectName()+".txt");
    }
    QFile file( m_dir.filePath(project->getProjectName()));
    if (!file.open(QIODevice::ReadWrite))
    {
        ROS_ERROR("Cannot open this Project!");
        return;
    }
    QTextStream inStream(&file);
    project->fileText = inStream.readAll();
}

void ProjectHandler::loadProject(Project * proj, Interpreter * interpreters)
{
    // Open file.
    if(proj->getProjectName().size() == 0)
    {
        ROS_WARN("No Project is Select.");
        return;
    }

    if (!proj->getProjectName().endsWith(".txt")) {
        proj->setProjectName(proj->getProjectName()+".txt");
    }
    QFile file( m_dir.filePath(proj->getProjectName()));
    if (!file.open(QIODevice::ReadOnly))
    {
        ROS_ERROR("Cannot open this Project!");
        return;
    }

    proj->projectPropertiesInit();
//    loadBasicInfo();
//    loadPoints();

    QTextStream inStream(&file);
    QString line;
    int lineNo = 0;

    do {
        // 1.Read line.
        QString _line = inStream.readLine();
        QStringList find = _line.split(QRegExp("\\s"));
        line = _line.mid(find.at(0).length()+1);

        // 2.Check grammar, Load commandlist for Teach.
        if(interpreters->processor_MOVJ_LSPB.CanProcess(line))
        {
            proj->commandList.push_back(interpreters->processor_MOVJ_LSPB.Load(line));
        }
        else if(interpreters->processor_MOVL_RPY_LSPB.CanProcess(line))
        {
            proj->commandList.push_back(interpreters->processor_MOVL_RPY_LSPB.Load(line));
        }
        else if(interpreters->processor_MOVJ_SPLINE_LSPB.CanProcess(line))
        {
            proj->commandList.push_back(interpreters->processor_MOVJ_SPLINE_LSPB.Load(line));
        }
        else if(interpreters->processor_SET.CanProcess(line))
        {
            proj->commandList.push_back(interpreters->processor_SET.Load(line));
        }
        else if(interpreters->processor_RESET.CanProcess(line))
        {
            proj->commandList.push_back(interpreters->processor_RESET.Load(line));
        }
        else if(interpreters->processor_WAIT.CanProcess(line))
        {
            proj->commandList.push_back(interpreters->processor_WAIT.Load(line));
        }
        else if(interpreters->processor_PROG.CanProcess(line))
        {
            proj->commandList.push_back(interpreters->processor_PROG.Load(line));
        }
        else
        {
            ROS_ERROR("Unknown Grammar at Line: %d", lineNo);
	    proj->commandList.clear();
	    proj->lineList.clear();
	    projectLoadFailed();
	    return;
        }

        // 3. Load lineList for Playback GUI.
        proj->lineList.append(line);

        lineNo++;

    } while (!inStream.atEnd());
}

// Project save functions ( Accessible from GUI )
void ProjectHandler::saveFile(Project *project)
{
    if (project->getProjectName().size() == 0) {
        ROS_WARN("Cannot save file, empty filename.");
        return;
    }

    if (!project->getProjectName().endsWith(".txt")) {
        project->setProjectName(project->getProjectName()+".txt");
    }

    QFile file(m_dir.filePath(project->getProjectName()));
    if (file.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream outStream(&file);
        QList<QString>::Iterator it;
        int i = 0;

        for(it = project->lineList.begin(); it != project->lineList.end(); it++, i++)
        {
            QString one;
            one = *it;
            outStream << i << " " << one << "\n";
        }
    }

    file.close();
    directoryChanged();
}

void ProjectHandler::saveText(Project *project)//fileText should go through Parser......
{
    if (project->getProjectName().size() == 0) {
        qWarning() << "Cannot save file, empty filename.";
        return;
    }

    if (!project->getProjectName().endsWith(".txt")) {
        project->setProjectName(project->getProjectName()+".txt");
    }

    ROS_WARN("save!!!");
    QFile file(m_dir.filePath(project->getProjectName()));
    if (file.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream outStream(&file);
        outStream << project->fileText;
        file.close();
        directoryChanged();
    }
}


