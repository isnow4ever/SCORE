/*
* License:The MIT License (MIT)
*
* Copyright (c) 2013,2014 Yanyu Su
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
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <string>
#include <iomanip>
#include "constants.h"

class JointStateHandller
{
public:
    void callback ( const sensor_msgs::JointState::ConstPtr & message )
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    JointStateHandller()
        : mCrtMsg()
        , mNewMsg ( false )
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const sensor_msgs::JointState & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    sensor_msgs::JointState mCrtMsg;
    bool mNewMsg;
};

std::ofstream outfile;
sensor_msgs::JointState CrtMsg;

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "x50_calibration" );
  ros::NodeHandle                     NODE;
  JointStateHandller                   JOINT_STATE_HANDLLER;
  ros::Subscriber                        subscriber_joint_states =
          NODE.subscribe<sensor_msgs::JointState> (
              INTERFACE_JOINT_STATES, 1024,  &JointStateHandller::callback,
              &JOINT_STATE_HANDLLER );

  char tmp[2048];
  std::cout << "input file name>";
  std::cin.getline(tmp,2048);

  outfile.open(tmp,std::ios_base::app);
  if(!outfile.is_open())
  {
      std::cout << "cannot open file to write." << std::endl;
      return 0;
  }
  outfile << std::endl << std::endl << "new record" << std::endl;
  while ( ros::ok() )
  {
      //std::cout << "run" << std::endl;
      ros::spinOnce();
      if ( JOINT_STATE_HANDLLER.newMsg() )
      {
          //std::cout << "new" << std::endl;
          CrtMsg = JOINT_STATE_HANDLLER.fetchMsg();
          std::cout << ros::Time::now() << " ";
          outfile << ros::Time::now() << " ";
          for(size_t i = 0; i < CrtMsg.position.size(); ++i)
          {
              std::cout << std::setprecision(12) << CrtMsg.position[i]/3.1415926*180 << " ";
              outfile << std::setprecision(12) << CrtMsg.position[i]/3.1415926*180 << " ";
          }
          std::cout << std::endl;
          outfile << std::endl;
      }

      ros::Duration(0.0125).sleep();
  }
  std::cout << "save file" << std::endl;
  outfile.close();

  return 0;
}
