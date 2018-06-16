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

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


int main(int argc, char **argv)
{
    ros::init (argc, argv, "scene_operator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle node_handle;

    /* ATTACH AN OBJECT*/
    /* First advertise and wait for a connection*/
    ros::Publisher attached_object_publisher = node_handle.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
    while(attached_object_publisher.getNumSubscribers() < 1)
    {
        ROS_INFO("waiting.");
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    /* Define the attached object message*/
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "root";
    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = "world";
    /* The id of the object */
    attached_object.object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.5;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    /* An attach operation requires an ADD */
    attached_object.object.operation = attached_object.object.ADD;

    /* Publish and sleep (to view the visualized results)*/
    attached_object_publisher.publish(attached_object);

    ros::shutdown();
    return 0;
}
