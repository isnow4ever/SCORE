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

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "x50_pipline");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // Construct a planning scene
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // SETUP THE PLANNING PIPELINE*/
    planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

    // Sleep a little to allow time to startup rviz, etc. */
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

    // CREATE A MOTION PLAN REQUEST FOR THE RIGHT ARM OF THE PR2 */
    // We will ask the end-effector of the PR2 to go to a desired location */
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    // A desired pose */
    geometry_msgs::PointStamped pose;
    pose.header.frame_id = "root";
    pose.point.x = 0.0;
    pose.point.y = 1.1;
    pose.point.z = 1.7;

    // Create the request */
    req.group_name = "manipulator";
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("tool", pose, 0.5);
    req.goal_constraints.push_back(pose_goal);

    // Call the pipeline */
    //  planning_pipeline->generatePlan((planning_scene::PlanningSceneConstPtr) planning_scene, req, res);
    planning_pipeline->generatePlan(planning_scene, req, res);

    // Check that the planning was successful */
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    // Visualize the generated plan */
    // Publisher for display */
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    sleep_time.sleep();

    ROS_INFO("Done");
    return 0;
}
