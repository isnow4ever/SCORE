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
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "x50_planner" );
  ros::AsyncSpinner spinner ( 1 );
  spinner.start();
  ros::NodeHandle node_handle ( "~" );

  // Load the robot model
  robot_model_loader::RobotModelLoader robot_model_loader ( "robot_description" );
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // Construct a planning scene
  planning_scene::PlanningScenePtr planning_scene ( new planning_scene::PlanningScene ( robot_model ) );

  // Load a planner by name.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if ( !node_handle.getParam ( "planning_plugin", planner_plugin_name ) )
  {
    //ROS_FATAL_STREAM("Could not find planner plugin name");
    planner_plugin_name = "ompl_interface/OMPLPlanner";
  }

  try
  {
    planner_plugin_loader.reset (
      new pluginlib::ClassLoader<planning_interface::PlannerManager> (
        "moveit_core", "planning_interface::PlannerManager" ) );
  }
  catch ( pluginlib::PluginlibException& ex )
  {
    ROS_FATAL_STREAM ( "Exception while creating planning plugin loader " << ex.what() );
  }

  try
  {
    planner_instance.reset ( planner_plugin_loader->createUnmanagedInstance ( planner_plugin_name ) );

    if ( !planner_instance->initialize ( robot_model, node_handle.getNamespace() ) )
      ROS_FATAL_STREAM ( "Could not initialize planner instance" );

    ROS_INFO_STREAM ( "Using planning interface '" << planner_instance->getDescription() << "'" );
  }
  catch ( pluginlib::PluginlibException& ex )
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;

    for ( std::size_t i = 0 ; i < classes.size() ; ++i )
      ss << classes[i] << " ";

    ROS_ERROR_STREAM ( "Exception while loading planner '"
                       << planner_plugin_name << "': " << ex.what() << std::endl
                       << "Available plugins: " << ss.str() );
  }

  // Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time ( 10.0 );
  sleep_time.sleep();

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "root";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 1.1;
  pose.pose.position.z = 1.3;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose ( 3, 0.01 );
  std::vector<double> tolerance_angle ( 3, 0.01 );

  // `kinematic_constraints`_
  req.group_name = "manipulator";
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints ( "link6", pose, tolerance_pose, tolerance_angle );
  req.goal_constraints.push_back ( pose_goal );

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext ( planning_scene, req, res.error_code_ );
  context->solve ( res );

  if ( res.error_code_.val != res.error_code_.SUCCESS )
  {
    ROS_ERROR ( "Could not compute plan successfully" );
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory> ( "/move_group/display_planned_path", 1, true );
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO ( "Visualizing the trajectory" );
  moveit_msgs::MotionPlanResponse response;
  res.getMessage ( response );

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back ( response.trajectory );
  display_publisher.publish ( display_trajectory );

  sleep_time.sleep();

  ROS_INFO ( "Done" );

  return 0;
}
