#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "test_node");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Publisher jointstate_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

	ros::Rate loop_rate(10);


	sensor_msgs::JointState js;

	js.name.push_back("joint1");
	js.name.push_back("joint2");
	js.name.push_back("joint3");
	js.name.push_back("joint4");
	
	for(int i = 0; i < 4; i++)
	{
		js.position.push_back(0.0);
		js.velocity.push_back(0.1);
		js.effort.push_back(0.3);			
	}

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	double count = 0.1;
	while (ros::ok())
	{
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		//ROS_INFO("%s", msg.data.c_str());

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		//chatter_pub.publish(msg);



		for(int i = 0; i < 4; i++)
		{
			if(js.position[i] > 100)
			{
				js.position[i] = 0.00;
				count = 0;
			}
			js.position[i] = 0.00 + count;
			count = count + 0.1;			
		}
		
		std::stringstream sss;
		sss << js.position.at(0) << " " << js.position.at(1) << " " << js.position.at(2) << " " << js.position.at(3) << std::endl;
		ROS_INFO("%s", sss.str().c_str());
		jointstate_pub.publish(js);

		ros::spinOnce();

		loop_rate.sleep();


	}

	return 0;
}