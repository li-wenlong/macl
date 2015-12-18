#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

#define MIN_SCAN_ANGLE_RAD -45.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD +45.0/180*M_PI

bool obstacleFound = false;

void readSensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensor_msgs);

int main(int argc, char** argv)
{
	if(argc<2)
	{
		ROS_ERROR("You must specify robot id.");
		return -1;
	}

	char *robot_name = argv[1];

	ros::init(argc,argv,"random_walk");
	ros::NodeHandle nh;

    string cmd_vel_topic_name = "cmd_vel";

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

    string laser_topic_name = "base_scan";

	ros::Subscriber base_scan_sub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic_name, 1, &readSensorCallback);

	geometry_msgs::Twist moveForwardCommand;
	moveForwardCommand.linear.x = 0.2;

	geometry_msgs::Twist turnCommand;
	turnCommand.angular.z = 0.2;

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		if(obstacleFound)
		{
			cmd_vel_pub.publish(turnCommand);
		}
		else
		{
			cmd_vel_pub.publish(moveForwardCommand);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void readSensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	bool isObstacle = false;

	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

	for (int i=minIndex; i<=maxIndex; i++)
	{
		if(scan->ranges[i] < 1)
		{
			isObstacle = true;
		}
	}

	if(isObstacle)
	{
		obstacleFound = true;
	}
	else
	{
		obstacleFound = false;
	}

}