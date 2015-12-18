#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define UNKNOWN_COV 999
#define LASER_UNCERTAINTY 0.0005

using namespace std;

bool observed_msg_received = false;
bool observer_msg_received = false;
bool observer_amcl_msg_received = false;

geometry_msgs::PoseWithCovarianceStamped observed_pose;
geometry_msgs::PoseWithCovarianceStamped observer_pose;
geometry_msgs::PoseWithCovarianceStamped observer_amcl_pose;

void observed_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void observer_amcl_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void observer_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

int main(int argc, char ** argv)
{	
	if (argc < 2) {
		ROS_ERROR("You must observed robot id.");
		return -1;
	}
	char *observed_id = argv[1];
	
	ros::init(argc, argv, "observer");
	ros::NodeHandle nh;

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_comm", 1000);

	//get observed and observer pose and observer amcl pose 
	string observed_str = "/robot_";
	observed_str += observed_id;
	observed_str += "/amcl_pose";
	ros::Subscriber observed_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(observed_str, 1, &observed_pose_received);
	ros::Subscriber observer_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &observer_pose_received);
	ros::Subscriber observer_amcl_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose_amcl", 1, &observer_amcl_pose_received);

	ros::Rate loopRate(10);
	while (ros::ok()) {
		if(observed_msg_received == true && observer_msg_received == true && observer_amcl_msg_received == true)
		{
			// cout<<"Observed Time Stamp: "<<observed_pose.header.stamp<<endl<<"mean:"<<endl<<observed_pose.pose.pose.position.x<<", "<<observed_pose.pose.pose.position.y<<endl;
			// cout<<"Observer Time Stamp: "<<observer_pose.header.stamp<<endl<<"mean:"<<endl<<observer_pose.pose.pose.position.x<<", "<<observer_pose.pose.pose.position.y<<endl;
			// cout<<"Observer amcl Time Stamp: "<<observer_amcl_pose.header.stamp<<endl<<"mean:"<<endl<<observer_amcl_pose.pose.pose.position.x<<", "<<observer_amcl_pose.pose.pose.position.y<<endl;
			//prepare the amcl pose of observed from observer
			//use (acccurate observer pose - acccurate observed pose) to mimic accurate laser scan
			geometry_msgs::PoseWithCovarianceStamped amcl_pose_observed;
			amcl_pose_observed = observer_amcl_pose;
			amcl_pose_observed.header.stamp = observed_pose.header.stamp;
			amcl_pose_observed.header.frame_id = observed_id;
			amcl_pose_observed.pose.pose.position.x = observed_pose.pose.pose.position.x - observer_pose.pose.pose.position.x + observer_amcl_pose.pose.pose.position.x;
			amcl_pose_observed.pose.pose.position.y = observed_pose.pose.pose.position.y - observer_pose.pose.pose.position.y + observer_amcl_pose.pose.pose.position.y;
			amcl_pose_observed.pose.pose.orientation.z = observed_pose.pose.pose.orientation.z;
			for(int i=0;i<2;i++)
			{
				for(int j=0;j<2;j++)
				{
					amcl_pose_observed.pose.covariance[i*6+j] += LASER_UNCERTAINTY;
				}
			}
			amcl_pose_observed.pose.covariance[35] = UNKNOWN_COV;
			// cout<<"published amcl Time Stamp: "<<amcl_pose_observed.header.stamp<<endl<<"mean:"<<endl<<amcl_pose_observed.pose.pose.position.x<<", "<<amcl_pose_observed.pose.pose.position.y<<endl;

			//publish amcl_pose_observed
			pose_pub.publish(amcl_pose_observed);

			observed_msg_received = false;
			observer_msg_received = false;
			observer_amcl_msg_received = false;
		}
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

void observed_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	observed_msg_received = true;
	observed_pose.header = msg->header;
	observed_pose.pose = msg->pose;
}

void observer_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	observer_msg_received = true;
	observer_pose.header = msg->header;
	observer_pose.pose = msg->pose;
}

void observer_amcl_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	observer_amcl_msg_received = true;
	observer_amcl_pose.header = msg->header;
	observer_amcl_pose.pose = msg->pose;
}