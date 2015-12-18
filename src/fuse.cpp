#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "Eigen/LU"
#include <iostream>
#include <fstream>

#define EXPIRE_TIME 0.2

using namespace std;

int robot_id;

int received_stamp;
int self_amcl_stamp;

bool received_msg_received = false;
bool self_amcl_msg_received = false;

geometry_msgs::PoseWithCovarianceStamped received_pose;
geometry_msgs::PoseWithCovarianceStamped self_amcl_pose;
geometry_msgs::PoseWithCovarianceStamped self_pose;
void received_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void self_amcl_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void self_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

int main(int argc, char ** argv)
{	
	if (argc < 2) {
		ROS_ERROR("You must specify robot id.");
		return -1;
	}
	char *id = argv[1];
	robot_id = atoi(id);

	ros::init(argc, argv, "fuse");
	ros::NodeHandle nh;

	ros::Subscriber received_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pose_comm", 1, &received_pose_received);
	ros::Subscriber self_amcl_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose_amcl", 1, &self_amcl_pose_received);
	ros::Subscriber self_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &self_pose_received);
	ros::Publisher initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);

	string file_name = "/home/kimiwings/Desktop/robot_";
	file_name += id;
	file_name += "_track.txt";
	ofstream fout(file_name.c_str());
	int counter = 0;

	ros::Rate loopRate(10);
	while (ros::ok()) 
	{
		if(received_msg_received == true && self_amcl_msg_received == true)
		{
			//check if expired
			if((self_amcl_stamp-received_stamp)*(self_amcl_stamp-received_stamp) < EXPIRE_TIME*EXPIRE_TIME)
			{
				//cout<<"Self Time Stamp: "<<self_amcl_pose.header.stamp<<endl<<"mean:"<<endl<<self_amcl_pose.pose.pose.position.x<<", "<<self_amcl_pose.pose.pose.position.y<<endl;
				//convolve
				Eigen::MatrixXf received_pose_cov(2,2);
				for(int i=0; i<2; i++)
				{
					for(int j=0; j<2; j++)
					{
						received_pose_cov(i,j) = received_pose.pose.covariance[6*i+j]*((self_amcl_stamp-received_stamp)*(self_amcl_stamp-received_stamp)+1);
					}
				}

				Eigen::MatrixXf self_amcl_pose_cov(2,2);
				for(int i=0; i<2; i++)
				{
					for(int j=0; j<2; j++)
					{
						self_amcl_pose_cov(i,j) = self_amcl_pose.pose.covariance[6*i+j];
					}
				}


				Eigen::MatrixXf convolved_cov(2,2);
				convolved_cov = received_pose_cov.inverse() + self_amcl_pose_cov.inverse();
				convolved_cov = convolved_cov.inverse();

				Eigen::VectorXf received_pose_mean(2);
				received_pose_mean(0) = received_pose.pose.pose.position.x;
				received_pose_mean(1) = received_pose.pose.pose.position.y;

				Eigen::VectorXf self_amcl_pose_mean(2);
				self_amcl_pose_mean(0) = self_amcl_pose.pose.pose.position.x;
				self_amcl_pose_mean(1) = self_amcl_pose.pose.pose.position.y;

				Eigen::VectorXf convolved_mean(2);
				convolved_mean = convolved_cov * (received_pose_cov.inverse()*received_pose_mean + self_amcl_pose_cov.inverse()*self_amcl_pose_mean);
				// cout<<"---------------------------------------"<<endl;
				// cout<<"robot_"<<robot_id<<endl;
				// cout<<"received "<<endl<<received_pose_mean<<endl;
				// cout<<"self amcl"<<endl<<self_amcl_pose_mean<<endl;
				// cout<<"convolved "<<endl<<convolved_mean<<endl;
				// cout<<"true "<<endl<<self_pose.pose.pose.position.x<<endl<<self_pose.pose.pose.position.y<<endl;
				// cout<<"---------------------------------------"<<endl;
				if(!fout)
				{
					cout << "File could not be opened." << endl;
				}
				else 
				{
					cout<<"Write robot "<<id<<" track record no. "<<counter++<<endl;
					fout<<self_amcl_pose.pose.pose.position.x<<" "<<self_amcl_pose.pose.pose.position.y<<" "<<self_pose.pose.pose.position.x<<" "<<self_pose.pose.pose.position.y<<" "<<convolved_mean(0)<<" "<<convolved_mean(1)<<";"<<endl;
				}
				double convolved_pose_x = convolved_mean(0);	
				double convolved_pose_y = convolved_mean(1);	

				geometry_msgs::PoseWithCovarianceStamped convolved_pose = self_amcl_pose;
				convolved_pose.header.stamp = ros::Time::now();
				convolved_pose.header.frame_id = "/map";
				convolved_pose.pose.pose.position.x = convolved_pose_x;
				convolved_pose.pose.pose.position.y = convolved_pose_y;
				for(int i=0; i<2; i++)
				{
					for(int j=0; j<2; j++)
					{
						convolved_pose.pose.covariance[6*i+j] = convolved_cov(i,j);
					}
				}
				//ROS_INFO("Publisher");
				initialpose_pub.publish(convolved_pose);
			}

			received_msg_received = false;
			self_amcl_msg_received = false;
		}
		ros::spinOnce();
		loopRate.sleep();
	}
	cout<<"close file";
	fout.close();
	return 0;
}

void received_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	// cout<<"Received Time Stamp: "<<msg->header.stamp<<endl;
	received_stamp = msg->header.stamp.toSec();
	char robot_id_char[2];
	robot_id_char[0] = robot_id + '0';
	robot_id_char[1] = '\0';
	string received_id = msg->header.frame_id;
	const char *received_id_char = received_id.c_str();
	if(strcmp(received_id_char, robot_id_char) == 0)
	{
		received_msg_received = true;
		received_pose.header = msg->header;
		received_pose.pose = msg->pose;
	}
}

void self_amcl_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	// cout<<"Self Time Stamp: "<<msg->header.stamp<<endl;
	self_amcl_stamp = msg->header.stamp.toSec();
	self_amcl_msg_received = true;
	self_amcl_pose.header = msg->header;
	self_amcl_pose.pose = msg->pose;
}

void self_pose_received(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	// cout<<"Self Time Stamp: "<<msg->header.stamp<<endl;
	self_pose.header = msg->header;
	self_pose.pose = msg->pose;
}