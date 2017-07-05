#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

std::string node_name = "message_translator";
std::string msg_header = "[MSG TRANS]";
int publish_rate = 10; //Hz
std::string uav_name = "uav_0";
std::string uav_id_str = "0";
int uav_id = 0;

//Subscriber & Publisher
std::string topic_sub_pos_update = "/uav_0/mavros/local_position/pose";
std::string topic_sub_vel_update = "/uav_0/mavros/local_position/velocity";
std::string topic_sub_vel_command = "/uav_0/cmd_vel";
std::string topic_pub_vel_command = "/uav_0/mavros/setpoint_velocity/cmd_vel";
std::string topic_pub_vel_update = "/uav_0/velocity";
std::string topic_pub_pos_update = "/uav_0/position";

ros::Subscriber sub_pos_update;
ros::Subscriber sub_vel_update;
ros::Subscriber sub_vel_command;
ros::Publisher pub_vel_command;
ros::Publisher pub_vel_update;
ros::Publisher pub_pos_update;

//Logic functions
int initialize();
void prepare_msgs();
void publish_msgs();
void notify_errors();
bool is_timeout(ros::Time stamp_1, ros::Time stamp_2, double timeout);

//Callback functions
void pos_update_cb(const geometry_msgs::PoseStamped _pos);
void vel_update_cb(const geometry_msgs::TwistStamped _vel);
void vel_command_cb(const geometry_msgs::TwistStamped _vel_cmd);

//Variables
geometry_msgs::PoseStamped last_pose_update;
geometry_msgs::TwistStamped last_vel_update;
geometry_msgs::TwistStamped last_vel_command;

geometry_msgs::Vector3 pos_update_msg;
geometry_msgs::Vector3 vel_update_msg;
geometry_msgs::TwistStamped vel_command_msg;

double timeout_pose_update = 1.0;
double timeout_vel_update = 1.0;
double timeout_vel_command = 1.0;

bool flag_pose_update_timeout = true;
bool flag_vel_update_timeout = true;
bool flag_vel_command_timeout = true;

bool disable_z_vel_cmd = true;

ros::Time last_notify_time;
double error_notify_interval = 5.0; //sec

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	nh_param.param<std::string>("uav_name", uav_name, uav_name);
	nh_param.param<int>("uav_id", uav_id, uav_id);
	nh_param.param<std::string>("topic_sub_pos_update", topic_sub_pos_update, topic_sub_pos_update);
	nh_param.param<std::string>("topic_sub_vel_update", topic_sub_vel_update, topic_sub_vel_update);
	nh_param.param<std::string>("topic_sub_vel_command", topic_sub_vel_command, topic_sub_vel_command);
	nh_param.param<std::string>("topic_pub_vel_command", topic_pub_vel_command, topic_pub_vel_command);
	nh_param.param<std::string>("topic_pub_vel_update", topic_pub_vel_update, topic_pub_vel_update);
	nh_param.param<std::string>("topic_pub_pos_update", topic_pub_pos_update, topic_pub_pos_update);
	nh_param.param<double>("timeout_pose_update", timeout_pose_update, timeout_pose_update);
	nh_param.param<double>("timeout_vel_update", timeout_vel_update, timeout_vel_update);
	nh_param.param<double>("timeout_vel_command", timeout_vel_command, timeout_vel_command);
	nh_param.param<bool>("disable_z_vel_cmd", disable_z_vel_cmd, disable_z_vel_cmd);


	sub_pos_update = nh.subscribe<geometry_msgs::PoseStamped>(topic_sub_pos_update, 10, pos_update_cb);
	sub_vel_update = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_vel_update, 10, vel_update_cb);
	sub_vel_command = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_vel_command, 10, vel_command_cb);

	pub_vel_command = nh.advertise<geometry_msgs::TwistStamped>(topic_pub_vel_command, 10);
	pub_vel_update = nh.advertise<geometry_msgs::Vector3>(topic_pub_vel_update, 10);
	pub_pos_update = nh.advertise<geometry_msgs::Vector3>(topic_pub_pos_update, 10);

	ros::Rate loop_rate(publish_rate);

	if (initialize() == 0){	
		ROS_INFO("%s Initialized.", msg_header.c_str());
	} else {
		ROS_ERROR("%s Failed to initialized.", msg_header.c_str());
		return -1;
	}

	while (ros::ok()){
		prepare_msgs();
		notify_errors();
		publish_msgs();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


void pos_update_cb(const geometry_msgs::PoseStamped _pos){
	last_pose_update = _pos;
}
void vel_update_cb(const geometry_msgs::TwistStamped _vel){
	last_vel_update = _vel;
}
void vel_command_cb(const geometry_msgs::TwistStamped _vel_cmd){
	last_vel_command = _vel_cmd;
}

int initialize(){
	//Get UAV_id in string
	std::ostringstream convert;
	convert << uav_id;
	uav_id_str = convert.str();

	return 0;
}

void prepare_msgs(){
	ros::Time common_stamp = ros::Time::now();
	//Position update
	if (is_timeout(last_pose_update.header.stamp, common_stamp, timeout_pose_update)){
		flag_pose_update_timeout = true;
	} else {
		flag_pose_update_timeout = false;
	}
	pos_update_msg.x = last_pose_update.pose.position.x;
	pos_update_msg.y = last_pose_update.pose.position.y;
	pos_update_msg.z = last_pose_update.pose.position.z;

	//Velocity update
	if (is_timeout(last_vel_update.header.stamp, common_stamp, timeout_vel_update)){
		vel_update_msg.x = 0.0;
		vel_update_msg.y = 0.0;
		vel_update_msg.z = 0.0;
		flag_vel_update_timeout = true;
	} else {
		vel_update_msg.x = last_vel_update.twist.linear.x;
		vel_update_msg.y = last_vel_update.twist.linear.y;
		vel_update_msg.z = last_vel_update.twist.linear.z;
		flag_vel_update_timeout = false;
	}

	//Velocity command
	if (is_timeout(last_vel_command.header.stamp, common_stamp, timeout_vel_command)){
		vel_command_msg.twist.linear.x = 0.0;
		vel_command_msg.twist.linear.y = 0.0;
		vel_command_msg.twist.linear.z = 0.0;
		flag_vel_command_timeout = true;
	} else {
		vel_command_msg.twist.linear.x = last_vel_command.twist.linear.x;
		vel_command_msg.twist.linear.y = last_vel_command.twist.linear.y;
		if (disable_z_vel_cmd) { vel_command_msg.twist.linear.z = 0.0; }
		else { vel_command_msg.twist.linear.z = last_vel_command.twist.linear.z; }
		flag_vel_command_timeout = false;
	}
}

void publish_msgs(){
	//Position update
	pub_pos_update.publish(pos_update_msg);

	//Velocity update
	pub_vel_update.publish(vel_update_msg);

	//Velocity command
	pub_vel_command.publish(vel_command_msg);
}

void notify_errors(){
	if (is_timeout(last_notify_time, ros::Time::now(), error_notify_interval)){
		if (flag_pose_update_timeout) ROS_INFO("%s POS EST timeout. Publish last received values.", msg_header.c_str());
		if (flag_vel_update_timeout) ROS_INFO("%s VEL EST timeout. Assume and publish zero values.", msg_header.c_str());
		if (flag_vel_command_timeout) ROS_INFO("%s VEL CMD timeout. Safety mode: Attempt position hold.", msg_header.c_str());
		last_notify_time = ros::Time::now();
	}
}

bool is_timeout(ros::Time stamp_1, ros::Time stamp_2, double timeout){
	if (fabs(stamp_1.toSec() - stamp_2.toSec()) >= fabs(timeout)) return true;
	else return false;
}