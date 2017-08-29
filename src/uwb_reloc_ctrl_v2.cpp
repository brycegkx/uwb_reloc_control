#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/Range.h>

#include <uwb_reloc_msgs/RelativeInfoStamped.h>
#include <uwb_reloc_msgs/TwistWithDistanceStamped.h>

#include <mavros_msgs/RCIn.h>

#include <string>

#define no_of_uav 3
#define control_rate 20

std::string node_name = "fruitdove_reloc_ctrl";
std::string message_header = "[UAV_RELOC_CTRL]";

// Identifier
std::string uav_name = "uav_0";
int uav_id = 0;
std::string uav_id_str;

// Control param - Model: Double integrator
double delta_t = 0.5; //sec
double epsilon_1 = 1.0;
double epsilon_2 = 0.007;
double kappa = 1.0;

double target_altitude = 1.5; //m
double altitude_tolerance = 0.1;

// State storage
uwb_reloc_msgs::TwistWithDistanceStamped uav_states[no_of_uav];
double desired_distance[no_of_uav];
bool uniform_formation = true;
double uniform_distance = 4.0; //meters

// Output variables
double acc_sp[3] = {0.0, 0.0, 0.0};
double vel_sp[3] = {0.0, 0.0, 0.0};
double pos_sp[3] = {0.0, 0.0, 0.0};

double vel_cmd_lim_max[3] = {0.5, 0.5, 1.0};
double vel_cmd_lim_min[3] = {-0.5, -0.5, -1.0};

// Timeout variable
double stamp_timeout = 0.5; //sec
double inter_stamp_timeout = 0.5; //sec

// NaN detection
bool flag_nan_rel_pos = false;
ros::Time last_warning_nan_rel_pos;
double nan_notify_interval = 3.0; //sec

// Flight modes tracker
int sub_mode_channel = 6;
enum offb_sub_mode {mode_hov, mode_alt, mode_pos};
offb_sub_mode last_mode;
offb_sub_mode curr_mode;
int thres_1 = 1000;
int thres_2 = 1600;
ros::Time last_rc_signal_time;
double rc_signal_timeout = 1.0; //sec

// Local position tracker
geometry_msgs::PoseStamped curr_pose;
double pose_timeout = 0.5; //sec
sensor_msgs::Range curr_range;
ros::Time last_range_time;
double curr_altitude = 0.0;

//Subscribers and Publishers
std::string topic_sub_states = "/common/vicon/uav_states";
std::string topic_sub_auto_mode_rc = "/uav_0/mavros/rc/in";
std::string topic_sub_local_pos = "/uav_0/mavros/local_position/pose";
std::string topic_sub_range = "/uav_0/mavros/distance_sensor/hrlv_ez4_pub";

std::string topic_pub_vel_ctrl = "/uav_0/cmd_vel";
std::string topic_pub_vel_cmd_common = "/common/cmd_vel";

ros::Subscriber sub_states;
ros::Subscriber sub_rc;
ros::Subscriber sub_pose;
ros::Subscriber sub_range;
ros::Publisher pub_vel_cmd;
ros::Publisher pub_vel_cmd_common;

// State-machine functions
int initialize();
void calculate_control_setpoints_xy();
void calculate_control_setpoints_z();
void reset_control_setpoints();
void saturate_velocity_setpoints();
void publish_velocity_commands();
void notify_nan_relative_position();

void check_flight_mode();

// Utility functions
bool is_timeout(ros::Time stamp_1, ros::Time stamp_2, double timeout);

// Callback functions
void states_cb(const uwb_reloc_msgs::RelativeInfoStamped state);
void rc_signal_cb(const mavros_msgs::RCIn rc_signals);
void pose_cb(const geometry_msgs::PoseStamped _pose);
void range_cb(const sensor_msgs::Range _range);

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	nh_param.param<std::string>("uav_name", uav_name, uav_name);
	nh_param.param<int>("uav_id", uav_id, uav_id);
	nh_param.param<int>("sub_mode_channel", sub_mode_channel, sub_mode_channel);
	nh_param.param<std::string>("topic_sub_states", topic_sub_states, topic_sub_states);
	nh_param.param<std::string>("topic_sub_auto_mode_rc", topic_sub_auto_mode_rc, topic_sub_auto_mode_rc);
	nh_param.param<std::string>("topic_sub_local_pos", topic_sub_local_pos, topic_sub_local_pos);
	nh_param.param<std::string>("topic_sub_range", topic_sub_range, topic_sub_range);
	nh_param.param<std::string>("topic_pub_vel_ctrl", topic_pub_vel_ctrl, topic_pub_vel_ctrl);
  	nh_param.param<std::string>("topic_pub_vel_cmd_common", topic_pub_vel_cmd_common, topic_pub_vel_cmd_common);
	nh_param.param<double>("delta_t", delta_t, delta_t);
	nh_param.param<double>("epsilon_1", epsilon_1, epsilon_1);
	nh_param.param<double>("epsilon_2", epsilon_2, epsilon_2);
	nh_param.param<double>("kappa", kappa, kappa);

	nh_param.param<bool>("uniform_formation", uniform_formation, uniform_formation);
	nh_param.param<double>("uniform_distance", uniform_distance, uniform_distance);

	sub_range = nh.subscribe<sensor_msgs::Range>(topic_sub_range, 10, range_cb);
	sub_states = nh.subscribe<uwb_reloc_msgs::RelativeInfoStamped>(topic_sub_states, 10, states_cb);
    sub_rc = nh.subscribe<mavros_msgs::RCIn>(topic_sub_auto_mode_rc, 10, rc_signal_cb);
    sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(topic_sub_local_pos, 10, pose_cb);

	pub_vel_cmd = nh.advertise<geometry_msgs::TwistStamped>(topic_pub_vel_ctrl, 10);
  	pub_vel_cmd_common = nh.advertise<geometry_msgs::TwistStamped>(topic_pub_vel_cmd_common, 10);

	if (initialize() != 0) return 1;
	ROS_WARN("%s: Initialized. UAV_ID: %d", message_header.c_str(), uav_id);

	ros::Rate loop_rate(control_rate);
	while (ros::ok()){
        reset_control_setpoints();
        check_flight_mode();
        switch (curr_mode){
            case mode_hov:
                break;
            case mode_alt:
                calculate_control_setpoints_z();
                break;
            case mode_pos:
                calculate_control_setpoints_xy();
                calculate_control_setpoints_z();
                break;
            default:
                break;
        }
        saturate_velocity_setpoints();
		publish_velocity_commands();

		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_WARN("%s: Exited cleanly. UAV_ID: %d", message_header.c_str(), uav_id);
	return 0;
}

void states_cb(const uwb_reloc_msgs::RelativeInfoStamped state){
	int rqstr_id = state.rqstrId;
	int rspdr_id = state.rspdrId;
	int target_index = rqstr_id;
	uwb_reloc_msgs::TwistWithDistanceStamped self_info;
	uwb_reloc_msgs::TwistWithDistanceStamped othr_info;
	if (rqstr_id == uav_id){
		//Update other state
		target_index = rspdr_id;
		othr_info.header = state.header;
		othr_info.uav_id = rspdr_id;
		othr_info.abs_vel = state.rspdrVel;
		othr_info.rel_pos.x = state.rlpos_RqstToRspdr.x;
    	othr_info.rel_pos.y = state.rlpos_RqstToRspdr.y;
    	othr_info.rel_pos.z = state.rlpos_RqstToRspdr.z;
		othr_info.distance = state.distance;

		//Update self state
		self_info.header = state.header;
		self_info.uav_id = rqstr_id;
		self_info.abs_vel = state.rqstrVel;
		self_info.rel_pos.x = 0.0;
		self_info.rel_pos.y = 0.0;
		self_info.rel_pos.z = 0.0;
		self_info.distance = state.distance;

		//Store information
		uav_states[target_index] = othr_info;
		uav_states[uav_id] = self_info;
	}
	if (rspdr_id == uav_id){
		//Update other state
		target_index = rqstr_id;
		othr_info.header = state.header;
		othr_info.uav_id = rqstr_id;
		othr_info.abs_vel = state.rqstrVel;
		othr_info.rel_pos.x = -state.rlpos_RqstToRspdr.x;
    	othr_info.rel_pos.y = -state.rlpos_RqstToRspdr.y;
    	othr_info.rel_pos.z = -state.rlpos_RqstToRspdr.z;
		othr_info.distance = state.distance;

		//Update self state
		self_info.header = state.header;
		self_info.uav_id = rspdr_id;
		self_info.abs_vel = state.rspdrVel;
		self_info.rel_pos.x = 0.0;
		self_info.rel_pos.y = 0.0;
		self_info.rel_pos.z = 0.0;
		self_info.distance = state.distance;

		//Store information
		uav_states[target_index] = othr_info;
		uav_states[uav_id] = self_info;
	}

}

void rc_signal_cb(const mavros_msgs::RCIn rc_signals){
    int rc_signal = rc_signals.channels[sub_mode_channel];
    last_mode = curr_mode;
    if (rc_signal <= thres_1) curr_mode = mode_hov;
    else {
        if (rc_signal > thres_1 && rc_signal < thres_2) curr_mode = mode_alt;
        else curr_mode = mode_pos;
    }
    last_rc_signal_time = ros::Time::now();
    // Notify
    if (last_mode != curr_mode){
        switch (curr_mode) {
            case mode_hov:
                ROS_WARN("%s Switched mode: HOVER", message_header.c_str());
                break;
            case mode_alt:
                ROS_WARN("%s Switched mode: CONTROL Z", message_header.c_str());
                break;
            case mode_pos:
                ROS_WARN("%s Switched mode: CONTROL XYZ", message_header.c_str());
                break;
        }
    }
}

void pose_cb(const geometry_msgs::PoseStamped _pose){
    curr_pose = _pose;
}

void range_cb(const sensor_msgs::Range _range){
	curr_range = _range;
	last_range_time = ros::Time::now();
	if (curr_range.range > curr_range.min_range && curr_range.range < curr_range.max_range){
		curr_altitude = curr_range.range;
	}
}

int initialize(){
  //Get UAV_id in string
  std::ostringstream convert;
  convert << uav_id;
  uav_id_str = convert.str();

	//Stupidity check: UAV_id is greater than (UAV_number - 1). Return 1 to indicate problem.
	if (uav_id > (no_of_uav - 1)){
		ROS_ERROR("%s: uav_id greater than number of UAV. Invalid. Exit!", message_header.c_str());
		return 1;
	}

	int i = 0;

	//Initialize desired distances between UAV. This decides the formation shape
	for (i = 0; i < no_of_uav; i++){
		desired_distance[i] = uniform_distance;
	}

	//Redundancy check: Ensure that vel_cmd_lim_max is actually greater than vel_cmd_lim_min
	double actual_max, actual_min;
	for (i = 0; i < 3; i++){
		if (vel_cmd_lim_max[i] >= vel_cmd_lim_min[i]) {
			actual_max = vel_cmd_lim_max[i];
			actual_min = vel_cmd_lim_min[i];
		} else {
			actual_max = vel_cmd_lim_min[i];
			actual_min = vel_cmd_lim_max[i];
		}
		vel_cmd_lim_max[i] = actual_max;
		vel_cmd_lim_min[i] = actual_min;
	}

	//Safety check: Reset control setpoints
	reset_control_setpoints();

	return 0;
}

void reset_control_setpoints(){
    for (int i = 0; i < 3; i++){
        acc_sp[i] = 0.0;
    	vel_sp[i] = 0.0;
    	pos_sp[i] = 0.0;

    }
}

void calculate_control_setpoints_xy(){
	bool has_valid_control = false;

	uwb_reloc_msgs::TwistWithDistanceStamped self_state;
	uwb_reloc_msgs::TwistWithDistanceStamped othr_state;

	double self_vel[2] = {0.0, 0.0};
	double othr_vel[2] = {0.0, 0.0};
	double rel_vel[2] = {0.0, 0.0};
	double othr_rel_pos[2] = {0.0, 0.0};

	double distance = 0.0;
	double diff_sqr = 0.0;

	double sum_rel_vel[2] = {0.0, 0.0};
	double sum_rel_pos[2] = {0.0, 0.0};

	acc_sp[0] = 0.0;
	acc_sp[1] = 0.0;
	acc_sp[2] = 0.0;

	for (int tmp_id = 0; tmp_id < no_of_uav; tmp_id++){
		if (tmp_id != uav_id){
			self_state = uav_states[uav_id];
			othr_state = uav_states[tmp_id];
			if (is_timeout(self_state.header.stamp, ros::Time::now(), stamp_timeout) ||
				is_timeout(self_state.header.stamp, othr_state.header.stamp, inter_stamp_timeout)){
				// Timeout
				sum_rel_vel[0] += 0.0;
				sum_rel_vel[1] += 0.0;
				sum_rel_pos[0] += 0.0;
				sum_rel_pos[1] += 0.0;
        	// ROS_INFO("TO");
			} else {
				// Valid
				self_vel[0] = self_state.abs_vel.x;
				self_vel[1] = self_state.abs_vel.y;

				othr_vel[0] = othr_state.abs_vel.x;
				othr_vel[1] = othr_state.abs_vel.y;

				othr_rel_pos[0] = othr_state.rel_pos.x;
				othr_rel_pos[1] = othr_state.rel_pos.y;

				rel_vel[0] = othr_vel[0] - self_vel[0];
				rel_vel[1] = othr_vel[1] - self_vel[1];

				distance = othr_state.distance;

				diff_sqr = distance * distance - desired_distance[tmp_id] * desired_distance[tmp_id];

				sum_rel_vel[0] += rel_vel[0];
				sum_rel_vel[1] += rel_vel[1];

				sum_rel_pos[0] += diff_sqr * othr_rel_pos[0];
				sum_rel_pos[1] += diff_sqr * othr_rel_pos[1];

				//Finally, check for NaN relative position
				if (isnan(othr_rel_pos[0]) ||
					isnan(othr_rel_pos[1]) ||
					isnan(sum_rel_pos[0]) ||
					isnan(sum_rel_pos[1]))
				{
					has_valid_control = false;
					flag_nan_rel_pos = true;
				}
				else has_valid_control = true;
			}
		}
	}

	if (has_valid_control){
		acc_sp[0] = epsilon_1 * kappa * sum_rel_vel[0] + 2 * epsilon_2 * sum_rel_pos[0];
		acc_sp[1] = epsilon_1 * kappa * sum_rel_vel[1] + 2 * epsilon_2 * sum_rel_pos[1];

		vel_sp[0] = self_vel[0] + acc_sp[0] * delta_t;
		vel_sp[1] = self_vel[1] + acc_sp[1] * delta_t;

		// pos_sp[0] = pos_sp[0] + pos_sp[0] * delta_t;
		// pos_sp[1] = pos_sp[1] + pos_sp[1] * delta_t;
	} else {
		reset_control_setpoints();
	}
}

void calculate_control_setpoints_z(){
    if (is_timeout(last_range_time, ros::Time::now(), pose_timeout)){
        vel_sp[2] = 0.0;
    } else {
        double altitude_error = target_altitude - curr_altitude;
        if (fabs(altitude_error) <= fabs(altitude_tolerance) ){
            vel_sp[2] = 0.0;
        } else {
            if (altitude_error > 0.0) vel_sp[2] = 0.5;
            else vel_sp[2] = -0.5;
        }
    }
}

void saturate_velocity_setpoints(){
	for (int i = 0; i < 3; i++){
		if (vel_sp[i] >= vel_cmd_lim_max[i]) vel_sp[i] = vel_cmd_lim_max[i];
		if (vel_sp[i] <= vel_cmd_lim_min[i]) vel_sp[i] = vel_cmd_lim_min[i];
	}
}

void publish_velocity_commands(){
	geometry_msgs::TwistStamped vel_cmd_msg;
	vel_cmd_msg.twist.linear.x = vel_sp[0];
	vel_cmd_msg.twist.linear.y = vel_sp[1];
	vel_cmd_msg.twist.linear.z = vel_sp[2];
	vel_cmd_msg.twist.angular.x = 0.0;
	vel_cmd_msg.twist.angular.y = 0.0;
	vel_cmd_msg.twist.angular.z = 0.0;
  	vel_cmd_msg.header.frame_id = uav_id_str;
  	vel_cmd_msg.header.stamp = ros::Time::now();

  	pub_vel_cmd.publish(vel_cmd_msg);
  	pub_vel_cmd_common.publish(vel_cmd_msg);
}

bool is_timeout(ros::Time stamp_1, ros::Time stamp_2, double timeout){
	if (fabs(stamp_1.toSec() - stamp_2.toSec()) >= fabs(timeout)) return true;
	else return false;
}

void notify_nan_relative_position(){
	if (flag_nan_rel_pos == true &&
		ros::Time::now().toSec() - last_warning_nan_rel_pos.toSec() > nan_notify_interval){
		ROS_WARN("[RELOC CTRL] NaN relative position");
		last_warning_nan_rel_pos = ros::Time::now();
	}
}

void check_flight_mode(){
    if (is_timeout(last_rc_signal_time, ros::Time::now(), rc_signal_timeout)) {
        curr_mode = mode_hov;
    }
}
