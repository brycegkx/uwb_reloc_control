#include <stdio.h>
#include <stdlib.h>

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <uwb_reloc_msgs/RelativeInfoStamped.h>
#include <uwb_reloc_msgs/uwbTalkData.h>
#include <uwb_reloc/uwbTalkData.h>

#include "rl_format.h"
#include "local_pos_sp_format.h"
#include "csv_parse_engine.h"

std::string node_name = "csv_parser";

std::string topic_sub_uav_info = "/uav_0/rlForControl";
std::string topic_pub_markers = "/visualization_marker_array";

std::string topic_sub_uav_cmd_d0 = "/uav_0/mavros/setpoint_velocity/cmd_vel";
std::string topic_sub_uav_cmd_d1 = "/uav_1/mavros/setpoint_velocity/cmd_vel";
std::string topic_sub_uav_cmd_d2 = "/uav_2/mavros/setpoint_velocity/cmd_vel";
std::string topic_sub_uav_vel_d0 = "/uav_0/mavros/local_position/velocity";
std::string topic_sub_uav_vel_d1 = "/uav_1/mavros/local_position/velocity";
std::string topic_sub_uav_vel_d2 = "/uav_2/mavros/local_position/velocity";

// Upper board data
std::string upp_basic_path = "//home//chung//data//UPPER//";
std::string upp_data_set = "0822_scienceCentre//";
std::string upp_uav_id = "d0//";
std::string upp_file_name = "rl0.csv";

// PX4 data
std::string px4_basic_path = "//home//chung//data//PX4//";
std::string px4_data_set = "0822_scienceCentre//";
std::string px4_uav_id = "d0//";
std::string px4_file_name = "sess001//log002//log002_vehicle_local_position_setpoint_0.csv";

//Callback functions
void uav_info_cb(const uwb_reloc_msgs::RelativeInfoStamped info);
void uav_cmd_d0_cb(const geometry_msgs::TwistStamped vel);
void uav_cmd_d1_cb(const geometry_msgs::TwistStamped vel);
void uav_cmd_d2_cb(const geometry_msgs::TwistStamped vel);
void uav_vel_d0_cb(const geometry_msgs::TwistStamped vel);
void uav_vel_d1_cb(const geometry_msgs::TwistStamped vel);
void uav_vel_d2_cb(const geometry_msgs::TwistStamped vel);

double viz_rate = 20.0;

double d_01, d_12, d_02;
std::string d_str_01, d_str_02, d_str_12;
geometry_msgs::TwistStamped cmd_d0, cmd_d1, cmd_d2;
geometry_msgs::TwistStamped vel_d0, vel_d1, vel_d2;

rl temp_rl_info;

int main(int argc, char** argv) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    ros::Publisher pub_pose_d0 = nh.advertise<geometry_msgs::PoseStamped>("poseupdate_d0", 10);
    ros::Publisher pub_pose_d1 = nh.advertise<geometry_msgs::PoseStamped>("poseupdate_d1", 10);
    ros::Publisher pub_pose_d2 = nh.advertise<geometry_msgs::PoseStamped>("poseupdate_d2", 10);

    ros::Publisher pub_point_d0 = nh.advertise<geometry_msgs::PointStamped>("point_d0", 10);
    ros::Publisher pub_point_d1 = nh.advertise<geometry_msgs::PointStamped>("point_d1", 10);
    ros::Publisher pub_point_d2 = nh.advertise<geometry_msgs::PointStamped>("point_d2", 10);

    ros::Publisher pub_markers = nh.advertise<visualization_msgs::MarkerArray>(topic_pub_markers, 10);

    ros::Subscriber sub_uav_info = nh.subscribe<uwb_reloc_msgs::RelativeInfoStamped>(topic_sub_uav_info, 10, uav_info_cb);
    ros::Subscriber sub_uav_d0_cmd = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_uav_cmd_d0, 10, uav_cmd_d0_cb);
    ros::Subscriber sub_uav_d1_cmd = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_uav_cmd_d1, 10, uav_cmd_d1_cb);
    ros::Subscriber sub_uav_d2_cmd = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_uav_cmd_d2, 10, uav_cmd_d2_cb);
    ros::Subscriber sub_uav_d0_vel = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_uav_vel_d0, 10, uav_vel_d0_cb);
    ros::Subscriber sub_uav_d1_vel = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_uav_vel_d1, 10, uav_vel_d1_cb);
    ros::Subscriber sub_uav_d2_vel = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_uav_vel_d2, 10, uav_vel_d2_cb);

    ros::Rate publish_rate(100);


    ros::Time start_time = ros::Time::now();
    long time_stamp_ms = 0;
    long ros_ellapsed_time_ms = 0;
    int allowable_time_diff = 3; //ms

    double x_0, x_1, x_2;
    double y_0, y_1, y_2;
    double a, b;

    geometry_msgs::PoseStamped temp_pose_msg;
    geometry_msgs::PointStamped temp_point_msg;
    geometry_msgs::Point temp_point_d0, temp_point_d1, temp_point_d2;
    geometry_msgs::Point temp_point_d01, temp_point_d12, temp_point_d02;

    ros::Duration marker_life_time (1/viz_rate * 0.8);
    while (ros::ok()){
        // -------------------- SOLVE FOR THE POSITIONS --------------------
        x_1 = 3.0;          y_1 = 0.0;
        x_0 = x_1 - d_01;   y_0 = 0.0;

        // Solve by model x = a*y + b
        a = -(y_1 - y_0) / (x_1 - x_0);
        b = -((d_12*d_12 - d_02*d_02)-(x_1*x_1 - x_0*x_0)-(y_1*y_1 - y_0*y_0))/(2.0 * (x_1 - x_0));

        // Gradient descend
        y_2 = 0.0;
        double delta_y = 0.001;
        double residue = 100.0;
        int cycle_limit = 10000;
        int cycle_count = 0;
        while (residue > 0.0){
            y_2 = y_2 + delta_y;
            x_2 = a * y_2 + b;
            residue = (d_12 * d_12) - (x_2 - x_1) * (x_2 - x_1) - (y_2 - y_1) * (y_2 - y_1);
            cycle_count++;
            if (cycle_count >= cycle_limit){
                break;
                ROS_ERROR("FAILED");
            }
        }

        // -------------------- PLOT DATA IN RVIZ --------------------
        int marker_id = -1;
        temp_pose_msg.header.stamp = ros::Time::now();
        temp_pose_msg.header.frame_id = "base_link";

        temp_pose_msg.pose.position.x = x_0;
        temp_pose_msg.pose.position.y = y_0;
        pub_pose_d0.publish(temp_pose_msg);

        temp_pose_msg.pose.position.x = x_1;
        temp_pose_msg.pose.position.y = y_1;
        pub_pose_d1.publish(temp_pose_msg);

        temp_pose_msg.pose.position.x = x_2;
        temp_pose_msg.pose.position.y = y_2;
        pub_pose_d2.publish(temp_pose_msg);

        temp_point_d0.x = x_0;
        temp_point_d0.y = y_0;

        temp_point_d1.x = x_1;
        temp_point_d1.y = y_1;

        temp_point_d2.x = x_2;
        temp_point_d2.y = y_2;

        temp_point_d01.x = (x_0 + x_1) / 2.0;
        temp_point_d01.y = (y_0 + y_1) / 2.0;

        temp_point_d12.x = (x_1 + x_2) / 2.0;
        temp_point_d12.y = (y_1 + y_2) / 2.0;

        temp_point_d02.x = (x_0 + x_2) / 2.0;
        temp_point_d02.y = (y_0 + y_2) / 2.0;

        temp_point_msg.header.stamp = ros::Time::now();
        temp_point_msg.header.frame_id = "base_link";

        temp_point_msg.point = temp_point_d0;
        pub_point_d0.publish(temp_point_msg);

        temp_point_msg.point = temp_point_d1;
        pub_point_d1.publish(temp_point_msg);

        temp_point_msg.point = temp_point_d2;
        pub_point_d2.publish(temp_point_msg);

        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker ending_marker;
        ros::Time common_stamp = ros::Time();

        // Distance label formatting
        visualization_msgs::Marker temp_dist_label_format;
        temp_dist_label_format.header.stamp = common_stamp;
        temp_dist_label_format.header.frame_id = "base_link";
        temp_dist_label_format.scale.x = 0.3;
        temp_dist_label_format.scale.y = 0.3;
        temp_dist_label_format.scale.z = 0.3;
        temp_dist_label_format.color.a = 1.0;
        temp_dist_label_format.lifetime = marker_life_time;
        temp_dist_label_format.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        // Distance lable 0_1
        visualization_msgs::Marker temp_dist_label_01 = temp_dist_label_format;
        temp_dist_label_01.id = marker_id++;
        temp_dist_label_01.pose.position = temp_point_d01;
        if (fabs(d_01 - 3.0) < 0.5) temp_dist_label_01.color.g = 1.0;
        else temp_dist_label_01.color.r = 1.0;
        temp_dist_label_01.text = d_str_01.c_str();
        marker_array.markers.push_back(temp_dist_label_01);

        // Distance label 1_2
        visualization_msgs::Marker temp_dist_label_12 = temp_dist_label_format;
        temp_dist_label_12.id = marker_id++;
        temp_dist_label_12.pose.position = temp_point_d12;
        if (fabs(d_12 - 3.0) < 0.5) temp_dist_label_12.color.g = 1.0;
        else temp_dist_label_12.color.r = 1.0;
        temp_dist_label_12.text = d_str_12.c_str();
        marker_array.markers.push_back(temp_dist_label_12);

        // Distance label 0_2
        visualization_msgs::Marker temp_dist_label_02 = temp_dist_label_format;
        temp_dist_label_02.id = marker_id++;
        temp_dist_label_02.pose.position = temp_point_d02;
        if (fabs(d_02 - 3.0) < 0.5) temp_dist_label_02.color.g = 1.0;
        else temp_dist_label_02.color.r = 1.0;
        temp_dist_label_02.text = d_str_02.c_str();
        marker_array.markers.push_back(temp_dist_label_02);

        // UAV label formatting
        visualization_msgs::Marker temp_uav_label_format;
        temp_uav_label_format.header.stamp = common_stamp;
        temp_uav_label_format.header.frame_id = "base_link";
        temp_uav_label_format.scale.x = 0.3;
        temp_uav_label_format.scale.y = 0.3;
        temp_uav_label_format.scale.z = 0.3;
        temp_uav_label_format.color.a = 1.0;
        temp_uav_label_format.lifetime = marker_life_time;
        temp_uav_label_format.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        // UAV 0 label
        visualization_msgs::Marker temp_uav_label_0 = temp_uav_label_format;
        temp_uav_label_0.id = marker_id++;
        temp_uav_label_0.pose.position = temp_point_d0;
        temp_uav_label_0.color.r = 1.0;
        temp_uav_label_0.color.b = 1.0;
        temp_uav_label_0.text = "D0: FOLLOWER";
        marker_array.markers.push_back(temp_uav_label_0);

        // Distance label 1_2
        visualization_msgs::Marker temp_uav_label_1 = temp_uav_label_format;
        temp_uav_label_1.id = marker_id++;
        temp_uav_label_1.pose.position = temp_point_d1;
        temp_uav_label_1.color.b = 1.0;
        temp_uav_label_1.color.g = 1.0;
        temp_uav_label_1.text = "D1: LEADER";
        marker_array.markers.push_back(temp_uav_label_1);

        // Distance label 0_2
        visualization_msgs::Marker temp_uav_label_2 = temp_uav_label_format;
        temp_uav_label_2.id = marker_id++;
        temp_uav_label_2.pose.position = temp_point_d2;
        temp_uav_label_2.color.r = 1.0;
        temp_uav_label_2.color.b = 1.0;
        temp_uav_label_2.text = "D2: FOLLOWER";
        marker_array.markers.push_back(temp_uav_label_2);

        // Line formatting
        visualization_msgs::Marker temp_line_format;
        temp_line_format.header.stamp = common_stamp;
        temp_line_format.header.frame_id = "base_link";
        temp_line_format.scale.x = 0.1;
        temp_line_format.scale.y = 0.1;
        temp_line_format.scale.z = 0.1;
        temp_line_format.color.a = 0.5;
        temp_line_format.color.b = 1.0;
        temp_line_format.lifetime = marker_life_time;
        temp_line_format.type = visualization_msgs::Marker::LINE_STRIP;

        // Line 0_1
        visualization_msgs::Marker temp_line_01 = temp_line_format;
        temp_line_01.id = marker_id++;
        temp_line_01.points.push_back(temp_point_d0);
        temp_line_01.points.push_back(temp_point_d1);
        marker_array.markers.push_back(temp_line_01);

        // Line 1_2
        visualization_msgs::Marker temp_line_02 = temp_line_format;
        temp_line_02.id = marker_id++;
        temp_line_02.points.push_back(temp_point_d0);
        temp_line_02.points.push_back(temp_point_d2);
        marker_array.markers.push_back(temp_line_02);

        // Line 0_2
        visualization_msgs::Marker temp_line_12 = temp_line_format;
        temp_line_12.id = marker_id++;
        temp_line_12.points.push_back(temp_point_d1);
        temp_line_12.points.push_back(temp_point_d2);
        marker_array.markers.push_back(temp_line_12);


        // Arrow formatting
        visualization_msgs::Marker temp_arrow_format;
        temp_arrow_format.header.stamp = common_stamp;
        temp_arrow_format.header.frame_id = "base_link";
        temp_arrow_format.scale.x = 0.1;
        temp_arrow_format.scale.y = 0.2;
        temp_arrow_format.scale.z = 0.1;
        temp_arrow_format.color.a = 0.5;
        temp_arrow_format.color.r = 1.0;
        temp_arrow_format.lifetime = marker_life_time;
        temp_arrow_format.type = visualization_msgs::Marker::ARROW;

        // cmd d0
        visualization_msgs::Marker temp_arrow_cmd0 = temp_arrow_format;
        geometry_msgs::Point cmd0_endpoint = temp_point_d0;
        cmd0_endpoint.x += cmd_d0.twist.linear.x * 2;
        cmd0_endpoint.y += cmd_d0.twist.linear.y * 2;
        temp_arrow_cmd0.id = marker_id++;
        temp_arrow_cmd0.points.push_back(temp_point_d0);
        temp_arrow_cmd0.points.push_back(cmd0_endpoint);
        marker_array.markers.push_back(temp_arrow_cmd0);

        // cmd d1
        visualization_msgs::Marker temp_arrow_cmd1 = temp_arrow_format;
        geometry_msgs::Point cmd1_endpoint = temp_point_d1;
        cmd1_endpoint.x += cmd_d1.twist.linear.x * 2;
        cmd1_endpoint.y += cmd_d1.twist.linear.y * 2;
        temp_arrow_cmd1.id = marker_id++;
        temp_arrow_cmd1.points.push_back(temp_point_d1);
        temp_arrow_cmd1.points.push_back(cmd1_endpoint);
        marker_array.markers.push_back(temp_arrow_cmd1);

        // cmd d2
        visualization_msgs::Marker temp_arrow_cmd2 = temp_arrow_format;
        geometry_msgs::Point cmd2_endpoint = temp_point_d2;
        cmd2_endpoint.x += cmd_d2.twist.linear.x * 2;
        cmd2_endpoint.y += cmd_d2.twist.linear.y * 2;
        temp_arrow_cmd2.id = marker_id++;
        temp_arrow_cmd2.points.push_back(temp_point_d2);
        temp_arrow_cmd2.points.push_back(cmd2_endpoint);
        marker_array.markers.push_back(temp_arrow_cmd2);

        // Arrow formatting
        temp_arrow_format.color.r = 0.0;
        temp_arrow_format.color.g = 1.0;

        // vel d0
        visualization_msgs::Marker temp_arrow_vel0 = temp_arrow_format;
        geometry_msgs::Point vel0_endpoint = temp_point_d0;
        vel0_endpoint.x += vel_d0.twist.linear.x * 2;
        vel0_endpoint.y += vel_d0.twist.linear.y * 2;
        temp_arrow_vel0.id = marker_id++;
        temp_arrow_vel0.points.push_back(temp_point_d0);
        temp_arrow_vel0.points.push_back(vel0_endpoint);
        marker_array.markers.push_back(temp_arrow_vel0);

        // vel d1
        visualization_msgs::Marker temp_arrow_vel1 = temp_arrow_format;
        geometry_msgs::Point vel1_endpoint = temp_point_d1;
        vel1_endpoint.x += vel_d1.twist.linear.x * 2;
        vel1_endpoint.y += vel_d1.twist.linear.y * 2;
        temp_arrow_vel1.id = marker_id++;
        temp_arrow_vel1.points.push_back(temp_point_d1);
        temp_arrow_vel1.points.push_back(vel1_endpoint);
        marker_array.markers.push_back(temp_arrow_vel1);

        // vel d2
        visualization_msgs::Marker temp_arrow_vel2 = temp_arrow_format;
        geometry_msgs::Point vel2_endpoint = temp_point_d2;
        vel2_endpoint.x += vel_d2.twist.linear.x * 2;
        vel2_endpoint.y += vel_d2.twist.linear.y * 2;
        temp_arrow_vel2.id = marker_id++;
        temp_arrow_vel2.points.push_back(temp_point_d2);
        temp_arrow_vel2.points.push_back(vel2_endpoint);
        marker_array.markers.push_back(temp_arrow_vel2);

        ending_marker.id = marker_id++;
        ending_marker.action = 3;
        marker_array.markers.push_back(ending_marker);

        pub_markers.publish(marker_array);

        ros::spinOnce();
        publish_rate.sleep();
    }
    ROS_INFO("END OF FILE");
    return 0;
}

void uav_info_cb(const uwb_reloc_msgs::RelativeInfoStamped info){
    int rqstr_id = info.rqstrId;
    int rspdr_id = info.rspdrId;
    double dist = info.distance;
    if ((rqstr_id == 0 && rspdr_id == 1) || (rqstr_id == 1 && rspdr_id == 0)) d_01 = dist;
    if ((rqstr_id == 0 && rspdr_id == 2) || (rqstr_id == 2 && rspdr_id == 0)) d_02 = dist;
    if ((rqstr_id == 1 && rspdr_id == 2) || (rqstr_id == 2 && rspdr_id == 1)) d_12 = dist;

    std::ostringstream strs_01, strs_02, strs_12;
    strs_01 << d_01;
    d_str_01 = strs_01.str();
    strs_02 << d_02;
    d_str_02 = strs_02.str();
    strs_12 << d_12;
    d_str_12 = strs_12.str();
}

void uav_cmd_d0_cb(const geometry_msgs::TwistStamped vel){
    cmd_d0 = vel;
}

void uav_cmd_d1_cb(const geometry_msgs::TwistStamped vel){
    cmd_d1 = vel;
}

void uav_cmd_d2_cb(const geometry_msgs::TwistStamped vel){
    cmd_d2 = vel;
}

void uav_vel_d0_cb(const geometry_msgs::TwistStamped vel){
    vel_d0 = vel;
}

void uav_vel_d1_cb(const geometry_msgs::TwistStamped vel){
    vel_d1 = vel;
}

void uav_vel_d2_cb(const geometry_msgs::TwistStamped vel){
    vel_d2 = vel;
}
