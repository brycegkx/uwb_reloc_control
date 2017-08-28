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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "rl_format.h"
#include "local_pos_sp_format.h"
#include "csv_parse_engine.h"

std::string node_name = "csv_parser";

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

double viz_rate = 20.0;

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

    ros::Publisher pub_markers = nh.advertise<visualization_msgs::MarkerArray>("markers", 10);

    ros::Rate publish_rate(100);

    // UPPER board csv file
    std::string upp_file_path = upp_basic_path + upp_data_set + upp_uav_id + upp_file_name;
    std::ifstream upp_file(upp_file_path.c_str());
    CSVRow upp_file_row;
    long upp_file_row_counter = 0;
    bool upp_file_mutex_locked = false;
    bool upp_file_has_start_time = false;
    long upp_file_start_time_ms = 0;
    long upp_file_ellapsed_time_ms = 0;
    long upp_file_offset_time_ms = 0;
    rl temp_rl_info;

    // PX4 csv file
    std::string px4_file_path = px4_basic_path + px4_data_set + px4_uav_id + px4_file_name;
    std::ifstream px4_file(px4_file_path.c_str());
    CSVRow px4_file_row;
    long px4_file_row_counter = 0;
    bool px4_file_mutex_locked = false;
    bool px4_file_has_start_time = false;
    long px4_file_start_time_ms = 0;
    long px4_file_ellapsed_time_ms = 0;
    long px4_file_offset_time_ms = 500;
    local_pos_sp temp_local_pos_sp_info;

    ros::Time start_time = ros::Time::now();
    long time_stamp_ms = 0;
    long ros_ellapsed_time_ms = 0;
    int allowable_time_diff = 3; //ms

    double d_01, d_12, d_02;
    std::string d_str_01, d_str_12, d_str_02;
    double x_0, x_1, x_2;
    double y_0, y_1, y_2;
    double a, b;

    geometry_msgs::PoseStamped temp_pose_msg;
    geometry_msgs::PointStamped temp_point_msg;
    geometry_msgs::Point temp_point_d0, temp_point_d1, temp_point_d2;
    geometry_msgs::Point temp_point_d01, temp_point_d12, temp_point_d02;

    ros::Duration marker_life_time (1/viz_rate * 0.8);
    while (ros::ok()){
        // -------------------- GET DATA FROM CSV --------------------
        try {
            if (!upp_file_mutex_locked){
                if (upp_file.eof()) break;
                upp_file_row.readNextRow(upp_file);
                // if (upp_file_row.size() < 10) continue;
                upp_file_row_counter++;
                if (upp_file_row_counter > 1) {
                    upp_file_row.parse_to_rl(temp_rl_info);
                    if (!upp_file_has_start_time) {
                        upp_file_start_time_ms = temp_rl_info.uwb_time_ms;
                        upp_file_ellapsed_time_ms = 0;
                        upp_file_has_start_time = true;
                    } else {
                        upp_file_ellapsed_time_ms = temp_rl_info.uwb_time_ms - upp_file_start_time_ms;
                    }
                    if ((temp_rl_info.rqstr_id == 0 && temp_rl_info.rspdr_id == 1) ||
                        (temp_rl_info.rqstr_id == 1 && temp_rl_info.rspdr_id == 0)) {
                        d_01 = temp_rl_info.dist;
                        d_str_01 = temp_rl_info.dist_str;
                    }
                    if ((temp_rl_info.rqstr_id == 1 && temp_rl_info.rspdr_id == 2) ||
                        (temp_rl_info.rqstr_id == 2 && temp_rl_info.rspdr_id == 1)) {
                        d_12 = temp_rl_info.dist;
                        d_str_12 = temp_rl_info.dist_str;
                    }
                    if ((temp_rl_info.rqstr_id == 0 && temp_rl_info.rspdr_id == 2) ||
                        (temp_rl_info.rqstr_id == 2 && temp_rl_info.rspdr_id == 0)) {
                        d_02 = temp_rl_info.dist;
                        d_str_02 = temp_rl_info.dist_str;
                    }

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
                }
            }
        } catch (const std::exception& e) { /* */ }

        try {
            if (!px4_file_mutex_locked){
                if (px4_file.eof()) break;
                px4_file_row.readNextRow(px4_file);
                // if (px4_file_row.size() < 5) continue;
                px4_file_row_counter++;
                if (px4_file_row_counter > 1){
                    px4_file_row.parse_to_local_pos_sp(temp_local_pos_sp_info);
                    if (!px4_file_has_start_time) {
                        px4_file_start_time_ms = temp_local_pos_sp_info.timestamp_us / 1000;
                        px4_file_ellapsed_time_ms = 0 + px4_file_offset_time_ms;
                        px4_file_has_start_time = true;
                    } else {
                        px4_file_ellapsed_time_ms = temp_local_pos_sp_info.timestamp_us / 1000
                                                    - px4_file_start_time_ms + px4_file_offset_time_ms;
                    }
                }
            }
        } catch (const std::exception& e) { /* */ }

        // -------------------- SYNC CSV WITH ROS --------------------
        ros_ellapsed_time_ms = (ros::Time::now().toSec() - start_time.toSec()) * 1000;
        if (ros_ellapsed_time_ms >= upp_file_ellapsed_time_ms - allowable_time_diff &&
                ros_ellapsed_time_ms <= upp_file_ellapsed_time_ms) {
            upp_file_mutex_locked = false;
        } else {
            if (ros_ellapsed_time_ms < upp_file_ellapsed_time_ms - allowable_time_diff)    upp_file_mutex_locked = true;
            if (ros_ellapsed_time_ms > upp_file_ellapsed_time_ms) upp_file_mutex_locked = false;
        }

        if (ros_ellapsed_time_ms >= px4_file_ellapsed_time_ms - allowable_time_diff &&
                ros_ellapsed_time_ms <= px4_file_ellapsed_time_ms) {
            px4_file_mutex_locked = false;
        } else {
            if (ros_ellapsed_time_ms < px4_file_ellapsed_time_ms - allowable_time_diff) px4_file_mutex_locked = true;
            if (ros_ellapsed_time_ms > px4_file_ellapsed_time_ms) px4_file_mutex_locked = false;
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

        // Vel d0
        visualization_msgs::Marker temp_arrow_v0 = temp_arrow_format;
        geometry_msgs::Point v0_endpoint = temp_point_d0;
        v0_endpoint.x += temp_local_pos_sp_info.vy * 2;
        v0_endpoint.y += temp_local_pos_sp_info.vx * 2;
        temp_arrow_v0.id = marker_id++;
        temp_arrow_v0.points.push_back(temp_point_d0);
        temp_arrow_v0.points.push_back(v0_endpoint);
        marker_array.markers.push_back(temp_arrow_v0);

        // // Vel d1
        // visualization_msgs::Marker temp_arrow_v1 = temp_arrow_format;
        // geometry_msgs::Point v1_endpoint = temp_point_d1;
        // v1_endpoint.x += temp_local_pos_sp_info.vy * 2;
        // v1_endpoint.y += temp_local_pos_sp_info.vx * 2;
        // temp_arrow_v1.id = marker_id++;
        // temp_arrow_v1.points.push_back(temp_point_d1);
        // temp_arrow_v1.points.push_back(v1_endpoint);
        // marker_array.markers.push_back(temp_arrow_v1);

        // // Vel d2
        // visualization_msgs::Marker temp_arrow_v2 = temp_arrow_format;
        // geometry_msgs::Point v2_endpoint = temp_point_d2;
        // v2_endpoint.x += temp_local_pos_sp_info.vy * 2;
        // v2_endpoint.y += temp_local_pos_sp_info.vx * 2;
        // temp_arrow_v2.id = marker_id++;
        // temp_arrow_v2.points.push_back(temp_point_d2);
        // temp_arrow_v2.points.push_back(v2_endpoint);
        // marker_array.markers.push_back(temp_arrow_v2);

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
