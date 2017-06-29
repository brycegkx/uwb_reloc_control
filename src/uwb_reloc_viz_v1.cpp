#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/transform_datatypes.h>

#include <uwb_reloc_msgs/RelativeInfoStamped.h>
#include <uwb_reloc_msgs/uwbTalkData.h>
#include <uwb_reloc/uwbTalkData.h>


#define no_of_uav 3
#define viz_rate 20

std::string node_name = "uwb_reloc_viz";

//Subscribers and Publishers
std::string topic_sub_uav_info = "/rlForUAV";
std::string topic_sub_uav_cmd = "/common/cmd_vel";
std::string topic_pub_markers = "/visualization_marker_array";

ros::Subscriber sub_uav_info;
ros::Subscriber sub_uav_cmd;
ros::Publisher pub_markers;

//UAV information
geometry_msgs::Point uav_pos[no_of_uav];
geometry_msgs::Vector3 uav_vel[no_of_uav];
geometry_msgs::Vector3 uav_cmd[no_of_uav];
geometry_msgs::Quaternion uav_vel_dir[no_of_uav];
double uav_vel_mag[no_of_uav];
geometry_msgs::Quaternion uav_cmd_dir[no_of_uav];
double uav_cmd_mag[no_of_uav];

//RViz tool
// rviz_visual_tools::RvizVisualTools my_rviz_tool;

//Logic functions


//Callback functions
void uav_info_cb(const uwb_reloc::uwbTalkData info);
void uav_cmd_cb(const geometry_msgs::TwistStamped cmd);

int main(int argc, char** argv){
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");

  nh_param.param<std::string>("topic_sub_uav_info", topic_sub_uav_info, topic_sub_uav_info);
  nh_param.param<std::string>("topic_sub_uav_cmd", topic_sub_uav_cmd, topic_sub_uav_cmd);
  nh_param.param<std::string>("topic_pub_markers", topic_pub_markers, topic_pub_markers);

  sub_uav_info = nh.subscribe<uwb_reloc::uwbTalkData>(topic_sub_uav_info, 10, uav_info_cb);
  sub_uav_cmd = nh.subscribe<geometry_msgs::TwistStamped>(topic_sub_uav_cmd, 10, uav_cmd_cb);
  pub_markers = nh.advertise<visualization_msgs::MarkerArray>(topic_pub_markers, 10);

  ros::Duration marker_life_time (1/viz_rate * 0.8);

  ros::Rate loop_rate(viz_rate);

  while (ros::ok()){
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker ending_marker;
    // my_rviz_tool.deleteAllMarkers();
    for (int i=0; i < no_of_uav; i++){
      visualization_msgs::Marker temp_pos_marker;
      visualization_msgs::Marker temp_vel_marker;
      visualization_msgs::Marker temp_cmd_marker;
      ros::Time common_stamp = ros::Time();
      //Position as red sphere
      temp_pos_marker.id = i * 3 + 0;
      temp_pos_marker.header.stamp = common_stamp;
      temp_pos_marker.header.frame_id = "base_link";
      temp_pos_marker.type = visualization_msgs::Marker::SPHERE;
      // temp_cmd_marker.action = visualization_msgs::Marker::DELETE;
      temp_pos_marker.pose.position = uav_pos[i];
      temp_pos_marker.scale.x = 0.2;
      temp_pos_marker.scale.y = 0.2;
      temp_pos_marker.scale.z = 0.2;
      temp_pos_marker.color.r = 1.0;
      temp_pos_marker.color.a = 0.5;
      temp_pos_marker.lifetime = marker_life_time;
      marker_array.markers.push_back(temp_pos_marker);

      //Velocity as green arrow
      temp_vel_marker.id = i * 3 + 1;
      temp_vel_marker.header.stamp = common_stamp;
      temp_vel_marker.header.frame_id = "base_link";
      temp_vel_marker.type = visualization_msgs::Marker::ARROW;
      temp_vel_marker.pose.position = uav_pos[i];
      temp_vel_marker.pose.orientation = uav_vel_dir[i];
      temp_vel_marker.scale.y = 0.05;
      temp_vel_marker.scale.z = 0.05;
      temp_vel_marker.scale.x = uav_vel_mag[i];
      temp_vel_marker.color.g = 1.0;
      temp_vel_marker.color.a = 1.0;
      temp_vel_marker.lifetime = marker_life_time;
      marker_array.markers.push_back(temp_vel_marker);

      //Command as blue arrow
      temp_cmd_marker.id = i * 3 + 2;
      temp_cmd_marker.header.stamp = common_stamp;
      temp_cmd_marker.header.frame_id = "base_link";
      temp_cmd_marker.type = visualization_msgs::Marker::ARROW;
      temp_cmd_marker.pose.position = uav_pos[i];
      temp_cmd_marker.pose.orientation = uav_cmd_dir[i];
      temp_cmd_marker.scale.y = 0.05;
      temp_cmd_marker.scale.z = 0.05;
      temp_cmd_marker.scale.x = uav_cmd_mag[i];
      temp_cmd_marker.color.b = 1.0;
      temp_cmd_marker.color.a = 1.0;
      temp_cmd_marker.lifetime = marker_life_time;
      marker_array.markers.push_back(temp_cmd_marker);
    }
    ending_marker.id = 9;
    ending_marker.action = 3;
    marker_array.markers.push_back(ending_marker);
    pub_markers.publish(marker_array);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void uav_info_cb(const uwb_reloc::uwbTalkData info){
  int temp_id = 0;
  geometry_msgs::Point temp_pos;
  geometry_msgs::Vector3 temp_vel;
  geometry_msgs::Quaternion temp_quat;
  tf::Quaternion temp_quat_tf;
  double temp_yaw_angle;
  double temp_magnitude;
  //Requester
  temp_id = info.rqstrId;
  // ROS_INFO("Here: %d", temp_id);
  if (temp_id < no_of_uav){
    temp_pos.x = info.rqstr_x;
    temp_pos.y = info.rqstr_y;
    temp_pos.z = info.rqstr_z;
    temp_vel.x = info.rqstr_vx;
    temp_vel.y = info.rqstr_vy;
    temp_vel.z = info.rqstr_vz;

    temp_yaw_angle = atan2(temp_vel.y, temp_vel.x);
    temp_magnitude = sqrt(temp_vel.y * temp_vel.y + temp_vel.x * temp_vel.x);

    temp_quat_tf.setEuler(0.0, 0.0, temp_yaw_angle);
    tf::quaternionTFToMsg(temp_quat_tf, temp_quat);

    uav_pos[temp_id] = temp_pos;
    uav_vel[temp_id] = temp_vel;
    uav_vel_dir[temp_id] = temp_quat;
    uav_vel_mag[temp_id] = temp_magnitude;
  }
  //Responder
  // temp_id = info.rspdrId;
  // if (temp_id < no_of_uav){
  //   temp_pos.x = info.self_x;
  //   temp_pos.y = info.self_y;
  //   temp_pos.z = info.self_z;
  //   temp_vel.x = info.self_vx;
  //   temp_vel.y = info.self_vy;
  //   temp_vel.z = info.self_vz;

  //   temp_yaw_angle = atan2(temp_vel.y, temp_vel.x);
  //   temp_magnitude = sqrt(temp_vel.y * temp_vel.y + temp_vel.x * temp_vel.x);

  //   temp_quat_tf.setEuler(0.0, 0.0, temp_yaw_angle);
  //   tf::quaternionTFToMsg(temp_quat_tf, temp_quat);

  //   uav_pos[temp_id] = temp_pos;
  //   uav_vel[temp_id] = temp_vel;
  //   uav_vel_dir[temp_id] = temp_quat;
  //   uav_vel_mag[temp_id] = temp_magnitude;
  // }
}

void uav_cmd_cb(const geometry_msgs::TwistStamped cmd){
  int temp_id = std::atoi(cmd.header.frame_id.c_str());
  geometry_msgs::Vector3 temp_cmd;
  geometry_msgs::Quaternion temp_quat;
  tf::Quaternion temp_quat_tf;
  double temp_yaw_angle;
  double temp_magnitude;
  if (temp_id < no_of_uav){
    temp_cmd.x = cmd.twist.linear.x;
    temp_cmd.y = cmd.twist.linear.y;
    temp_cmd.z = cmd.twist.linear.z;

    temp_yaw_angle = atan2(temp_cmd.y, temp_cmd.x);
    temp_magnitude = sqrt(temp_cmd.y * temp_cmd.y + temp_cmd.x * temp_cmd.x);

    temp_quat_tf.setEuler(0.0, 0.0, temp_yaw_angle);
    tf::quaternionTFToMsg(temp_quat_tf, temp_quat);

    uav_cmd[temp_id] = temp_cmd;
    uav_cmd_dir[temp_id] = temp_quat;
    uav_cmd_mag[temp_id] = temp_magnitude;
  }
}
