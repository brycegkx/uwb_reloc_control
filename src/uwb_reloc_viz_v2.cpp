#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
// #include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <uwb_reloc_msgs/RelativeInfoStamped.h>
#include <uwb_reloc_msgs/uwbTalkData.h>
#include <uwb_reloc/uwbTalkData.h>

std::string node_name = "uwb_reloc_viz_v2";

// UAVs information
geometry_msgs::Pose local_origin_pose;
bool local_origin_acquired = false;

geometry_msgs::Pose uav_0_curr_local_pose;
geometry_msgs::Pose uav_1_curr_local_pose;
geometry_msgs::Pose uav_2_curr_local_pose;

// Callbacks
void uav_0_global_pose_cb(const geometry_msgs::PoseWithCovarianceStamped _pose);
void uav_2_global_pose_cb(const nav_msgs::Odometry _pose);

// Util functions
inline void myPoseMsgToTF(const geometry_msgs::Pose& msg, tf::Transform& bt);

int main(int argc, char** argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    //Subscriber
    ros::Subscriber uav_0_global_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/uav_0/mavros/global_position/local", 10, uav_0_global_pose_cb);
    ros::Subscriber uav_2_global_pose_sub = nh.subscribe<nav_msgs::Odometry>("/uav_2/mavros/global_position/local", 10, uav_2_global_pose_cb);

    //Broadcaster
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));

    //Find start pose of all uavs
    ros::Rate loop_rate(10);
    while (ros::ok()){ //Draw tf, poses
        // uav 0 tf
        myPoseMsgToTF(uav_0_curr_local_pose, transform);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav_0"));

        // uav 2 tf
        myPoseMsgToTF(uav_2_curr_local_pose, transform);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav_2"));

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void uav_0_global_pose_cb(const geometry_msgs::PoseWithCovarianceStamped _pose){
    if (local_origin_acquired){
        uav_0_curr_local_pose.position.x = _pose.pose.pose.position.x - local_origin_pose.position.x;
        uav_0_curr_local_pose.position.y = _pose.pose.pose.position.y - local_origin_pose.position.y;
        uav_0_curr_local_pose.position.z = _pose.pose.pose.position.z - local_origin_pose.position.z;
        uav_0_curr_local_pose.orientation = _pose.pose.pose.orientation;
    } else {
        local_origin_pose = _pose.pose.pose;
        local_origin_acquired = true;
        ROS_WARN("Acquired local origin");
    }
}

void uav_2_global_pose_cb(const nav_msgs::Odometry _pose){
    if (local_origin_acquired){
        uav_2_curr_local_pose.position.x = _pose.pose.pose.position.x - local_origin_pose.position.x;
        uav_2_curr_local_pose.position.y = _pose.pose.pose.position.y - local_origin_pose.position.y;
        uav_2_curr_local_pose.position.z = _pose.pose.pose.position.z - local_origin_pose.position.z;
        uav_2_curr_local_pose.orientation = _pose.pose.pose.orientation;
    }
}

inline void myPoseMsgToTF(const geometry_msgs::Pose& msg, tf::Transform& bt){
    bt = tf::Transform
        ( tf::Quaternion(   msg.orientation.x,
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w),
            tf::Vector3(    msg.position.x,
                            msg.position.y,
                            msg.position.z)
        );
}
