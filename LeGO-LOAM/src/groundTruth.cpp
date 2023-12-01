#include "utility.h"
#include <nav_msgs/Path.h> // Include the nav_msgs/Path header

nav_msgs::Path path;

void posestampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    static tf::TransformBroadcaster tfBroadcaster;

    tf::Quaternion z_body_to_sensor_ori = tf::Quaternion(0, 0, 0);
    tf::Vector3 z_body_to_sensor_pos = tf::Vector3(0.870, -0.001, -0.670);
    
    /*
    tf::Quaternion body_init_to_NED_ori = tf::Quaternion(-0.009740638500177649, 0.0006472508840350607, 0.7569846883637315, -0.6533596885412861);
    tf::Vector3 body_init_to_NED_pos = tf::Vector3(-33.2670281695695, -72.67735566501135, 0.46049959737605317);
    tf::Quaternion camera_init_to_map_ori = tf::Quaternion(PI/2,0,PI/2);
    tf::Vector3 camera_init_to_map_pos = tf::Vector3(0,0,0);
    tf::Quaternion camera_orientation = tf::Quaternion(3.141, -0.002, -0.107);
    tf::Vector3 camera_position = tf::Vector3(0,0,0);
    tf::Pose camera_init_to_map = tf::Pose(camera_init_to_map_ori,camera_init_to_map_pos);
    tf::Pose body_init_to_NED = tf::Pose(body_init_to_NED_ori,body_init_to_NED_pos);
    tf::Pose camera_pose = tf::Pose(camera_orientation,camera_position);*/

    tf::Pose z_body_to_sensor = tf::Pose(z_body_to_sensor_ori,z_body_to_sensor_pos);

    tf::Quaternion rotate_axes = tf::Quaternion(M_PI/2,0,-M_PI/2);
    tf::Vector3 rotate_axes_ori = tf::Vector3(0,0,0);
    tf::Pose rotate_axes_pose = tf::Pose(rotate_axes,rotate_axes_ori);

    tf::Pose body;
    tf::poseMsgToTF(msg->pose, body);

    tf::Pose ground_truth = body * z_body_to_sensor * rotate_axes_pose; // shift the frame to the sensor frame, so that everything aligns

    geometry_msgs::Pose ground_truth_msg;
    tf::poseTFToMsg(ground_truth, ground_truth_msg);

    geometry_msgs::PoseStamped ground_truth_stamped_msg;
    ground_truth_stamped_msg.pose = ground_truth_msg;
    ground_truth_stamped_msg.header.frame_id = "NED";

    // broadcast transform
    tf::StampedTransform broadcast_pose;
    broadcast_pose.setRotation(ground_truth.getRotation());
    broadcast_pose.setOrigin(ground_truth.getOrigin());
    broadcast_pose.stamp_ = msg->header.stamp;
    broadcast_pose.frame_id_ = "NED";
    broadcast_pose.child_frame_id_ = "ground_truth";
    tfBroadcaster.sendTransform(broadcast_pose);

    path.header.stamp = ros::Time::now();  // Update the timestamp
    path.header.frame_id = "NED"; // Set the frame ID

    // // Append the received PoseStamped message to the Path
    path.poses.push_back(ground_truth_stamped_msg);
    //path.poses.push_back(*msg);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");
    ros::NodeHandle nh;

    ros::Subscriber poseStampedSub = nh.subscribe("/seapath/pose", 10, posestampedCallback); // subscribe to pose
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("/ground_truth", 10); // publish path

    ros::Rate rate(100);

    while (ros::ok())
    {
        pathPub.publish(path);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
