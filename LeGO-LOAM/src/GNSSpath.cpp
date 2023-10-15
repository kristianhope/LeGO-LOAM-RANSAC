#include "utility.h"

double reference_latitude = 63.43967123888433; // Set your reference latitude here
double reference_longitude = 10.398784289136529; // Set your reference longitude here
const double meters_per_degree_lat = 111139; // Approximate meters per degree of latitude
const double meters_per_degree_lon = 111139; // Approximate meters per degree of longitude

nav_msgs::Path path;
ros::Publisher path_pub;

void navsatfixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // Convert latitude and longitude to NED coordinates
    double lat = msg->latitude;
    double lon = msg->longitude;
    double N = (lat - reference_latitude) * meters_per_degree_lat;
    double E = (lon - reference_longitude) * meters_per_degree_lon;

    tf::Vector3 translation = tf::Vector3(E,N,0);

    tf::Pose body_to_init;
    body_to_init.setOrigin(translation);
    body_to_init.setRotation(tf::Quaternion(0,0,0));


    tf::Pose body_to_NED = body_to_init;

    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg(body_to_NED, pose_msg);

    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.pose = pose_msg;
    pose_stamped_msg.header.frame_id = "map";


    path.poses.push_back(pose_stamped_msg);

    // Publish the updated path
    path.header.stamp = msg->header.stamp;
    path.header.frame_id = "map"; // Set your frame ID
    path_pub.publish(path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");
    ros::NodeHandle nh;

    path_pub = nh.advertise<nav_msgs::Path>("/GNSS_path", 10); // Replace with your desired topic name

    ros::Subscriber navsat_sub = nh.subscribe("/seapath/fix", 10, navsatfixCallback); // Replace with your NavSatFix topic

    // Set your reference latitude and longitude here

    ros::spin();

    return 0;
}