#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cm_setup_tf");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    while (n.ok()) {
        broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
            ros::Time::now(), "map", "base_link"));
        broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.2,0,0)),
            ros::Time::now(), "base_link", "camera_depth_frame"));
        broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
            ros::Time::now(), "base_link", "laser"));
        r.sleep();
    }
}
