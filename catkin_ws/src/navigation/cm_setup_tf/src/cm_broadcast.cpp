#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>

//void poseCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
//    if (!msg->markers.empty()) {
//        static tf::TransformBroadcaster br;
//        tf::Transform transform;
//        //transform.setOrigin( tf::Vector3(msg->markers[0].pose.position.z, -msg->markers[0].pose.position.x, 0.0) );
//        transform.setOrigin( tf::Vector3(0.2, 0.15, 0.0) );
//        transform.setRotation( tf::Quaternion (0, 0, 0, 1) );
//        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_rgb_optical_frame"));
//        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "apriltag_frame"));
//    }
//}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cm_setup_tf");
    ros::NodeHandle n;
    //ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/apriltags/maker_array_ls", 1);
    //ros::Subscriber sub = n.subscribe("/apriltags/marker_array", 1, &poseCallback);
    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    while (n.ok()) {
        broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
            ros::Time::now(), "map", "base_link"));
        broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
            ros::Time::now(), "base_link", "map"));
        broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
            ros::Time::now(), "base_link", "laser"));
        //broadcaster.sendTransform(
        //  tf::StampedTransform(
        //    tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.2,0.15,0)),
        //    ros::Time::now(), "base_link", "camera_depth_frame"));
        //broadcaster.sendTransform(
        //  tf::StampedTransform(
        //    tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.0,0.0,0)),
        //    ros::Time::now(), "base_link", "camera_depth_frame"));
        //broadcaster.sendTransform(
        //  tf::StampedTransform(
        //    tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
        //    ros::Time::now(), "base_link", "odom"));
        //broadcaster.sendTransform(
        //  tf::StampedTransform(
        //    tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
        //    ros::Time::now(), "base_link", "base_imu_link"));
        //broadcaster.sendTransform(
        //  tf::StampedTransform(
        //    tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.0,0.0,0)),
        //    ros::Time::now(), "camera_depth_frame", "base_link"));
        //broadcaster.sendTransform(
        //  tf::StampedTransform(
        //    tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
        //    ros::Time::now(), "base_link", "laser"));
        //broadcaster.sendTransform(
        //  tf::StampedTransform(
        //    tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.2,0.15,0)),
        //    ros::Time::now(), "base_link", "camera_rgb_optical_frame"));
        //broadcaster.sendTransform(
        //  tf::StampedTransform(
        //    tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.2,0.15,0)),
        //    ros::Time::now(), "base_link", "apriltag_frame"));
        broadcaster.sendTransform(
          tf::StampedTransform(
            tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.0,0.0,0)),
            ros::Time::now(), "base_link", "apriltag_frame"));
        //ros::spinOnce();
        r.sleep();
    }
    return 0;
}
