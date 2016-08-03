#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include "fake_trajectory/TrajectoryVector.h"
#include <random_numbers/random_numbers.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "faketraj");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<fake_trajectory::TrajectoryVector>("/trajectory", 1);
    ros::Rate loop_rate(10);
    random_numbers::RandomNumberGenerator rng;

    while (ros::ok()) {
        fake_trajectory::TrajectoryVector traj;

        geometry_msgs::Point p1;
        p1.x = rng.uniformReal(-5,5); p1.y = rng.uniformReal(0,5); p1.z = 0;
        traj.trajectory.push_back(p1);
        geometry_msgs::Point p2;
        p2.x = rng.uniformReal(-5,5); p2.y = rng.uniformReal(0,5); p1.z = 0;
        traj.trajectory.push_back(p2);
        geometry_msgs::Point p3;
        p3.x = rng.uniformReal(-5,5); p3.y = rng.uniformReal(0,5); p1.z = 0;
        traj.trajectory.push_back(p3);

        pub.publish(traj);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
