#include "trajectory_cost.h"


void costmapInitCallback(const nav_msgs::OccupancyGridConstPtr& costmsg) {
    resolution = int(1/costmsg->info.resolution);
    width = costmsg->info.width;
    height = costmsg->info.height;
}

void costmapCallback(const map_msgs::OccupancyGridUpdateConstPtr& costmsg) {
    data = costmsg->data;

    int cost = 0;
    for (int i = 0; i < latestTrajectory.trajectory.size(); i++) {
        double x = latestTrajectory.trajectory[i].x;
        double y = latestTrajectory.trajectory[i].y;
        ROS_INFO("(%f, %f)", x, y);

        // (0,0) is costmap[0] is bottom right corner
        // (0,y) is costmap[y] is top right corner
        // (x,0) is costmap[x*height] is bottom left corner
        // (x,y) is costmap[x*height + y] is top left corner
        int cx = width/2 - round(latestTrajectory.trajectory[i].x * resolution);
        int cy = height/2 + round(latestTrajectory.trajectory[i].y * resolution);
        ROS_INFO("(%d, %d)", cx, cy);

        // check bounds
        if (cx <= width && cx >= 0 && cy <= height && cy >= 0) {
            ROS_INFO("cost: %d", data[cx*width + cy]);
            if (data[cx*width + cy] == -1) {
                cost = cost + 25;
            } 
            else {
                cost = cost + data[cx*width + cy];
            }
        } 
        else {
            cost = cost + 25;
        }
    }
    tc.trajectoryCost = cost;
}

void trajectoryCallback(const fake_trajectory::TrajectoryVector::ConstPtr& trajmsg) {
    if (!trajmsg->trajectory.empty()) {
        latestTrajectory.trajectory.clear();
        for (unsigned int i = 0; i < trajmsg->trajectory.size(); i++) {
            latestTrajectory.trajectory.push_back(trajmsg->trajectory[i]);
            //ROS_INFO("(%f, %f)", latestTrajectory.trajectory[i].x, latestTrajectory.trajectory[i].y);
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajcost");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<trajectory_cost::TrajectoryCostVector>("/trajectory/cost", 1);
    ros::Subscriber sub0 = nh.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &costmapInitCallback);
    ros::Subscriber sub1 = nh.subscribe<map_msgs::OccupancyGridUpdate>("/costmap_node/costmap/costmap_updates", 1, &costmapCallback);
    ros::Subscriber sub2 = nh.subscribe<fake_trajectory::TrajectoryVector>("/trajectory", 1, &trajectoryCallback);
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        pub.publish(tc);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
