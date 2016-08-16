#include "trajectory_cost.h"

void costmapInitCallback(const nav_msgs::OccupancyGridConstPtr& costmsg) {
    resolution = int(1/costmsg->info.resolution);
    width = costmsg->info.width;
    height = costmsg->info.height;
}

void costmapCallback(const map_msgs::OccupancyGridUpdateConstPtr& costmsg) {
    int ts_size = latestTrajectorySet.trajectorysims.size();
    if (ts_size > 0) {
        data = costmsg->data;
        std::vector<int> costVector;

        // iterate over each trajectory
        for (int i = 0; i < latestTrajectorySet.trajectorysims.size(); i++) {
            int cost = 0;
            fake_trajectory::TrajectoryVector latestTrajectory = latestTrajectorySet.trajectorysims[i];
            // iterate over points in each trajectory
            for (int j = 0; j < latestTrajectory.trajectory.size(); j++) {
                double x = latestTrajectory.trajectory[j].x;
                double y = latestTrajectory.trajectory[j].y;
                //ROS_INFO("(%f, %f)", x, y);

                // (0,0) is costmap[0] is bottom right corner
                // (0,y) is costmap[y] is top right corner
                // (x,0) is costmap[x*height] is bottom left corner
                // (x,y) is costmap[x*height + y] is top left corner
                int cx = width/2 - round(latestTrajectory.trajectory[j].x * resolution);
                int cy = height/2 + round(latestTrajectory.trajectory[j].y * resolution);
                //ROS_INFO("(%d, %d)", cx, cy);

                // check bounds
                if (cx <= width && cx >= 0 && cy <= height && cy >= 0) {
                    //ROS_INFO("cost: %d", data[cx*width + cy]);
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
            costVector.push_back(cost);
        }
        //for (int i = 0; i < costVector.size(); i++) {
        //    printf("%d ", costVector[i]);
        //}
        //printf("\n");

        // return index of min cost
        std::vector<int>::iterator result = std::min_element(std::begin(costVector), std::end(costVector));
        int minElem = std::distance(std::begin(costVector), result);
        trajID.trajID = latestTrajectorySet.trajectorysims[minElem].trajID;
    }
}

void trajectoryCallback(const fake_trajectory::TrajectorySims::ConstPtr& trajSetMsg) {
    if (!trajSetMsg->trajectorysims.empty()) {
        latestTrajectorySet.trajectorysims.clear();
        // iterate over full trajectories
        for (unsigned int i = 0; i < trajSetMsg->trajectorysims.size(); i++) {
            fake_trajectory::TrajectoryVector singleTrajectory = trajSetMsg->trajectorysims[i];
            //if (!singleTrajectory.trajectory.empty()) {
            //    // iterate over trajectory points
            //    for (unsigned int i = 0; i < singleTrajectory.trajectory.size(); i++) {
            //        latestTrajectory.trajectory.push_back(singleTrajectory.trajectory[i]);
            //        //ROS_INFO("(%f, %f)", latestTrajectory.trajectory[i].x, latestTrajectory.trajectory[i].y);
            //    }
            //}
            latestTrajectorySet.trajectorysims.push_back(singleTrajectory);
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajcost");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<trajectory_cost::TrajectoryID>("/trajectory/costID", 1);
    ros::Subscriber sub0 = nh.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &costmapInitCallback);
    ros::Subscriber sub1 = nh.subscribe<map_msgs::OccupancyGridUpdate>("/costmap_node/costmap/costmap_updates", 1, &costmapCallback);
    ros::Subscriber sub2 = nh.subscribe<fake_trajectory::TrajectorySims>("/trajectory", 1, &trajectoryCallback);
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        pub.publish(trajID);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
