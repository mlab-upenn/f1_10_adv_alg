#include "trajectory_cost.h"
//#define DEBUG
#define RVIZ

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
            trajectory_brain::TrajectoryVector latestTrajectory = latestTrajectorySet.trajectorysims[i];
            // iterate over points in each trajectory
            for (int j = 0; j < latestTrajectory.trajectory.size(); j++) {
                double x = latestTrajectory.trajectory[j].x;
                double y = latestTrajectory.trajectory[j].y;

                // (0,0) is costmap[0] is bottom right corner
                // (0,y) is costmap[y] is top right corner
                // (x,0) is costmap[x*height] is bottom left corner
                // (x,y) is costmap[x*height + y] is top left corner
                int cx = width/2 + round(latestTrajectory.trajectory[j].x * resolution);
                int cy = height/2 + round(latestTrajectory.trajectory[j].y * resolution);
                //ROS_INFO("(%f, %f) -> (%d, %d) : %d", x,y, cx, cy, data[cy*width + cx]);

                // check bounds
                if (cx <= width && cx >= 0 && cy <= height && cy >= 0) {
                    // Modify costs
                    if (data[cy*height + cx] == -1 || data[cy*height+cx] >= 99) { // UNK, Inflation, Lethal
                        cost = std::numeric_limits<int>::max();
                        break;
                    } 
                    else {
                        cost = cost + data[cy*width + cx];
                    }
                } 
                else { // Out of Bounds
                    cost = std::numeric_limits<int>::max();
                    break;
                }
            }
            costVector.push_back(cost);
        }

        // print costVector
        for (int i = 0; i < costVector.size(); i++) {
            printf("%d ", costVector[i]);
        }
        printf("\n");

        // return index of min cost
        int minElem = -1;
        int minElemVal = std::numeric_limits<int>::max();
        for (int i = 0; i < costVector.size(); i++) {
            if (costVector[i] > 0 && costVector[i] < minElemVal) {
                minElemVal = costVector[i];
                minElem = i;
            }
        }
        // all trajectories are invalid
        if (minElem == -1) {
            trajID.trajID = -1;
        }
        // only swap trajectory if previous one is no longer valid
        else if (trajID.trajID == -1 || costVector[trajID.trajID] == std::numeric_limits<int>::max()) {
            trajID.trajID = latestTrajectorySet.trajectorysims[minElem].trajID;
        }
    }
}

void trajectoryCallback(const trajectory_brain::TrajectorySims::ConstPtr& trajSetMsg) {
    if (!trajSetMsg->trajectorysims.empty()) {
        latestTrajectorySet.trajectorysims.clear();
        line_array.markers.clear();
        // iterate over full trajectories
        for (unsigned int i = 0; i < trajSetMsg->trajectorysims.size(); i++) {
            trajectory_brain::TrajectoryVector singleTrajectory = trajSetMsg->trajectorysims[i];
#ifdef DEBUG
            if (!singleTrajectory.trajectory.empty()) {
                // iterate over trajectory points
                for (unsigned int i = 0; i < singleTrajectory.trajectory.size(); i++) {
                    ROS_INFO("(%f, %f)", singleTrajectory.trajectory[i].x, singleTrajectory.trajectory[i].y);
                }
            }
#endif
            latestTrajectorySet.trajectorysims.push_back(singleTrajectory);

#ifdef RVIZ
            // publish visualization messages
            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "base_link";
            line_strip.header.stamp = ros::Time();
            line_strip.action = visualization_msgs::Marker::ADD;
  
            // Define message id and scale (thickness)
            line_strip.id = i;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            
            // Set the color and transparency (blue and solid)
            line_strip.scale.x = 0.01;
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;
            
            // Add points
            for (unsigned int i = 0; i < singleTrajectory.trajectory.size(); i++) {
                line_strip.points.push_back(singleTrajectory.trajectory[i]);
            }
            line_array.markers.push_back(line_strip);
#endif
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajcost");
    ros::NodeHandle nh;

    ros::Publisher pub_id = nh.advertise<trajectory_cost::TrajectoryID>("/trajectory/costID", 1);
    ros::Publisher pub_vis = nh.advertise<visualization_msgs::MarkerArray>("/trajectory/marker_array", 1);
    ros::Subscriber sub0 = nh.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &costmapInitCallback);
    ros::Subscriber sub1 = nh.subscribe<map_msgs::OccupancyGridUpdate>("/costmap_node/costmap/costmap_updates", 1, &costmapCallback);
    ros::Subscriber sub2 = nh.subscribe<trajectory_brain::TrajectorySims>("/trajectorysims", 1, &trajectoryCallback);
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        pub_id.publish(trajID);

#ifdef RVIZ
        // modify selected trajectory 
        if (!line_array.markers.empty()) {
            ROS_INFO("trajID: %d", trajID.trajID);
            if (trajID.trajID != -1) {
                line_array.markers[trajID.trajID].scale.x = 0.01; 
                line_array.markers[trajID.trajID].color.g = 1.0;
                line_array.markers[trajID.trajID].color.a = 1.0;
            }
            pub_vis.publish(line_array);
        }
#endif
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
