#ifndef TRAJECTORY_COST_H_
#define TRAJECTORY_COST_H_ 

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "trajectory_brain/TrajectoryVector.h"
#include "trajectory_brain/TrajectorySims.h"
#include "trajectory_cost/TrajectoryID.h"
#include <algorithm>
#include <vector>

#include <visualization_msgs/Marker.h>


trajectory_brain::TrajectorySims latestTrajectorySet;
int resolution;
int width;
int height;
std::vector<signed char> data;
trajectory_cost::TrajectoryID trajID;

visualization_msgs::Marker line_strip;

#endif
