#ifndef TRAJECTORY_COST_H_
#define TRAJECTORY_COST_H_ 

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "fake_trajectory/TrajectoryVector.h"
#include "fake_trajectory/TrajectorySims.h"
#include "trajectory_cost/TrajectoryID.h"
#include <algorithm>
#include <vector>


fake_trajectory::TrajectorySims latestTrajectorySet;
int resolution;
int width;
int height;
std::vector<signed char> data;
trajectory_cost::TrajectoryID trajID;

#endif