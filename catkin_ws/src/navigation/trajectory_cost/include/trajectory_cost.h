#ifndef TRAJECTORY_COST_H_
#define TRAJECTORY_COST_H_ 

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "fake_trajectory/TrajectoryVector.h"
#include "trajectory_cost/TrajectoryCostVector.h"


fake_trajectory::TrajectoryVector latestTrajectory;
int resolution;
int width;
int height;
std::vector<signed char> data;
trajectory_cost::TrajectoryCostVector tc;

#endif
