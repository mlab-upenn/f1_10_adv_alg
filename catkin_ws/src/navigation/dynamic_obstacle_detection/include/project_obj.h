#ifndef PROJECT_OBJ_H_
#define PROJECT_OBJ_H_

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random_numbers/random_numbers.h>

static const int NUM_PARTICLES = 10;
static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char OCCLUDED = 128;
static const unsigned char FREE_SPACE = 0;

// geometry is const (m)
static const double geo_w = 0.55118; 
static const double geo_l = 0.29718;

bool init = true;
Eigen::Quaternion<double> q;
struct _pf_meas;

typedef struct _pf_meas {
    // x, y
    double pose_z;
    double pose_x;

    // theta
    double pose_th;

    // velocity
    int time_s, time_ns;
    double vel_z;
    double vel_x;

} pf_meas;

pf_meas prev;

// virtual scan
bool vs_init = true;

double maxRadius;
double radiusStep;
double rayAngleSteps;
int x_size;
int y_size;
double rstep;
int resolution;
int* occMap;
std::vector<int> cm2om;
unsigned char* costmap_;

#endif
