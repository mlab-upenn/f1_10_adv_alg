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
#include <algorithm>
#include <vector>

static const int NUM_PARTICLES = 10;
static const float DT = 0.1;
static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char OCCLUDED = 128;
static const unsigned char FREE_SPACE = 0;

// geometry is const (m)
static const float geo_l = 0.55118; 
static const float geo_w = 0.29718;
struct Point { float x, y; };
Point rotate_point (float cx, float cy, float angle, float p);

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
void createLaserScan(std::vector<pf_meas> pf_list);
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
unsigned char* pf_costmap;
unsigned char* ls_costmap;

// virtual scan port
double angleIncrement;
double rayAngleSize;
double minAngle;
double maxAngle;
double vbuffer;
float maxRangeVal;
int numRadiusSteps;
int numAngleSteps;
int numRanges;

void initVirtualScan(const sensor_msgs::LaserScanConstPtr& laser_scan);
unsigned char* calcVirtualScan(std::vector<float> laserscanArray);
void compareVirtualScan(unsigned char* pf_costmap);

#endif
