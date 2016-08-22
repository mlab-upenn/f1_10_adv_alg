/*
*  trajectory_brain.cpp
*  Trajectory Generator for Cubic Spline Trajectory Generation with RBF-N
*
*  Created by Matthew O'Kelly on 10/7/15.
*  Copyright (c) 2016 Matthew O'Kelly. All rights reserved.
*  mokelly@seas.upenn.edu
*
*/

/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
*  * Neither the name of Autoware nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

// Standard ROS crap...
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>


// Visualization stuff...
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

// Change this directory name...? Think this is right...
#include "trajectory_brain/TrajectoryVector.h"
#include "trajectory_brain/TrajectorySims.h"
#include "trajectory_cost/TrajectoryID.h"
#include "trajectory_brain.h"

// These are all the RBFA networks...
#include <rbfa.h>
//#include <rbfa1.c>
//#include <rbfa2.c>
//#include <rbfa3.c>
//#include <rbfa4.c>
//#include <rbfa5.c>

#include <omp.h>

//#define DEBUG

/* CONSTANTS */
// Seed point for state lattice in meters
static const double LOOKAHEAD = 6.5;
// How wide is the track typically, meters
static const double TRACKWIDTH = 2.0;
// Loop rate for node in Hz.
static const int LOOP_RATE = 20;
// Timestep
static const double TIME_STEP = 0.2;

/* LOCKS */
static bool costComputed = TRUE;
static bool havePose = TRUE;

/* VEHICLE STATE */
// Global variable to hold vehicle state
union State vehGlobal;

// Global variable to store all possible trajectories
union Parameters trajLattice[LOOP_COUNT];

// The ID of the best trajectory from the lattice
int bestTrajID;

// Store the previous trajectory to compensate for delays
union Parameters prev_traj;


// The current trajectory so everyone can see...
union Parameters current_traj;


/////////////////////////////////////////////////////////////////
// Compute trajectory
// Computes valid trajctories for a particular goal...
/////////////////////////////////////////////////////////////////

// NOTE: This will be part of the kernel STEP 2.
union Parameters rbfTrajectory(double sx, double sy, double theta)
{
  // Setup union of structures to hold the results of the trajectory generation
  union Spline curvature;
  union Parameters reparam;

  #ifdef DEBUG
  printf("sx: %f, sy: %f, theta %f\n", sx, sy, theta);
  #endif

  reparam.s = 0.0;
  reparam.a = 0.0;
  reparam.b = 0.0;
  reparam.c = 0.0;
  reparam.e = 0.0;
  reparam.success = FALSE;

  if (sx < 10.00 && sx > 4.00 && sy > -4.00 && sy <4.00)
  {
    #ifdef DEBUG
    printf("Generating a trajectory\n");
    #endif
    // This is the arc length of the trajectory,
    curvature.s = rbfa1(sx, sy, theta*10.0);
    // This is the curvature at the knot point 0
    curvature.kappa_0 = rbfa2(sx, sy, theta*10.0);
    // This is the curvature at knot point 1
    curvature.kappa_1 = rbfa3(sx, sy, theta*10.0);
    // This is the curvature at knot point 2
    curvature.kappa_2 = rbfa4(sx, sy, theta*10.0);
    // This is the curvature at knot point 3
    curvature.kappa_3 = rbfa5(sx, sy, theta*10.0);
    // Note that the request was succesfull
    curvature.success= TRUE;

    // Now compute reparameterization
    reparam.s = curvature.s;
    reparam.a = curvature.kappa_0;
    reparam.b = ((-0.50)*(-2*curvature.kappa_3 + 11*curvature.kappa_0 - 18*curvature.kappa_1 + 9*curvature.kappa_2)/curvature.s);
    reparam.c = ((4.50)*(-curvature.kappa_3 + 2*curvature.kappa_0 - 5*curvature.kappa_1 +4*curvature.kappa_2)/(curvature.s*curvature.s));
    reparam.e = ((-4.50)*(-curvature.kappa_3 + curvature.kappa_0 - 3*curvature.kappa_1 + 3*curvature.kappa_2)/(curvature.s*curvature.s*curvature.s));
    reparam.success = TRUE;
  }

  // Return the results
  return reparam;
}

/////////////////////////////////////////////////////////////////
// Pose Callback Function
// Handle new poses coming from localization and state estimation.
/////////////////////////////////////////////////////////////////

void poseCallback(const nav_msgs::Odometry& pose_msg)
{
  vehGlobal.sx = pose_msg.pose.pose.position.x;
  vehGlobal.sy = pose_msg.pose.pose.position.y;
  vehGlobal.theta = pose_msg.pose.pose.orientation.z;
  vehGlobal.kappa = pose_msg.twist.twist.angular.z;

  double vx = pose_msg.twist.twist.linear.x;
  double vy = pose_msg.twist.twist.linear.y;

  vehGlobal.v = sqrt(pow(vx,2) + pow(vy,2));
  vehGlobal.vdes = vehGlobal.v;

  ROS_INFO_STREAM("Got a state update");
  havePose = TRUE;
}


/////////////////////////////////////////////////////////////////
// Trajectory Cost Recieved
// Get the best trajectory from the lattice
/////////////////////////////////////////////////////////////////
void trajCostCallback( const trajectory_cost::TrajectoryID& cost_msg)
{
  ROS_INFO("cost_id: %d", cost_msg.trajID);
  costComputed = TRUE;
  bestTrajID = cost_msg.trajID;
  current_traj = trajLattice[bestTrajID];

  std_msgs::Float64MultiArray spline;
  spline.data.clear();

  for(int i = 0; i < 6;i++)
  {
    spline.data.push_back(current_traj.param_value[i]);
  }

  spline_parameters_pub.publish(spline);

}

/////////////////////////////////////////////////////////////////
// Sample Trajectory Poses
// This function can move the vehicle state forward along
// a trajectory.
// Can be used for for dealing with latency.
/////////////////////////////////////////////////////////////////
// NOTE: Should this be part of the kernel, alternate parallelize with OMP post kernel. Maybe try both ways
trajectory_brain::TrajectoryVector trajFwdSim( union Parameters traj, union State veh, unsigned int trajID, unsigned int numPts = 60)
{
  if (veh.v == 0) {
    veh.v = 1;
  }
  if (traj.s == 0) {
    traj.s = 1;
  }
  double trajectory_time = traj.s/veh.v;
  double simtime = 0.0;

  //double a = traj.a;
  double b = traj.b;
  double c = traj.c;
  double e = traj.e;

  trajectory_brain::TrajectoryVector samples;
  samples.trajID = trajID;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = 0.0;

  double kappa = veh.kappa;
  double theta = veh.theta;
  double v = veh.v;

  double dt = trajectory_time / numPts;

  #ifdef DEBUG
  ROS_INFO("traj.s: %f, veh.v: %f, traj time: %f\n", traj.s, veh.v, trajectory_time);
  #endif

  while (simtime < trajectory_time)
  {

    double dkappa = b*v + 2*c*v*v*simtime + 3*e*v*v*v*pow(simtime,2);
    kappa = kappa + dkappa;
    double dtheta = v*kappa;
    theta = theta + dtheta;

    p.x = p.x + v*cos(theta);
    p.y = p.y + v*sin(theta);
    p.z = 0;
    samples.trajectory.push_back(p);

    simtime = simtime + dt;
  }

  return samples;


}

/////////////////////////////////////////////////////////////////
// Compute State Lattice Points
// This function computes an array of points to generate
// trajectories for. Makes it easy to parallelize.
/////////////////////////////////////////////////////////////////

// NOTE: This is part of the kernel but you need to get rid of the for loop
// switch it to depend on the thread id. The TRACKWIDTH variable should probably
// be changed to be a function of the number of threads so we compute a lattice
// that stays within the bounds. STEP 1.
StateLattice computeStateLattice( union State veh)
{

  // Read in the current orientation of the vehicle
  double theta = veh.theta;

  // Compute x_0, the x seed point for the state lattice
  double x_0 = LOOKAHEAD*cos(theta);

  // Compute y_0, the y seed point for the state lattice
  double y_0 = LOOKAHEAD*sin(theta);

  // Compute the spacing
  double dlattice_x = (TRACKWIDTH/(SAMPLES-1.00))*sin(theta);
  double dlattice_y = (TRACKWIDTH/(SAMPLES-1.00))*cos(theta);

  union StateLattice lattice;

  for (int i=0; i<LOOP_COUNT; i++)
  {
    // Compute offsets for positive and negative lattice points
    double offset_x = ((double) i) * dlattice_x;
    double offset_y = ((double) i) * dlattice_y;

    // Put them in the data structure relative to the seed point
    lattice.x[i] = x_0 + offset_x;
    lattice.x[i+(int)SAMPLES] = x_0 - offset_x;
    lattice.y[i] = y_0 + offset_y;
    lattice.y[i+(int)SAMPLES] = y_0 - offset_y;
  }

  return lattice;


}

#define TRAJECTORY_PTS 60

//#define CUDA_KERNEL
#ifdef CUDA_KERNEL
__global__ void trajgen_cuda_kernel(State *state, StateLattice *lattice,
                                    geometry_msgs::Point *trajectory_data,
                                    unsigned int *trajectory_lengths,
                                    bool *trajstatus) {

  Parameters traj = rbfTrajectory(lattice->x[blockIdx.x], lattice->y[blockIdx.x], 0.0);
  trajstatus[blockIdx.x] = traj.success;

  if (traj.success == true) {
    trajectory_brain::TrajectoryVector tv = trajFwdSim(trajLattice[blockIdx.x], *state, blockIdx.x, TRAJECTORY_PTS);
    std::copy(tv.trajectory.begin(), tv.trajectory.end(), blockIdx.x * tv.trajectory.size() + trajectory_data);
  }
}
#endif

/////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////



int main(int argc, char **argv)
{
  prev_traj.s = 0.0;
  prev_traj.a = 0.0;
  prev_traj.b = 0.0;
  prev_traj.c = 0.0;
  prev_traj.e = 0.0;
  prev_traj.success = TRUE;
  current_traj.s = 0.0;
  current_traj.a = 0.0;
  current_traj.b = 0.0;
  current_traj.c = 0.0;
  current_traj.e = 0.0;
  current_traj.success = TRUE;


  // These are local variables for keeping track of the previous
  // timestamp and orientation. Used in the estimate of curvature...

  // Write to console that we are starting trajectory generation
  ROS_INFO_STREAM("Trajectory Generation Begins: ");

  // Set up ROS, TO DO: change to proper name (same with rest of the file)
  ros::init(argc, argv, "trajectory_brain");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");


  // Publish the following topics:
  spline_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("spline", 10);
  ros::Publisher state_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("state", 10);
  ros::Publisher pub = nh.advertise<trajectory_brain::TrajectorySims>("/trajectorysims", 1);



  // Subscribe to the following topics:
  ros::Subscriber odometry_subscriber = nh.subscribe("/odom",1, poseCallback);
  ros::Subscriber cost_subscriber = nh.subscribe("/trajectory/costID", 1, trajCostCallback);
  // Set the loop rate unit is Hz
  ros::Rate loop_rate(LOOP_RATE);

  bool trajstatus[LOOP_COUNT];

  #ifdef CUDA_KERNEL
    State *dev_state;
    StateLattice *dev_lattice;
    geometry_msgs::Point dev_pts[TRAJECTORY_PTS * LOOP_COUNT], pts[TRAJECTORY_PTS * LOOP_COUNT];
    bool dev_trajstatus[LOOP_COUNT];

    memset(trajstatus, false, LOOP_COUNT * sizeof(bool));

    cudaMalloc((void **) &dev_state, sizeof(State));
    cudaMalloc((void **) &dev_lattice, sizeof(lattice));
    cudaMalloc((void **) &dev_pts, sizeof(geometry_msgs::Point) * TRAJECTORY_PTS);
    cudaMalloc((void **) &dev_trajstatus, sizeof(bool) * LOOP_COUNT);
  #endif

  // Here we go....
  while (ros::ok())
  {
    ROS_INFO_STREAM("Running Trajectory Generator");
    std_msgs::Bool _lf_stat;

    if (costComputed == TRUE)
    {
      costComputed=FALSE;
      // Compute set of goals...
      prev_traj = current_traj;
      // Store all the trajectory sims
      trajectory_brain::TrajectorySims sims;
      union StateLattice lattice;
      // Compute the state lattice
      lattice = computeStateLattice(vehGlobal);
      sims.trajectorysims.clear();
      sims.trajectorysims.resize(LOOP_COUNT);

      #ifdef CUDA_KERNEL
      cudaMemcpy(dev_state, &vehGlobal, sizeof(State), cudaMemcpyHostToDevice);
      cudaMemcpy(dev_lattice, &lattice, sizeof(StateLattice), cudaMemcpyHostToDevice);

      trajgen_cuda_kernel<<<LOOP_COUNT, 1>>>(dev_state, dev_lattice, dev_vector, dev_trajstatus);

      cudaMemcpy(pts, dev_pts, sizeof(geometry_msgs::Point) * TRAJECTORY_PTS, cudaMemcpyDeviceToHost);
      cudaMemcpy(trajstatus, dev_trajstatus, sizeof(bool) * LOOP_COUNT, cudaMemcpyDeviceToHost);

      for (int i = 0; i < LOOP_COUNT; i++) {
        sims.trajectorysims[i].trajID = i;
        sims.trajectorysims[i].trajectory.resize(TRAJECTORY_PTS);
        memcpy(sims.trajectorysims[i].trajectory.data(), pts[i * TRAJECTORY_PTS], TRAJECTORY_PTS);
      }

      #else
      #pragma omp parallel for
      for(int i=0; i<LOOP_COUNT; i++)
      {
        trajLattice[i] = rbfTrajectory(lattice.x[i], lattice.y[i], 0.0);
        trajstatus[i] = trajLattice[i].success;

        if(trajLattice[i].success == TRUE)
        {
          trajectory_brain::TrajectoryVector tv = trajFwdSim(trajLattice[i],vehGlobal, i, TRAJECTORY_PTS);
          sims.trajectorysims[i] = tv;
          #ifdef DEBUG
          printf("I have %d threads\n", omp_get_num_threads());
          printf("pushed %d onto sims\n", i);
          printf("id: %d\n", tv.trajID);
          for (int j = 0; j < tv.trajectory.size(); j++) {
            printf("px: %f py: %f\n", tv.trajectory[j].x, tv.trajectory[j].y);
          }
          #endif
        }
      }

      #ifdef DEBUG
      printf("sims.trajectorysims.size: %d\n", sims.trajectorysims.size());
      for (int j = 0; j < sims.trajectorysims.size(); j++) {
        printf("traj %i size: %d, ", j, sims.trajectorysims[j].trajectory.size());
        printf("x: %f ", sims.trajectorysims[j].trajectory[0].x);
      }
      printf("\n");
      #endif
      #endif

      ROS_INFO("Trajectory published\n");
      pub.publish(sims);
      if (initialPass) {
        costComputed = TRUE;
        initialPass = false;
      }
    }
    else
    {
      ROS_INFO_STREAM("Cost Compute Blocking...");
    }
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
