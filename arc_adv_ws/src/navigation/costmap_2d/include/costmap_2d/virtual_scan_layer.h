#ifndef VIRTUAL_SCAN_LAYER_H_
#define VIRTUAL_SCAN_LAYER_H_

#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <costmap_2d/ObstaclePluginConfig.h>
#include <costmap_2d/footprint.h>

namespace costmap_2d {
    class VirtualScanLayer : public CostmapLayer {
    public:
      VirtualScanLayer() {
        costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
      }
    
      virtual ~VirtualScanLayer();
      virtual void onInitialize();
      virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    
      virtual void activate();
      virtual void deactivate();
      virtual void reset();
    
      void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                             const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
     
    protected:
      virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
    
      std::string global_frame_;  ///< @brief The global frame for the costmap
      double max_obstacle_height_;  ///< @brief Max Obstacle Height
    
      laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds
    
      std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
      std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
      std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors
      std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles
      
      std::vector<costmap_2d::Observation> static_marking_observations_;
    
      bool rolling_window_;
      dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig> *dsrv_;
    
      int combination_method_;
      bool init_;
      double cm_width_, cm_height_, cm_resolution_;
      double maxRadius, radiusStep;
      int rayAngleSteps;  
      int* occMap;
      std::vector<int> cm2om;
    
    private:
      void reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level);
    };
}
#endif
