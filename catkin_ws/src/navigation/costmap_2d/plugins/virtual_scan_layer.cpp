#include <costmap_2d/virtual_scan_layer.h>
#include <pluginlib/class_list_macros.h>
#include <time.h>

struct timespec start, end;
static double TimeSpecToSeconds(struct timespec* ts) {
    return (double)ts->tv_sec + (double)ts->tv_nsec / 1000000000.0;
}
PLUGINLIB_EXPORT_CLASS(costmap_2d::VirtualScanLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::OCCLUDED;

namespace costmap_2d {
    void VirtualScanLayer::onInitialize()
    {
      ros::NodeHandle nh("~/" + name_), g_nh;
      rolling_window_ = layered_costmap_->isRolling();
      default_value_ = NO_INFORMATION;
    
      VirtualScanLayer::matchSize();
      current_ = true;
      init_ = true;
    
      global_frame_ = layered_costmap_->getGlobalFrameID();
    
      std::string source;
      nh.param("virtual_scan_source", source, std::string(""));
      ROS_INFO("    Subscribed to Topics: %s", source.c_str());
      nh.param("width", cm_width_, 10.0);
      nh.param("height", cm_height_, 10.0);
      nh.param("resolution", cm_resolution_, 0.1);
    
      // get our tf prefix
      ros::NodeHandle prefix_nh;
      const std::string tf_prefix = tf::getPrefixParam(prefix_nh);
    
      ros::NodeHandle source_node(nh, source);
    
      // get the parameters for the specific topic
      //double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
      std::string topic, sensor_frame, data_type;
    
      source_node.param("topic", topic, source);
      source_node.param("sensor_frame", sensor_frame, std::string(""));
      source_node.param("data_type", data_type, std::string("LaserScan"));

      source_node.param("max_radius", maxRadius, 3.0);
      source_node.param("radius_step", radiusStep, 0.1);
      source_node.param("ray_angle_steps", rayAngleSteps, 10);

      if (!sensor_frame.empty()) {
        sensor_frame = tf::resolve(tf_prefix, sensor_frame);
      }
    
      observation_buffers_.push_back(
          boost::shared_ptr < ObservationBuffer
              > (new ObservationBuffer(topic, 0, 0, 0, 2, 2.5, 3.0, *tf_, global_frame_, sensor_frame, 0.2)));
    
      marking_buffers_.push_back(observation_buffers_.back());
    
      if (data_type == "LaserScan") {
        boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
            > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 1));
    
        boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
            > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 1));
    
        filter->registerCallback(
            boost::bind(&VirtualScanLayer::laserScanCallback, this, _1, observation_buffers_.back()));
    
        observation_subscribers_.push_back(sub);
        observation_notifiers_.push_back(filter);
      }
    
      if (sensor_frame != "") {
        std::vector < std::string > target_frames;
        target_frames.push_back(global_frame_);
        target_frames.push_back(sensor_frame);
        observation_notifiers_.back()->setTargetFrames(target_frames);
      }
    
      dsrv_ = NULL;
      setupDynamicReconfigure(nh);
    }
    
    void VirtualScanLayer::setupDynamicReconfigure(ros::NodeHandle& nh) {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
      dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
          &VirtualScanLayer::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    
    VirtualScanLayer::~VirtualScanLayer() {
        if (dsrv_)
            delete dsrv_;
    }
    void VirtualScanLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level) {
      enabled_ = config.enabled;
      max_obstacle_height_ = config.max_obstacle_height;
      combination_method_ = config.combination_method;
    }
    
    void VirtualScanLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan,
                                             const boost::shared_ptr<ObservationBuffer>& buffer) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &start);

        float vbuffer = radiusStep / 2;
        int numRanges = laser_scan->ranges.size();
        float rayAngleSize = laser_scan->angle_increment * rayAngleSteps; // sweep step of angles

        int numRadiusSteps = std::ceil(maxRadius / radiusStep) - 1;
        int numAngleSteps = (numRanges / rayAngleSteps) - 1;
        occMap = new int[numRadiusSteps * numAngleSteps];

        int x_size = int(cm_width_);
        int y_size = int(cm_height_);
        int resolution = int(1/cm_resolution_);
        float rstep = cm_resolution_;
        float max_x;
        float max_y;
        if (init_ == true) {
            cm2om.reserve(x_size*y_size*resolution);

            max_x = maxRadius * std::max(std::sin(laser_scan->angle_max), std::abs(std::sin(laser_scan->angle_min)));
            max_y = maxRadius;
            for (int ci = 0; ci < x_size*resolution; ci++) {
                for (int cj = 0; cj < y_size*resolution; cj++) {
                    float x_dist = std::abs((ci * rstep) - x_size/2);
                    float y_dist = std::abs((cj * rstep) - y_size/2);
                    if (!(x_dist > max_x || y_dist > max_y)) {
                       // iterate over occMap
                       bool found = false;
                       for (int range = 0; range < numRadiusSteps; range++) {
                           int angleStart, angleEnd;
                           if (ci < x_size*resolution/2) {
                               angleStart = 0;
                               angleEnd = (numAngleSteps / 2) + 1;
                           }
                           else {
                               angleStart = (numAngleSteps / 2) - 1;
                               angleEnd = numAngleSteps;
                           }
                           for (int angle = angleStart; angle < angleEnd-1; angle++) {
                               float r1 = range * radiusStep;
                               float r2 = (range+1) * radiusStep;
                               float a1 = (angle * rayAngleSize) - laser_scan->angle_max;
                               float a2 = ((angle+1) * rayAngleSize) - laser_scan->angle_max;
                               float x_h = std::max(std::abs(r2 * std::sin(a1)), std::abs(r2 * std::sin(a2)));
                               float x_l = std::min(std::abs(r1 * std::sin(a1)), std::abs(r1 * std::sin(a2)));
                               float y_h = std::max(std::abs(r2 * std::cos(a1)), std::abs(r2 * std::cos(a2)));
                               float y_l = std::max(std::abs(r1 * std::cos(a1)), std::abs(r1 * std::cos(a2)));
                               float x_h1 = x_h;
                               float y_h1 = y_h;
                               float x_l1 = x_l;
                               float y_l1 = y_l;
                               if (a1 < (rayAngleSize/2) || a1 > (-rayAngleSize/2) ||
                                   a2 < (rayAngleSize/2) || a2 > (-rayAngleSize/2)) {
                                   x_h1 = x_h + 0.05;
                                   y_h1 = y_h + 0.05;
                                   x_l1 = x_l - 0.05;
                                   y_l1 = y_l - 0.05;
                               }
                               // check occMap limits
                               if ((x_dist < x_h1 && x_dist >= x_l1 && y_dist < y_h1 && y_dist >= y_l1)) {
                                   cm2om.push_back(range * numAngleSteps + angle);
                                   found = true;
                                   break;
                               }
                           }
                           if (found) {
                               break;
                           }
                           if (range == numRadiusSteps - 1) {
                               cm2om.push_back(-1);
                           }
                       }
                    }
                    else {
                       cm2om.push_back(-1);
                    }
                }
            }
            init_ = false;
        }

        for (int range = 0; range < numRadiusSteps; range++) { // sweep across distance
            for (int angle = 0; angle < numAngleSteps; angle++) { // sweep across rays
                int startIdx = angle * rayAngleSteps;
                for (int step = 0; step < rayAngleSteps; step++) {
                    int curIdx = std::min(startIdx + step, numRanges); 
		    float dp = laser_scan->ranges[curIdx];
                    if (std::isnan(dp)) {
			dp = laser_scan->range_max;
                    }
		    if (dp > (range*radiusStep - vbuffer) && dp < (range*radiusStep + vbuffer)) {
                        occMap[range * numAngleSteps + angle] = LETHAL_OBSTACLE;
			break;
                    }
		    else if ((range*radiusStep + vbuffer) >= dp) {
                        occMap[range * numAngleSteps + angle] = OCCLUDED;
                        break;
                    }
                    else {
                        occMap[range * numAngleSteps + angle] = FREE_SPACE;
                    }
                }
            }
        }
        for (int ci = 0; ci < x_size*resolution; ci++) {
            for (int cj = y_size*resolution/2; cj < y_size*resolution; cj++) {
                int idx = cm2om[ci*y_size*resolution+cj];
                costmap_[ci*y_size*resolution + cj] = NO_INFORMATION;
                if (idx != -1) {
                    costmap_[ci*y_size*resolution + cj] = occMap[idx];
                }
            }
        }
        clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        printf("laserscan callback time: %f\n", TimeSpecToSeconds(&end) - TimeSpecToSeconds(&start));
    }
    
    
    void VirtualScanLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                        double* min_y, double* max_x, double* max_y) {
        touch(-200, -200, min_x, min_y, max_x, max_y);
        touch(200, 200, min_x, min_y, max_x, max_y);
    }
    
    void VirtualScanLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
          return;

        for (int j = min_j; j < max_j; j++) {
            for (int i = min_i; i < max_i; i++) {
                int index = getIndex(i, j);
                master_grid.setCost(i, j, costmap_[index]);
            }
        }
    }
    
    void VirtualScanLayer::activate()
    {
      // if we're stopped we need to re-subscribe to topics
      for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
      {
        if (observation_subscribers_[i] != NULL)
          observation_subscribers_[i]->subscribe();
      }
    
      for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
      {
        if (observation_buffers_[i])
          observation_buffers_[i]->resetLastUpdated();
      }
    }
    void VirtualScanLayer::deactivate()
    {
      for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
      {
        if (observation_subscribers_[i] != NULL)
          observation_subscribers_[i]->unsubscribe();
      }
    }
    
    void VirtualScanLayer::reset()
    {
        deactivate();
        resetMaps();
        current_ = true;
        activate();
    }
} // end namespace
