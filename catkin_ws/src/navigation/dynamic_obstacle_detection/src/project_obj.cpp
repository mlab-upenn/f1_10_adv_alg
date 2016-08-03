#include "project_obj.h"


using namespace visualization_msgs;
using namespace nav_msgs;
using namespace message_filters;

//void velCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_msg,
//                 const nav_msgs::Odometry::ConstPtr& odom_msg){
//}

void apriltagCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    if (!msg->markers.empty()) {
        int time_s = msg->markers[0].header.stamp.sec;
        int time_ns = msg->markers[0].header.stamp.nsec;
        double pose_z = msg->markers[0].pose.position.x;
        double pose_x = msg->markers[0].pose.position.y;

        // Transform Quaternion to RPY
        q = Eigen::Quaternion<double>(msg->markers[0].pose.orientation.x, 
                                      msg->markers[0].pose.orientation.y, 
                                      msg->markers[0].pose.orientation.z, 
                                      msg->markers[0].pose.orientation.w);
        Eigen::Matrix3d m(q);
        Eigen::Vector3d ea = m.eulerAngles(0, 1, 2);
        double pose_th = ea[1];

        // Linear velocity
        int dtime = (time_s - prev.time_s) * 1E9 + (time_ns - prev.time_ns);
        double vel = sqrt(pow(pose_z - prev.pose_z, 2)
                     + pow(pose_x - prev.pose_x, 2)) / (dtime / 1E9);
        double vel_z = (pose_z - prev.pose_z) / (dtime / 1E9);
        double vel_x = (pose_x - prev.pose_x) / (dtime / 1E9);
        prev.pose_z = pose_z;
        prev.pose_x = pose_x;
        prev.time_s = time_s;
        prev.time_ns = time_ns;
        //ROS_INFO("\n(%f %f %f %f)", pose_z, pose_x, pose_th, vel);
        printf("%f %f %f %f\n", pose_z, pose_x, pose_th, vel);

        random_numbers::RandomNumberGenerator rng;
        std::vector<pf_meas> pf_list;
        for (int i = 0; i < NUM_PARTICLES; i++) {
            pf_meas *pf = new pf_meas;

            // Create multivariate gaussian 
            pf->pose_z = rng.gaussian(pose_z,0.1);
            pf->pose_x = rng.gaussian(pose_x,0.1);
            pf->pose_th = rng.gaussian(pose_th,0.1);
            pf->vel_z = rng.gaussian(vel_z,0.1);
            pf->vel_x = rng.gaussian(vel_x,0.1);
            pf_list.push_back(*pf);
        }
        
        //for (int j = 0; j < NUM_PARTICLES; j++) {
        //    pf_meas *pf1 = new pf_meas; 
        //    *pf1 = pf_list[j];
        //    //ROS_INFO("(%f %f %f %f)", pf1->pose_z, pf1->pose_x, pf1->pose_th, pf1->vel);
        //    printf("%f %f %f %f\n", pf1->pose_z, pf1->pose_x, pf1->pose_th, pf1->vel);
        //}
    }
}

void vsCallback(const sensor_msgs::LaserScanConstPtr& laser_scan) {
    double vbuffer = radiusStep / 2;
    int numRanges = laser_scan->ranges.size();
    double rayAngleSize = laser_scan->angle_increment * rayAngleSteps; // sweep step of angles
    int numRadiusSteps = std::ceil(maxRadius / radiusStep) - 1;
    int numAngleSteps = (numRanges / rayAngleSteps) - 1;
    double max_x = maxRadius * std::max(std::sin(laser_scan->angle_max), std::abs(std::sin(laser_scan->angle_min)));
    double max_y = maxRadius;
    if (vs_init) {
        occMap = new int[numRadiusSteps * numAngleSteps];
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
                           double r1 = range * radiusStep;
                           double r2 = (range+1) * radiusStep;
                           double a1 = (angle * rayAngleSize) - laser_scan->angle_max;
                           double a2 = ((angle+1) * rayAngleSize) - laser_scan->angle_max;
                           double x_h = std::max(std::abs(r2 * std::sin(a1)), std::abs(r2 * std::sin(a2)));
                           double x_l = std::min(std::abs(r1 * std::sin(a1)), std::abs(r1 * std::sin(a2)));
                           double y_h = std::max(std::abs(r2 * std::cos(a1)), std::abs(r2 * std::cos(a2)));
                           double y_l = std::max(std::abs(r1 * std::cos(a1)), std::abs(r1 * std::cos(a2)));
                           double x_h1 = x_h;
                           double y_h1 = y_h;
                           double x_l1 = x_l;
                           double y_l1 = y_l;
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
        vs_init = false;
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
    costmap_ = new unsigned char[x_size*resolution*y_size*resolution];
    for (int ci = 0; ci < x_size*resolution; ci++) {
        for (int cj = y_size*resolution/2; cj < y_size*resolution; cj++) {
            int idx = cm2om[ci*y_size*resolution+cj];
            costmap_[ci*y_size*resolution + cj] = NO_INFORMATION;
            if (idx != -1) {
                costmap_[ci*y_size*resolution + cj] = occMap[idx];
            }
        }
    }
}

void initVsParams() {
      maxRadius = 5.0;
      radiusStep = 0.1;
      rayAngleSteps = 10;
      x_size = 10;
      y_size = 10;
      rstep = 0.05;
      resolution = int(1/rstep);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "project_obj");
    ros::NodeHandle nh;

    initVsParams();

    ros::Subscriber sub1 = nh.subscribe("/apriltags/marker_array", 1, &apriltagCallback);
    ros::Subscriber sub2 = nh.subscribe("/scan_dl", 1, &vsCallback);

    //ros::Publisher pub = nh.advertise<MarkerArray>("/apriltags/maker_array_ls", 1);
    //message_filters::Subscriber<MarkerArray> sub1(nh, "/apriltags/marker_array", 1);
    //message_filters::Subscriber<Odometry> sub2(nh, "/odom", 1);

    //typedef sync_policies::ApproximateTime<Odometry, Odometry> MySyncPolicy;
    //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub2, sub2);
    //sync.registerCallback(boost::bind(&velCallback, _1, _2));

    ros::spin();
    return 0;
}
