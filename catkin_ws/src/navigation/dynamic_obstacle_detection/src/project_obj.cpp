#include "project_obj.h"


using namespace visualization_msgs;
using namespace nav_msgs;
using namespace message_filters;


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
        createLaserScan(pf_list);
    }
}

Point rotate_point(float cx, float cy, float angle, Point p) {
    float s = std::sin(angle);
    float c = std::cos(angle);

    float x = p.x - cx;
    float y = p.y - cy;

    Point pr;
    pr.x = (x * c) - (y * s) + cx;
    pr.y = (x * s) + (y * c) + cy;
    return pr;
}

void createLaserScan(std::vector<pf_meas> pf_list) {
    for (int i = 0; i < pf_list.size(); i++) {
        // advance particle by velocity
        float x_pos = pf_list[i].pose_x + pf_list[i].vel_x*DT;
        float z_pos = pf_list[i].pose_z + pf_list[i].vel_z*DT;
        //printf("cent: [%f, %f]\n", x_pos, z_pos);

        // get corners of vehicle
        Point p1 = {x_pos - geo_w/2, z_pos - geo_l/2};
        Point p2 = {x_pos - geo_w/2, z_pos + geo_l/2};
        Point p3 = {x_pos + geo_w/2, z_pos - geo_l/2};
        Point p4 = {x_pos + geo_w/2, z_pos + geo_l/2};
        //printf("orig: [[%f, %f];[%f, %f];[%f, %f];[%f, %f]]\n", p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

        // rotate vehicle by -theta, cw is positive so reverse
        p1 = rotate_point(x_pos, z_pos, -pf_list[i].pose_th, p1);
        p2 = rotate_point(x_pos, z_pos, -pf_list[i].pose_th, p2);
        p3 = rotate_point(x_pos, z_pos, -pf_list[i].pose_th, p3);
        p4 = rotate_point(x_pos, z_pos, -pf_list[i].pose_th, p4);
        //printf("rotate: [[%f, %f];[%f, %f];[%f, %f];[%f, %f]]\n", p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

        std::vector<float> x_v {p1.x, p2.x, p3.x, p4.x};
        std::vector<float> y_v {p1.y, p2.y, p3.y, p4.y};
        std::vector<float>::iterator min_x = std::min_element(std::begin(x_v), std::end(x_v));
        std::vector<float>::iterator max_x = std::max_element(std::begin(x_v), std::end(x_v));
        std::vector<float>::iterator min_y = std::min_element(std::begin(y_v), std::end(y_v));
        int min_x_pos = std::distance(std::begin(x_v), min_x);
        int max_x_pos = std::distance(std::begin(x_v), max_x);
        int min_y_pos = std::distance(std::begin(y_v), min_y);
        float min_th = std::atan(x_v[min_x_pos]/y_v[min_x_pos]);
        float mid_th = std::atan(x_v[min_y_pos]/y_v[min_y_pos]);
        float max_th = std::atan(x_v[max_x_pos]/y_v[max_x_pos]);

        // ergo vehicle at (0,0)
        std::vector<float> laserscanArray;
        for (float rayangle = minAngle; rayangle <= maxAngle; rayangle += angleIncrement) {
            // angle lies on left half of ergo vehicle
            if (rayangle >= min_th && rayangle < mid_th) {
                // extrapolate distance based on angle
                float ratio_th = (mid_th - rayangle) / (mid_th - min_th);
                float dist_mid = sqrt(pow(x_v[min_y_pos],2) + pow(y_v[min_y_pos],2));
                float dist_left = sqrt(pow(x_v[min_x_pos],2) + pow(y_v[min_x_pos],2));
                float dist_cur = dist_mid - ratio_th * (dist_mid - dist_left);
                laserscanArray.push_back(dist_cur);
                //printf("dist1: %f %f %f\n", dist_left, dist_cur, dist_mid);
            } 
            else if (rayangle >= mid_th  && rayangle <= max_th) {
                // extrapolate distance based on angle
                float ratio_th = (max_th - rayangle) / (max_th - mid_th);
                float dist_mid = sqrt(pow(x_v[min_y_pos],2) + pow(y_v[min_y_pos],2));
                float dist_right = sqrt(pow(x_v[max_x_pos],2) + pow(y_v[max_x_pos],2));
                float dist_cur = dist_right - ratio_th * (dist_right - dist_mid);
                laserscanArray.push_back(dist_cur);
                //printf("dist2: %f %f %f\n", dist_mid, dist_cur, dist_right);
            } 
            else { // outside vehicle angle range
                laserscanArray.push_back(-1);
            }
            //printf("angle: %f\n", rayangle);
        }
        laserscanArray.push_back(-1); // 1 extra to match original laserscan
        unsigned char* temp = calcVirtualScan(laserscanArray);
        pf_costmap = new unsigned char[x_size*resolution*y_size*resolution];
        for (int ci = 0; ci < x_size*resolution; ci++) {
            for (int cj = 0; cj < y_size*resolution; cj++) {
                pf_costmap[ci*y_size*resolution + cj] = temp[ci*y_size*resolution + cj];
            }
        }
        compareVirtualScan(pf_costmap);
    }
}

void compareVirtualScan(unsigned char* pf_costmap) {
    int total = 0;
    for (int i = 0; i < x_size*resolution; i++) {
        for (int j = 0; j < y_size*resolution; j++) {
            total += (ls_costmap[i*y_size*resolution + j] != pf_costmap[i*y_size*resolution +j]);
            if (i < 10 && j < 10) {
                printf("%d ", ls_costmap[i*y_size*resolution +j]);
                printf("%d\n", pf_costmap[i*y_size*resolution +j]);
            }
        }
    }
    printf("total: %d\n");
}

void initVirtualScan(const sensor_msgs::LaserScanConstPtr& laser_scan) {
    vbuffer = radiusStep / 2;
    numRanges = laser_scan->ranges.size();
    maxRangeVal = laser_scan->range_max;
    angleIncrement = laser_scan->angle_increment;
    rayAngleSize = laser_scan->angle_increment * rayAngleSteps; // sweep step of angles
    minAngle = laser_scan->angle_min;
    maxAngle = laser_scan->angle_max;
    numRadiusSteps = std::ceil(maxRadius / radiusStep) - 1;
    numAngleSteps = (numRanges / rayAngleSteps) - 1;
    double max_x = maxRadius * std::max(std::sin(laser_scan->angle_max), std::abs(std::sin(laser_scan->angle_min)));
    double max_y = maxRadius;
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
                    if (found) { break;}
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

unsigned char* calcVirtualScan(std::vector<float> laserscanArray) {
    for (int range = 0; range < numRadiusSteps; range++) { // sweep across distance
        for (int angle = 0; angle < numAngleSteps; angle++) { // sweep across rays
            int startIdx = angle * rayAngleSteps;
            for (int step = 0; step < rayAngleSteps; step++) {
                int curIdx = std::min(startIdx + step, numRanges); 
    	        float dp = laserscanArray[curIdx];
                if (std::isnan(dp) || dp == -1) {
    		    dp = maxRangeVal;
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
    unsigned char* costmap_ = new unsigned char[x_size*resolution*y_size*resolution];
    for (int ci = 0; ci < x_size*resolution; ci++) {
        for (int cj = 0; cj < y_size*resolution; cj++) {
            int idx = cm2om[ci*y_size*resolution+cj];
            costmap_[ci*y_size*resolution + cj] = NO_INFORMATION;
            if (idx != -1) {
                costmap_[ci*y_size*resolution + cj] = occMap[idx];
            }
        }
    }
    return costmap_;
}

void vsCallback(const sensor_msgs::LaserScanConstPtr& laser_scan) {
    if (vs_init) {
        initVirtualScan(laser_scan);
    }
    std::vector<float> laserscanArray;
    for (int i = 0; i < laser_scan->ranges.size(); i++) {
        laserscanArray.push_back(laser_scan->ranges[i]);
    }
    unsigned char* temp = calcVirtualScan(laserscanArray);
    ls_costmap = new unsigned char[x_size*resolution*y_size*resolution];
    for (int ci = 0; ci < x_size*resolution; ci++) {
        for (int cj = 0; cj < y_size*resolution; cj++) {
            ls_costmap[ci*y_size*resolution + cj] = temp[ci*y_size*resolution + cj];
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
    ros::Subscriber sub1 = nh.subscribe("/scan_dl", 1, &vsCallback);
    ros::Duration(1).sleep();
    ros::Subscriber sub2 = nh.subscribe("/apriltags/marker_array", 1, &apriltagCallback);
    //ros::Publisher pub = nh.advertise<MarkerArray>("/apriltags/maker_array_ls", 1);
    //message_filters::Subscriber<MarkerArray> sub1(nh, "/apriltags/marker_array", 1);
    //message_filters::Subscriber<Odometry> sub2(nh, "/odom", 1);

    //typedef sync_policies::ApproximateTime<Odometry, Odometry> MySyncPolicy;
    //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub2, sub2);
    //sync.registerCallback(boost::bind(&velCallback, _1, _2));

    ros::spin();
    return 0;
}
