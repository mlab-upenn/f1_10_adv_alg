#include "trajectory_visualization.h"

// Loop rate for node in Hz.
static const int LOOP_RATE = 20;
static const float plot_step_size = 0.2;

// Callback function to get control parameters 
void splineCallback(const std_msgs::Float64MultiArray::ConstPtr& spline) {
    for (int i = 0; i < 6; i++) {
        ROS_INFO("%d", spline->data[i]);
    }
  //drawSpline(curvature, veh, 0,0);
  //SPLINE_INDEX++;
  //ROS_INFO_STREAM("Spline published to RVIZ");
}

/////////////////////////////////////////////////////////////////
// Draw Spline
/////////////////////////////////////////////////////////////////
static void drawSpline(union Spline curvature, union State veh, int flag, int selected) {
    static double vdes=veh.vdes;
    // Setup up line strips
    visualization_msgs:: Marker line_strip;
    line_strip.header.frame_id = "/base_link";
    line_strip.header.stamp = ros::Time();
    line_strip.action = visualization_msgs::Marker::ADD;
  
    // Define message id and scale (thickness)
    line_strip.id = flag;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    
    // Set the color and transparency (blue and solid) if not selected
    if (selected > 0) {
        line_strip.scale.x = 0.08;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;
    }
  
    // Set the color and transparency (green and solid) if selected  
    else {
        line_strip.scale.x = 0.1;
        line_strip.color.g = 1.0;
        line_strip.color.a = 1.0;
    }
  
    // Init temp state for storing results of genLineStrip
    union State temp;
  
    // Init time
    double sim_time = 0.0;
  
    // Figure out sim time horizon
    double horizon = curvature.s/veh.vdes;
  
    // Init points
    geometry_msgs::Point p;
  
    // Create veritices
    while(sim_time < horizon && curvature.success == TRUE) {
        temp = genLineStrip(veh, curvature, vdes, sim_time);
        p.x = temp.sx;
        p.y = temp.sy;
        p.z = 0.0;
        line_strip.points.push_back(p);
        veh = temp;
        sim_time = sim_time + plot_step_size;
    }
  
    // Publish trajectory line strip (to RViz)
    g_marker_pub.publish(line_strip);
}

int main(int argc, char **argv) {
    // Set up ROS, TO DO: change to proper name (same with rest of the file)
    ros::init(argc, argv, "trajectory_visualization");

    // Create node handles 
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Publish the following topics: 
    ros::Subscriber spline_sub = nh.subscribe("/spline",1, splineCallback);
    ros::Publisher g_marker_pub = nh.advertise<visualization_msgs::Marker>("/trajectory/spline", 1);

    // Add subscription to splines topic ala NK.

    // Set the loop rate unit is Hz
    ros::Rate loop_rate(LOOP_RATE); 
    while (ros::ok()) {
       ros::spinOnce();
       loop_rate.sleep();
    }
    return 0;
}
