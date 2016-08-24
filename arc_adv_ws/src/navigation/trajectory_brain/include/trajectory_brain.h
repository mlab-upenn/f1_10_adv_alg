#define TRUE 1
#define FALSE 0
#define SAMPLES (8)
#define LOOP_COUNT (SAMPLES*2)
union State
{
    struct 
    {
        double sx;
        double sy;
        double theta; 
        double kappa;
        double v;
        double vdes;
        double timestamp;
    };

    double state_value[7];
};

union Parameters 
{
    struct 
    {
        double a;
        double b;
        double c;
        double e;
        double s;
        bool success;
    };

    double param_value[6];


};

union Spline
{
    struct
    {
        double s;
        double kappa_1;
        double kappa_2;   
        double kappa_0;
        double kappa_3;
        bool success;
        
    };

    double spline_value[6];
};

union StateLattice
{
    struct
    {
          double x[LOOP_COUNT];
          double y[LOOP_COUNT];
    };

    double lattice_param[2];
};

union Parameters rbfTrajectory(double sx, double sy, double theta);

void poseCallback(const nav_msgs::Odometry& pose_msg);

void trajCostCallback( const trajectory_cost::TrajectoryID& cost_msg);

trajectory_brain::TrajectoryVector trajFwdSim( union Parameters traj, union State veh, unsigned int trajID);

StateLattice computeStateLattice( union State veh);

ros::Publisher spline_parameters_pub;

bool initialPass = true;
