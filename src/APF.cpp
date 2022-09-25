#include <artificial_potential_fields/APF.h>

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    speed = sqrt(pow(odometry_msg->twist.twist.linear.x, 2) + pow(odometry_msg->twist.twist.linear.y, 2));
}

void attractiveVelocityCallback(const geometry_msgs::Twist& command_msg){
    velocity_d << command_msg.linear.x, command_msg.linear.y, command_msg.linear.z, command_msg.angular.z;
}

double distance(pcl::PointXYZ p){
    return sqrt(pow(p.x, 2) + pow(p.y, 2) );
}

void filterSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    //ROS_INFO_STREAM("[APF] cloud->size() = " << cloud->size());
    for(int i = 0; i < cloud->size(); ++i)
        if(distance(cloud->at(i)) <= UAV_radius)
            inliers->indices.push_back(i);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
    //ROS_INFO_STREAM("[APF] inliers->indices.size() = " << inliers->indices.size());
    //ROS_INFO_STREAM("[APF] cloud->size() = " << cloud->size());
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 msg_cloud;
    projector.projectLaser(*scan, msg_cloud,0.4);
    pcl::fromROSMsg(msg_cloud, *cloud);
    filterSphere(cloud); // filters out points inside the bounding sphere
    cloud->header.frame_id = "base_footprint";

    // Filter the point cloud(abandon)

}
void dynamicReconfigureCallback(artificial_potential_fields::setAPFConfig &config, uint32_t level){
    k_repulsive = config.k_repulsive;
    eta_0 = config.eta_0;
}

// Constructor
APF::APF(int argc, char** argv){
    ros::init(argc, argv, "APF");
    ros::NodeHandle node_handle;

    // Subscribers
    odometry_subscriber = node_handle.subscribe("/odom", 1, odometryCallback);
    attractive_velocity_subscriber = node_handle.subscribe("/attractive_velocity", 1, attractiveVelocityCallback);
    laser_subscriber = node_handle.subscribe("/scan", 1, laserCallback);
    // Publishers
    force_publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);//TwistStamped
    attractive_publisher = node_handle.advertise<geometry_msgs::Vector3>("/potential_fields/attractive", 1); // for debug
    repulsive_publisher = node_handle.advertise<geometry_msgs::Vector3>("/potential_fields/repulsive", 1); // for debug
    point_cloud_publisher = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/potential_fieldspoint_cloud", 1); // for debug

    ROS_INFO_STREAM("[APF] UAV_radius = " << UAV_radius);

    transformation = MatrixXf::Identity(4, 4);
}

// Destructor
APF::~APF(){
    ros::shutdown();
    exit(0);
}

void APF::run(){
    dynamic_reconfigure::Server<artificial_potential_fields::setAPFConfig> server;
    dynamic_reconfigure::Server<artificial_potential_fields::setAPFConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    Vector3f force;
    Vector3f repulsive_force;

    ros::Rate rate(30);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        // Compute the attractive forces
        repulsive_force << 0, 0, 0;
        int points = 0;
        for(int i = 0; i < cloud->size(); ++i){
            Vector3f obstacle(cloud->points[i].x , cloud->points[i].y ,0); // point obstacle
            float eta = distance(obstacle);
            if(eta < eta_0 && eta > UAV_radius){ // does not consider points further than eta_0 AND closer than UAV_radius (inside the bounding sphere)
                obstacle -= obstacle/eta*UAV_radius; // move the UAV's center to the point on the sfere surrounding UAV and closest to the obstacle
                //repulsive_force += pow(1/eta - 1/eta_0, 2)/2*obstacle/eta;
                repulsive_force += (1/eta - 1/eta_0)/pow(eta, 2)*obstacle;
                ++points;
            }
        }
        if(points){ // normalise repulsive force
            repulsive_force /= points;
        }

        repulsive_force *= ((velocity_d[0]+0.0115)/7.5);//TODO
        ROS_INFO_STREAM("[APF] repulsive_force = " << repulsive_force);
        force = velocity_d.head(3) - repulsive_force;
        //TODO
        if(abs(force(0))<0.04)
        {force(1)*=2;}
        geometry_msgs::Twist force_msg;
        force_msg.linear.x = force(0);
        force_msg.linear.y = force(1);
        force_msg.angular.z = velocity_d(3);
        force_publisher.publish(force_msg);

        geometry_msgs::Vector3 debug_msg;
        debug_msg.x = velocity_d(0);
        debug_msg.y = velocity_d(1);
        debug_msg.z = velocity_d(2);
        attractive_publisher.publish(debug_msg);
        debug_msg.x = -repulsive_force(0);
        debug_msg.y = -repulsive_force(1);
        debug_msg.z = 0;
        repulsive_publisher.publish(debug_msg);

        point_cloud_publisher.publish(cloud);
    }
}

double APF::distance(Vector3f v){
    return sqrt(pow(v(0), 2) + pow(v(1), 2));
}

int main(int argc, char** argv){
    cout << "[APF] Artificial potential fields running..." << endl;

    APF* apf = new APF(argc, argv);
    apf->run();
}
