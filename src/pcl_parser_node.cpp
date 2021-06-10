#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

//tf
#include "tf/transform_datatypes.h"
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ros
#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

class Parser
{
    public:
        Parser(ros::NodeHandle& nh);        
        ~Parser();
        void callbackDepthCloud(const sensor_msgs::PointCloud2ConstPtr& depthCloudMsgPtr);
        ros::NodeHandle nh_;
        
    private:
        ros::Subscriber subDepthCloud;
        // ros::Subscriber subLocalPath;

        ros::Publisher pubBorderCloud; 
        
};

Parser::Parser(ros::NodeHandle& nh) : nh_(nh)
{
    subDepthCloud = nh_.subscribe("/camera/depth/color/points", 1, &Parser::callbackDepthCloud, this);
    pubBorderCloud = nh_.advertise<sensor_msgs::PointCloud2>("/border_cloud", 1, true);
   
};

Parser::~Parser() 
{    
    ROS_INFO("Parser destructor.");
}

void Parser::callbackDepthCloud(const sensor_msgs::PointCloud2ConstPtr& depthCloudMsgPtr)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*depthCloudMsgPtr, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*depth_cloud);


    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // transforming depth cloud to the map link    
    Eigen::AngleAxis<float> y_axis_rotation(M_PI / 2, Eigen::Vector3f(0,1,0));
    Eigen::AngleAxis<float> x_axis_rotation(-M_PI / 2, Eigen::Vector3f(1,0,0));

    Eigen::Matrix4f rotation;
    rotation.block<3,3>(0,0) = (x_axis_rotation * y_axis_rotation).matrix();

    pcl::transformPointCloud(*depth_cloud, *depth_cloud, rotation);
    
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(depth_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(-0.5, 0.05);
    // pass.setFilterLimitsNegative (true);
    pass.filter(*depth_cloud);
    
    // final_cloud = depth_cloud;
    // publishing border points
    sensor_msgs::PointCloud2 borderCloudMsg;
    pcl::toROSMsg(*depth_cloud, borderCloudMsg);
    
    borderCloudMsg.header.frame_id = "map";
    borderCloudMsg.header.stamp = depthCloudMsgPtr->header.stamp;
    pubBorderCloud.publish(borderCloudMsg);

    std::cout << "recived depth cloud" << std::endl;

}

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "pcl_parser_node");
    // for subscribe
    ros::NodeHandle nh;
    Parser planner(nh);

    ros::spin();
    return 0;

}
