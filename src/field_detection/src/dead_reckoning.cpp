#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <pcl/memory.h> // for pcl::make_shared
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

#include "core_msgs/robot_position.h"

using namespace std;
using namespace cv;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

int spincounter = 0;
bool isenabled = false;

// Global variable
sensor_msgs::PointCloud2 msg_cloud;

PointCloud::Ptr result(new PointCloud), target, accumulated_result(new PointCloud);
Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
ros::Publisher pub;

random_device rd;
mt19937 mersenne(rd());
uniform_int_distribution<> die(1, 10);

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
    MyPointRepresentation()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray(const PointNormalT &p, float *out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	/*
    if (spincounter++ % 3 != 0) {
	    return;
    }
    */
    // angle in radian
    float angle_min = scan->angle_min;
    float angle_max = scan->angle_max;
    float angle_increment = scan->angle_increment;
    vector<float> range = scan->ranges;

    // 1. LaserScan msg to PCL::PointXYZ

    // initializae pointcloud
    PointCloud::Ptr cloud(new::PointCloud);

    // Create the extract object for removal of infinite ditance points
    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr inf_points(new pcl::PointIndices());

    // Fill the pointcloud
    int len = range.size(); // size of range vector
    float angle_temp;
    cloud->is_dense = false;
    cloud->width = len;
    cloud->height = 1;
    cloud->points.resize(len);
    for(int i = 0; i < len; i++){
        angle_temp = angle_min + i*angle_increment;
        if (std::isinf(range[i])==false){
            cloud->points[i].x = range[i]*cos(angle_temp);
            cloud->points[i].y = range[i]*sin(angle_temp);
            cloud->points[i].z = 0;
        }
        else{
            // indices of infinite distance points
            inf_points->indices.push_back(i);
        }
    }

    // Remove infinite distance points from cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inf_points);
    extract.setNegative(true);
    extract.filter(*cloud);

    pcl::transformPointCloud(*cloud, *cloud, GlobalTransform.inverse());

    core_msgs::robot_position msg;
    if (!::target) {
        ::target = cloud;
    }
    else {
        PointCloud::Ptr temp(new PointCloud);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(::target);
        icp.setInputTarget(cloud);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        
        pairTransform = icp.getFinalTransformation();

        pcl::transformPointCloud(*cloud, *temp, pairTransform.inverse());
        GlobalTransform = GlobalTransform * pairTransform;

        *::target += *temp;

        // Coonvert PCL type to sensor_msgs/PointCloud2 type
        pcl::toROSMsg(*::target, msg_cloud);

        // Extract 10% of total points to prevent too many points
        pcl::PointIndices::Ptr random_points(new pcl::PointIndices());
        len = ::target->size(); /* number of target */
        for(int i = 0; i < len; i++){
            if (die(mersenne) <= 1) {
                random_points->indices.push_back(i);
            }
        }
        extract.setInputCloud(::target);
        extract.setIndices(random_points);
        extract.setNegative(true);
        extract.filter(*::target);
    }

    cloud.reset(new PointCloud);
    std::cout << "Transformation mtx" << std::endl;

    Eigen::Matrix4f invglobal = GlobalTransform.inverse();
    cout << invglobal << endl;

    float robotangle;
    if (invglobal(0, 0) == 0.) {
        robotangle = (invglobal(1, 0) > 0)?(3.1415926/2):(-3.1415926/2);
    } else if (invglobal(0, 0) < 0.) {
        if (invglobal(1, 0) > 0.) {
            robotangle = atan(invglobal(1, 0)/invglobal(0, 0)) + 3.1415926;
        } else if (invglobal(1, 0) < 0.) {
            robotangle = atan(invglobal(1, 0)/invglobal(0, 0)) - 3.1415926;
        } else {
            robotangle = 3.1415926;
        }
    } else {
        robotangle = atan(invglobal(1, 0)/invglobal(0, 0));
    }


    if (isenabled) {
        msg.data = {
            invglobal(0, 0), invglobal(0, 1), invglobal(0, 2), invglobal(0, 3),
            invglobal(1, 0), invglobal(1, 1), invglobal(1, 2), invglobal(1, 3),
            invglobal(2, 0), invglobal(2, 1), invglobal(2, 2), invglobal(2, 3),
            invglobal(3, 0), invglobal(3, 1), invglobal(3, 2), invglobal(3, 3)
        };
        pub.publish(msg);
    }
    
    std::cout << "Robot pose (x, y, theta) : " << invglobal(0, 3) << ", " << invglobal(1, 3) << ", " << robotangle << std::endl;
}

int main (int argc, char **argv) {
    ros::init (argc, argv, "dead_reckoning_node");
    ros::NodeHandle nh;
    pub = nh.advertise<core_msgs::robot_position>("/robot_position",1);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, lidar_Callback);
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_cloud", 1);
    
    ros::Rate loop_rate(5);

    while(ros::ok()){
        ros::spinOnce();

        bool entrance_finished;
        if (nh.getParam("/entrance_finished", entrance_finished) && !isenabled) {
            if (entrance_finished) {
                isenabled = true;
                GlobalTransform = Eigen::Matrix4f::Identity();
                (::target).reset(new PointCloud);
                pcl::toROSMsg(*::target, msg_cloud);
                std::cout << "Initialized current position as origin!!" << std::endl;
            }
        }

        msg_cloud.header.frame_id = "base_scan"; 
        msg_cloud.header.stamp = ros::Time::now();
        pub_cloud.publish(msg_cloud);

        loop_rate.sleep();
    }
    return 0;
}
