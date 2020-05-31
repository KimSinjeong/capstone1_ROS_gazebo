#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

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

using namespace std;
using namespace cv;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

int spincounter = 0;

// Global variable
sensor_msgs::PointCloud2 msg_cloud;

PointCloud::Ptr result(new PointCloud), target, accumulated_result(new PointCloud);
Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

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

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize(0.05, 0.05, 0.05);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues(alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon(1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance(2.0);
    // Set the point representation
    reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);
    for (int i = 0; i < 50; ++i)
    {
        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

        prev = reg.getLastIncrementalTransformation();
    }

    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    //
    // Transform target back in source frame
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

    //add the source to the transformed target
    // *output += *cloud_src;

    final_transform = targetToSource;
}

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (spincounter++ % 5 != 0) {
        return;
    }
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

    if (!::target) {
        ::target = cloud;
    }
    else {
        PointCloud::Ptr temp(new PointCloud);
        pairAlign(::target, cloud, temp, GlobalTransform, true);
        //GlobalTransform *= pairTransform;

        *target += *temp;

        // Coonvert PCL type to sensor_msgs/PointCloud2 type
        pcl::toROSMsg(*target, msg_cloud);
    }

    cloud.reset(new PointCloud);
    cout << GlobalTransform << endl;
}

int main (int argc, char **argv) {
    ros::init (argc, argv, "obstacle_detect_node");
    ros::NodeHandle nh;
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_cloud", 1);

    ros::Rate loop_rate(5);

    while(ros::ok()){
        ros::spinOnce();

        msg_cloud.header.frame_id = "base_scan"; 
        msg_cloud.header.stamp = ros::Time::now();
        pub_cloud.publish(msg_cloud);

        loop_rate.sleep();
    }
    return 0;
}
