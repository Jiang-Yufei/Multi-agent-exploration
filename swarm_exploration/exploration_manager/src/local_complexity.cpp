#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <exploration_manager/fast_exploration_fsm.h>
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <time.h>



using namespace Eigen;
namespace fast_planner {
void FastExplorationFSM::droneLocalComplexityCalculate(const sensor_msgs::PointCloud2::ConstPtr& msg){
    
    // ROS_INFO("Local Map received!");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    //ROS_INFO("Cloud size: %d", cloud.size());
    if (cloud.size() == 0) {
        ROS_INFO("Empty cloud!");
        return;
    }
    // calculate time cost
    ros::Time t1 = ros::Time::now();
    auto t2 = t1;
    
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_ptr = cloud;
    normal_estimator.setInputCloud(cloud_ptr);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);
    
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*cloud_ptr, *indices);
    
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (10000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud_ptr);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    
    // std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    // std::cout << "These are the indices of the points of the initial" << 
    // std::endl << "cloud that belong to the first cluster:" << std::endl;

    // Visualizate clusters
    pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
    for (int i = 0; i < clusters.size(); i++) {
        // centers of clusters
        pcl::PointXYZ center;
        center.x = 0;
        center.y = 0;
        center.z = 0;
        for (int j = 0; j < clusters[i].indices.size(); j++) {
            center.x += cloud[clusters[i].indices[j]].x;
            center.y += cloud[clusters[i].indices[j]].y;
            center.z += cloud[clusters[i].indices[j]].z;
        }
        center.x /= clusters[i].indices.size();
        center.y /= clusters[i].indices.size();
        center.z /= clusters[i].indices.size();
        cluster_cloud.push_back(center);
    
    }


    sensor_msgs::PointCloud2 cluster_cloud_msg;
    pcl::toROSMsg(cluster_cloud, cluster_cloud_msg);
    cluster_cloud_msg.header.frame_id = "world";
    cluster_cloud_msg.header.stamp = ros::Time::now();
    cluster_cloud_pub.publish(cluster_cloud_msg);
    //ROS_INFO("Cluster cloud center published!");
    t2 = ros::Time::now();
    //ROS_INFO("Time cost: %f", (t2 - t1).toSec());

    double complexity = cal_local_complexity(clusters.size(), 5);
    exploration_manager::DroneLocalComplexity complexity_msg;
    complexity_msg.drone_id = getId();
    complexity_msg.local_complexity = complexity;
    drone_local_complexity_pub_.publish(complexity_msg);
    //ROS_INFO("drone_complexity=%f", complexity);
}

double FastExplorationFSM::cal_local_complexity(int cluster_num, int a){
    double complexity = 1 - a * std::exp(-cluster_num);
    //ROS_INFO("Local complexity: %f", complexity);

    return complexity;
}

}