// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <memory>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "laserMappingClass.h"
#include "lidar.h"
std::string subOdomTopic = "receive_odom";
std::string subPclTopic = "receive_velodyne_cloud";
std::string pubPclTopic = "receive_cloudMap";

LaserMappingClass laserMapping;
lidar::Lidar lidar_param;
//std::mutex mutex_lock;
//std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
//std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

ros::Publisher map_pub;

void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*laserCloud2, *pointcloud_in);
    ros::Time pointcloud_time = laserCloud2->header.stamp;

    Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
    current_pose.rotate(Eigen::Quaterniond(odometry->pose.pose.orientation.w,odometry->pose.pose.orientation.x,odometry->pose.pose.orientation.y,odometry->pose.pose.orientation.z));  
    current_pose.pretranslate(Eigen::Vector3d(odometry->pose.pose.position.x,odometry->pose.pose.position.y,odometry->pose.pose.position.z));
            
    laserMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping.getMap();
    sensor_msgs::PointCloud2 PointsMsg;
    pcl::toROSMsg(*pc_map, PointsMsg);
    PointsMsg.header.stamp = pointcloud_time;
    PointsMsg.header.frame_id = "map";
    map_pub.publish(PointsMsg); 
}
/*
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();
}

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}


void laser_mapping(){
    while(1){
        if(!odometryBuf.empty() && !pointCloudBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
                ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
                odometryBuf.pop();
                ROS_INFO("time stamp unaligned with path final, pls check your data --> laser mapping node");
                mutex_lock.unlock();
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();
            

            laserMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping.getMap();
            sensor_msgs::PointCloud2 PointsMsg;
            pcl::toROSMsg(*pc_map, PointsMsg);
            PointsMsg.header.stamp = pointcloud_time;
            PointsMsg.header.frame_id = "map";
            map_pub.publish(PointsMsg); 
            


        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMappingNodeDds");
    ros::NodeHandle nh;

    int scan_line = 16;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 90.0;
    double min_dis = 0.3;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/pubPclTopic", pubPclTopic);
    nh.getParam("/subOdomTopic", subOdomTopic);
    nh.getParam("/subPclTopic", subPclTopic);


    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserMapping.init(map_resolution);
    //ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, velodyneHandler);
    //ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>(pubPclTopic, 100);
    //std::thread laser_mapping_process{laser_mapping};

    message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
    typedef message_filters::Synchronizer<syncPolicy> Sync; 
    std::shared_ptr<Sync> sync_;
    subOdometry.subscribe(nh, subOdomTopic, 100);
    subLaserCloud.subscribe(nh, subPclTopic, 100);
    sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
    sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

    ros::spin();

    return 0;
}
