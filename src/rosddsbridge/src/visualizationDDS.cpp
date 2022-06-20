#include <cmath>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>
#include "rosddsbridge/PclWithOdom.h"

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

class VisualizationDDS
{
private:
  ros::NodeHandle nh;

  ros::Subscriber subVelodyneCloud;
  ros::Subscriber subMapCloud;
  ros::Subscriber subOriginVelodyneCloud;
  ros::Subscriber subPclWithOdom;
  ros::Subscriber subPclWithOdom2;
  ros::Subscriber subOdom;
  ros::Publisher pubVelodyneCloud;
  ros::Publisher pubMapCloud;
  ros::Publisher pubRoomVis;
  ros::Publisher pubVelodyneHistory;
  ros::Publisher pubVelodyneWithMapFrame;
  ros::Publisher pubOdom;
  ros::Publisher pubVelodyneWithMapFrame2;
  ros::Publisher pubOdom2;
  ros::Publisher pubVelodyneHistory2;

  const double exploredAreaVoxelSize = 0.3;
  // VLP-16
  const double PI = 3.1415926;
  const bool useCloudRing = false;
  const int N_SCAN = 16;
  const int Horizon_SCAN = 1800;
  const float ang_res_x = 0.2;
  const float ang_res_y = 2.0;
  const float ang_bottom = 15.0 + 0.1;
  const float sensorMinimumRange = 0.4;

  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudTemp;
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudWhole;
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudWholeHistory;
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudTemp2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudWhole2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudWholeHistory2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloudTemp;
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloudWhole;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDDS;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInMapFrame;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudMap;

  pcl::VoxelGrid<pcl::PointXYZI> velodyneHistoryDwzFilter;

  cv::Mat rangeMat;
  vector<vector<double>> rangeVector;

  mutex mt;

  nav_msgs::Odometry odomData;
public:
  VisualizationDDS() : nh("~")
  {
    //receive velodyne with frame_id /map from dds
    subVelodyneCloud = nh.subscribe<sensor_msgs::PointCloud2>("/receive_velodyne_cloud", 5, &VisualizationDDS::velodyneCloudHandler, this);
    //receive map with frame_id /map from dds
    subMapCloud = nh.subscribe<sensor_msgs::PointCloud2>("/receive_map_cloud", 5, &VisualizationDDS::mapCloudHandler, this);
    //reveive whole velodyne pointclouds with frame id  /velodyne from this node
    subOriginVelodyneCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &VisualizationDDS::originVelodyneCloudHandler, this);
    //receive pcl with odom from dds
    subPclWithOdom = nh.subscribe<rosddsbridge::PclWithOdom>("/pclWithOdom", 5, &VisualizationDDS::pclWithOdomHandler, this);
    subPclWithOdom2 = nh.subscribe<rosddsbridge::PclWithOdom>("/pclWithOdom2", 5, &VisualizationDDS::pclWithOdomHandler2, this);
    //publish whole map with map_frame
    pubMapCloud = nh.advertise<sensor_msgs::PointCloud2>("/receive_map_cloud_whole", 5);
    //publish a velodyne with velodyne_frame
    pubVelodyneCloud = nh.advertise<sensor_msgs::PointCloud2>("/receive_velodyne_cloud_whole", 5);
    //publish all velodyne points
    pubVelodyneHistory = nh.advertise<sensor_msgs::PointCloud2>("/receive_velodyne_cloud_history", 5);
    //publish room vis with
    pubRoomVis = nh.advertise<visualization_msgs::Marker>("/roomVisLocal", 1);
    //publish
    pubVelodyneWithMapFrame = nh.advertise<sensor_msgs::PointCloud2>("/receive_velodyne_cloud", 1);
    pubOdom = nh.advertise<nav_msgs::Odometry>("/receive_odom", 10);
    pubVelodyneWithMapFrame2 = nh.advertise<sensor_msgs::PointCloud2>("/receive_velodyne_cloud2", 1);
    pubOdom2 = nh.advertise<nav_msgs::Odometry>("/receive_odom2", 10);
    pubVelodyneHistory2 = nh.advertise<sensor_msgs::PointCloud2>("/receive_velodyne_cloud_history2", 5);
    
    subOdom = nh.subscribe<nav_msgs::Odometry>("/state_estimation",10,&VisualizationDDS::odomHandler,this);

    allocateMemory();
    resetParameters();
  }

  void odomHandler(const nav_msgs::Odometry::ConstPtr &msg)
  {
    odomData = *msg;
  }

  void pclWithOdomHandler2(const rosddsbridge::PclWithOdom::ConstPtr &msg)
  {
    sensor_msgs::PointCloud2 laserCloud = msg->pcl;
    nav_msgs::Odometry odometry = msg->odom;
    odometry.child_frame_id = "sensor2";

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z));
    tf::Quaternion q(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "sensor2"));

    pubOdom2.publish(odometry);
    velodyneCloudTemp2->clear();
    pcl::fromROSMsg(laserCloud, *velodyneCloudTemp2);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*velodyneCloudTemp2, *velodyneCloudTemp2, indices);
    tf::StampedTransform transformToMap;
    transformToMap.setOrigin(tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z));
    transformToMap.setRotation(tf::Quaternion(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w));
    pcl::PointXYZI p1;
    tf::Vector3 vec;
    int laserCloudInNum = velodyneCloudTemp2->points.size();
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInMapFrame2(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < laserCloudInNum; i++)
    {
      p1 = velodyneCloudTemp2->points[i];
      vec.setX(p1.x);
      vec.setY(p1.y);
      vec.setZ(p1.z);

      vec = transformToMap * vec;

      p1.x = vec.x();
      p1.y = vec.y();
      p1.z = vec.z();
      p1.intensity = velodyneCloudTemp2->points[i].intensity;
      laserCloudInMapFrame2->points.push_back(p1);
    }
    sensor_msgs::PointCloud2 scan_data;
    pcl::toROSMsg(*laserCloudInMapFrame2, scan_data);
    scan_data.header.stamp = laserCloud.header.stamp;
    scan_data.header.frame_id = "/map";
    pubVelodyneWithMapFrame2.publish(scan_data);

    *velodyneCloudWhole2 += *laserCloudInMapFrame2;
    *velodyneCloudWholeHistory2 += *laserCloudInMapFrame2;
    unique_lock<mutex> lk(mt);
    velodyneHistoryDwzFilter.setInputCloud(velodyneCloudWholeHistory2);
    velodyneHistoryDwzFilter.filter(*velodyneCloudWholeHistory2);
    lk.unlock();

    if (laserCloudInNum < 1790)
    {
      sensor_msgs::PointCloud2 scan_data2;
      pcl::toROSMsg(*velodyneCloudWhole2, scan_data2);
      scan_data2.header.stamp = laserCloud.header.stamp;
      scan_data2.header.frame_id = "/map";
      pubVelodyneWithMapFrame2.publish(scan_data2);
      velodyneCloudWhole2->clear();
    }

    if (pubVelodyneHistory2.getNumSubscribers() != 0)
    {
      sensor_msgs::PointCloud2 scan_data3;
      pcl::toROSMsg(*velodyneCloudWholeHistory2, scan_data3);
      scan_data3.header.stamp = laserCloud.header.stamp;
      scan_data3.header.frame_id = "/map";
      pubVelodyneWithMapFrame2.publish(scan_data3);
    }
  }

  void pclWithOdomHandler(const rosddsbridge::PclWithOdom::ConstPtr &msg)
  {
    sensor_msgs::PointCloud2 laserCloud = msg->pcl;
    nav_msgs::Odometry odometry = msg->odom;
    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform odomTrans;
    odomTrans.stamp_ = odometry.header.stamp;
    odomTrans.frame_id_ = "/map";
    odomTrans.child_frame_id_ = "/receive_vehicle";
    odomTrans.setRotation(tf::Quaternion(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w));
    odomTrans.setOrigin(tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z));
    tfBroadcaster.sendTransform(odomTrans);
    /*
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z) );
    tf::Quaternion q(odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "sensor"));
    */
    pubOdom.publish(odometry);
    laserCloudDDS->clear();
    laserCloudInMapFrame->clear();
    pcl::fromROSMsg(laserCloud, *laserCloudDDS);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudDDS, *laserCloudDDS, indices);
    tf::StampedTransform transformToMap;
    transformToMap.setOrigin(tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z));
    transformToMap.setRotation(tf::Quaternion(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w));
    pcl::PointXYZI p1;
    tf::Vector3 vec;
    int laserCloudInNum = laserCloudDDS->points.size();
    for (int i = 0; i < laserCloudInNum; i++)
    {
      p1 = laserCloudDDS->points[i];
      vec.setX(p1.x);
      vec.setY(p1.y);
      vec.setZ(p1.z);

      vec = transformToMap * vec;

      p1.x = vec.x();
      p1.y = vec.y();
      p1.z = vec.z();
      p1.intensity = laserCloudDDS->points[i].intensity;
      laserCloudInMapFrame->points.push_back(p1);
    }
    sensor_msgs::PointCloud2 scan_data;
    pcl::toROSMsg(*laserCloudInMapFrame, scan_data);
    scan_data.header.stamp = laserCloud.header.stamp;
    scan_data.header.frame_id = "/map";
    pubVelodyneWithMapFrame.publish(scan_data);
  }

  void originVelodyneCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
    //velodyneCloudIn->clear();
    pcl::fromROSMsg(*laserCloudIn, *velodyneCloudIn);
    measureRoom(velodyneCloudIn);
  }

  void measureRoom(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud)
  {
    // VLP-16
    /*
    const double PI = 3.1415926;
    const bool useCloudRing=false;
    const int N_SCAN = 16;
    const int Horizon_SCAN = 1800;
    const float ang_res_x = 0.2;
    const float ang_res_y = 2.0;
    const float ang_bottom = 15.0+0.1;
    */
    // range image projection
    rangeVector.resize(4, vector<double>(8, -1));
    float minX = 0;
    float minY = 0;
    float maxX = 0;
    float maxY = 0;
    float verticalAngle, horizonAngle, range;
    int rowIdn, columnIdn, index;
    pcl::PointXYZI thisPoint;
    int cloudSize = laserCloud->points.size();

    for (int i = 0; i < cloudSize; ++i)
    {
      thisPoint.x = laserCloud->points[i].x;
      thisPoint.y = laserCloud->points[i].y;
      thisPoint.z = laserCloud->points[i].z;

      // find the row and column index in the iamge for this point
      if (useCloudRing == true)
      {
        //rowIdn = laserCloudInRing->points[i].ring;
      }
      else
      {
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
      }
      if (rowIdn < 0 || rowIdn >= N_SCAN)
      {
        continue;
      }

      //horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
      horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;

      //columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
      columnIdn = (horizonAngle + 180.0 + 0.5) / ang_res_x;
      if (columnIdn >= Horizon_SCAN)
        columnIdn -= Horizon_SCAN;

      if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
        continue;

      range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
      if (range < sensorMinimumRange)
        continue;

      //rangeMat.at<float>(rowIdn, columnIdn) = range;
      /*
      int rangeNum = (50 + 1) / 2;
      if (rowIdn > 7)
      {
        if (abs(columnIdn - 0) < rangeNum)
        {
          rangeVector[0][rowIdn - 8] = range;
          //minX=max(minX,range*cos( ( (rowIdn-8)*2+1)*PI/180));
          minX = max(minX, abs(thisPoint.x));
        }
        else if (abs(columnIdn - 450) < rangeNum)
        {
          //rangeVector[1][rowIdn-8]=range;
          minY = max(minY, abs(thisPoint.y));
        }
        else if (abs(columnIdn - 900) < rangeNum)
        {
          //rangeVector[2][rowIdn-8]=range;
          maxX = max(maxX, abs(thisPoint.x));
        }
        else if (abs(columnIdn - 1350) < rangeNum)
        {
          //rangeVector[3][rowIdn-8]=range;
          maxY = max(maxY, abs(thisPoint.y));
        }
      }
    
    */
    float angle = atan2(thisPoint.y,thisPoint.x)/PI*180;
    float rangeAngle = 1;
    if(angle>0-rangeAngle && angle<0+rangeAngle){
      maxX = max(maxX, abs(thisPoint.x));
    }else if(angle>90-rangeAngle && angle<90+rangeAngle){
      maxY = max(maxY, abs(thisPoint.y));
    }else if(angle>-90-rangeAngle && angle<-90+rangeAngle){
      minY = max(minY, abs(thisPoint.y));
    }else if(angle>180-rangeAngle || angle<-180+rangeAngle){
      minX = max(minX, abs(thisPoint.x));
    }
    }
    if (minX == 0 && minY == 0 && maxX == 0 && maxY == 0)
      return;
    std::cout << minX << ' ' << minY << ' ' << maxX << ' ' << maxY << std::endl;
    visualRoom(-minX, -minY, maxX, maxY, 0, 1);
    //drawRoom(-minX,-minY,maxX,maxY);
  }
  void drawRoom(double minX_, double minY_, double maxX_, double maxY_)
  {
    Mat img(1000, 1000, CV_8UC1);
    Rect r(250, 250, 120, 200);
    rectangle(img, r, Scalar(0, 255, 255), 3);
    imshow("room", img);
    cout << "show picture" << endl;
    //imwrite("test.png", image);
  }
  void visualRoom(double minX_, double minY_, double maxX_, double maxY_, double minZ_, double maxZ_)
  {
    visualization_msgs::Marker box;
    box.header.stamp = ros::Time::now();
    box.header.frame_id = "/vehicle";
    box.id = 1;
    box.ns = "boundary";
    box.type = visualization_msgs::Marker::LINE_LIST;
    box.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point box_p1, box_p2, box_p3, box_p4, box_p5, box_p6, box_p7, box_p8;
    box_p1.x = maxX_;
    box_p1.y = maxY_;
    box_p1.z = maxZ_;
    box_p2.x = minX_;
    box_p2.y = maxY_;
    box_p2.z = maxZ_;
    box_p3.x = minX_;
    box_p3.y = minY_;
    box_p3.z = maxZ_;
    box_p4.x = maxX_;
    box_p4.y = minY_;
    box_p4.z = maxZ_;
    box_p5.x = maxX_;
    box_p5.y = maxY_;
    box_p5.z = minZ_;
    box_p6.x = minX_;
    box_p6.y = maxY_;
    box_p6.z = minZ_;
    box_p7.x = minX_;
    box_p7.y = minY_;
    box_p7.z = minZ_;
    box_p8.x = maxX_;
    box_p8.y = minY_;
    box_p8.z = minZ_;

    box.scale.x = 0.1; //line width
    box.color.r = 0.0;
    box.color.g = 0.0;
    box.color.b = 255.0 / 255.0;

    box.color.a = 1;
    box.points.push_back(box_p1);
    box.points.push_back(box_p2);
    box.points.push_back(box_p2);
    box.points.push_back(box_p3);
    box.points.push_back(box_p3);
    box.points.push_back(box_p4);
    box.points.push_back(box_p4);
    box.points.push_back(box_p1);
    box.points.push_back(box_p1);
    box.points.push_back(box_p5);
    box.points.push_back(box_p2);
    box.points.push_back(box_p6);
    box.points.push_back(box_p3);
    box.points.push_back(box_p7);
    box.points.push_back(box_p4);
    box.points.push_back(box_p8);
    box.points.push_back(box_p5);
    box.points.push_back(box_p6);
    box.points.push_back(box_p6);
    box.points.push_back(box_p7);
    box.points.push_back(box_p7);
    box.points.push_back(box_p8);
    box.points.push_back(box_p8);
    box.points.push_back(box_p5);
    tf::Quaternion quat2;
    quat2.setEuler(0.0, 0.0, 0.0);
    box.pose.orientation.x = quat2.x();
    box.pose.orientation.y = quat2.y();
    box.pose.orientation.z = quat2.z();
    box.pose.orientation.w = quat2.w();
    box.lifetime = ros::Duration(0.0);
    box.frame_locked = false;
    pubRoomVis.publish(box);

    visualization_msgs::Marker textX;
    textX.header.frame_id = "/vehicle";
    textX.header.stamp = ros::Time::now();
    textX.ns = "textX";
    textX.action = visualization_msgs::Marker::ADD;
    textX.pose.orientation.w = 1.0;
    textX.id = 0;
    textX.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textX.scale.z = 1.0;
    textX.color.b = 0;
    textX.color.g = 0;
    textX.color.r = 1;
    textX.color.a = 1;
    textX.pose.position.x = (maxX_ + minX_) / 2;
    textX.pose.position.y = maxY_ - 1;
    textX.pose.position.z = maxZ_;
    textX.text = "SIZE_X: " + to_string(abs(maxX_) + abs(minX_)) + "=" + to_string(abs(minX_)) + "+" + to_string(abs(maxX_));
    pubRoomVis.publish(textX);

    visualization_msgs::Marker textY;
    textY.header.frame_id = "/vehicle";
    textY.header.stamp = ros::Time::now();
    textY.ns = "textY";
    textY.action = visualization_msgs::Marker::ADD;
    textY.pose.orientation.w = 1.0;
    textY.id = 0;
    textY.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textY.scale.z = 1;
    textY.color.b = 0;
    textY.color.g = 0;
    textY.color.r = 1;
    textY.color.a = 1;
    textY.pose.position.x = maxX_ - 2;
    textY.pose.position.y = (maxY_ + minY_) / 2;
    textY.pose.position.z = maxZ_;
    textY.text = "SIZE_Y: " + to_string(abs(maxY_) + abs(minY_)) + "=" + to_string(abs(minY_)) + "+" + to_string(abs(maxY_));
    pubRoomVis.publish(textY);
  }

  void velodyneCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn)
  {
    velodyneCloudTemp->clear();
    pcl::fromROSMsg(*laserCloudIn, *velodyneCloudTemp);
    *velodyneCloudWhole += *velodyneCloudTemp;
    *velodyneCloudWholeHistory += *velodyneCloudTemp;
    if (velodyneCloudTemp->points.size() < 1799)
    {
      sensor_msgs::PointCloud2 velodynePoints;
      pcl::toROSMsg(*velodyneCloudWhole, velodynePoints);
      velodynePoints.header = laserCloudIn->header;
      pubVelodyneCloud.publish(velodynePoints);
      //measureRoom(velodyneCloudWhole);
      velodyneCloudWhole->clear();
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudWholeHistoryTemp(new pcl::PointCloud<pcl::PointXYZI>());
    unique_lock<mutex> ul(mt);
    velodyneHistoryDwzFilter.setInputCloud(velodyneCloudWholeHistory);
    velodyneHistoryDwzFilter.filter(*velodyneCloudWholeHistoryTemp);
    ul.unlock();
    velodyneCloudWholeHistory = velodyneCloudWholeHistoryTemp;
    sensor_msgs::PointCloud2 velodynePointsHistory;
    pcl::toROSMsg(*velodyneCloudWhole, velodynePointsHistory);
    velodynePointsHistory.header = laserCloudIn->header;
    pubVelodyneHistory.publish(velodynePointsHistory);
  }

  void mapCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn)
  {
    mapCloudTemp->clear();
    pcl::fromROSMsg(*laserCloudIn, *mapCloudTemp);
    *mapCloudWhole += *mapCloudTemp;

    if (mapCloudTemp->points.size() < 1790)
    {
      sensor_msgs::PointCloud2 mapPoints;
      pcl::toROSMsg(*mapCloudWhole, mapPoints);
      mapPoints.header = laserCloudIn->header;
      pubMapCloud.publish(mapPoints);
      mapCloudWhole->clear();
    }
  }

  void allocateMemory()
  {
    velodyneCloudTemp.reset(new pcl::PointCloud<pcl::PointXYZI>());
    velodyneCloudWhole.reset(new pcl::PointCloud<pcl::PointXYZI>());
    mapCloudTemp.reset(new pcl::PointCloud<pcl::PointXYZI>());
    mapCloudWhole.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudInMapFrame.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
    velodyneCloudWholeHistory.reset(new pcl::PointCloud<pcl::PointXYZI>());
    velodyneCloudTemp2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    velodyneCloudWhole2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    velodyneCloudWholeHistory2.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  void resetParameters()
  {
    velodyneHistoryDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    rangeVector.resize(4, vector<double>(8, -1));
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualizationDDS");

  auto VD = new VisualizationDDS();

  ROS_INFO("\033[1;32m---->\033[0m VisualDDS Started.");

  ros::spin();

  return 0;
}
