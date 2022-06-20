#include <cmath>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class VisualizationDDS
{
private:
  ros::NodeHandle nh;

  ros::Subscriber subVelodyneCloud;
  ros::Publisher pubVelodynePoint;
  ros::Publisher pubRoomVis;

  const double PI = 3.1415926;

public:
  VisualizationDDS() : nh("~")
  {
    subVelodyneCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 5, &VisualizationDDS::velodyneCloudHandler, this);
    pubVelodynePoint = nh.advertise<sensor_msgs::PointCloud2>("/velodynePoints", 5);
    pubRoomVis = nh.advertise<visualization_msgs::Marker>("/roomVisLocal", 1);

    allocateMemory();
    resetParameters();
  }

 void velodyneCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyneCloudTemp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*laserCloudIn, *velodyneCloudTemp);
    measureRoom(velodyneCloudTemp);
  }

  void measureRoom(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud)
  {
    float minX = 0;
    float minY = 0;
    float maxX = 0;
    float maxY = 0;
    float verticalAngle, horizonAngle, range;
    int rowIdn, columnIdn, index;
    cv::Mat rangeMat = cv::Mat(16, 1800, CV_32FC1, cv::Scalar::all(-1.0));
    pcl::PointXYZI thisPoint;

    int cloudSize = laserCloud->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
      thisPoint.x = laserCloud->points[i].x;
      thisPoint.y = laserCloud->points[i].y;
      thisPoint.z = laserCloud->points[i].z;

      verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
      rowIdn = (verticalAngle + 15.0 + 0.1) / 2.0;
      if (rowIdn < 0 || rowIdn >= 16)
        continue;

      horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;
      columnIdn = (horizonAngle + 180.0 - 0.1) / 0.2;
      if (columnIdn >= 1800)
        columnIdn -= 1800;
      if (columnIdn < 0 || columnIdn >= 1800)
        continue;

      range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
      if (range < 0.3)
        continue;
      rangeMat.at<float>(rowIdn, columnIdn) = range;

      //measure room
      float angle = atan2(thisPoint.y, thisPoint.x) / PI * 180;
      float rangeAngle = 1;
      if (angle > 0 - rangeAngle && angle < 0 + rangeAngle)
      {
        maxX = max(maxX, abs(thisPoint.x));
      }
      else if (angle > 90 - rangeAngle && angle < 90 + rangeAngle)
      {
        maxY = max(maxY, abs(thisPoint.y));
      }
      else if (angle > -90 - rangeAngle && angle < -90 + rangeAngle)
      {
        minY = max(minY, abs(thisPoint.y));
      }
      else if (angle > 180 - rangeAngle || angle < -180 + rangeAngle)
      {
        minX = max(minX, abs(thisPoint.x));
      }
    }
    if (minX == 0 && minY == 0 && maxX == 0 && maxY == 0)
      return;
    std::cout << minX << ' ' << minY << ' ' << maxX << ' ' << maxY << std::endl;
    visualRoom(-minX, -minY, maxX, maxY, 0, 1);

    mat2pcl(rangeMat);
  }

void mat2pcl(cv::Mat &img)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr velodyneCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ point;

    for (int i = 0; i < 16; ++i)
    {
      for (int j = 0; j < 1800; ++j)
      {
        float range = img.at<float>(i, j);
        if (range <= 0)
          continue;
        double angleZ = (i * 2.0 - 15.0) * PI / 180.0;
        double angleXY = (j * 0.2 - 180.0 + 0.2) * PI / 180.0;
        point.z = range * sin(angleZ);
        point.x = range * cos(angleZ) * cos(angleXY);
        point.y = range * cos(angleZ) * sin(angleXY);
        velodyneCloud->points.emplace_back(point);
      }
    }

    sensor_msgs::PointCloud2 velodynePoints;
    pcl::toROSMsg(*velodyneCloud, velodynePoints);
    velodynePoints.header.frame_id = "velodyne";
    pubVelodynePoint.publish(velodynePoints);
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

  void allocateMemory()
  {
  }

  void resetParameters()
  {
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl2mat");

  auto VD = new VisualizationDDS();

  ROS_INFO("\033[1;32m---->\033[0m VisualDDS Started.");

  ros::spin();

  return 0;
}
