//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h> 
#include <std_msgs/Float32.h>
//C++ lib
#include <string>
#include <boost/thread.hpp> 
#include <cmath>
//dds lib
#include <fastrtps/Domain.h>
#include <fastrtps/log/Log.h>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
//dds msg
#include "PclMsgPubSubTypes.h"
#include "OctomapMsgPubSubTypes.h"
#include "TwistMsgPubSubTypes.h"
#include "TfMsgPubSubTypes.h"
#include "ImageMsgPubSubTypes.h"
#include "TfMsgBasePubSubTypes.h"
#include "MarkerMsgPubSubTypes.h"
#include "OdometryMsgPubSubTypes.h"
#include "MeasureMsgPubSubTypes.h"
#include "OctomapMsg.h"
#include "TwistMsg.h"
#include "TfMsg.h"
#include "ImageMsg.h"
#include "PclMsg.h"
#include "TfMsgBase.h"
#include "MarkerMsg.h"
#include "OdometryMsg.h"
#include "MeasureMsg.h"
//user lib
#include "DDSPublisher.h"
#include "optionparser.h"
//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>  //点类型相关定义
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace std;

#define MAXPCLSIZE 7000000
#define MAXIMAGESIZE 1000000
#define LOG 1
#define UNIT 1800

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

const double PI = 3.1415926;
const bool useCloudRing=false;
const int N_SCAN = 16;
const int Horizon_SCAN = 1800;
const float ang_res_x = 0.2;
const float ang_res_y = 2.0;
const float ang_bottom = 15.0+0.1;
const float sensorMinimumRange = 0.4;

std::string wan_ip ="127.0.0.1";
int port =56452;
std::string ros_sub_PointCloud2_topic = "/velodyne_points";
std::string ros_sub_Dis_topic = "/dis";
std::string dds_pub_Measure_topic=   "MeasureTopic";

const float minZ = 1;
float maxZ = 0;

int pcl_count=0;
void subpclCallback(const sensor_msgs::PointCloud2::ConstPtr& pclin, DDSPublisher* dds_pub)
{
  if(pcl_count==0){
    pcl_count=10;
  }else{
    --pcl_count;
    std::cout<<pcl_count<<std::endl;
    return;
  }
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud (new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*pclin, *laserCloud);
  // range image projection
  float minX=0;
  float minY=0;
  float maxX=0;
  float maxY=0;
  float verticalAngle, horizonAngle, range;
  int rowIdn, columnIdn, index; 
  pcl::PointXYZI thisPoint;
  int  cloudSize = laserCloud->points.size();

  for (int i = 0; i < cloudSize; ++i){
    thisPoint.x = laserCloud->points[i].x;
    thisPoint.y = laserCloud->points[i].y;
    thisPoint.z = laserCloud->points[i].z;

    // find the row and column index in the iamge for this point
    if (useCloudRing == true){
    //rowIdn = laserCloudInRing->points[i].ring;
    }else{
      verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
      rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
    }
    if (rowIdn < 0 || rowIdn >= N_SCAN){
      continue;
    }

    //horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
    horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;

    //columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
    columnIdn = (horizonAngle + 180.0 + 0.5)/ang_res_x;
    if (columnIdn >= Horizon_SCAN)
      columnIdn -= Horizon_SCAN;

    if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
      continue;

    range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
    if (range < sensorMinimumRange)
      continue;
            
    //rangeMat.at<float>(rowIdn, columnIdn) = range;
    int rangeNum=(50+1)/2;
    if(rowIdn>7){
      if(abs(columnIdn-0)<rangeNum){
        minX=max(minX, abs(thisPoint.x));
      }else if(abs(columnIdn-450) < rangeNum){
        minY=max(minY,abs(thisPoint.y));
      }else if(abs(columnIdn-900)<rangeNum){
        maxX=max(maxX, abs(thisPoint.x));
      } else if(abs(columnIdn-1350)<rangeNum){
        maxY=max(maxY,abs(thisPoint.y));
      }
    }
  }
  MeasureMsg measure;
  measure.minX() = minX;
  measure.minY() = minY;
  measure.minZ() = minZ;
  measure.maxX() = maxX;
  measure.maxY() = maxY;
  measure.maxZ() = maxZ;
  eprosima::fastdds::dds::DataWriter* writer = dds_pub->getWriter(dds_pub_Measure_topic);
  auto listener = dds_pub->getListener(dds_pub_Measure_topic);
  if(listener->connected_){
    writer->write((void*)&measure);
    if(LOG) std::cout<<"pub measure data"<<std::endl;
  }
  
}

void disCallback(const std_msgs::Float32::ConstPtr &msg){
  maxZ=msg->data;
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"robotVelodynePub");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  DDSPublisher *dds_pub= new DDSPublisher();

  nhPrivate.getParam("wan_ip",wan_ip);
  nhPrivate.getParam("port",port);
  nhPrivate.getParam("ros_sub_PointCloud2_topic",ros_sub_PointCloud2_topic);
  nhPrivate.getParam("ros_sub_Dis_topic",ros_sub_Dis_topic);
  nhPrivate.getParam("dds_pub_Measure_topic",dds_pub_Measure_topic);

  if (dds_pub->init(wan_ip, static_cast<uint16_t>(port),0, 0))//ip,port,domainID,client=0 or server=1
       ROS_INFO("init  dds succeed!");
  else
  {
      ROS_INFO("init dds failed!");
  }   
 
  eprosima::fastdds::dds::TypeSupport type_pcl(new MeasureMsgPubSubType());
  eprosima::fastdds::dds::DataWriterQos wqos_pcl;

  wqos_pcl.transport_priority().value=16;
  wqos_pcl.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_pcl.history().depth = 1;
  wqos_pcl.resource_limits().max_samples = 50;
  wqos_pcl.resource_limits().allocated_samples = 20;
  wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
  wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
  wqos_pcl.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
  //wqos_pcl.writer_resource_limits().matched_subscriber_allocation =eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);

  dds_pub->createWriter(dds_pub_Measure_topic,wqos_pcl,type_pcl);

  ros::Subscriber sub_pcl= nh.subscribe<sensor_msgs::PointCloud2>(ros_sub_PointCloud2_topic, 10 , boost::bind(&subpclCallback,_1,dds_pub));
  ros::Subscriber subZDis = nh.subscribe<std_msgs::Float32>(ros_sub_Dis_topic,10,disCallback);

  ros::spin();
	
  return 0;
}
