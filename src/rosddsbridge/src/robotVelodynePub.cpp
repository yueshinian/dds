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
//C++ lib
#include <string>
#include <boost/thread.hpp> 
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
#include "OctomapMsg.h"
#include "TwistMsg.h"
#include "TfMsg.h"
#include "ImageMsg.h"
#include "PclMsg.h"
#include "TfMsgBase.h"
#include "MarkerMsg.h"
#include "OdometryMsg.h"
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#define MAXPCLSIZE 7000000
#define MAXIMAGESIZE 1000000
#define LOG 1
#define UNIT 1800

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

PclMsg pcl_;

std::string wan_ip ="127.0.0.1";
int port =56452;
std::string ros_sub_PointCloud2_topic = "/registered_scan";
std::string dds_pub_PointCloud2_topic=   "PclTopic";

int pcl_count=0;
void subpclCallback(const sensor_msgs::PointCloud2::ConstPtr& pclin, DDSPublisher* dds_pub)
{
  if(pcl_count==0){
    pcl_count=5;
  }else{
    --pcl_count;
    std::cout<<pcl_count<<std::endl;
    return;
  }
  /*
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*pclin, *laserCloudIn);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_volxel (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud (laserCloudIn);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered_volxel);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_box(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::CropBox<pcl::PointXYZI> cropBoxFilter;
  cropBoxFilter.setMin(Eigen::Vector4f(-30, -30, -1, 1.0));
  cropBoxFilter.setMax(Eigen::Vector4f(30, 30, 2, 1.0));
  cropBoxFilter.setNegative(false);    
  cropBoxFilter.setInputCloud(cloud_filtered_volxel);
  cropBoxFilter.filter(*cloud_filtered_box);

  sensor_msgs::PointCloud2 pcl;
  pcl::toROSMsg(*cloud_filtered_box,pcl);
  pcl.header=pclin->header;
  std::cout<<"laserCloudIn format size is : "<<laserCloudIn->points.size()<<std::endl;
  std::cout<<"cloud_filtered_volxel format size is : "<<cloud_filtered_volxel->points.size()<<std::endl;
  std::cout<<"cloud_filtered_box format size is : "<<cloud_filtered_box->points.size()<<std::endl;

  dds_pub->pcl_.datacount()=pcl.data.size();
  if(dds_pub->pcl_.datacount()<MAXPCLSIZE)
  {
    dds_pub->pcl_.data().reserve(dds_pub->pcl_.datacount());
    dds_pub->pcl_.seq()=pcl.header.seq;
    dds_pub->pcl_.secs()=pcl.header.stamp.sec;
    dds_pub->pcl_.nsecs()=pcl.header.stamp.nsec;
    dds_pub->pcl_.frame_id()=pcl.header.frame_id;
    dds_pub->pcl_.height()=pcl.height;
    dds_pub->pcl_.width()=pcl.width;
    dds_pub->pcl_.PointFileds_name()[0]=pcl.fields[0].name;
    dds_pub->pcl_.PointFileds_name()[1]=pcl.fields[1].name;
    dds_pub->pcl_.PointFileds_name()[2]=pcl.fields[2].name;
    dds_pub->pcl_.PointFileds_name()[3]=pcl.fields[3].name;
    dds_pub->pcl_.PointFileds_offset()[0]=pcl.fields[0].offset;
    dds_pub->pcl_.PointFileds_offset()[1]=pcl.fields[1].offset;
    dds_pub->pcl_.PointFileds_offset()[2]=pcl.fields[2].offset;
    dds_pub->pcl_.PointFileds_offset()[3]=pcl.fields[3].offset;
    dds_pub->pcl_.PointFileds_datatype()[0]=pcl.fields[0].datatype;
    dds_pub->pcl_.PointFileds_datatype()[1]=pcl.fields[1].datatype;
    dds_pub->pcl_.PointFileds_datatype()[2]=pcl.fields[2].datatype;
    dds_pub->pcl_.PointFileds_datatype()[3]=pcl.fields[3].datatype;
    dds_pub->pcl_.PointFileds_count()[0]=pcl.fields[0].count;
    dds_pub->pcl_.PointFileds_count()[1]=pcl.fields[1].count;
    dds_pub->pcl_.PointFileds_count()[2]=pcl.fields[2].count;
    dds_pub->pcl_.PointFileds_count()[3]=pcl.fields[3].count;
    dds_pub->pcl_.is_bigendian()=pcl.is_bigendian;
    dds_pub->pcl_.point_step()=pcl.point_step;
    dds_pub->pcl_.row_step()=pcl.row_step;
    dds_pub->pcl_.data().assign(pcl.data.begin(),pcl.data.end());
    dds_pub->pcl_.is_dense()=pcl.is_dense;
    dds_pub->publish(dds_pub_PointCloud2_topic,false,4);
    dds_pub->pcl_.data().clear(); 
  }
  else
  {
    ROS_INFO("> MAXPCLSIZE lost");
  }
  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*pclin, *laserCloudIn);
  int size=laserCloudIn->points.size();
  std::cout<<"laserCloudIn format size is : "<<size<<std::endl;
  for(int i=0;i<size;++i){
    if(i%UNIT==0 || i==size-1){
      std::cout<<"laserCloudTemp format size is : "<<laserCloudTemp->points.size()<<std::endl;
      sensor_msgs::PointCloud2 pcl;
      pcl::toROSMsg(*laserCloudTemp,pcl);
      pcl.header=pclin->header;
      dds_pub->pcl_.data().reserve(2000);
      dds_pub->pcl_.seq()=pcl.header.seq;
      dds_pub->pcl_.secs()=pcl.header.stamp.sec;
      dds_pub->pcl_.nsecs()=pcl.header.stamp.nsec;
      dds_pub->pcl_.frame_id()=pcl.header.frame_id;
      dds_pub->pcl_.height()=pcl.height;
      dds_pub->pcl_.width()=pcl.width;
      dds_pub->pcl_.PointFileds_name()[0]=pcl.fields[0].name;
      dds_pub->pcl_.PointFileds_name()[1]=pcl.fields[1].name;
      dds_pub->pcl_.PointFileds_name()[2]=pcl.fields[2].name;
      dds_pub->pcl_.PointFileds_name()[3]=pcl.fields[3].name;
      dds_pub->pcl_.PointFileds_offset()[0]=pcl.fields[0].offset;
      dds_pub->pcl_.PointFileds_offset()[1]=pcl.fields[1].offset;
      dds_pub->pcl_.PointFileds_offset()[2]=pcl.fields[2].offset;
      dds_pub->pcl_.PointFileds_offset()[3]=pcl.fields[3].offset;
      dds_pub->pcl_.PointFileds_datatype()[0]=pcl.fields[0].datatype;
      dds_pub->pcl_.PointFileds_datatype()[1]=pcl.fields[1].datatype;
      dds_pub->pcl_.PointFileds_datatype()[2]=pcl.fields[2].datatype;
      dds_pub->pcl_.PointFileds_datatype()[3]=pcl.fields[3].datatype;
      dds_pub->pcl_.PointFileds_count()[0]=pcl.fields[0].count;
      dds_pub->pcl_.PointFileds_count()[1]=pcl.fields[1].count;
      dds_pub->pcl_.PointFileds_count()[2]=pcl.fields[2].count;
      dds_pub->pcl_.PointFileds_count()[3]=pcl.fields[3].count;
      dds_pub->pcl_.is_bigendian()=pcl.is_bigendian;
      dds_pub->pcl_.point_step()=pcl.point_step;
      dds_pub->pcl_.row_step()=pcl.row_step;
      dds_pub->pcl_.data().assign(pcl.data.begin(),pcl.data.end());
      dds_pub->pcl_.is_dense()=pcl.is_dense;
      dds_pub->publish(dds_pub_PointCloud2_topic,true,4);
      dds_pub->pcl_.data().clear(); 
      laserCloudTemp->clear();
    }else{
      laserCloudTemp->push_back(laserCloudIn->points[i]);
    }
  }
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
  nhPrivate.getParam("dds_pub_PointCloud2_topic",dds_pub_PointCloud2_topic);

  if (dds_pub->init(wan_ip, static_cast<uint16_t>(port),0, 0))//ip,port,domainID,client=0 or server=1
       ROS_INFO("init  dds succeed!");
  else
  {
      ROS_INFO("init dds failed!");
  }   
 
  eprosima::fastdds::dds::TypeSupport type_pcl(new PclMsgPubSubType());
  eprosima::fastdds::dds::DataWriterQos wqos_pcl;

  wqos_pcl.transport_priority().value=16;
  wqos_pcl.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_pcl.history().depth = 16;
  wqos_pcl.resource_limits().max_samples = 50;
  wqos_pcl.resource_limits().allocated_samples = 20;
  wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
  wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
  wqos_pcl.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
  //wqos_pcl.writer_resource_limits().matched_subscriber_allocation =eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);

  dds_pub->createWriter(dds_pub_PointCloud2_topic,wqos_pcl,type_pcl);

  ros::Subscriber sub_pcl= nh.subscribe<sensor_msgs::PointCloud2>(ros_sub_PointCloud2_topic, 10 , boost::bind(&subpclCallback,_1,dds_pub));

  ros::spin();
	
  return 0;
}
