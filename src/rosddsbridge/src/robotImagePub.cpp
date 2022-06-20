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
#include <mutex>
#include <memory>
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

#define MAXPCLSIZE 7000000
#define MAXIMAGESIZE 1000000
#define LOG 1
#define UNIT 1800

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace std;

std::string wan_ip ="127.0.0.1";
int port =56452;
std::string ros_sub_image1_topic= "/image/compressed";
std::string ros_sub_image2_topic= "/image/compressed";
std::string dds_pub_image1_topic= "ImageTopic1";
std::string dds_pub_image2_topic= "ImageTopic2";

mutex mt;

const int SkipCount = 10;
int skipCount1=0;
int skipCount2=0;

void subimgCallback1(const sensor_msgs::CompressedImage::ConstPtr& image_msg, DDSPublisher* dds_pub)
{
  if(skipCount1==0){
    skipCount1 = SkipCount;
  }else{
    --skipCount1;
    return;
  }
  ImageMsg image_;
  image_.datacount()=image_msg->data.size();
  if(LOG) ROS_INFO("image data size %d",image_msg->data.size());
  if(image_.datacount()<MAXIMAGESIZE)
  {
    image_.data().resize(image_.datacount());
    image_.seq()=image_msg->header.seq;
    image_.secs()=image_msg->header.stamp.sec;
    image_.nsecs()=image_msg->header.stamp.nsec;
    image_.frame_id()=image_msg->header.frame_id;
    image_.format()=image_msg->format;
    image_.data().assign(image_msg->data.begin(),image_msg->data.end());

    eprosima::fastdds::dds::DataWriter* writer = dds_pub->getWriter(dds_pub_image1_topic);
    auto listener = dds_pub->getListener(dds_pub_image1_topic);
    if(listener->connected_){
      writer->write((void*)&image_);
      if(LOG) std::cout<<"pub image1 data"<<std::endl;
    }
  }else{
      ROS_INFO("> MAXIMAGESIZE lost");
  }
}

void subimgCallback2(const sensor_msgs::CompressedImage::ConstPtr& image_msg, DDSPublisher* dds_pub)
{
  if(skipCount2==0){
    skipCount2 = SkipCount;
  }else{
    --skipCount2;
    return;
  }
  ImageMsg image_;
  image_.datacount()=image_msg->data.size();
  if(LOG) ROS_INFO("image data size %d",image_msg->data.size());
  if(image_.datacount()<MAXIMAGESIZE)
  {
    image_.data().resize(image_.datacount());
    image_.seq()=image_msg->header.seq;
    image_.secs()=image_msg->header.stamp.sec;
    image_.nsecs()=image_msg->header.stamp.nsec;
    image_.frame_id()=image_msg->header.frame_id;
    image_.format()=image_msg->format;
    image_.data().assign(image_msg->data.begin(),image_msg->data.end());

    eprosima::fastdds::dds::DataWriter* writer = dds_pub->getWriter(dds_pub_image2_topic);
    auto listener = dds_pub->getListener(dds_pub_image2_topic);
    if(listener->connected_){
      writer->write((void*)&image_);
      if(LOG) std::cout<<"pub image2 data"<<std::endl;
    }
  }else{
      ROS_INFO("> MAXIMAGESIZE lost");
  }
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"robotImagePub");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  DDSPublisher *dds_pub= new DDSPublisher();

  nhPrivate.getParam("wan_ip",wan_ip);
  nhPrivate.getParam("port",port);
  nhPrivate.getParam("ros_sub_image1_topic",ros_sub_image1_topic);
  nhPrivate.getParam("ros_sub_image2_topic",ros_sub_image2_topic);
  nhPrivate.getParam("dds_pub_image1_topic",dds_pub_image1_topic);
  nhPrivate.getParam("dds_pub_image2_topic",dds_pub_image2_topic);

  if (dds_pub->init(wan_ip, static_cast<uint16_t>(port),0, 0))//ip,port,domainID,client=0 or server=1
       ROS_INFO("init  dds succeed!");
  else
  {
      ROS_INFO("init dds failed!");
  }   
 
  eprosima::fastdds::dds::TypeSupport type_pcl(new ImageMsgPubSubType());
  eprosima::fastdds::dds::DataWriterQos wqos_pcl;

  wqos_pcl.transport_priority().value=32;
  wqos_pcl.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_pcl.history().depth = 1;
  wqos_pcl.resource_limits().max_samples = 50;
  wqos_pcl.resource_limits().allocated_samples = 20;
  wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
  wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
  wqos_pcl.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
  //wqos_pcl.writer_resource_limits().matched_subscriber_allocation =eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);

  dds_pub->createWriter(dds_pub_image1_topic,wqos_pcl,type_pcl);
  dds_pub->createWriter(dds_pub_image2_topic,wqos_pcl,type_pcl);
  
  ros::Subscriber sub_img1=nh.subscribe<sensor_msgs::CompressedImage>(ros_sub_image1_topic,10,boost::bind(&subimgCallback1,_1,dds_pub));
  ros::Subscriber sub_img2=nh.subscribe<sensor_msgs::CompressedImage>(ros_sub_image2_topic,10,boost::bind(&subimgCallback2,_1,dds_pub));

  ros::spin();
	
  return 0;
}
