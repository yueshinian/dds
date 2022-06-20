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

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

OdometryMsg  odom_;

std::string ros_sub_odometry_topic = "/odom";
std::string dds_pub_odometry_topic = "OdometryTopic";
int odom_count=0;
void subodomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,DDSPublisher* dds_pub ){
  if(odom_count==0){
    odom_count=200;
  }else{
    --odom_count;
    return;
  }
  dds_pub->odom_.seq()=odom_msg->header.seq;
  dds_pub->odom_.secs()=odom_msg->header.stamp.sec;
  dds_pub->odom_.nsecs()=odom_msg->header.stamp.nsec;
  dds_pub->odom_.frame_id()=odom_msg->header.frame_id;
  dds_pub->odom_.child_frame_id()=odom_msg->child_frame_id;
  dds_pub->odom_.position_x()=odom_msg->pose.pose.position.x;
  dds_pub->odom_.position_y()=odom_msg->pose.pose.position.y;
  dds_pub->odom_.position_z()=odom_msg->pose.pose.position.z;
  dds_pub->odom_.orientation_x()=odom_msg->pose.pose.orientation.x;
  dds_pub->odom_.orientation_y()=odom_msg->pose.pose.orientation.y;
  dds_pub->odom_.orientation_z()=odom_msg->pose.pose.orientation.z;
  dds_pub->odom_.orientation_w()=odom_msg->pose.pose.orientation.w;
  dds_pub->publish(dds_pub_odometry_topic,true,6);
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"robotOdomPub");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  std::string wan_ip ="127.0.0.1";
  int port =56452;

  nhPrivate.getParam("wan_ip",wan_ip);
  nhPrivate.getParam("port",port);
  nhPrivate.getParam("ros_sub_odometry_topic",ros_sub_odometry_topic);
  nhPrivate.getParam("dds_pub_odometry_topic",dds_pub_odometry_topic);

  DDSPublisher *dds_pub= new DDSPublisher();
  if (dds_pub->init(wan_ip, static_cast<uint16_t>(port),0, 0))//ip,port,domainID,client=0 or server=1
       ROS_INFO("init  dds succeed!");
  else
  {
      ROS_INFO("init dds failed!");
  }   
 
  eprosima::fastdds::dds::TypeSupport type_odom(new OdometryMsgPubSubType());
  eprosima::fastdds::dds::DataWriterQos wqos_odom;
  wqos_odom.transport_priority().value=2;
  wqos_odom.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_odom.history().depth = 1;
  wqos_odom.resource_limits().max_samples = 50;
  wqos_odom.resource_limits().allocated_samples = 20;
  wqos_odom.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
  wqos_odom.reliable_writer_qos().times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
  wqos_odom.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_odom.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;

  dds_pub->createWriter(dds_pub_odometry_topic,wqos_odom,type_odom);

  ros::Subscriber sub_odom=nh.subscribe<nav_msgs::Odometry>(ros_sub_odometry_topic,1,boost::bind(&subodomCallback,_1,dds_pub));

  ros::spin();
	
  return 0;
}
