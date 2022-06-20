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

#define MAXPCLSIZE 7000000
#define MAXIMAGESIZE 1000000
#define LOG 1

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

PclMsg pcl_;
OctomapMsg octomap_;
TfMsg tf_;
ImageMsg image_;
TwistMsg twist_;
MarkerMsg marker_;
OdometryMsg  odom_;


std::string ros_sub_PointCloud2_topic = "/velodyne_points_filtered";
std::string ros_sub_Octomap_topic= "octomap_full";
std::string ros_sub_image_topic= "/image/compressed";
std::string ros_sub_tf_topic = "/tf";
std::string ros_sub_twist_topic= "/cmd_vel";
std::string ros_sub_marker_topic ="/visual_pub";
std::string ros_sub_frontvideo_topic= "/frontvideo/compressed";
std::string ros_sub_backvideo_topic= "/backvideo/compressed";
std::string ros_sub_topvideo_topic= "/topvideo/compressed";
std::string ros_sub_odometry_topic = "/odom";

std::string dds_pub_PointCloud2_topic=   "PclTopic";
std::string dds_pub_Octomap_topic= "OctomapTopic";
std::string dds_pub_image_topic= "ImageTopic";
std::string dds_pub_tf_topic = "TfTopic";
std::string dds_pub_twist_topic=     "TwistTopic";
std::string dds_pub_marker_topic= "MarkerTopic";
std::string dds_pub_frontvideo_topic= "frontVideoTopic";
std::string dds_pub_backvideo_topic= "backVideoTopic";
std::string dds_pub_topvideo_topic= "topVideoTopic";
std::string dds_pub_odometry_topic = "OdometryTopic";

void subodomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,DDSPublisher* dds_pub ){
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
  dds_pub->publish(dds_pub_odometry_topic,false,6);
}

void subvelCallback(const geometry_msgs::Twist::ConstPtr& vel_msg, DDSPublisher* dds_pub)
{
  dds_pub->twist_.linear_x()=vel_msg->linear.x;
  dds_pub->twist_.linear_y()=vel_msg->linear.y;
  dds_pub->twist_.linear_z()=vel_msg->linear.z;
  dds_pub->twist_.angular_x()=vel_msg->angular.x;
  dds_pub->twist_.angular_y()=vel_msg->angular.y;
  dds_pub->twist_.angular_z()=vel_msg->angular.z;
  dds_pub->publish(dds_pub_twist_topic,false,0);
}
int pcl_count=0;
void subpclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl, DDSPublisher* dds_pub)
{
  if(pcl_count==0){
    pcl_count=2;
  }else{
    --pcl_count;
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*pcl, *laserCloudIn);
  /*
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_volxel (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud (laserCloudIn);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered_volxel);
  sensor_msgs::PointCloud2Ptr pclin;
  pcl::toROSMsg(*cloud_filtered_volxel,pclin);
  pcl->header=pclin->header;
*/
  std::cout<<"pcl format size is : "<<laserCloudIn->points.size()<<std::endl;
  
  dds_pub->pcl_.datacount()=pcl->data.size();
  if(dds_pub->pcl_.datacount()<MAXPCLSIZE)
  {
    dds_pub->pcl_.data().resize(dds_pub->pcl_.datacount());
    dds_pub->pcl_.seq()=pcl->header.seq;
    dds_pub->pcl_.secs()=pcl->header.stamp.sec;
    dds_pub->pcl_.nsecs()=pcl->header.stamp.nsec;
    dds_pub->pcl_.frame_id()=pcl->header.frame_id;
    dds_pub->pcl_.height()=pcl->height;
    dds_pub->pcl_.width()=pcl->width;
    dds_pub->pcl_.PointFileds_name()[0]=pcl->fields[0].name;
    dds_pub->pcl_.PointFileds_name()[1]=pcl->fields[1].name;
    dds_pub->pcl_.PointFileds_name()[2]=pcl->fields[2].name;
    dds_pub->pcl_.PointFileds_name()[3]=pcl->fields[3].name;
    dds_pub->pcl_.PointFileds_offset()[0]=pcl->fields[0].offset;
    dds_pub->pcl_.PointFileds_offset()[1]=pcl->fields[1].offset;
    dds_pub->pcl_.PointFileds_offset()[2]=pcl->fields[2].offset;
    dds_pub->pcl_.PointFileds_offset()[3]=pcl->fields[3].offset;
    dds_pub->pcl_.PointFileds_datatype()[0]=pcl->fields[0].datatype;
    dds_pub->pcl_.PointFileds_datatype()[1]=pcl->fields[1].datatype;
    dds_pub->pcl_.PointFileds_datatype()[2]=pcl->fields[2].datatype;
    dds_pub->pcl_.PointFileds_datatype()[3]=pcl->fields[3].datatype;
    dds_pub->pcl_.PointFileds_count()[0]=pcl->fields[0].count;
    dds_pub->pcl_.PointFileds_count()[1]=pcl->fields[1].count;
    dds_pub->pcl_.PointFileds_count()[2]=pcl->fields[2].count;
    dds_pub->pcl_.PointFileds_count()[3]=pcl->fields[3].count;
    dds_pub->pcl_.is_bigendian()=pcl->is_bigendian;
    dds_pub->pcl_.point_step()=pcl->point_step;
    dds_pub->pcl_.row_step()=pcl->row_step;
    dds_pub->pcl_.data().assign(pcl->data.begin(),pcl->data.end());
    dds_pub->pcl_.is_dense()=pcl->is_dense;
    dds_pub->publish(dds_pub_PointCloud2_topic,false,4);
    dds_pub->pcl_.data().clear(); 
  }
  else
  {
    ROS_INFO("> MAXPCLSIZE lost");
  }
}

void submarkerCallback(const visualization_msgs::Marker::ConstPtr& marker_msg, DDSPublisher* dds_pub)
{
  dds_pub->marker_.seq()=marker_msg->header.seq;
  dds_pub->marker_.secs()=marker_msg->header.stamp.sec;
  dds_pub->marker_.nsecs()=marker_msg->header.stamp.nsec;
  dds_pub->marker_.frame_id()=marker_msg->header.frame_id;
  dds_pub->marker_.ns()=marker_msg->ns;
  dds_pub->marker_.id()=marker_msg->id;
  dds_pub->marker_.type()=marker_msg->type;
  dds_pub->marker_.action()=marker_msg->action;
  dds_pub->marker_.position_x()=marker_msg->pose.position.x;
  dds_pub->marker_.position_y()=marker_msg->pose.position.y;
  dds_pub->marker_.position_z()=marker_msg->pose.position.z;
  dds_pub->marker_.orientation_x()=marker_msg->pose.orientation.x;
  dds_pub->marker_.orientation_y()=marker_msg->pose.orientation.y;
  dds_pub->marker_.orientation_z()=marker_msg->pose.orientation.z;
  dds_pub->marker_.orientation_w()=marker_msg->pose.orientation.w;
  dds_pub->marker_.scale_x()=marker_msg->scale.x;
  dds_pub->marker_.scale_y()=marker_msg->scale.y;
  dds_pub->marker_.scale_z()=marker_msg->scale.z;
  dds_pub->marker_.color_r()=marker_msg->color.r;
  dds_pub->marker_.color_g()=marker_msg->color.g;
  dds_pub->marker_.color_b()=marker_msg->color.b;
  dds_pub->marker_.color_a()=marker_msg->color.a;
  dds_pub->publish(dds_pub_marker_topic,false,5);
}

void subimgCallback(const sensor_msgs::CompressedImage::ConstPtr& image_msg, DDSPublisher* dds_pub)
{
  dds_pub->image_.datacount()=image_msg->data.size();
  if(LOG) ROS_INFO("image data size %d",image_msg->data.size());
  if(dds_pub->image_.datacount()<MAXIMAGESIZE)
  {
    dds_pub->image_.data().resize(dds_pub->image_.datacount());
    dds_pub->image_.seq()=image_msg->header.seq;
    dds_pub->image_.secs()=image_msg->header.stamp.sec;
    dds_pub->image_.nsecs()=image_msg->header.stamp.nsec;
    dds_pub->image_.frame_id()=image_msg->header.frame_id;
    dds_pub->image_.format()=image_msg->format;
    dds_pub->image_.data().assign(image_msg->data.begin(),image_msg->data.end());
    dds_pub->publish(dds_pub_image_topic,false,3);
    dds_pub->image_.data().clear();
  }
  else
  {
    ROS_INFO("> MAXIMAGESIZE lost");
  }
}

void subfrontvideoCallback(const sensor_msgs::CompressedImage::ConstPtr& image_msg, DDSPublisher* dds_pub)
{
  ImageMsg video_;
  video_.datacount()=image_msg->data.size();
  if(LOG) ROS_INFO("video data size %d",video_.datacount());
  if(video_.datacount()<MAXIMAGESIZE)
  {
    video_.data().resize(video_.datacount());
    video_.seq()=image_msg->header.seq;
    video_.secs()=image_msg->header.stamp.sec;
    video_.nsecs()=image_msg->header.stamp.nsec;
    video_.frame_id()=image_msg->header.frame_id;
    video_.format()=image_msg->format;
    video_.data().assign(image_msg->data.begin(),image_msg->data.end());
    eprosima::fastdds::dds::DataWriter* writer = dds_pub->getWriter(dds_pub_frontvideo_topic);
    writer->write((void*)&video_);
    if(LOG) std::cout << "[RUDP-frontVideo] seq: " << video_.seq() << " with time: "<< video_.secs() << " SENT" << std::endl;
  }
  else
  {
    ROS_INFO("> MAXIMAGESIZE lost");
  }
}

void subbackvideoCallback(const sensor_msgs::CompressedImage::ConstPtr& image_msg, DDSPublisher* dds_pub)
{
  ImageMsg video_;
  video_.datacount()=image_msg->data.size();
  if(video_.datacount()<MAXIMAGESIZE)
  {
    video_.data().resize(video_.datacount());
    video_.seq()=image_msg->header.seq;
    video_.secs()=image_msg->header.stamp.sec;
    video_.nsecs()=image_msg->header.stamp.nsec;
    video_.frame_id()=image_msg->header.frame_id;
    video_.format()=image_msg->format;
    video_.data().assign(image_msg->data.begin(),image_msg->data.end());
    eprosima::fastdds::dds::DataWriter* writer = dds_pub->getWriter(dds_pub_backvideo_topic);
    writer->write((void*)&video_);
    if(LOG) std::cout << "[RUDP-backVideo] seq: " << video_.seq() << " with time: "<< video_.secs() << " SENT" << std::endl;
  }
  else
  {
    ROS_INFO("> MAXIMAGESIZE lost");
  }
}

void subtopvideoCallback(const sensor_msgs::CompressedImage::ConstPtr& image_msg, DDSPublisher* dds_pub)
{
  ImageMsg video_;
  video_.datacount()=image_msg->data.size();
  if(video_.datacount()<MAXIMAGESIZE)
  {
    video_.data().resize(video_.datacount());
    video_.seq()=image_msg->header.seq;
    video_.secs()=image_msg->header.stamp.sec;
    video_.nsecs()=image_msg->header.stamp.nsec;
    video_.frame_id()=image_msg->header.frame_id;
    video_.format()=image_msg->format;
    video_.data().assign(image_msg->data.begin(),image_msg->data.end());
    eprosima::fastdds::dds::DataWriter* writer = dds_pub->getWriter(dds_pub_topvideo_topic);
    writer->write((void*)&video_);
    if(LOG) std::cout << "[RUDP-topVideo] seq: " << video_.seq() << " with time: "<< video_.secs() << " SENT" << std::endl;
  }
  else
  {
    ROS_INFO("> MAXIMAGESIZE lost");
  }
}

void suboctomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg, DDSPublisher* dds_pub)
{
	dds_pub->octomap_.datacount()=octomap_msg->data.size();
  dds_pub->octomap_.data().resize(dds_pub->octomap_.datacount());
  dds_pub->octomap_.seq()=octomap_msg->header.seq;
	dds_pub->octomap_.secs()=octomap_msg->header.stamp.sec;
	dds_pub->octomap_.nsecs()=octomap_msg->header.stamp.nsec;
	dds_pub->octomap_.frame_id()=octomap_msg->header.frame_id;
	dds_pub->octomap_.binary()=octomap_msg->binary;
	dds_pub->octomap_.id()=octomap_msg->id;
	dds_pub->octomap_.resolution()=octomap_msg->resolution;
	dds_pub->octomap_.data().assign(octomap_msg->data.begin(),octomap_msg->data.end());
	dds_pub->publish(dds_pub_Octomap_topic,false,1); 
  dds_pub->octomap_.data().clear();
}

void subtfCallback(const tf2_msgs::TFMessage::ConstPtr& tf_msg, DDSPublisher* dds_pub)
{
  int count_tf=0;
  //dds_pub->tf_.tf_sequence().resize(1);
  for(int i=0; i< tf_msg->transforms.size();i++)
  {
    if( tf_msg->transforms[i].header.frame_id== "odom" ||tf_msg->transforms[i].child_frame_id=="/camera")//&& tf_msg->transforms[i].child_frame_id=="/camera" tf_msg->transforms[i].header.frame_id=="/camera_init" ||
    {
    	//if(LOG) std::cout<<"tf size "<<tf_msg->transforms.size()<<std::endl;
      TfMsgBase tfbase;
    	tfbase.seq()=tf_msg->transforms[i].header.seq;
    	tfbase.secs()=tf_msg->transforms[i].header.stamp.sec;
    	tfbase.nsecs()=tf_msg->transforms[i].header.stamp.nsec;
    	tfbase.frame_id()=tf_msg->transforms[i].header.frame_id;
    	tfbase.child_frame_id()=tf_msg->transforms[i].child_frame_id;
    	tfbase.translation_x()=tf_msg->transforms[i].transform.translation.x;
    	tfbase.translation_y()=tf_msg->transforms[i].transform.translation.y;
    	tfbase.translation_z()=tf_msg->transforms[i].transform.translation.z;
    	tfbase.rotation_x()=tf_msg->transforms[i].transform.rotation.x;
    	tfbase.rotation_y()=tf_msg->transforms[i].transform.rotation.y;
    	tfbase.rotation_z()=tf_msg->transforms[i].transform.rotation.z;
      tfbase.rotation_w()=tf_msg->transforms[i].transform.rotation.w;
    	dds_pub->tf_.tf_sequence().push_back(tfbase);
      count_tf++;
    }
  }
  if(count_tf!=0)
  {
      dds_pub->publish(dds_pub_tf_topic,false,2);
      dds_pub->tf_.tf_sequence().clear();
  }
}


int main(int argc,char **argv)
{
  ros::init(argc,argv,"ros2dds");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  DDSPublisher *dds_pub= new DDSPublisher();
  /*
  DDSPublisher *dds_pub_octomap= new DDSPublisher();
  DDSPublisher *dds_pub_image= new DDSPublisher();
  DDSPublisher *dds_pub_tf= new DDSPublisher();
  DDSPublisher *dds_pub_twist= new DDSPublisher();
  DDSPublisher *dds_pub_marker= new DDSPublisher();
  DDSPublisher *dds_pub_video= new DDSPublisher();
  DDSPublisher *dds_pub_odom= new DDSPublisher();
  */
  std::string wan_ip ="127.0.0.1";
  int port =56452;

  nhPrivate.getParam("wan_ip",wan_ip);
  nhPrivate.getParam("port",port);
  nhPrivate.getParam("ros_sub_PointCloud2_topic",ros_sub_PointCloud2_topic);
  nhPrivate.getParam("ros_sub_Octomap_topic",ros_sub_Octomap_topic);
  nhPrivate.getParam("ros_sub_image_topic",ros_sub_image_topic);
  nhPrivate.getParam("ros_sub_tf_topic",ros_sub_tf_topic);
  nhPrivate.getParam("ros_sub_twist_topic",ros_sub_twist_topic);
  nhPrivate.getParam("ros_sub_marker_topic",ros_sub_marker_topic);
  nhPrivate.getParam("ros_sub_frontvideo_topic",ros_sub_frontvideo_topic);
  nhPrivate.getParam("ros_sub_backvideo_topic",ros_sub_backvideo_topic);
  nhPrivate.getParam("ros_sub_topkvideo_topic",ros_sub_topvideo_topic);
  nhPrivate.getParam("ros_sub_odometry_topic",ros_sub_odometry_topic);

  nhPrivate.getParam("dds_pub_PointCloud2_topic",dds_pub_PointCloud2_topic);
  nhPrivate.getParam("dds_pub_Octomap_topic",dds_pub_Octomap_topic);
  nhPrivate.getParam("dds_pub_image_topic",dds_pub_image_topic);
  nhPrivate.getParam("dds_pub_tf_topic",dds_pub_tf_topic);
  nhPrivate.getParam("dds_pub_twist_topic",dds_pub_twist_topic);
  nhPrivate.getParam("dds_pub_marker_topic",dds_pub_marker_topic);
  nhPrivate.getParam("dds_pub_frontvideo_topic",dds_pub_frontvideo_topic);
  nhPrivate.getParam("dds_pub_backvideo_topic",dds_pub_backvideo_topic);
  nhPrivate.getParam("dds_pub_odometry_topic",dds_pub_odometry_topic);

  if (dds_pub->init(wan_ip, static_cast<uint16_t>(port),0, 0))//ip,port,domainID,client=0 or server=1
       ROS_INFO("init  dds succeed!");
  else
  {
      ROS_INFO("init dds failed!");
  }   
 
  eprosima::fastdds::dds::TypeSupport type_pcl(new PclMsgPubSubType());
  eprosima::fastdds::dds::TypeSupport type_octomap(new OctomapMsgPubSubType());
  eprosima::fastdds::dds::TypeSupport type_tf(new TfMsgPubSubType());
  eprosima::fastdds::dds::TypeSupport type_twist(new TwistMsgPubSubType());
  eprosima::fastdds::dds::TypeSupport type_marker(new MarkerMsgPubSubType());
  eprosima::fastdds::dds::TypeSupport type_image(new ImageMsgPubSubType());
  eprosima::fastdds::dds::TypeSupport type_odom(new OdometryMsgPubSubType());
  
  eprosima::fastdds::dds::DataWriterQos wqos_octomap,wqos_tf,wqos_image,wqos_pcl,wqos_marker,wqos_odom;

  wqos_octomap.transport_priority().value=32;
  wqos_octomap.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_octomap.history().depth = 30;
  wqos_octomap.resource_limits().max_samples = 50;
  wqos_octomap.resource_limits().allocated_samples = 20;
  wqos_octomap.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
  wqos_octomap.reliable_writer_qos().times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
  wqos_octomap.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_octomap.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
    
  wqos_tf.transport_priority().value=2;
  wqos_tf.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_tf.history().depth = 1;
  wqos_tf.resource_limits().max_samples = 50;
  wqos_tf.resource_limits().allocated_samples = 20;
  wqos_tf.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
  wqos_tf.reliable_writer_qos().times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
  wqos_tf.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_tf.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;

  wqos_image.transport_priority().value=16;
  wqos_image.history().kind = KEEP_ALL_HISTORY_QOS;
  //wqos_image.history().depth = 1;
  wqos_image.resource_limits().max_samples = 50;
  wqos_image.resource_limits().allocated_samples = 20;
  wqos_image.reliable_writer_qos().times.heartbeatPeriod.seconds = 0.5;
  wqos_image.reliable_writer_qos().times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
  wqos_image.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_image.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;

  eprosima::fastdds::dds::DataWriterQos wqos_video;
  wqos_video.transport_priority().value=2;
  wqos_video.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_video.history().depth = 1;
  wqos_video.resource_limits().max_samples = 50;
  wqos_video.resource_limits().allocated_samples = 20;
  wqos_video.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
  wqos_video.reliable_writer_qos().times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
  wqos_video.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
  wqos_video.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
  //wqos_video.publish_mode().kind=ASYNCHRONOUS_PUBLISH_MODE;
  //eprosima::fastrtps::rtps::ThroughputControllerDescriptor slowPublisherThroughputController{3000, 1000};//b/s
  //wqos_video.throughput_controller(slowPublisherThroughputController);

  wqos_pcl.transport_priority().value=16;
  wqos_pcl.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_pcl.history().depth = 1;
  wqos_pcl.resource_limits().max_samples = 50;
  wqos_pcl.resource_limits().allocated_samples = 20;
  wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
  wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
  wqos_pcl.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
  wqos_pcl.writer_resource_limits().matched_subscriber_allocation =eprosima::fastrtps::ResourceLimitedContainerConfig::fixed_size_configuration(1u);

  wqos_marker.transport_priority().value=4;
  wqos_marker.history().kind = KEEP_ALL_HISTORY_QOS;
  //wqos_image.history().depth = 1;
  wqos_marker.resource_limits().max_samples = 50;
  wqos_marker.resource_limits().allocated_samples = 20;
  wqos_marker.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
  wqos_marker.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
  wqos_marker.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_marker.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;

  wqos_odom.transport_priority().value=2;
  wqos_odom.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_odom.history().depth = 1;
  wqos_odom.resource_limits().max_samples = 50;
  wqos_odom.resource_limits().allocated_samples = 20;
  wqos_odom.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
  wqos_odom.reliable_writer_qos().times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
  wqos_odom.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_odom.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;

  dds_pub->createWriter(dds_pub_PointCloud2_topic,wqos_pcl,type_pcl);
  dds_pub->createWriter(dds_pub_Octomap_topic,wqos_octomap,type_octomap);
  dds_pub->createWriter(dds_pub_image_topic,wqos_image,type_image);
  dds_pub->createWriter(dds_pub_tf_topic,wqos_tf,type_tf);
  dds_pub->createWriter(dds_pub_marker_topic,wqos_marker,type_marker);
  dds_pub->createWriter(dds_pub_frontvideo_topic,wqos_video,type_image);
  dds_pub->createWriter(dds_pub_backvideo_topic,wqos_video,type_image);
  dds_pub->createWriter(dds_pub_odometry_topic,wqos_odom,type_odom);

  ros::Subscriber sub_pcl= nh.subscribe<sensor_msgs::PointCloud2>(ros_sub_PointCloud2_topic, 10 , boost::bind(&subpclCallback,_1,dds_pub));
  //ros::Subscriber sub_tf = nh.subscribe<tf2_msgs::TFMessage>(ros_sub_tf_topic, 1, boost::bind(&subtfCallback,_1,dds_pub));
  //ros::Subscriber sub_octomap=nh.subscribe<octomap_msgs::Octomap>(ros_sub_Octomap_topic,10,boost::bind(&suboctomapCallback,_1,dds_pub));
  //ros::Subscriber sub_img=nh.subscribe<sensor_msgs::CompressedImage>(ros_sub_image_topic,10,boost::bind(&subimgCallback,_1,dds_pub));
  //ros::Subscriber sub_marker=nh.subscribe<visualization_msgs::Marker>(ros_sub_marker_topic,10,boost::bind(&submarkerCallback,_1,dds_pub));
  //ros::Subscriber sub_frontvideo=nh.subscribe<sensor_msgs::CompressedImage>(ros_sub_frontvideo_topic,1,boost::bind(&subfrontvideoCallback,_1,dds_pub));
  //ros::Subscriber sub_backvideo=nh.subscribe<sensor_msgs::CompressedImage>(ros_sub_backvideo_topic,1,boost::bind(&subbackvideoCallback,_1,dds_pub));
  ros::Subscriber sub_odom=nh.subscribe<nav_msgs::Odometry>(ros_sub_odometry_topic,1,boost::bind(&subodomCallback,_1,dds_pub));

  ros::spin();
	
  return 0;
}
