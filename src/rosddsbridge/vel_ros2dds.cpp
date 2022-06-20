//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
//C++ lib
#include <string>
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
#include "OctomapMsg.h"
#include "TwistMsg.h"
#include "TfMsg.h"
#include "ImageMsg.h"
#include "PclMsg.h"
#include "TfMsgBase.h"
#include "MarkerMsg.h"
//user lib
#include "DDSPublisher.h"
#include "optionparser.h"

//wan_ip = "192.168.1.108";
#define WAN_IP  "127.0.0.1"
#define PORT  56460

//define config
#define ros_sub_PointCloud2_topic  "/segmented_cloud_pure"
#define ros_sub_Octomap_topic "octomap_full"
#define ros_sub_image_topic "/image/compressed"
#define ros_sub_tf_topic  "/tf"
#define ros_sub_twist_topic "/cmd_vel"
#define ros_sub_marker_topic "/visual_pub"

#define dds_pub_PointCloud2_topic   "PclTopic"
#define dds_pub_Octomap_topic "OctomapTopic"
#define dds_pub_image_topic "ImageTopic"
#define dds_pub_tf_topic  "TfTopic"
#define dds_pub_twist_topic     "TwistTopic"
#define dds_pub_marker_topic "MarkerTopic"

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

void subvelCallback(const geometry_msgs::Twist::ConstPtr& vel_msg, DDSPublisher* dds_pub)
{
  /*
  dds_pub->twist_.linear_x()=vel_msg->linear.x;
  dds_pub->twist_.linear_y()=vel_msg->linear.y;
  dds_pub->twist_.linear_z()=vel_msg->linear.z;
  dds_pub->twist_.angular_x()=vel_msg->angular.x;
  dds_pub->twist_.angular_y()=vel_msg->angular.y;
  dds_pub->twist_.angular_z()=vel_msg->angular.z;
  dds_pub->publish(dds_pub_twist_topic,false,0);
  */
  TwistMsg twist;
  twist.linear_x()=vel_msg->linear.x;
  twist.linear_y()=vel_msg->linear.y;
  twist.linear_z()=vel_msg->linear.z;
  twist.angular_x()=vel_msg->angular.x;
  twist.angular_y()=vel_msg->angular.y;
  twist.angular_z()=vel_msg->angular.z;
  //dds_pub->publish(dds_pub_twist_topic,false,0);
  eprosima::fastdds::dds::DataWriter* writer = dds_pub->getWriter(dds_pub_twist_topic);
  writer->write((void*)&twist);
  std::cout << "[RUDP-Twist] linear_x: " << twist.linear_x() << " with angular_z: "<< twist.angular_z() << " SENT" << std::endl;
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"ros2dds");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  ros::Subscriber sub_vel;
  DDSPublisher *dds_pub= new DDSPublisher();
  std::string wan_ip =WAN_IP;
  int port =PORT;
  nhPrivate.getParam("wan_ip",wan_ip);
  nhPrivate.getParam("port",port);

  if (dds_pub->init(wan_ip, static_cast<uint16_t>(port),0, 1))
       ROS_INFO("init  dds succeed!");
  else
  {
      ROS_INFO("init dds failed!");
  }   
 

  eprosima::fastdds::dds::TypeSupport type_twist(new TwistMsgPubSubType());
  
  eprosima::fastdds::dds::DataWriterQos wqos_twist;

  wqos_twist.transport_priority().value=32;
  wqos_twist.history().kind = KEEP_LAST_HISTORY_QOS;
  wqos_twist.history().depth =1;
  wqos_twist.resource_limits().max_samples = 50;
  wqos_twist.resource_limits().allocated_samples = 20;
  wqos_twist.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
  wqos_twist.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
  wqos_twist.reliability().kind = RELIABLE_RELIABILITY_QOS;
  wqos_twist.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    
  dds_pub->createWriter(dds_pub_twist_topic,wqos_twist,type_twist);

  sub_vel=nh.subscribe<geometry_msgs::Twist>(ros_sub_twist_topic, 10 , boost::bind(&subvelCallback,_1,dds_pub));

  ros::spin();
  
  return 0;
}
