//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
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
#include "dds_msgs.h"
//user lib
#include "DDSPublisher.h"
#include "optionparser.h"

//wan_ip = "192.168.1.108";
#define WAN_IP  "127.0.0.1"
#define PORT  56490

//define config
#define dds_joy_topic_name "JoyTopic"
#define dds_pose_topic_name "PoseTopic"

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;

eprosima::fastdds::dds::DataWriter* writer_joy;
eprosima::fastdds::dds::DataWriter* writer_pose;

class PubListener : public eprosima::fastdds::dds::DataWriterListener
    {
    public:

        PubListener(std::string topicName): topicName_(topicName), matched_(0), connected_(false){ }

        PubListener(): matched_(0), connected_(false){}

        ~PubListener() override{}

        void on_publication_matched(
                eprosima::fastdds::dds::DataWriter* writer,
                const eprosima::fastdds::dds::PublicationMatchedStatus& info ) 
        {
        	if (info.current_count_change == 1)
            {
        		matched_ ++;
        		connected_ = true;
        		std::cout << "[RUDP] Publisher ["<<this->topicName_<<"] ["<<matched_<<"st] matched!" << std::endl;
    		}
    		else if (info.current_count_change == -1)
    		{
        		connected_=false;
        		std::cout << "[RUDP] Publisher ["<<this->topicName_<<"] unmatched" << std::endl;
    		}
    		else
    		{
        		std::cout << info.current_count_change
                  << " is not a valid value for PublicationMatchedStatus current count change" << std::endl;
    		}
        }

        int matched_;
        bool connected_;
        std::string topicName_;
    } ;

void subjoyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  JoyMsg joy;
  joy.seq()=joy_msg->header.seq;
  joy.secs()=joy_msg->header.stamp.sec;
  joy.nsecs()=joy_msg->header.stamp.nsec;
  joy.frame_id()=joy_msg->header.frame_id;
  joy.axes().assign(joy_msg->axes.begin(),joy_msg->axes.end());
  joy.buttons().assign(joy_msg->buttons.begin(),joy_msg->buttons.end());
  writer_joy->write((void*)&joy);
  /*
  for(int i=0; i< joy_msg->axes.size();i++)
  	std::cout<<"axes "<<i<<" "<<joy_msg->axes[i]<<std::endl;
  for(int i=0; i< joy_msg->axes.size();i++)
  	std::cout<<"axes "<<i<<" "<<joy_msg->axes[i]<<std::endl;
  */
  //std::cout << "[RUDP-Twist] linear_x: " << joy_msg->axes << " with angular_z: "<< joy_msg->buttons << " SENT" << std::endl;
  std::cout << "[RUDP-Joy]  SENT" << std::endl;
}

void subgoalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  PoseMsg pose;
  pose.seq()=pose_msg->header.seq;
  pose.secs()=pose_msg->header.stamp.sec;
  pose.nsecs()=pose_msg->header.stamp.nsec;
  pose.frame_id()=pose_msg->header.frame_id;
  pose.position_x()=pose_msg->pose.position.x;
  pose.position_y()=pose_msg->pose.position.y;
  pose.position_z()=pose_msg->pose.position.z;
  pose.orientation_x()=pose_msg->pose.orientation.x;
  pose.orientation_y()=pose_msg->pose.orientation.y;
  pose.orientation_z()=pose_msg->pose.orientation.z;
  pose.orientation_w()=pose_msg->pose.orientation.w;
  writer_pose->write((void*)&pose);
  std::cout << "[RUDP-Pose] seq:"<<pose_msg->header.seq<<std::endl<<"  position:x  " <<pose_msg->pose.position.x<<"  y:  "<<pose_msg->pose.position.y<<"  z:  "<<pose_msg->pose.position.z<<  std::endl
  << "  oriention:x  "<<pose_msg->pose.orientation.x<< "  y:  "<<pose_msg->pose.orientation.y<<"  z:  "<<pose_msg->pose.orientation.z<< "  w:   "<<pose_msg->pose.orientation.w  << " SENT" << std::endl;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"pcgoalPub");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    std::string wan_ip =WAN_IP;
    int port =PORT;
    nhPrivate.getParam("wan_ip",wan_ip);
    nhPrivate.getParam("port",port);

    DomainParticipantQos pqos;
    pqos.name("Participant_pub");
    //ip&port
    int32_t kind = LOCATOR_KIND_UDPv4;
    Locator_t initial_peer_locator;
    initial_peer_locator.kind = kind;
    IPLocator::setIPv4(initial_peer_locator, wan_ip);
    initial_peer_locator.port = port;

    // Configure the current participant as SERVER
    std::cout << " pcJoyPub ros node publisher "<<wan_ip << ":" << port << std::endl;
    pqos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);
    //pqos.wire_protocol().builtin.discovery_config.discoveryServer_client_syncperiod =eprosima::fastrtps::Duration_t(0, 250000000);
    //pqos.wire_protocol().builtin.discovery_config.discoveryProtocol = eprosima::fastrtps::rtps::DiscoveryProtocol::SERVER;
    //pqos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(initial_peer_locator);
    //std::istringstream("72.61.73.70.66.61.72.6d.74.65.73.74") >> pqos.wire_protocol().prefix;
    pqos.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
    pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod =eprosima::fastrtps::Duration_t(5, 0);
        

    // UDP
    auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
    pqos.transport().user_transports.push_back(udp_transport);
    pqos.transport().use_builtin_transports = false;
    pqos.transport().send_socket_buffer_size = 1048576;
    pqos.transport().listen_socket_buffer_size =  4194304;

    eprosima::fastdds::dds::DomainParticipant* participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (participant_ == nullptr)
    {
        std::cout<<"Creat participant faild"<<std::endl;
    }
    //CREATE THE PUBLISHER
    eprosima::fastdds::dds::Publisher* publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
    if (publisher_ == nullptr)
    {
        std::cout<<"Creat publisher faild"<<std::endl;
    }

    //CREATE THE TOPIC
    //REGISTER THE TYPE*******************************************************************************************************************
    eprosima::fastdds::dds::TypeSupport type_joy(new JoyMsgPubSubType());
    eprosima::fastdds::dds::TypeSupport type_pose(new PoseMsgPubSubType());
    type_joy.register_type(participant_);
    type_pose.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_joy;
    tqos_joy.transport_priority().value=32;
    tqos_joy.durability().kind= eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    //CREATE THE TOPIC*********************************************************************************************************************
    eprosima::fastdds::dds::Topic* topic_joy = participant_->create_topic(dds_joy_topic_name, type_joy.get_type_name(), tqos_joy);
    eprosima::fastdds::dds::Topic* topic_pose = participant_->create_topic(dds_pose_topic_name, type_pose.get_type_name(), tqos_joy);
  
    eprosima::fastdds::dds::DataWriterQos wqos_joy;
    wqos_joy.transport_priority().value=32;
    wqos_joy.history().kind = KEEP_LAST_HISTORY_QOS;
    wqos_joy.history().depth =1;
    wqos_joy.resource_limits().max_samples = 50;
    wqos_joy.resource_limits().allocated_samples = 20;
    wqos_joy.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
    wqos_joy.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
    wqos_joy.reliability().kind = RELIABLE_RELIABILITY_QOS;
    wqos_joy.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    /***                                                                                                 listener and writter                                                                                                                                                                               ***/
    PubListener *listener_joy=new  PubListener(dds_joy_topic_name);
    PubListener *listener_pose=new  PubListener(dds_pose_topic_name);
    writer_joy= publisher_->create_datawriter(topic_joy, wqos_joy, listener_joy);
    writer_pose= publisher_->create_datawriter(topic_pose, wqos_joy, listener_pose);

    if(writer_joy == nullptr || writer_pose==nullptr)
        std::cout<<"creat topic "<<"joy or pose"<<" failed"<<std::endl;
    else
        std::cout<<"creat topic "<<"joy and pose"<<" success"<<std::endl;

    ros::Subscriber sub_joy=nh.subscribe<sensor_msgs::Joy>("/joy", 10 , boost::bind(&subjoyCallback,_1));
    ros::Subscriber sub_goal=nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,boost::bind(&subgoalCallback,_1));

    ros::spin();
  
  return 0;
}
