//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
//dds lib
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
//namespace
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;
//topic name
//define config
#define dds_joy_topic_name "JoyTopic"
#define dds_pose_topic_name "PoseTopic"
#define ros_joy_topic_name "joy"
#define ros_joy_topic_name "ddsgoal"

std::string wan_ip="127.0.0.1";
int port=56490;

class SubListener : public eprosima::fastdds::dds::DataReaderListener
{
public:
        SubListener(int mode , ros::NodeHandle nh, std::string topicName)
            : mode_(mode)
            , matched_(0)
            , topicName_(topicName)
        {
            this->np=nh;
            joy_pub=np.advertise<sensor_msgs::Joy>(dds_joy_topic_name,10);
            pose_pub=np.advertise<geometry_msgs::PoseStamped>(dds_pose_topic_name,1);
        }

        ~SubListener() 
        {
        }

        void on_data_available(eprosima::fastdds::dds::DataReader* reader) 
        {
            SampleInfo info;
            switch(mode_)
            {
                default:
                case 0:
                    if (reader->take_next_sample(&joy_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            // Print your structure data here.
                            sensor_msgs::Joy joy_msg;
                            joy_msg.header.seq=joy_.seq();
                            joy_msg.header.stamp.sec=joy_.secs();
                            joy_msg.header.stamp.nsec=joy_.nsecs();
                            joy_msg.header.frame_id=joy_.frame_id();
                            joy_msg.axes.assign(joy_.axes().begin(),joy_.axes().end());
                            joy_msg.buttons.assign(joy_.buttons().begin(),joy_.buttons().end());
                            joy_pub.publish(joy_msg);
                            std::cout << "[RUDP] Message joy RECEIVED" << std::endl;
                            std::cout<<"*******************************"<<std::endl;
                        }
                    }
                    break;

                case 1:
                    if (reader->take_next_sample(&pose_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            // Print your structure data here.
                            geometry_msgs::PoseStamped pose_msg;
                            pose_msg.header.seq=pose_.seq();
                            pose_msg.header.stamp.sec=pose_.secs();
                            pose_msg.header.stamp.nsec=pose_.nsecs();
                            pose_msg.header.frame_id=pose_.frame_id();
                            pose_msg.pose.position.x = pose_.position_x();
                            pose_msg.pose.position.y = pose_.position_y();
                            pose_msg.pose.position.z = pose_.position_z();
                            pose_msg.pose.orientation.x = pose_.orientation_x();
                            pose_msg.pose.orientation.y = pose_.orientation_y();
                            pose_msg.pose.orientation.z = pose_.orientation_z();
                            pose_msg.pose.orientation.w = pose_.orientation_w();
                            pose_pub.publish(pose_msg);
                            std::cout << "[RUDP-Pose] seq: "<<std::endl<<pose_msg.header.seq<<"  position:x  " <<pose_msg.pose.position.x<<"  y:  "<<pose_msg.pose.position.y<<"  z:  "<<pose_msg.pose.position.z<<std::endl
  << " oriention:x  "<<pose_msg.pose.orientation.x<< "  y:  "<<pose_msg.pose.orientation.y<<"  z:  "<<pose_msg.pose.orientation.z<< "  w:  "<<pose_msg.pose.orientation.w  << " SENT" << std::endl;
                        }
                    }
                    break;
            }
        }

        void on_subscription_matched(eprosima::fastdds::dds::DataReader* reader, const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) 
        {
            if (info.current_count_change == 1)
            {
                matched_++;
                std::cout << "[RUDP] Subscriber "<<topicName_<<" "<<matched_<<" matched" << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "[RUDP] Subscriber "<<topicName_<<" unmatched" << std::endl;
            }
            else
            {
                std::cout << info.current_count_change << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

        int matched_;
        std::string topicName_;
        int mode_;

        JoyMsg joy_;
        PoseMsg pose_;
        ros::NodeHandle np ;

        FILE *fp;
        char result_buf[1024];
        ros::Publisher joy_pub, pose_pub;
};

int main(int argc ,char **argv)
{
    ros::init(argc, argv, "robotJoySub");
    ros::NodeHandle nh;

    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("wan_ip",wan_ip);
    nhPrivate.getParam("port",port);

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Subscriber* subscriber_;

    //CREATE THE PARTICIPANT
    //IP&PORT
    int32_t kind = LOCATOR_KIND_UDPv4;
    Locator_t initial_peer_locator;
    initial_peer_locator.kind = kind;
    IPLocator::setIPv4(initial_peer_locator, wan_ip);
    initial_peer_locator.port = port;
    //std::cout << " subscriber as client"<<wan_ip << ":" << port << std::endl;
    DomainParticipantQos pqos;
    pqos.name("Participant_sub");

    // Configure the current participant as CLIENT
    std::cout << " robotJoySub ros node as subscriber  "<<wan_ip << ":" << port << std::endl;
    pqos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(initial_peer_locator);
    //pqos.wire_protocol().builtin.discovery_config.discoveryServer_client_syncperiod =eprosima::fastrtps::Duration_t(0, 250000000);
    //pqos.wire_protocol().builtin.discovery_config.discoveryProtocol = eprosima::fastrtps::rtps::DiscoveryProtocol::CLIENT;
    //eprosima::fastdds::rtps::RemoteServerAttributes remote_server_attr;
    //remote_server_attr.metatrafficUnicastLocatorList.push_back(initial_peer_locator);
    // Set the GUID prefix to identify the remote server
    //remote_server_attr.ReadguidPrefix("72.61.73.70.66.61.72.6d.74.65.73.74");
    // Connect to the SERVER at the previous locator
    //pqos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr);
    pqos.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
    pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod = eprosima::fastrtps::Duration_t(5, 0);
    
    // UDP
    auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
    udp_transport->sendBufferSize = 1048576;
    udp_transport->receiveBufferSize = 4194304;
    udp_transport->non_blocking_send = true;
    pqos.transport().user_transports.push_back(udp_transport);
    pqos.transport().use_builtin_transports = false;
    pqos.transport().send_socket_buffer_size = 12582912;
    pqos.transport().listen_socket_buffer_size = 12582912;
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (participant_ == nullptr)
    {
        std::cout<<"Create participant failed!"<<std::endl;
        return 0;
    }

    //CREATE THE SUBSCRIBER
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    if (subscriber_ == nullptr)
    {
        std::cout<<"Creat subscriber faild"<<std::endl;
        return 0;
    }

    //CREATE THE TOPIC
    //REGISTER THE TYPE
    eprosima::fastdds::dds::TypeSupport type_joy(new JoyMsgPubSubType());
    eprosima::fastdds::dds::TypeSupport type_pose(new PoseMsgPubSubType());
    type_joy.register_type(participant_);
    type_pose.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_joy;
    tqos_joy.transport_priority().value=32;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic* topic_joy = participant_->create_topic(dds_joy_topic_name, type_joy.get_type_name(), tqos_joy);
    eprosima::fastdds::dds::Topic* topic_pose = participant_->create_topic(dds_pose_topic_name, type_pose.get_type_name(), tqos_joy);

    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener* listener_joy= new SubListener(0,nh,dds_joy_topic_name);
    SubListener* listener_pose= new SubListener(1,nh,dds_pose_topic_name);
    //READERQOS
    DataReaderQos rqos_joy;
    rqos_joy.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_joy.history().depth = 1;
    rqos_joy.resource_limits().max_samples = 50;
    rqos_joy.resource_limits().allocated_samples = 20;
    rqos_joy.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_joy.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_joy = subscriber_->create_datareader(topic_joy, rqos_joy, listener_joy);
    eprosima::fastdds::dds::DataReader* reader_pose = subscriber_->create_datareader(topic_pose, rqos_joy, listener_pose);
    if(reader_joy == nullptr || reader_pose == nullptr )
        std::cout<<"creat topic "<<"joy or pose"<<" failed"<<std::endl;
    else
        std::cout<<"creat topic "<<"joy and pose"<<" success"<<std::endl;
    while(ros::ok());
    return 0;
}
