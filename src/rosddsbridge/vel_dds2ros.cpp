//C++ lib
#include <vector>
#include <string.h>
#include <sstream>
//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
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
//C++ lib
#include <vector>
#include <string>
//namespace
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;
//topic name

std::string ros_pub_Twist_topic= "reveive_twist";
std::string dds_sub_Twist_topic= "TwistTopic";
std::string wan_ip="127.0.0.1";
int port=56460;

class SubListener : public eprosima::fastdds::dds::DataReaderListener
{
public:

        SubListener(int mode , ros::NodeHandle nh, std::string topicName)
            : mode_(mode)
            , matched_(0)
            , topicName_(topicName)
        {
            this->np=nh;
            twist_pub=np.advertise<geometry_msgs::Twist>(ros_pub_Twist_topic,10);
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
                    if (reader->take_next_sample(&twist_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            geometry_msgs::Twist vel;
                            vel.linear.x=twist_.linear_x();
                            vel.linear.y=twist_.linear_y();
                            vel.linear.y=twist_.linear_y();
                            vel.angular.x=twist_.angular_x();
                            vel.angular.y=twist_.angular_y();
                            vel.angular.z=twist_.angular_z();
                            std::cout << "[RUDP-Twist] linear_x: " << twist_.linear_x() << " with angular_z: "<< twist_.angular_z() << " RECEIVED" << std::endl;
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
                std::cout << "[RTCP] Subscriber "<<topicName_<<" "<<matched_<<" matched" << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "[RTCP] Subscriber "<<topicName_<<"unmatched" << std::endl;
            }
            else
            {
                std::cout << info.current_count_change << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

        int matched_;
        std::string topicName_;
        int mode_;

        TwistMsg twist_;
        ros::NodeHandle np ;
        ros::Publisher twist_pub;
};

int main(int argc ,char **argv)
{
    ros::init(argc, argv, "vel_dds2ros");
    ros::NodeHandle nh;

    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("wan_ip",wan_ip);
    nhPrivate.getParam("port",port);
    nhPrivate.getParam("ros_pub_Twist_topic",ros_pub_Twist_topic);
    nhPrivate.getParam("dds_sub_Twist_topic",dds_sub_Twist_topic);

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
    std::cout << " vel_dds2ros node as client "<<wan_ip << ":" << port << std::endl;
    pqos.wire_protocol().builtin.discovery_config.discoveryServer_client_syncperiod =eprosima::fastrtps::Duration_t(0, 250000000);
    pqos.wire_protocol().builtin.discovery_config.discoveryProtocol = eprosima::fastrtps::rtps::DiscoveryProtocol::CLIENT;
    eprosima::fastdds::rtps::RemoteServerAttributes remote_server_attr;
    remote_server_attr.metatrafficUnicastLocatorList.push_back(initial_peer_locator);
    // Set the GUID prefix to identify the remote server
    remote_server_attr.ReadguidPrefix("72.61.73.70.66.61.72.6d.74.65.73.74");
    // Connect to the SERVER at the previous locator
    pqos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr);
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
    eprosima::fastdds::dds::TypeSupport type_twist(new TwistMsgPubSubType());
    type_twist.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_twist;
    tqos_twist.transport_priority().value=32;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic* topic_twist = participant_->create_topic(dds_sub_Twist_topic, type_twist.get_type_name(), tqos_twist);

    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener* listener_twist= new SubListener(0,nh,dds_sub_Twist_topic);
    //READERQOS
    DataReaderQos rqos_twist;
    rqos_twist.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_twist.history().depth = 1;
    rqos_twist.resource_limits().max_samples = 50;
    rqos_twist.resource_limits().allocated_samples = 20;
    rqos_twist.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_twist.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_twist = subscriber_->create_datareader(topic_twist, rqos_twist, listener_twist);
    if(reader_twist == nullptr)
        std::cout<<"creat topic "<<"Twist"<<" failed"<<std::endl;
    else
        std::cout<<"creat topic "<<"Twist"<<" success"<<std::endl;
    std::cin.ignore();
    return 0;
}
