//ros lib
#include <ros/ros.h>
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
#include "CmdMsgPubSubTypes.h"
#include "CmdMsg.h"
//user lib
#include "DDSPublisher.h"
#include "optionparser.h"

//wan_ip = "192.168.1.108";
#define WAN_IP  "192.168.1.172"
#define PORT  56470

//define config
#define dds_cmd_topic_name "CmdTopic"

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;

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

int main(int argc,char **argv)
{
    ros::init(argc,argv,"pcCmdPub");
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
    std::cout << " pcCmdPub ros node publisher "<<wan_ip << ":" << port << std::endl;
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
    //REGISTER THE TYPE
    eprosima::fastdds::dds::TypeSupport type_cmd(new CmdMsgPubSubType());
    type_cmd.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_cmd;
    tqos_cmd.transport_priority().value=32;
    tqos_cmd.durability().kind= eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic* topic_cmd = participant_->create_topic(dds_cmd_topic_name, type_cmd.get_type_name(), tqos_cmd);
  
    eprosima::fastdds::dds::DataWriterQos wqos_cmd;
    wqos_cmd.transport_priority().value=32;
    wqos_cmd.history().kind = KEEP_LAST_HISTORY_QOS;
    wqos_cmd.history().depth =1;
    wqos_cmd.resource_limits().max_samples = 50;
    wqos_cmd.resource_limits().allocated_samples = 20;
    wqos_cmd.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
    wqos_cmd.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
    wqos_cmd.reliability().kind = RELIABLE_RELIABILITY_QOS;
    wqos_cmd.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    
    PubListener *listener=new  PubListener(dds_cmd_topic_name);
    eprosima::fastdds::dds::DataWriter* writer_cmd = publisher_->create_datawriter(topic_cmd, wqos_cmd, listener);

    if(writer_cmd == nullptr)
        std::cout<<"creat topic "<<"cmd"<<" failed"<<std::endl;
    else
        std::cout<<"creat topic "<<"cmd"<<" success"<<std::endl;

    CmdMsg cmd_;
    std::string  str;
    while(ros::ok())
    {
    	std::cout<< "input " << cmd_.index() + 1 <<" cmd"<<std::endl;
    	std::getline(std::cin,str);
        cmd_.index(cmd_.index() + 1);
        cmd_.str(str);
        writer_cmd->write((void*)&cmd_);
        std::cout << "[RUDP] Message: " << cmd_.str() << " with index: "
                          << cmd_.index() << " SENT" << std::endl;
        std::cout<<"*******************************"<<std::endl;
    }
  
  return 0;
}
