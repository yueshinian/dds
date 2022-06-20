//ros lib
#include <ros/ros.h>
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
#include "CmdMsgPubSubTypes.h"
#include "CmdMsg.h"
//namespace
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;
//topic name
#define dds_cmd_topic_name "CmdTopic"

std::string wan_ip="192.168.1.172";
int port=56470;

class SubListener : public eprosima::fastdds::dds::DataReaderListener
{
public:
        SubListener(int mode , ros::NodeHandle nh, std::string topicName)
            : mode_(mode)
            , matched_(0)
            , topicName_(topicName)
        {
            this->np=nh;
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
                    if (reader->take_next_sample(&cmd_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            // Print your structure data here.
                            const char *sysCommand= cmd_.str().data();
                            fp=popen(sysCommand,"r");
                            if(fp==NULL)
                                std::cout<<"popen failed!"<<std::endl;
                            std::cout<<"*******************************"<<std::endl;
                            while(fgets(result_buf,sizeof(result_buf),fp)!=NULL)
                            {
                                if(result_buf[strlen(result_buf)-1]='\n')
                                    result_buf[strlen(result_buf)-1]='\0';
                                
                                std::cout<<result_buf<<std::endl;
                            }
                            std::cout << "[RUDP] Message ->> '" << cmd_.str() << "' <<- " << cmd_.index() << " RECEIVED" << std::endl;
                            std::cout<<"*******************************"<<std::endl;
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

        CmdMsg cmd_;
        ros::NodeHandle np ;

        FILE *fp;
        char result_buf[1024];
};

int main(int argc ,char **argv)
{
    ros::init(argc, argv, "robotCmdSub");
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
    std::cout << " robotCmdSub ros node as subscriber  "<<wan_ip << ":" << port << std::endl;
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
    eprosima::fastdds::dds::TypeSupport type_cmd(new CmdMsgPubSubType());
    type_cmd.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_cmd;
    tqos_cmd.transport_priority().value=32;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic* topic_cmd = participant_->create_topic(dds_cmd_topic_name, type_cmd.get_type_name(), tqos_cmd);

    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener* listener_cmd= new SubListener(0,nh,dds_cmd_topic_name);
    //READERQOS
    DataReaderQos rqos_cmd;
    rqos_cmd.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_cmd.history().depth = 1;
    rqos_cmd.resource_limits().max_samples = 50;
    rqos_cmd.resource_limits().allocated_samples = 20;
    rqos_cmd.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_cmd.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_cmd = subscriber_->create_datareader(topic_cmd, rqos_cmd, listener_cmd);
    if(reader_cmd == nullptr)
        std::cout<<"creat topic "<<"cmd"<<" failed"<<std::endl;
    else
        std::cout<<"creat topic "<<"cmd"<<" success"<<std::endl;
    while(ros::ok());
    return 0;
}
