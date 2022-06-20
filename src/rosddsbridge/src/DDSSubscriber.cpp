//dds lib
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/utils/IPLocator.h>
//#include <fastrtps/Domain.h>
/*
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/transport/TCPv6TransportDescriptor.h>
#include <fastrtps/transport/UDPv6TransportDescriptor.h>
#include <fastrtps/Domain.h>
#include <fastrtps/utils/IPLocator.h>
*/
#include "DDSSubscriber.h"
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/Domain.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastdds/rtps/flowcontrol/ThroughputControllerDescriptor.h>
//ros lib
#include <thread>
#include <stdio.h>
#include "ros/ros.h"

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;

DDSSubscriber::~DDSSubscriber()
{
    if (subscriber_ != nullptr)
    {
        participant_->delete_subscriber(subscriber_);
    }
    if (publisher_ != nullptr)
    {
        participant_->delete_publisher(publisher_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

bool DDSSubscriber::init(
        const std::string& wan_ip,
        unsigned short port,
        bool use_tls,
        const std::vector<std::string>& whitelist,
        unsigned int domainid
        )
{
    //CREATE THE PARTICIPANT
    DomainParticipantQos pqos;

    int32_t kind = LOCATOR_KIND_UDPv4;
    Locator_t initial_peer_locator;
    initial_peer_locator.kind = kind;
    IPLocator::setIPv4(initial_peer_locator, wan_ip);
    initial_peer_locator.port = port;
    std::cout << " subscriber as server "<<wan_ip << ":" << port << std::endl;
    
    pqos.wire_protocol().builtin.discovery_config.discoveryServer_client_syncperiod =eprosima::fastrtps::Duration_t(0, 250000000);
    pqos.wire_protocol().builtin.discovery_config.discoveryProtocol = eprosima::fastrtps::rtps::DiscoveryProtocol::SERVER;
    pqos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(initial_peer_locator);
    std::istringstream("72.61.73.70.66.61.72.6d.74.65.73.74") >> pqos.wire_protocol().prefix;
    pqos.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
    pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod =eprosima::fastrtps::Duration_t(5, 0);
    pqos.name("Participant_sub");

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
        return false;
    }
    return true;
}

eprosima::fastdds::dds::Topic* DDSSubscriber::getTopic(const std::string& topicName , const eprosima::fastdds::dds::DataReaderQos& qos, const eprosima::fastdds::dds::TypeSupport data_type, const int transport_priority)
{
    eprosima::fastdds::dds::Topic* topic;
    std::map<std::string, eprosima::fastdds::dds::Topic*>::iterator iterTopicMap;

  iterTopicMap = topic_map.find(topicName);
  if (iterTopicMap != topic_map.end())
  {
    topic = iterTopicMap->second;
    return topic;
  }

  {
    boost::mutex::scoped_lock lock(topic_map_mutex);

    iterTopicMap = topic_map.find(topicName);
    if (iterTopicMap != topic_map.end())
      topic = iterTopicMap->second;
    else
    {
      data_type.register_type(participant_, data_type.get_type_name());
      eprosima::fastdds::dds::TopicQos topic_qos;
      topic_qos.transport_priority().value=transport_priority;
      topic_qos.durability().kind=qos.durability().kind;
      topic = participant_->create_topic(topicName, data_type.get_type_name(), topic_qos);
      if(topic==NULL)
        std::cout<<"Failed to creat topic "<<topicName<<std::endl;
      //insert it into the map
      topic_map.insert(std::pair<std::string, eprosima::fastdds::dds::Topic*>(topicName, topic));
    }
  }

  return topic;
}

bool DDSSubscriber::createReader(std::string topicName, const eprosima::fastdds::dds::DataReaderQos& qos_ops, const eprosima::fastdds::dds::TypeSupport data_type, const int transport_priority, int mode, ros::NodeHandle nh)
{
  //eprosima::fastdds::dds::Topic* topic = getTopic(topicName,qos_ops,data_type,transport_priority);
  eprosima::fastdds::dds::TopicQos topic_qos;
  data_type.register_type(participant_, data_type.get_type_name());
  eprosima::fastdds::dds::Topic* topic=participant_->create_topic(topicName, data_type.get_type_name(), topic_qos);
  if(topic==NULL)
    std::cout<<"when create reader, get topic failed"<<std::endl;

  eprosima::fastdds::dds::DataReader* reader;
  std::map<std::string, eprosima::fastdds::dds::DataReader*>::iterator iterReaderMap;

  iterReaderMap = reader_map.find(topicName);
  if (iterReaderMap != reader_map.end())
  {
    reader = iterReaderMap->second;
    return reader;
  }

  {
    boost::mutex::scoped_lock lock(reader_map_mutex);
    iterReaderMap = reader_map.find(topicName);
    if (iterReaderMap != reader_map.end())
      reader = iterReaderMap->second;
    else
    {
      eprosima::fastdds::dds::DataReaderQos rqos;
      DDSSubscriber::SubListener *listener=new  DDSSubscriber::SubListener();
std::cout<<"init  dds failed!"<<std::endl;
      reader = subscriber_->create_datareader(topic, rqos, listener);
std::cout<<"init  dds failed!"<<std::endl;
      if(reader==NULL)
      {
        std::cout<<"create "<<topicName<<" reader filed!"<<std::endl;
        return false;
      }
std::cout<<"init  dds failed!"<<std::endl;
      reader_map.insert(std::pair<std::string, eprosima::fastdds::dds::DataReader*>(topicName, reader));
      listener_map.insert(std::pair<std::string, SubListener*>(topicName, listener));
std::cout<<"init  dds failed!"<<std::endl;
    }
  }
  std::cout<<"Create Topic "<<topicName<<" succeed!"<<std::endl;
  return true;
}

eprosima::fastdds::dds::DataReader* DDSSubscriber::getReader(std::string topicName)
{
  eprosima::fastdds::dds::DataReader* reader;
  std::map<std::string, eprosima::fastdds::dds::DataReader*>::iterator iterReaderMap;

  iterReaderMap = reader_map.find(topicName);
  if (iterReaderMap != reader_map.end())
  {
    reader = iterReaderMap->second;
    return reader;
  }
  else
    return NULL;
}

DDSSubscriber::SubListener* DDSSubscriber::getListener(std::string topicName)
{
  SubListener* listener;
  std::map<std::string, SubListener*>::iterator iterListenerMap;

  iterListenerMap = listener_map.find(topicName);
  if (iterListenerMap != listener_map.end())
  {
    listener = iterListenerMap->second;
    return listener;
  }
  else
    return NULL;
}

/*
void DDSSubscriber::SubListener::on_subscription_matched(eprosima::fastdds::dds::DataReader* reader, const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        matched_ ++;
        connected_ = true;
        std::cout << "[RUDP] Sublisher ["<<this->topicName_<<"] ["<<matched_<<"st] matched!" << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        connected_=false;
        std::cout << "[RUDP] Sublisher ["<<this->topicName_<<"] unmatched" << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SublicationMatchedStatus current count change" << std::endl;
    }
}
*/



