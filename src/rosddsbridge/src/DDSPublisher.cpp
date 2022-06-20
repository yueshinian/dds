// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file DDSPublisher.cpp
 *
 */
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
#include "DDSPublisher.h"
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

DDSPublisher::DDSPublisher()
    : participant_(nullptr)
    , publisher_(nullptr)
    , subscriber_(nullptr)
{

}

DDSPublisher::~DDSPublisher()
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

bool DDSPublisher::init(
        const std::string& wan_ip,
        unsigned short port,
        unsigned int domainid,
        int udpType
        )
{
    DomainParticipantQos pqos;
    pqos.name("Participant_pub");
    // UDP
    auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
    pqos.transport().user_transports.push_back(udp_transport);
    pqos.transport().use_builtin_transports = false;
    pqos.transport().send_socket_buffer_size = 1048576;
    pqos.transport().listen_socket_buffer_size =  4194304;

    participant_ = DomainParticipantFactory::get_instance()->create_participant(domainid, pqos);
    if (participant_ == nullptr)
    {
        std::cout<<"Creat participant faild"<<std::endl;
    }

    //CREATE THE PUBLISHER
    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
    if (publisher_ == nullptr)
    {
        return false;
    }
    return true;
}

eprosima::fastdds::dds::Topic* DDSPublisher::getTopic(const std::string& topicName , const eprosima::fastdds::dds::DataWriterQos& qos,const eprosima::fastdds::dds::TypeSupport data_type)
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
      topic_qos.transport_priority().value=qos.transport_priority().value;
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

bool DDSPublisher::createWriter(std::string topicName, const eprosima::fastdds::dds::DataWriterQos& qos_ops,const eprosima::fastdds::dds::TypeSupport data_type)
{
  eprosima::fastdds::dds::Topic* topic = getTopic(topicName,qos_ops,data_type);
  if(topic==NULL)
    std::cout<<"when create writer, get topic failed"<<std::endl;

  eprosima::fastdds::dds::DataWriter* writer;
  std::map<std::string, eprosima::fastdds::dds::DataWriter*>::iterator iterWriterMap;

  iterWriterMap = writer_map.find(topicName);
  if (iterWriterMap != writer_map.end())
  {
    writer = iterWriterMap->second;
    return writer;
  }

  {
    boost::mutex::scoped_lock lock(writer_map_mutex);
    iterWriterMap = writer_map.find(topicName);
    if (iterWriterMap != writer_map.end())
      writer = iterWriterMap->second;
    else
    {
      eprosima::fastdds::dds::DataWriterQos wqos;
      DDSPublisher::PubListener *listener=new  DDSPublisher::PubListener(topicName);
      writer = publisher_->create_datawriter(topic, qos_ops, listener);

      if(writer==NULL)
      {
        std::cout<<"create "<<topicName<<" writer filed!"<<std::endl;
        return false;
      }
      writer_map.insert(std::pair<std::string, eprosima::fastdds::dds::DataWriter*>(topicName, writer));
      listener_map.insert(std::pair<std::string, PubListener*>(topicName, listener));
    }
  }
  std::cout<<"Create Topic "<<topicName<<" succeed!"<<std::endl;
  return true;
}

eprosima::fastdds::dds::DataWriter* DDSPublisher::getWriter(std::string topicName)
{
  eprosima::fastdds::dds::DataWriter* writer;
  std::map<std::string, eprosima::fastdds::dds::DataWriter*>::iterator iterWriterMap;

  iterWriterMap = writer_map.find(topicName);
  if (iterWriterMap != writer_map.end())
  {
    writer = iterWriterMap->second;
    return writer;
  }
  else
    return NULL;
}

DDSPublisher::PubListener* DDSPublisher::getListener(std::string topicName)
{
  PubListener* listener;
  std::map<std::string, PubListener*>::iterator iterListenerMap;

  iterListenerMap = listener_map.find(topicName);
  if (iterListenerMap != listener_map.end())
  {
    listener = iterListenerMap->second;
    return listener;
  }
  else
    return NULL;
}

void DDSPublisher::PubListener::on_publication_matched(eprosima::fastdds::dds::DataWriter*,const eprosima::fastdds::dds::PublicationMatchedStatus& info)
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

