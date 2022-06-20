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
 * @file DDSPublisher.h
 *
 */

#ifndef DDSPUBLISHER_H_
#define DDSPUBLISHER_H_

//dds lib
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
#include "VelodyneWithOdometryMsgPubSubTypes.h"
#include "OctomapMsg.h"
#include "TwistMsg.h"
#include "TfMsg.h"
#include "ImageMsg.h"
#include "PclMsg.h"
#include "TfMsgBase.h"
#include "MarkerMsg.h"
#include "OdometryMsg.h"
#include "VelodyneWithOdometryMsg.h"
//C++ lib
#include <vector>
#include <string.h>
#include <cstring>
#include <map>
#include <boost/thread/mutex.hpp>

#define LOG 1

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

class DDSPublisher
{
    class PubListener : public eprosima::fastdds::dds::DataWriterListener
    {
    public:

        PubListener(std::string topicName): topicName_(topicName), matched_(0), connected_(false){ }

        PubListener(): matched_(0), connected_(false){}

        ~PubListener() override{}

        void on_publication_matched(
                eprosima::fastdds::dds::DataWriter* writer,
                const eprosima::fastdds::dds::PublicationMatchedStatus& info ) override;

        int matched_;

        bool connected_;

        std::string topicName_;
    } ;

    
    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Publisher* publisher_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::TopicQos  tqos;

    std::map<std::string, eprosima::fastdds::dds::Topic*> topic_map;
    std::map<std::string, eprosima::fastdds::dds::DataWriter*> writer_map;
    std::map<std::string, eprosima::fastdds::dds::DataReader*> reader_map;
    std::map<std::string, PubListener*> listener_map;

    boost::mutex topic_map_mutex;
    boost::mutex writer_map_mutex;
    boost::mutex reader_map_mutex;

    eprosima::fastdds::dds::Topic* getTopic(const std::string& topicName , const eprosima::fastdds::dds::DataWriterQos& qos, const eprosima::fastdds::dds::TypeSupport data_type);
    eprosima::fastdds::dds::DataReader* getReader(std::string topicName);
public:
    DDSPublisher();
    virtual ~DDSPublisher();
    //!Initialize
    bool init(const std::string& wan_ip,unsigned short port, unsigned int domainid, int udpType);
    
    eprosima::fastdds::dds::DataWriter* getWriter(std::string topicName);    
    PubListener* getListener(std::string topicName);
    bool createReader(std::string topicName, const eprosima::fastdds::dds::DataReaderQos& qos_ops, int mode);
    bool createWriter(std::string topicName, const eprosima::fastdds::dds::DataWriterQos& qos_ops,const eprosima::fastdds::dds::TypeSupport data_type);
    
    //!Publish a sample
    PclMsg pcl_;
    OctomapMsg octomap_;
    TfMsg tf_;
    ImageMsg image_;
    TwistMsg twist_;
    MarkerMsg marker_;
    OdometryMsg odom_;
    VelodyneWithOdometryMsg velodyneWithOdometry_;
    bool publish( std::string topicName, bool waitForListener,int choice)
    {
        eprosima::fastdds::dds::DataWriter* writer_ = getWriter(topicName);
        PubListener* listener_ = getListener(topicName);
        if (!waitForListener  || listener_->connected_  )
        {
            switch(choice)
            {
                default:
                case 0:
                    writer_->write((void*)&twist_);
                    if(LOG) std::cout << "[RUDP-Twist] linear_x: " << twist_.linear_x() << " with angular_z: "<< twist_.angular_z() << " SENT" << std::endl;
                    break;
                case 1:
                    writer_->write((void*)&octomap_);
                    if(LOG)  std::cout << "[RUDP-Octomap] Seq: " << octomap_.seq() << " with time: "<< octomap_.secs() << " data size "<<octomap_.datacount() << " SENT" << std::endl;
                    break;
                case 2:
                    writer_->write((void*)&tf_);
                    if(LOG)  std::cout << "[RUDP-Tf] Seq: " << tf_.tf_sequence()[0].seq() << " with time: "<< tf_.tf_sequence()[0].secs() << " SENT" << std::endl;
                    break;
                case 3:
                    writer_->write((void*)&image_);
                    if(LOG)  std::cout << "[RUDP-Image] Seq: " << image_.seq() << " with time: "<< image_.secs() << " SENT" << std::endl;
                    break;
                case 4:
                    writer_->write((void*)&pcl_);
                    if(LOG)  std::cout << "[RUDP-Pcl] Seq: " << pcl_.seq() << " with time: "<< pcl_.secs() << " data size "<<pcl_.datacount() << " SENT" << std::endl;
                    break;
                case 5:
                    writer_->write((void*)&marker_);
                    if(LOG)  std::cout << "[RUDP-Marker] Seq: " << marker_.seq() << " with time: "<< marker_.secs() << " SENT" << std::endl;
                    break;
                case 6:
                    writer_->write((void*)&odom_);
                    if(LOG)  std::cout << "[RUDP-Odom] Seq: " << odom_.seq() << " with time: "<< odom_.secs() << " SENT" << std::endl;
                    break;
                case 7:
                    writer_->write((void*)&velodyneWithOdometry_);
                    if(LOG) std::cout<<"[RUDP-VelodyneWithOdom] Seq: "<<velodyneWithOdometry_.pcl().seq()
                                                    <<" with time: "<< velodyneWithOdometry_.pcl().secs() << " SENT" << std::endl;
                    break;
            }
            return true;
        }
        return false;
    }
};

#endif /* DDSPUBLISHER_H_ */
