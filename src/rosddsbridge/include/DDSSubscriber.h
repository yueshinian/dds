#ifndef DDSSUBSCRIBER_H_
#define DDSSUBSCRIBER_H_

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
#include <iostream>
#include <vector>
#include <string.h>
#include <cstring>
#include <map>
#include <boost/thread/mutex.hpp>
//namespace
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;
//topic name
#define ros_pub_PointCloud2_topic   "registered_scan"
#define ros_pub_Octomap_topic   "receive_octomap"
#define ros_pub_Twist_topic "reveive_twist"
#define ros_pub_Image_topic "receive_image"
#define ros_pub_Tf_topic    "receive_tf"
#define ros_pub_Marker_topic "receive_marker"
#define dds_sub_PointCloud2_topic   "PclTopic"
#define dds_sub_Octomap_topic   "OctomapTopic"
#define dds_sub_Twist_topic "TwistTopic"
#define dds_sub_Image_topic "ImageTopic"
#define dds_sub_Tf_topic    "TfTopic"
#define dds_sub_Marker_topic "MarkerTopic"
//wan_ip = "192.168.1.108"
#define WAN_IP  "127.0.0.1" 
#define PORT 56452
#define multiComm 0
#define SAVEIMAGE 0

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

class DDSSubscriber
{
public:
    virtual ~DDSSubscriber();
    //!Initialize
    bool init(const std::string& wan_ip,unsigned short port,bool use_tls,const std::vector<std::string>& whitelist,unsigned int domainid);
    
    bool createReader(std::string topicName, const eprosima::fastdds::dds::DataReaderQos& qos_ops, const eprosima::fastdds::dds::TypeSupport data_type, const int transport_priority, int mode,ros::NodeHandle nh);
    //bool createWriter(std::string topicName, const eprosima::fastdds::dds::DataWriterQos& qos_ops, const eprosima::fastdds::dds::TypeSupport data_type);
    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Publisher* publisher_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::TopicQos  tqos;

    std::map<std::string, eprosima::fastdds::dds::Topic*> topic_map;
    //std::map<std::string, eprosima::fastdds::dds::DataWriter*> writer_map;
    std::map<std::string, eprosima::fastdds::dds::DataReader*> reader_map;

    boost::mutex topic_map_mutex;
    boost::mutex writer_map_mutex;
    boost::mutex reader_map_mutex;

    eprosima::fastdds::dds::Topic* getTopic(const std::string& topicName , const eprosima::fastdds::dds::DataReaderQos& qos, const eprosima::fastdds::dds::TypeSupport data_type, const int transport_priority);
    //eprosima::fastdds::dds::DataWriter* getWriter(std::string topicName);
    eprosima::fastdds::dds::DataReader* getReader(std::string topicName);

    DDSSubscriber(): participant_(nullptr), publisher_(nullptr), subscriber_(nullptr)
    {
    }

    class SubListener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:

        //SubListener(std::string topicName, int mode ): topicName_(topicName),mode_(mode), matched_(0), connected_(false){ }
        SubListener() {}
        ~SubListener() {}
/*
        //void on_subscription_matched(eprosima::fastdds::dds::DataReader* reader, const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)  override;

        int matched_;
        bool connected_;
        std::string topicName_;
        int mode_;

        TwistMsg twist_;
        void on_subscription_matched(
                eprosima::fastdds::dds::DataReader* reader,
                const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) 
        {
            std::cout << "[RTCP] Waiting Subscriber matched" << std::endl;
            if (info.current_count_change == 1)
            {
                matched_ = info.total_count;
                std::cout << "[RTCP] Subscriber matched" << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                matched_ = info.total_count;
                std::cout << "[RTCP] Subscriber unmatched" << std::endl;
            }
            else
            {
                std::cout << info.current_count_change << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }
*/
    };   
    std::map<std::string, SubListener*> listener_map;
    SubListener* getListener(std::string topicName);
};

#endif /* DDSPUBLISHER_H_ */
