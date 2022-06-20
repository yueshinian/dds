//C++ lib
#include <vector>
#include <string.h>
#include <sstream>
#include <memory>
#include <mutex>
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
#include <nav_msgs/Odometry.h>
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
#include "OdometryMsgPubSubTypes.h"
#include "OctomapMsg.h"
#include "TwistMsg.h"
#include "TfMsg.h"
#include "ImageMsg.h"
#include "PclMsg.h"
#include "TfMsgBase.h"
#include "MarkerMsg.h"
#include "OdometryMsg.h"
//C++ lib
#include <vector>
#include <string>
//namespace
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;
using namespace std;
//topic name
#define PORT 56452
std::string wan_ip ="127.0.0.1";
int port =56452;
std::string ros_pub_Odom_topic = "/receive_odom";
std::string dds_sub_Odom_topic=   "OdometryTopic";
#define multiComm 0
#define SAVEIMAGE 1
#define LOG 1
class SubListener : public eprosima::fastdds::dds::DataReaderListener
{
public:

        SubListener(int mode , ros::NodeHandle nh, std::string dds_topicName, std::string ros_topicName)
            : mode_(mode)
            , matched_(0)
            , dds_topicName_(dds_topicName)
            , ros_topicName_(ros_topicName)
        {
            this->np=nh;
            odom_pub=np.advertise<nav_msgs::Odometry>(ros_topicName,10);
        }

        ~SubListener() 
        {
        }

        void on_data_available(eprosima::fastdds::dds::DataReader* reader) 
        {
            SampleInfo info;
            
                    if (reader->take_next_sample(&odom_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            if(LOG) std::cout << "[RUDP-Odom] Seq: " << odom_.seq() << " with time: "<<odom_.secs() << " RECEIVED" << std::endl;
                            nav_msgs::Odometry odom;
                            odom.header.seq=odom_.seq();
                            odom.header.stamp.sec=odom_.secs();
                            odom.header.stamp.nsec=odom_.nsecs();
                            odom.header.frame_id=odom_.frame_id();
                            odom.child_frame_id = odom_.child_frame_id();
                            odom.pose.pose.position.x=odom_.position_x();
                            odom.pose.pose.position.y=odom_.position_y();
                            odom.pose.pose.position.z=odom_.position_z();
                            odom.pose.pose.orientation.x=odom_.orientation_x();
                            odom.pose.pose.orientation.y=odom_.orientation_y();
                            odom.pose.pose.orientation.z=odom_.orientation_z();
                            odom.pose.pose.orientation.w=odom_.orientation_w();
                            odom_pub.publish(odom);
                            static tf::TransformBroadcaster br;
                            tf::Transform transform;
                            transform.setOrigin( tf::Vector3(odom_.position_x(), odom_.position_y(), odom_.position_z()) );
                            tf::Quaternion q(odom_.orientation_x(),odom_.orientation_y(),odom_.orientation_z(),odom_.orientation_w());
                            transform.setRotation(q);
                            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "receive_base_link"));
                            if(LOG) ROS_INFO("pubodom");
                        }
                    }
        }

        void on_subscription_matched(eprosima::fastdds::dds::DataReader* reader, const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) 
        {
            if (info.current_count_change == 1)
            {
                matched_++;
                std::cout << "[RTCP] Subscriber "<<dds_topicName_<<" "<<matched_<<" matched" << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "[RTCP] Subscriber "<<dds_topicName_<<"unmatched" << std::endl;
            }
            else
            {
                std::cout << info.current_count_change << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

        int matched_;
        std::string dds_topicName_;
        std::string ros_topicName_;
        int mode_;
        OdometryMsg odom_;
        ros::NodeHandle np ;
        ros::Publisher odom_pub;
};

int main(int argc ,char **argv)
{
    ros::init(argc, argv, "pcOdomSub");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("wan_ip",wan_ip);
    nhPrivate.getParam("port",port);
    nhPrivate.getParam("ros_pub_Odom_topic",ros_pub_Odom_topic);
    nhPrivate.getParam("dds_sub_Odom_topic",dds_sub_Odom_topic);

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
   
    DomainParticipantQos pqos;
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
        std::cout<<"Create participant failed!"<<std::endl;
        return 0;
    }
    else
        std::cout<<"init  dds succeed!!"<<std::endl;
    //CREATE THE SUBSCRIBER
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    if (subscriber_ == nullptr)
    {
        std::cout<<"Creat subscriber faild"<<std::endl;
        return 0;
    }

    //CREATE THE TOPIC
    //REGISTER THE TYPE
    eprosima::fastdds::dds::TypeSupport type_odom(new OdometryMsgPubSubType());
    type_odom.register_type(participant_);
    //TOPIC QOS
    TopicQos  tqos_odom;
    tqos_odom.transport_priority().value=22;
    //CREATE THE TOPIC
     eprosima::fastdds::dds::Topic* topic_odom = participant_->create_topic(dds_sub_Odom_topic, type_odom.get_type_name(), tqos_odom);

    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener* listener_odom =new SubListener(6,nh, dds_sub_Odom_topic, ros_pub_Odom_topic);
    //READERQOS
    DataReaderQos rqos_odom;
    rqos_odom.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_odom.history().depth = 1;
    rqos_odom.resource_limits().max_samples = 50;
    rqos_odom.resource_limits().allocated_samples = 20;
    rqos_odom.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_odom.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_odom = subscriber_->create_datareader(topic_odom, rqos_odom, listener_odom);
    if(reader_odom == nullptr)
    	std::cout<<"creat topic "<<dds_sub_Odom_topic<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<dds_sub_Odom_topic<<" success"<<std::endl;

    ros::spin();
    return 0;
}
