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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
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
#include "MeasureMsgPubSubTypes.h"
#include "OctomapMsg.h"
#include "TwistMsg.h"
#include "TfMsg.h"
#include "ImageMsg.h"
#include "PclMsg.h"
#include "TfMsgBase.h"
#include "MarkerMsg.h"
#include "OdometryMsg.h"
#include "MeasureMsg.h"
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
std::string wan_ip = "127.0.0.1";
int port = 56452;
std::string ros_pub_Measure_topic = "/roomVis";
std::string dds_sub_Measure_topic = "MeasureTopic";
#define multiComm 0
#define SAVEIMAGE 1
#define LOG 1
class SubListener : public eprosima::fastdds::dds::DataReaderListener
{
public:
    SubListener(int mode, ros::NodeHandle nh, std::string dds_topicName, std::string ros_topicName)
        : mode_(mode), matched_(0), dds_topicName_(dds_topicName), ros_topicName_(ros_topicName)
    {
        this->np = nh;
        pubRoomVis = nh.advertise<visualization_msgs::Marker>(ros_topicName, 1);
    }

    ~SubListener()
    {
    }

    void on_data_available(eprosima::fastdds::dds::DataReader *reader)
    {
        SampleInfo info;

        if (reader->take_next_sample(&measure_, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.valid_data)
            {
                if (LOG)
                    std::cout << "[RUDP-Measure] minX: " << measure_.minX() << " with maxX: " << measure_.maxX() << " RECEIVED"
                              << " minY: " << measure_.minY() << " with maxY: " << measure_.maxY()
                              << std::endl;
                visualRoom(measure_.minX(), measure_.minY(), measure_.minZ(),
                           measure_.maxX(), measure_.maxY(), measure_.maxZ());
                if (LOG)
                    ROS_INFO("pub measure room");
            }
        }
    }

    void on_subscription_matched(eprosima::fastdds::dds::DataReader *reader, const eprosima::fastdds::dds::SubscriptionMatchedStatus &info)
    {
        if (info.current_count_change == 1)
        {
            matched_++;
            std::cout << "[RUDP] Subscriber " << dds_topicName_ << " " << matched_ << " matched" << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            std::cout << "[RUDP] Subscriber " << dds_topicName_ << "unmatched" << std::endl;
        }
        else
        {
            std::cout << info.current_count_change << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
        }
    }

    void visualRoom(double minX_, double minY_, double maxX_, double maxY_, double minZ_, double maxZ_)
    {
        visualization_msgs::Marker box;
        box.header.stamp = ros::Time::now();
        box.header.frame_id = "/velodyne";
        box.id = 1;
        box.ns = "boundary";
        box.type = visualization_msgs::Marker::LINE_LIST;
        box.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point box_p1, box_p2, box_p3, box_p4, box_p5, box_p6, box_p7, box_p8;
        box_p1.x = maxX_;
        box_p1.y = maxY_;
        box_p1.z = maxZ_;
        box_p2.x = minX_;
        box_p2.y = maxY_;
        box_p2.z = maxZ_;
        box_p3.x = minX_;
        box_p3.y = minY_;
        box_p3.z = maxZ_;
        box_p4.x = maxX_;
        box_p4.y = minY_;
        box_p4.z = maxZ_;
        box_p5.x = maxX_;
        box_p5.y = maxY_;
        box_p5.z = minZ_;
        box_p6.x = minX_;
        box_p6.y = maxY_;
        box_p6.z = minZ_;
        box_p7.x = minX_;
        box_p7.y = minY_;
        box_p7.z = minZ_;
        box_p8.x = maxX_;
        box_p8.y = minY_;
        box_p8.z = minZ_;

        box.scale.x = 0.1; //line width
        box.color.r = 0.0;
        box.color.g = 0.0;
        box.color.b = 255.0 / 255.0;

        box.color.a = 1;
        box.points.push_back(box_p1);
        box.points.push_back(box_p2);
        box.points.push_back(box_p2);
        box.points.push_back(box_p3);
        box.points.push_back(box_p3);
        box.points.push_back(box_p4);
        box.points.push_back(box_p4);
        box.points.push_back(box_p1);
        box.points.push_back(box_p1);
        box.points.push_back(box_p5);
        box.points.push_back(box_p2);
        box.points.push_back(box_p6);
        box.points.push_back(box_p3);
        box.points.push_back(box_p7);
        box.points.push_back(box_p4);
        box.points.push_back(box_p8);
        box.points.push_back(box_p5);
        box.points.push_back(box_p6);
        box.points.push_back(box_p6);
        box.points.push_back(box_p7);
        box.points.push_back(box_p7);
        box.points.push_back(box_p8);
        box.points.push_back(box_p8);
        box.points.push_back(box_p5);
        tf::Quaternion quat2;
        quat2.setEuler(0.0, 0.0, 0.0);
        box.pose.orientation.x = quat2.x();
        box.pose.orientation.y = quat2.y();
        box.pose.orientation.z = quat2.z();
        box.pose.orientation.w = quat2.w();
        box.lifetime = ros::Duration(0.0);
        box.frame_locked = false;
        pubRoomVis.publish(box);

        visualization_msgs::Marker textX;
        textX.header.frame_id = "/velodyne";
        textX.header.stamp = ros::Time::now();
        textX.ns = "textX";
        textX.action = visualization_msgs::Marker::ADD;
        textX.pose.orientation.w = 1.0;
        textX.id = 0;
        textX.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textX.scale.z = 1.0;
        textX.color.b = 0;
        textX.color.g = 0;
        textX.color.r = 1;
        textX.color.a = 1;
        textX.pose.position.x = (maxX_ + minX_) / 2;
        textX.pose.position.y = maxY_ - 1;
        textX.pose.position.z = maxZ_;
        textX.text = "SIZE_X: " + to_string(abs(maxX_) + abs(minX_)) + "=" + to_string(abs(minX_)) + "+" + to_string(abs(maxX_));
        pubRoomVis.publish(textX);

        visualization_msgs::Marker textY;
        textY.header.frame_id = "/velodyne";
        textY.header.stamp = ros::Time::now();
        textY.ns = "textY";
        textY.action = visualization_msgs::Marker::ADD;
        textY.pose.orientation.w = 1.0;
        textY.id = 0;
        textY.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textY.scale.z = 1;
        textY.color.b = 0;
        textY.color.g = 0;
        textY.color.r = 1;
        textY.color.a = 1;
        textY.pose.position.x = maxX_ - 2;
        textY.pose.position.y = (maxY_ + minY_) / 2;
        textY.pose.position.z = maxZ_;
        textY.text = "SIZE_Y: " + to_string(abs(maxY_) + abs(minY_)) + "=" + to_string(abs(minY_)) + "+" + to_string(abs(maxY_));
        pubRoomVis.publish(textY);
    }

    int matched_;
    std::string dds_topicName_;
    std::string ros_topicName_;
    int mode_;

    MeasureMsg measure_;
    ros::NodeHandle np;
    ros::Publisher pubRoomVis;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcMeasureSub");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("wan_ip", wan_ip);
    nhPrivate.getParam("port", port);
    nhPrivate.getParam("ros_pub_Measure_topic", ros_pub_Measure_topic);
    nhPrivate.getParam("dds_sub_Measure_topic", dds_sub_Measure_topic);

    eprosima::fastdds::dds::DomainParticipant *participant_;
    eprosima::fastdds::dds::Subscriber *subscriber_;

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
        std::cout << "Create participant failed!" << std::endl;
        return 0;
    }
    else
        std::cout << "init  dds succeed!!" << std::endl;
    //CREATE THE SUBSCRIBER
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    if (subscriber_ == nullptr)
    {
        std::cout << "Creat subscriber faild" << std::endl;
        return 0;
    }

    //CREATE THE TOPIC
    //REGISTER THE TYPE
    eprosima::fastdds::dds::TypeSupport type_pcl(new MeasureMsgPubSubType());
    type_pcl.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_pcl;
    tqos_pcl.transport_priority().value = 32;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic *topic_pcl = participant_->create_topic(dds_sub_Measure_topic, type_pcl.get_type_name(), tqos_pcl);

    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener *listener_pcl = new SubListener(4, nh, dds_sub_Measure_topic, ros_pub_Measure_topic);
    //READERQOS
    DataReaderQos rqos_pcl;

    rqos_pcl.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_pcl.history().depth = 1;
    rqos_pcl.resource_limits().max_samples = 50;
    rqos_pcl.resource_limits().allocated_samples = 20;
    rqos_pcl.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader *reader_pcl = subscriber_->create_datareader(topic_pcl, rqos_pcl, listener_pcl);
    if (reader_pcl == nullptr)
        std::cout << "creat topic " << dds_sub_Measure_topic << " failed" << std::endl;
    else
        std::cout << "creat topic " << dds_sub_Measure_topic << " success" << std::endl;

    ros::spin();

    return 0;
}
