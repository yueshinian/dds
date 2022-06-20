//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
//C++ lib
#include <string>
#include <boost/thread.hpp>
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
//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> //文件输入输出
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>                //点类型相关定义
#include <pcl/visualization/cloud_viewer.h> //点类型相关定义
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

const bool LOG = 1;

std::string wan_ip = "127.0.0.1";
int port = 56452;
std::string ros_sub_PointCloud2_topic = "/laser_cloud_surround";
std::string dds_pub_PointCloud2_topic = "PclMapTopic";

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;

eprosima::fastdds::dds::DataWriter *writerPcl;

class PubListener : public eprosima::fastdds::dds::DataWriterListener
{
public:
    PubListener(std::string topicName) : topicName_(topicName), matched_(0), connected_(false) {}

    PubListener() : matched_(0), connected_(false) {}

    ~PubListener() override {}

    void on_publication_matched(
        eprosima::fastdds::dds::DataWriter *writer,
        const eprosima::fastdds::dds::PublicationMatchedStatus &info)
    {
        if (info.current_count_change == 1)
        {
            matched_++;
            connected_ = true;
            std::cout << "[RUDP] Publisher [" << this->topicName_ << "] [" << matched_ << "st] matched!" << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            connected_ = false;
            std::cout << "[RUDP] Publisher [" << this->topicName_ << "] unmatched" << std::endl;
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
};

PubListener *listener;
int pcl_count = 0;

void subpclCallback(const sensor_msgs::PointCloud2::ConstPtr &pclin)
{
    if (pcl_count == 0)
    {
        pcl_count = 1;
    }
    else
    {
        --pcl_count;
        std::cout << pcl_count << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pclin, *laserCloudIn);
    int size = laserCloudIn->points.size();
    if (size == 0)
        return;
    for (int i = 0; i < size; ++i)
    {
        if (laserCloudTemp->points.size() == 1800 || i == size - 1)
        {
             PclMsg pclMsg_;
            std::cout << "laserCloudTemp format size is : " << laserCloudTemp->points.size() << std::endl;
            sensor_msgs::PointCloud2 pcl;
            pcl::toROSMsg(*laserCloudTemp, pcl);
            pcl.header = pclin->header;
            pclMsg_.data().reserve(1900);
            pclMsg_.seq() = pcl.header.seq;
            pclMsg_.secs() = pcl.header.stamp.sec;
            pclMsg_.nsecs() = pcl.header.stamp.nsec;
            pclMsg_.frame_id() = "/map";
            pclMsg_.height() = pcl.height;
            pclMsg_.width() = pcl.width;
            pclMsg_.PointFileds_name()[0] = pcl.fields[0].name;
            pclMsg_.PointFileds_name()[1] = pcl.fields[1].name;
            pclMsg_.PointFileds_name()[2] = pcl.fields[2].name;
            pclMsg_.PointFileds_name()[3] = pcl.fields[3].name;
            pclMsg_.PointFileds_offset()[0] = pcl.fields[0].offset;
            pclMsg_.PointFileds_offset()[1] = pcl.fields[1].offset;
            pclMsg_.PointFileds_offset()[2] = pcl.fields[2].offset;
            pclMsg_.PointFileds_offset()[3] = pcl.fields[3].offset;
            pclMsg_.PointFileds_datatype()[0] = pcl.fields[0].datatype;
            pclMsg_.PointFileds_datatype()[1] = pcl.fields[1].datatype;
            pclMsg_.PointFileds_datatype()[2] = pcl.fields[2].datatype;
            pclMsg_.PointFileds_datatype()[3] = pcl.fields[3].datatype;
            pclMsg_.PointFileds_count()[0] = pcl.fields[0].count;
            pclMsg_.PointFileds_count()[1] = pcl.fields[1].count;
            pclMsg_.PointFileds_count()[2] = pcl.fields[2].count;
            pclMsg_.PointFileds_count()[3] = pcl.fields[3].count;
            pclMsg_.is_bigendian() = pcl.is_bigendian;
            pclMsg_.point_step() = pcl.point_step;
            pclMsg_.row_step() = pcl.row_step;
            pclMsg_.data().assign(pcl.data.begin(), pcl.data.end());
            pclMsg_.is_dense() = pcl.is_dense;
            if (listener->connected_)
            {
                writerPcl->write((void *)&pclMsg_);
                if (LOG)
                {
                    std::cout << "pub map " << laserCloudTemp->points.size() << std::endl;
                }
                ros::Duration(0.1).sleep();
            }
            std::cout << sizeof(pclMsg_) << std::endl;
            laserCloudTemp->clear();
        }
        else
        {
            float temp = laserCloudIn->points[i].x;
            laserCloudIn->points[i].x = laserCloudIn->points[i].z;
            laserCloudIn->points[i].z = laserCloudIn->points[i].y;
            laserCloudIn->points[i].y = temp;
            laserCloudTemp->push_back(laserCloudIn->points[i]);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotMapPubIn");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("wan_ip", wan_ip);
    nhPrivate.getParam("port", port);
    nhPrivate.getParam("ros_sub_PointCloud2_topic", ros_sub_PointCloud2_topic);
    nhPrivate.getParam("dds_pub_PointCloud2_topic", dds_pub_PointCloud2_topic);

    DomainParticipantQos pqos;
    pqos.name("Participant_pub");
    //ip&port
    int32_t kind = LOCATOR_KIND_UDPv4;
    Locator_t initial_peer_locator;
    initial_peer_locator.kind = kind;
    IPLocator::setIPv4(initial_peer_locator, wan_ip);
    initial_peer_locator.port = port;

    // UDP
    auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
    pqos.transport().user_transports.push_back(udp_transport);
    pqos.transport().use_builtin_transports = false;
    pqos.transport().send_socket_buffer_size = 1048576;
    pqos.transport().listen_socket_buffer_size = 4194304;

    eprosima::fastdds::dds::DomainParticipant *participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (participant_ == nullptr)
    {
        std::cout << "Creat participant faild" << std::endl;
    }
    //CREATE THE PUBLISHER
    eprosima::fastdds::dds::Publisher *publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
    if (publisher_ == nullptr)
    {
        std::cout << "Creat publisher faild" << std::endl;
    }

    //CREATE THE TOPIC
    //REGISTER THE TYPE
    eprosima::fastdds::dds::TypeSupport type_pcl(new PclMsgPubSubType());
    type_pcl.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_pcl;
    tqos_pcl.transport_priority().value = 32;
    tqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic *topic_pcl = participant_->create_topic(dds_pub_PointCloud2_topic, type_pcl.get_type_name(), tqos_pcl);

    eprosima::fastdds::dds::DataWriterQos wqos_pcl;
    wqos_pcl.transport_priority().value = 32;
    wqos_pcl.history().kind = KEEP_LAST_HISTORY_QOS;
    wqos_pcl.history().depth = 1;
    wqos_pcl.resource_limits().max_samples = 50;
    wqos_pcl.resource_limits().allocated_samples = 20;
    wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
    wqos_pcl.reliable_writer_qos().times.heartbeatPeriod.nanosec = 500 * 1000 * 1000;
    wqos_pcl.reliability().kind = RELIABLE_RELIABILITY_QOS;
    wqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;

    listener = new PubListener(dds_pub_PointCloud2_topic);
    writerPcl = publisher_->create_datawriter(topic_pcl, wqos_pcl, listener);
    //writer_joy= publisher_->create_datawriter(topic_cmd, wqos_cmd, listener);

    if (writerPcl == nullptr)
        std::cout << "creat topic " << dds_pub_PointCloud2_topic << " failed" << std::endl;
    else
        std::cout << "creat topic " << dds_pub_PointCloud2_topic << " success" << std::endl;

    ros::Subscriber sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>(ros_sub_PointCloud2_topic, 10, subpclCallback);

    ros::spin();

    return 0;
}
