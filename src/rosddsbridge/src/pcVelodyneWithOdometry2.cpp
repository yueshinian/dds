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
#include "rosddsbridge/PclWithOdom.h"
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
#include <string>
//namespace
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using Locator_t = eprosima::fastrtps::rtps::Locator_t;
using IPLocator = eprosima::fastrtps::rtps::IPLocator;
using namespace std;
//topic name
std::string wan_ip ="127.0.0.1";
int port =56452;
std::string ros_pub_PointCloud2WithOdom_topic = "/pclWithOdom";
std::string dds_sub_PointCloud2WithOdom_topic=   "PclWithOdomTopic";
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
            pclWithOdomPub = np.advertise<rosddsbridge::PclWithOdom>(ros_topicName_,10);
        }

        ~SubListener() 
        {
        }

        void on_data_available(eprosima::fastdds::dds::DataReader* reader) 
        {
            SampleInfo info;
           
                   if (reader->take_next_sample(&velodyneWithOdom, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            if(LOG) std::cout << "[RUDP-PclWithOdom] Seq: " << velodyneWithOdom.pcl().seq() << " with time: "<< velodyneWithOdom.pcl().secs() << " RECEIVED" << std::endl;
                            rosddsbridge::PclWithOdom pclWithOdom;
                            sensor_msgs::PointCloud2 pcl;
                            pcl.header.seq=velodyneWithOdom.pcl().seq();
                            //pcl.header.stamp.sec=pcl_.secs();
                            //pcl.header.stamp.nsec=pcl_.nsecs();
                            pcl.header.stamp=ros::Time::now();
                            pcl.header.frame_id=velodyneWithOdom.pcl().frame_id();
                            pcl.height=velodyneWithOdom.pcl().height();
                            pcl.width=velodyneWithOdom.pcl().width();
                            for(int i=0;i<4;i++)
                            {
                                sensor_msgs::PointField pfl;
                                pfl.name=velodyneWithOdom.pcl().PointFileds_name()[i].c_str();
                                pfl.offset=velodyneWithOdom.pcl().PointFileds_offset()[i];
                                pfl.datatype=velodyneWithOdom.pcl().PointFileds_datatype()[i];
                                pfl.count=velodyneWithOdom.pcl().PointFileds_count()[i];
                                pcl.fields.push_back(pfl);  
                            }
                            pcl.is_bigendian=velodyneWithOdom.pcl().is_bigendian();
                            pcl.point_step=velodyneWithOdom.pcl().point_step();
                            pcl.row_step=velodyneWithOdom.pcl().row_step();
			                pcl.data.assign(velodyneWithOdom.pcl().data().begin(),velodyneWithOdom.pcl().data().end());
                            pcl.is_dense=velodyneWithOdom.pcl().is_dense();

                            nav_msgs::Odometry odom;
                            odom.header.seq=velodyneWithOdom.odometry().seq();
                            odom.header.stamp.sec=velodyneWithOdom.odometry().secs();
                            odom.header.stamp.nsec=velodyneWithOdom.odometry().nsecs();
                            odom.header.frame_id=velodyneWithOdom.odometry().frame_id();
                            odom.child_frame_id = velodyneWithOdom.odometry().child_frame_id();
                            odom.pose.pose.position.x=velodyneWithOdom.odometry().position_x();
                            odom.pose.pose.position.y=velodyneWithOdom.odometry().position_y();
                            odom.pose.pose.position.z=velodyneWithOdom.odometry().position_z();
                            odom.pose.pose.orientation.x=velodyneWithOdom.odometry().orientation_x();
                            odom.pose.pose.orientation.y=velodyneWithOdom.odometry().orientation_y();
                            odom.pose.pose.orientation.z=velodyneWithOdom.odometry().orientation_z();
                            odom.pose.pose.orientation.w=velodyneWithOdom.odometry().orientation_w();
                            
                            pclWithOdom.pcl=pcl;
                            pclWithOdom.odom=odom;
                            pclWithOdomPub.publish(pclWithOdom);

                            static tf::TransformBroadcaster br;
                            tf::Transform transform;
                            transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) );
                            tf::Quaternion q(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w);
                            transform.setRotation(q);
                            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "receive_vehicle2"));
                            if(LOG) ROS_INFO("pub pcl");
                        
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

        VelodyneWithOdometryMsg velodyneWithOdom;
        ros::NodeHandle np ;
        ros::Publisher pcl_pub;
        ros::Publisher pclWithOdomPub;
};

int main(int argc ,char **argv)
{
    ros::init(argc, argv, "pcVelodyneWithOdomSub2");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("wan_ip",wan_ip);
    nhPrivate.getParam("port",port);
    nhPrivate.getParam("ros_pub_PointCloud2WithOdom_topic",ros_pub_PointCloud2WithOdom_topic);
    nhPrivate.getParam("dds_sub_PointCloud2WithOdom_topic",dds_sub_PointCloud2WithOdom_topic);

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
    eprosima::fastdds::dds::TypeSupport type_pcl(new VelodyneWithOdometryMsgPubSubType());
    type_pcl.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_pcl;
    tqos_pcl.transport_priority().value=32;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic* topic_pcl = participant_->create_topic(dds_sub_PointCloud2WithOdom_topic, type_pcl.get_type_name(), tqos_pcl);

    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener* listener_pcl = new SubListener(4,nh, dds_sub_PointCloud2WithOdom_topic, ros_pub_PointCloud2WithOdom_topic);
    //READERQOS
    DataReaderQos rqos_pcl;

    rqos_pcl.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_pcl.history().depth = 15;
    rqos_pcl.resource_limits().max_samples = 50;
    rqos_pcl.resource_limits().allocated_samples = 20;
    rqos_pcl.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_pcl = subscriber_->create_datareader(topic_pcl, rqos_pcl, listener_pcl);
    if(reader_pcl == nullptr)
    	std::cout<<"creat topic "<<"pclWithOdom"<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<"pclWithOdom"<<" success"<<std::endl;

    ros::spin();

    return 0;
}
