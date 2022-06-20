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
#define ros_pub_PointCloud2_topic   "receive_pcl"
#define ros_pub_Octomap_topic   "receive_octomap"
#define ros_pub_Twist_topic "reveive_twist"
#define ros_pub_Image_topic "receive_image"
#define ros_pub_Tf_topic    "receive_tf"
#define ros_pub_Marker_topic "receive_marker"
#define ros_pub_frontVideo_topic "front_video"
#define ros_pub_backVideo_topic "back_video"
#define ros_pub_Odom_topic "receive_odom"

#define dds_sub_PointCloud2_topic   "PclTopic"
#define dds_sub_Octomap_topic   "OctomapTopic"
#define dds_sub_Twist_topic "TwistTopic"
#define dds_sub_Image_topic "ImageTopic"
#define dds_sub_Tf_topic    "TfTopic"
#define dds_sub_Marker_topic "MarkerTopic"
#define dds_sub_frontVideo_topic "frontVideoTopic"
#define dds_sub_backVideo_topic "backVideoTopic"
#define dds_sub_Odom_topic "OdometryTopic"

#define WAN_IP  "127.0.0.1" 
#define PORT 56452
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
            switch(mode_)
            {
                default:
                case 0:
                    twist_pub=np.advertise<geometry_msgs::Twist>(ros_topicName_,10);
                    break;
                case 1:
                    octomap_pub = np.advertise<octomap_msgs::Octomap>(ros_topicName_, 10);
                    break;
                case 2:
                    tf_pub=np.advertise<tf2_msgs::TFMessage>(ros_topicName_,500);
                    break;
                case 3:
                    image_pub=np.advertise<sensor_msgs::Image>(ros_topicName_,10);
                    break;
                case 4:
                    pcl_pub = np.advertise<sensor_msgs::PointCloud2>(ros_topicName_, 10);
                    break;
                case 5:
                    marker_pub=np.advertise<visualization_msgs::Marker>(ros_topicName_,10);
                    break;
                case 6:
                    odom_pub=np.advertise<nav_msgs::Odometry>(ros_topicName,10);
            }
        }

        ~SubListener() 
        {
        }

        void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) 
        {
            SampleInfo info;
            switch(mode_)
            {
                default:
                case 0:
                    if (reader->take_next_sample(&twist_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            geometry_msgs::Twist vel;
                            vel.linear.x=twist_.linear_x();
                            vel.linear.y=twist_.linear_y();
                            vel.linear.y=twist_.linear_y();
                            vel.angular.x=twist_.angular_x();
                            vel.angular.y=twist_.angular_y();
                            vel.angular.z=twist_.angular_z();
                            if(LOG) std::cout << "[RUDP-Twist] linear_x: " << twist_.linear_x() << " with angular_z: "<< twist_.angular_z() << " RECEIVED" << std::endl;
                        }
                    }
                    break;
                case 1:
                    if (reader->take_next_sample(&octomap_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            if(LOG) std::cout << "[RUDP-Octomap] Seq " << octomap_.seq() << " Time " << octomap_.secs() << " RECEIVED" << std::endl;
                            octomap_msgs::Octomap octomap;
                            octomap.header.seq=octomap_.seq();
                            octomap.header.stamp.sec=octomap_.secs();
                            octomap.header.stamp.nsec=octomap_.nsecs();
                            octomap.header.frame_id=octomap_.frame_id();
                            octomap.binary=octomap_.binary();
                            octomap.id=octomap_.id();
                            octomap.resolution=octomap_.resolution();
                            octomap.data.assign(octomap_.data().begin(),octomap_.data().end());
                            octomap_pub.publish(octomap);

                            if(LOG) ROS_INFO("pub octomap");
                        }
                    }
                    break;
                case 2:
                    if (reader->take_next_sample(&tf_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            if(0) std::cout << "[RUDP-Tf] Seq: " << tf_.tf_sequence()[0].seq() << " with time: "<< tf_.tf_sequence()[0].secs() << " RECEIVED" << std::endl;
                            for (int i=0; i<tf_.tf_sequence().size();i++)
                            {
                                std_msgs::Header header;
                                header.stamp.sec=tf_.tf_sequence()[i].secs();
                                header.stamp.nsec=tf_.tf_sequence()[i].nsecs();
                                static tf::TransformBroadcaster br;
                                tf::Transform transform;
                                transform.setOrigin( tf::Vector3(tf_.tf_sequence()[i].translation_x(), tf_.tf_sequence()[i].translation_y(), tf_.tf_sequence()[i].translation_z()) );
                                double roll, pitch, yaw;
                                tf::Matrix3x3(tf::Quaternion(tf_.tf_sequence()[i].rotation_x(), tf_.tf_sequence()[i].rotation_y(), tf_.tf_sequence()[i].rotation_z(),tf_.tf_sequence()[i].rotation_w())).getRPY(roll, pitch, yaw);
                                tf::Quaternion q;
                                q.setRPY(roll,pitch,yaw);
                                transform.setRotation(q);
                                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_.tf_sequence()[i].frame_id(), tf_.tf_sequence()[i].child_frame_id()));
				                if(0) std::cout<<"tf time "<<header.stamp<<std::endl;
                            }
                            if(0) ROS_INFO("pub tf");
                        }
                    }
                    break;
                case 3:
		          try
                  { 
                    if (reader->take_next_sample(&image_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
			                               
			    	            if(LOG) std::cout << "[RUDP-Image] Seq: " << image_.seq() << " with time: "<< image_.secs() << " RECEIVED" << std::endl;
                            	sensor_msgs::CompressedImage image;
                            	image.header.seq=image_.seq();
                            	image.header.stamp.sec=image_.secs();
                            	image.header.stamp.nsec=image_.nsecs();
                            	image.header.frame_id=image_.frame_id();
                            	image.format=image_.format();
			                     image.data.assign(image_.data().begin(),image_.data().end());
                            	cv::Mat cv_image = cv::imdecode(cv::Mat(image.data),1);
                            	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
                            	image_pub.publish(*msg);

                            	if(ros_topicName_==ros_pub_Image_topic)
                            	{
                                	std::stringstream convert;
                                	convert<<"~/catkin_ws/ddscomm/src/rosddsbridge/saveImage"<< image_.seq()<<".jpg";
                                	cv::imwrite(convert.str(),cv_bridge::toCvShare(msg)->image);
                            	}
                            	if(LOG) ROS_INFO("pub image data size %d",msg->data.size());
                        }
                    }
                  }
			      catch(...){
				    std::cout<<"there is an error"<<std::endl;
				    throw;
			      }
                    break;
                case 4:
                   if (reader->take_next_sample(&pcl_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            if(LOG) std::cout << "[RUDP-Pcl] Seq: " << pcl_.seq() << " with time: "<< pcl_.secs() << " RECEIVED" << std::endl;
                            sensor_msgs::PointCloud2 pcl;
                            pcl.header.seq=pcl_.seq();
                            //pcl.header.stamp.sec=pcl_.secs();
                            //pcl.header.stamp.nsec=pcl_.nsecs();
                            pcl.header.stamp=ros::Time::now();
                            pcl.header.frame_id=pcl_.frame_id();
                            pcl.height=pcl_.height();
                            pcl.width=pcl_.width();
                            for(int i=0;i<4;i++)
                            {
                                sensor_msgs::PointField pfl;
                                pfl.name=pcl_.PointFileds_name()[i].c_str();
                                pfl.offset=pcl_.PointFileds_offset()[i];
                                pfl.datatype=pcl_.PointFileds_datatype()[i];
                                pfl.count=pcl_.PointFileds_count()[i];
                                pcl.fields.push_back(pfl);  
                            }
                            pcl.is_bigendian=pcl_.is_bigendian();
                            pcl.point_step=pcl_.point_step();
                            pcl.row_step=pcl_.row_step();
			                pcl.data.assign(pcl_.data().begin(),pcl_.data().end());
                            pcl.is_dense=pcl_.is_dense();
                            pcl_pub.publish(pcl);

                            pcl.fields.clear();
                            if(LOG) ROS_INFO("pub pcl");
                        }
                    }
                    break;
                case 5:
                    if (reader->take_next_sample(&marker_, &info) == ReturnCode_t::RETCODE_OK)
                    {
                        if (info.valid_data)
                        {
                            if(LOG) std::cout << "[RUDP-Marker] Seq: " << marker_.seq() << " with time: "<< marker_.secs() << " RECEIVED" << std::endl;
                            visualization_msgs::Marker marker;
                            marker.header.seq=marker_.seq();
                            marker.header.stamp.sec=marker_.secs();
                            marker.header.stamp.nsec=marker_.nsecs();
                            marker.header.frame_id=marker_.frame_id();
                            marker.id=marker_.id();
                            marker.ns=marker_.ns();
                            marker.type=marker_.type();
                            marker.action=marker_.action();
                            marker.pose.position.x=marker_.position_x();
                            marker.pose.position.y=marker_.position_y();
                            marker.pose.position.z=marker_.position_z();
                            marker.pose.orientation.x=marker_.orientation_x();
                            marker.pose.orientation.y=marker_.orientation_y();
                            marker.pose.orientation.z=marker_.orientation_z();
                            marker.pose.orientation.w=marker_.orientation_w();
                            marker.scale.x=marker_.scale_x();
                            marker.scale.y=marker_.scale_y();
                            marker.scale.z=marker_.scale_z();
                            marker.color.r=marker_.color_r();
                            marker.color.g=marker_.color_g();
                            marker.color.b=marker_.color_b();
                            marker.color.a=marker_.color_a();
                            marker_pub.publish(marker);
                            if(LOG) ROS_INFO("pub marker");
                        }
                    }
                    break;
                case 6:
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
                    break;
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

        TwistMsg twist_;
        PclMsg pcl_;
        OctomapMsg octomap_;
        TfMsg tf_;
        ImageMsg image_;
        MarkerMsg marker_;
        OdometryMsg odom_;
        ros::NodeHandle np ;
        ros::Publisher twist_pub,image_pub,octomap_pub,tf_pub,pcl_pub,marker_pub,odom_pub;//,
};

int main(int argc ,char **argv)
{
    ros::init(argc, argv, "dds2ros");
    ros::NodeHandle nh;
    std::string wan_ip=WAN_IP;
    bool isMultiComm=multiComm;
    int port=PORT;
    std::vector<std::string> whitelist;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("wan_ip",wan_ip);
    nhPrivate.getParam("port",port);
    nhPrivate.getParam("comm",isMultiComm);
    if(!isMultiComm)
        whitelist.push_back(wan_ip);

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Subscriber* subscriber_;

    //CREATE THE PARTICIPANT
    int32_t kind = LOCATOR_KIND_UDPv4;
    Locator_t initial_peer_locator;
    initial_peer_locator.kind = kind;
    IPLocator::setIPv4(initial_peer_locator, wan_ip);
    initial_peer_locator.port = port;
    std::cout << " subscriber as server "<<wan_ip << ":" << port << std::endl;
   
    DomainParticipantQos pqos;
    pqos.wire_protocol().builtin.discovery_config.discoveryServer_client_syncperiod =eprosima::fastrtps::Duration_t(0, 250000000);
    // Configure the current participant as SERVER
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
    eprosima::fastdds::dds::TypeSupport type_pcl(new PclMsgPubSubType());
    eprosima::fastdds::dds::TypeSupport type_octomap(new OctomapMsgPubSubType());
    eprosima::fastdds::dds::TypeSupport type_tf(new TfMsgPubSubType());
    eprosima::fastdds::dds::TypeSupport type_marker(new MarkerMsgPubSubType());
    eprosima::fastdds::dds::TypeSupport type_image(new ImageMsgPubSubType());
    eprosima::fastdds::dds::TypeSupport type_odom(new OdometryMsgPubSubType());
    type_octomap.register_type(participant_);
    type_tf.register_type(participant_);
    type_image.register_type(participant_);
    type_pcl.register_type(participant_);
    type_marker.register_type(participant_);
    type_odom.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_octomap,tqos_tf,tqos_image,tqos_pcl,tqos_marker, tqos_odom;
    tqos_octomap.transport_priority().value=16;
    tqos_tf.transport_priority().value=1;
    tqos_image.transport_priority().value=4;
    tqos_pcl.transport_priority().value=32;
    tqos_marker.transport_priority().value=22;
    tqos_odom.transport_priority().value=22;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic* topic_octomap = participant_->create_topic(dds_sub_Octomap_topic, type_octomap.get_type_name(), tqos_octomap);
    eprosima::fastdds::dds::Topic* topic_tf = participant_->create_topic(dds_sub_Tf_topic, type_tf.get_type_name(), tqos_tf);
    eprosima::fastdds::dds::Topic* topic_image = participant_->create_topic(dds_sub_Image_topic, type_image.get_type_name(), tqos_image);
    eprosima::fastdds::dds::Topic* topic_pcl = participant_->create_topic(dds_sub_PointCloud2_topic, type_pcl.get_type_name(), tqos_pcl);
    eprosima::fastdds::dds::Topic* topic_marker = participant_->create_topic(dds_sub_Marker_topic, type_marker.get_type_name(), tqos_marker);
     eprosima::fastdds::dds::Topic* topic_odom = participant_->create_topic(dds_sub_Odom_topic, type_odom.get_type_name(), tqos_odom);

    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener* listener_octomap =new SubListener(1,nh,dds_sub_Octomap_topic,ros_pub_Octomap_topic);
    SubListener* listener_tf =new SubListener(2,nh, dds_sub_Tf_topic, ros_pub_Tf_topic);
    SubListener* listener_image =new SubListener(3,nh,dds_sub_Image_topic, ros_pub_Image_topic);
    SubListener* listener_pcl = new SubListener(4,nh, dds_sub_PointCloud2_topic, ros_pub_PointCloud2_topic);
    SubListener* listener_marker =new SubListener(5,nh, dds_sub_Marker_topic, ros_pub_Marker_topic);
    SubListener* listener_odom =new SubListener(6,nh, dds_sub_Odom_topic, ros_pub_Odom_topic);
    //READERQOS
    DataReaderQos rqos_octomap,rqos_tf,rqos_image,rqos_pcl,rqos_marker, rqos_odom;

    rqos_octomap.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_octomap.history().depth = 30;
    rqos_octomap.resource_limits().max_samples = 50;
    rqos_octomap.resource_limits().allocated_samples = 20;
    rqos_octomap.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_octomap.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_octomap = subscriber_->create_datareader(topic_octomap, rqos_octomap, listener_octomap);
    if(reader_octomap == nullptr)
    	std::cout<<"creat topic "<<"Octomap"<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<"Octomap"<<" success"<<std::endl;

    rqos_tf.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_tf.history().depth = 1;
    rqos_tf.resource_limits().max_samples = 50;
    rqos_tf.resource_limits().allocated_samples = 20;
    rqos_tf.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_tf.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_tf = subscriber_->create_datareader(topic_tf, rqos_tf, listener_tf);
    if(reader_tf == nullptr)
    	std::cout<<"creat topic "<<"tf"<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<"tf"<<" success"<<std::endl;

    rqos_image.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_image.history().depth = 30;
    rqos_image.resource_limits().max_samples = 250;
    rqos_image.resource_limits().allocated_samples = 220;
    rqos_image.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_image.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_image = subscriber_->create_datareader(topic_image, rqos_image, listener_image);
    if(reader_image == nullptr)
    	std::cout<<"creat topic "<<"image"<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<"image"<<" success"<<std::endl;

    rqos_pcl.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_pcl.history().depth = 1;
    rqos_pcl.resource_limits().max_samples = 50;
    rqos_pcl.resource_limits().allocated_samples = 20;
    rqos_pcl.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_pcl.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_pcl = subscriber_->create_datareader(topic_pcl, rqos_pcl, listener_pcl);
    if(reader_pcl == nullptr)
    	std::cout<<"creat topic "<<"pcl"<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<"pcl"<<" success"<<std::endl;

    rqos_marker.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_marker.history().depth = 30;
    rqos_marker.resource_limits().max_samples = 50;
    rqos_marker.resource_limits().allocated_samples = 20;
    rqos_marker.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_marker.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_marker = subscriber_->create_datareader(topic_marker, rqos_marker, listener_marker);
    if(reader_marker == nullptr)
    	std::cout<<"creat topic "<<"marker"<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<"marker"<<" success"<<std::endl;

    //CREATE THE TOPIC
    //TOPIC QOS
    TopicQos tqos_video;
    tqos_video.transport_priority().value=2;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic* front_topic_video = participant_->create_topic(
        dds_sub_frontVideo_topic, type_image.get_type_name(), tqos_video);
    eprosima::fastdds::dds::Topic* back_topic_video = participant_->create_topic(
        dds_sub_backVideo_topic, type_image.get_type_name(), tqos_video);
    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener* listener_frontvideo= new SubListener(3,nh,dds_sub_frontVideo_topic, ros_pub_frontVideo_topic);
    SubListener* listener_backvideo= new SubListener(3,nh,dds_sub_backVideo_topic, ros_pub_backVideo_topic);
    //READERQOS
    DataReaderQos rqos_video;
    rqos_video.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_video.history().depth = 1;
    rqos_video.resource_limits().max_samples = 50;
    rqos_video.resource_limits().allocated_samples = 20;
    rqos_video.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
    rqos_video.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_frontvideo = subscriber_->create_datareader(
        front_topic_video, rqos_video, listener_frontvideo);
    eprosima::fastdds::dds::DataReader* reader_backvideo = subscriber_->create_datareader(
        back_topic_video, rqos_video, listener_backvideo);
    if(front_topic_video == nullptr || back_topic_video == nullptr )
        std::cout<<"creat topic "<<"video"<<" failed"<<std::endl;
    else
        std::cout<<"creat topic "<<"video"<<" success"<<std::endl;

    rqos_odom.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_odom.history().depth = 1;
    rqos_odom.resource_limits().max_samples = 50;
    rqos_odom.resource_limits().allocated_samples = 20;
    rqos_odom.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_odom.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_odom = subscriber_->create_datareader(topic_odom, rqos_odom, listener_odom);
    if(reader_tf == nullptr)
    	std::cout<<"creat topic "<<"tf"<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<"tf"<<" success"<<std::endl;
    ros::spin();
    //std::cin.ignore();
    return 0;
}
