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
std::string wan_ip ="127.0.0.1";
int port =56452;
std::string ros_pub_Image_topic = "testImage";
std::string ros_pub_Image1_topic = "/receive_image1";
std::string ros_pub_Image2_topic = "/receive_image2";
std::string dds_sub_Image1_topic = "ImageTopic1";
std::string dds_sub_Image2_topic = "ImageTopic2";
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
            image_pub=np.advertise<sensor_msgs::Image>(ros_topicName_,10);
        }

        ~SubListener() 
        {
        }

        void on_data_available(eprosima::fastdds::dds::DataReader* reader) 
        {
            SampleInfo info;
           
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
                                	//cv::imwrite(convert.str(),cv_bridge::toCvShare(msg)->image);
                            	}
                            	if(LOG) ROS_INFO("pub image data size %d",msg->data.size());
                        }
                    }
        }

        void on_subscription_matched(eprosima::fastdds::dds::DataReader* reader, const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) 
        {
            if (info.current_count_change == 1)
            {
                matched_++;
                std::cout << "[RUDP] Subscriber "<<dds_topicName_<<" "<<matched_<<" matched" << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "[RUDP] Subscriber "<<dds_topicName_<<"unmatched" << std::endl;
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

        ImageMsg image_;
        ros::NodeHandle np ;
        ros::Publisher image_pub;
};

int main(int argc ,char **argv)
{
    ros::init(argc, argv, "pcMeasureSub");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("wan_ip",wan_ip);
    nhPrivate.getParam("port",port);
    nhPrivate.getParam("ros_pub_Image1_topic",ros_pub_Image1_topic);
    nhPrivate.getParam("ros_pub_Image2_topic",ros_pub_Image2_topic);
    nhPrivate.getParam("dds_sub_Image1_topic",dds_sub_Image1_topic);
    nhPrivate.getParam("dds_sub_Image2_topic",dds_sub_Image2_topic);

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
    eprosima::fastdds::dds::TypeSupport type_image(new ImageMsgPubSubType());
    type_image.register_type(participant_);
    //TOPIC QOS
    TopicQos tqos_image;
    tqos_image.transport_priority().value=32;
    //CREATE THE TOPIC
    eprosima::fastdds::dds::Topic* topic_image1 = participant_->create_topic(dds_sub_Image1_topic, type_image.get_type_name(), tqos_image);
    eprosima::fastdds::dds::Topic* topic_image2 = participant_->create_topic(dds_sub_Image2_topic, type_image.get_type_name(), tqos_image);

    //CREATE THE DATAREADER
    //CREATE THE SUBLISTENER
    SubListener* listener_image1 = new SubListener(4,nh, dds_sub_Image1_topic, ros_pub_Image1_topic);
    SubListener* listener_image2 = new SubListener(4,nh, dds_sub_Image2_topic, ros_pub_Image2_topic);
    //READERQOS
    DataReaderQos rqos_image;

    rqos_image.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
    rqos_image.history().depth = 1;
    rqos_image.resource_limits().max_samples = 250;
    rqos_image.resource_limits().allocated_samples = 220;
    rqos_image.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    rqos_image.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    eprosima::fastdds::dds::DataReader* reader_image1 
        = subscriber_->create_datareader(topic_image1, rqos_image, listener_image1);
    eprosima::fastdds::dds::DataReader* reader_image2 
        = subscriber_->create_datareader(topic_image2, rqos_image, listener_image2);
    if(reader_image1 == nullptr || reader_image2==nullptr)
    	std::cout<<"creat topic "<<"image"<<" failed"<<std::endl;
    else
    	std::cout<<"creat topic "<<"image"<<" success"<<std::endl;

    ros::spin();

    return 0;
}
