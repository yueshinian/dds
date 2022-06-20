//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2_msgs/TFMessage.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
//dds lib
#include <fastrtps/Domain.h>
#include <fastrtps/log/Log.h>
#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/Domain.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/utils/IPLocator.h>
#include "optionparser.h"
//#include "HelloWorldSubscriber.h"

//dds msg
#include "PclMsgPubSubTypes.h"
#include "OctomapMsgPubSubTypes.h"
#include "PclMsg.h"
#include "OctomapMsg.h"
//C++ lib
#include <vector>
#include <string>

#define pcl_topic "test_pcl"
#define octomap_topic "test_octomap"

#define WAN_IP  "192.168.1.53" 

#define dds_pub_PointCloud2_topic 	"PclTopic"
#define dds_pub_Octomap_topic	"OctomapTopic"
#define dds_pub_image_topic	"ImageTopic"
#define dds_pub_tf_topic	"TfTopic"
#define dds_pub_twist_topic     "TwistTopic"

#define port_twist   5100
#define port_octomap 5200 
#define port_tf      5300
#define port_image   5400
#define port_pcl     5500

#define MULTCOMM 0
using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

class SubListener : public eprosima::fastrtps::SubscriberListener
    {
    public:
        SubListener()
            : n_matched(0)
            , n_samples(0)
        {};

        ~SubListener() {};

        void onSubscriptionMatched(Subscriber*,MatchingInfo& matching_info)
	{
		std::cout << "[RTCP] Waiting Subscriber matched" << std::endl;
    		if (matching_info.status == MATCHED_MATCHING)
    		{
       			 n_matched++;
        		std::cout << "[RTCP] Subscriber matched" << std::endl;
    		}
    		else
    		{
        		n_matched--;
        		std::cout << "[RTCP] Subscriber unmatched" << std::endl;
    		}
	}

	void onNewDataMessage(eprosima::fastrtps::Subscriber* sub)
	{
    		if (sub->takeNextData((void*)&pcl_, &info))
    		{
       			 if (info.sampleKind == ALIVE)
        		{
            			this->n_samples++;
	    			msgAvailale=true;
            			std::cout << "[RTCP] Seq " << pcl_.seq() << " Time" << pcl_.secs() << " RECEIVED" << std::endl;
        		}
    		}
   	}
	bool msgAvailale = false;
        PclMsg pcl_;
	OctomapMsg octomap_;
        eprosima::fastrtps::SampleInfo_t info;
        int n_matched;
        uint32_t n_samples;
    } listener;

int main(int argc,char **argv)
{
    std::string wan_ip=WAN_IP;
    int port = 5200;
    std::vector<std::string> whitelist;
    int count =0;
    long sleep =500;
    bool use_tls=false;
    if(!MULTCOMM)
   	whitelist.push_back(wan_ip);
    eprosima::fastrtps::Participant* participant_;
    eprosima::fastrtps::Subscriber* subscriber_;
    //OctomapMsgPubSubType type_;
    PclMsgPubSubType type_;

    ParticipantAttributes pparam;
    int32_t kind = LOCATOR_KIND_TCPv4;

    Locator_t initial_peer_locator;
    initial_peer_locator.kind = kind;

    std::shared_ptr<TCPv4TransportDescriptor> descriptor = std::make_shared<TCPv4TransportDescriptor>();

    for (std::string ip : whitelist)
    {
        descriptor->interfaceWhiteList.push_back(ip);
        std::cout << "Whitelisted " << ip << std::endl;
    }

    if (!wan_ip.empty())
    {
        IPLocator::setIPv4(initial_peer_locator, wan_ip);
        std::cout << wan_ip << ":" << port << std::endl;
    }
    else
    {
        IPLocator::setIPv4(initial_peer_locator, "127.0.0.1");
    }
    //initial_peer_locator.port = port_octomap;
    initial_peer_locator.port = port_pcl;
    pparam.rtps.builtin.initialPeersList.push_back(initial_peer_locator); // Publisher's meta channel

    pparam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
    pparam.rtps.builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(5, 0);
    pparam.rtps.setName("Participant_sub");

    pparam.rtps.useBuiltinTransports = false;

    if (use_tls)
    {
        using TLSVerifyMode = TCPTransportDescriptor::TLSConfig::TLSVerifyMode;
        using TLSOptions = TCPTransportDescriptor::TLSConfig::TLSOptions;
        descriptor->apply_security = true;
        descriptor->tls_config.password = "test";
        descriptor->tls_config.verify_file = "ca.pem";
        descriptor->tls_config.verify_mode = TLSVerifyMode::VERIFY_PEER;
        descriptor->tls_config.add_option(TLSOptions::DEFAULT_WORKAROUNDS);
    }

    descriptor->wait_for_tcp_negotiation = false;
    pparam.rtps.userTransports.push_back(descriptor);

    participant_ = Domain::createParticipant(pparam);
    if (participant_ == nullptr)
    {
        ROS_INFO("create participant failed!");
    }

    //REGISTER THE TYPE
    Domain::registerType(participant_, &type_);

    //CREATE THE SUBSCRIBER
    SubscriberAttributes rparam;
    rparam.topic.topicKind = NO_KEY;
    //rparam.topic.topicDataType = "OctomapMsg";
    //rparam.topic.topicName = dds_pub_Octomap_topic;
    rparam.topic.topicDataType = "PclMsg";
    rparam.topic.topicName = dds_pub_Pcl_topic;
    rparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    rparam.topic.historyQos.depth = 30;
    rparam.topic.resourceLimitsQos.max_samples = 50;
    rparam.topic.resourceLimitsQos.allocated_samples = 20;
    rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    rparam.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    subscriber_ = Domain::createSubscriber(participant_, rparam, (SubscriberListener*)&listener);
    if (subscriber_ == nullptr)
    {
       ROS_INFO("create subscriber failed!");
    }
    else
    {
       ROS_INFO("dds %s init success!",rparam.topic.topicName);
    }
  
    ros::init(argc, argv, "dds2ros");

    ros::NodeHandle nh;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic, 10);
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 10);
    ros::Rate loop_rate(100);
    int last_seq;
    while (ros::ok()) 
    {
        if (listener.msgAvailale){
		if(false)
		{
    			sensor_msgs::PointCloud2 pcl;
			pcl.header.seq=listener.pcl_.seq();
			last_seq=listener.pcl_.seq();
                	ROS_INFO("%d",listener.pcl_.seq());
	        	pcl.header.stamp.sec=listener.pcl_.secs();
			pcl.header.stamp.nsec=listener.pcl_.nsecs();
			pcl.header.frame_id=listener.pcl_.frame_id();
			pcl.height=listener.pcl_.height();
			pcl.width=listener.pcl_.width();
			for(int i=0;i<4;i++)
			{
				sensor_msgs::PointField pfl;
				pfl.name=listener.pcl_.PointFileds_name()[i].c_str();
				pfl.offset=listener.pcl_.PointFileds_offset()[i];
				pfl.datatype=listener.pcl_.PointFileds_datatype()[i];
				pfl.count=listener.pcl_.PointFileds_count()[i];
				pcl.fields.push_back(pfl);	
			}
			pcl.is_bigendian=listener.pcl_.is_bigendian();
			pcl.point_step=listener.pcl_.point_step();
			pcl.row_step=listener.pcl_.row_step();
 	                pcl.data.assign(listener.pcl_.data().begin(),listener.pcl_.data().end());
			pcl.is_dense=listener.pcl_.is_dense();
            		pcl_pub.publish(pcl);
			pcl.fields.clear();
			pcl.data.clear();
		}
		if(last_seq!=listener.octomap_.seq())
		{
    			octomap_msgs::Octomap octomap;
			octomap.header.seq=listener.octomap_.seq();
			last_seq=listener.octomap_.seq();
	        	octomap.header.stamp.sec=listener.octomap_.secs();
			octomap.header.stamp.nsec=listener.octomap_.nsecs();
			octomap.header.frame_id=listener.octomap_.frame_id();
			octomap.binary=listener.octomap_.binary();
			octomap.id=listener.octomap_.id();
			octomap.resolution=listener.octomap_.resolution();
	       	octomap.data.assign(listener.octomap_.data().begin(),listener.octomap_.data().end());
            		octomap_pub.publish(octomap);
			octomap.data.clear();
		}
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
