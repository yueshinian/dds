//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf2_msgs/TFMessage.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
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
#include "DDSSubscriber.h"

//C++ lib
#include <vector>
#include <string>

#define pcl_topic "test_pcl"
#define octomap_topic "test_octomap"

#define WAN_IP  "192.168.1.53" 

#define dds_sub_PointCloud2_topic 	"PclTopic"
#define dds_sub_Octomap_topic	"OctomapTopic"
#define dds_sub_image_topic	"ImageTopic"
#define dds_sub_tf_topic	"TfTopic"
#define dds_sub_twist_topic     "TwistTopic"

#define ros_pub_Twist_topic "cmd_vel"
#define ros_pub_PointCloud2_topic  "test_pcl"
#define ros_pub_Octomap_topic "test_octomap"
#define ros_pub_image_topic	"img"
#define ros_pub_tf_topic	"tf"

#define port_twist   5100
#define port_octomap 5200 
#define port_tf      5300
#define port_image   5400
#define port_pcl     5500

#define MULTCOMM 0

using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

class dds2ros
{
    DDSSubscriber twist_sub;
    DDSSubscriber octomap_sub;
    DDSSubscriber tf_sub;
    DDSSubscriber image_sub;
    DDSSubscriber pcl_sub;
    std::string wan_ip = WAN_IP;
    std::vector<std::string> whitelist;
    int count ;
    long sleep ;
public:
    dds2ros()
    {
        if(!MULTCOMM)
            whitelist.push_back(wan_ip);
    if (pcl_sub.init(wan_ip, static_cast<uint16_t>(port_pcl), false, whitelist,4,dds_sub_PointCloud2_topic,"PclMsg",0))
    {
       ROS_INFO("init pcl dds success!");  
    }

    if (octomap_sub.init(wan_ip, static_cast<uint16_t>(port_octomap), false, whitelist, 1,dds_sub_Octomap_topic,"OctomapMsg",32))
    {
       ROS_INFO("init octomap dds success!");
    }

    if (tf_sub.init(wan_ip, static_cast<uint16_t>(port_tf), false, whitelist, 2,dds_sub_tf_topic,"TfMsg",2))
    {
       ROS_INFO("init tf dds success!");
    }

    if (image_sub.init(wan_ip, static_cast<uint16_t>(port_image), false, whitelist, 3,dds_sub_image_topic,"ImageMsg",2))
    {
       ROS_INFO("init img dds success!");
    }

   if (twist_sub.init(wan_ip, static_cast<uint16_t>(port_twist), false, whitelist, 0,dds_sub_twist_topic,"TwistMsg",2))
    {
       ROS_INFO("init twist dds success!");
    }

    Domain::stopAll();
    };
    ~dds2ros(){};
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"dds2ros");
    
    dds2ros dds2ros1;

    ros::spin();

  	return 0;
}
