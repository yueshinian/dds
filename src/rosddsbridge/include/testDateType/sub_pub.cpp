//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <octomap_msgs/Octomap.h>
//C++ lib
#include <string>
//dds lib
#include <fastrtps/Domain.h>
#include <fastrtps/log/Log.h>
//user lib
#include "HelloWorldPublisher.h"
#include "optionparser.h"

#define octomap_topic_name "test"

using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

bool newMsg=false;

class sub_pub
{
 public:
    sub_pub();
    ~sub_pub(){};
    void subpclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl);
    void subtfCallback(const tf2_msgs::TFMessage::ConstPtr& tf);
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pcl,sub_tf,sub_img;
    ros::Publisher pub_pcl,pub_tf,pub_img;
    HelloWorldPublisher mypub;
    std::string wan_ip ;
   int port ;
   std::vector<std::string> whitelist;
   int count ;
   long sleep ;
};

void sub_pub::subpclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl)
{
newMsg=true;    
mypub.hello_.seq()=pcl->header.seq;
mypub.hello_.secs()=pcl->header.stamp.sec;
mypub.hello_.nsecs()=pcl->header.stamp.nsec;
mypub.hello_.frame_id()=pcl->header.frame_id;
mypub.hello_.height()=pcl->height;
mypub.hello_.width()=pcl->width;
mypub.hello_.PointFileds_name()[0]=pcl->fields[0].name;
mypub.hello_.PointFileds_name()[1]=pcl->fields[1].name;
mypub.hello_.PointFileds_name()[2]=pcl->fields[2].name;
mypub.hello_.PointFileds_name()[3]=pcl->fields[3].name;
mypub.hello_.PointFileds_offset()[0]=pcl->fields[0].offset;
mypub.hello_.PointFileds_offset()[1]=pcl->fields[1].offset;
mypub.hello_.PointFileds_offset()[2]=pcl->fields[2].offset;
mypub.hello_.PointFileds_offset()[3]=pcl->fields[3].offset;
//ROS_INFO("offset %d",pcl->fields[3].offset);
mypub.hello_.PointFileds_datatype()[0]=pcl->fields[0].datatype;
mypub.hello_.PointFileds_datatype()[1]=pcl->fields[1].datatype;
mypub.hello_.PointFileds_datatype()[2]=pcl->fields[2].datatype;
mypub.hello_.PointFileds_datatype()[3]=pcl->fields[3].datatype;
mypub.hello_.PointFileds_count()[0]=pcl->fields[0].count;
mypub.hello_.PointFileds_count()[1]=pcl->fields[1].count;
mypub.hello_.PointFileds_count()[2]=pcl->fields[2].count;
mypub.hello_.PointFileds_count()[3]=pcl->fields[3].count;
mypub.hello_.is_bigendian()=pcl->is_bigendian;
mypub.hello_.point_step()=pcl->point_step;
mypub.hello_.row_step()=pcl->row_step;
/*
int *p,*q;
*p=mypub.hello_.data()[0];
*q=pcl->data[0];
memcpy(p,q,sizeof(q));
*/

//memcpy(&mypub.hello_.data(),&pcl->data[0],pcl->data.size()*sizeof(uint8_t));
//for(int i=0;i<pcl->data.size();i++)
for(int i=0;i<10000;i++)
{
	mypub.hello_.data()[i]=pcl->data[i];
	//ROS_INFO("%d",uint8_t(mypub.hello_.data()[i]));
}
//ROS_INFO("size of pcl data %d",pcl->data.size());
//ROS_INFO("size of hello data %d",sizeof(mypub.hello_.data()));

mypub.hello_.is_dense()=pcl->is_dense;
mypub.publish(false); 
}

void sub_pub::subtfCallback(const tf2_msgs::TFMessage::ConstPtr& tf)
{
    ROS_INFO("receive tf");
}

sub_pub::sub_pub()
{
   //wan_ip = "192.168.1.108";
   wan_ip="192.168.1.77" ;
   port = 5100;
   whitelist.push_back(wan_ip);
   count = 0;
   sleep = 100;
  sub_pcl= nh_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, boost::bind(&sub_pub::subpclCallback,this,_1));
  sub_tf = nh_.subscribe<tf2_msgs::TFMessage>("tf", 10, boost::bind(&sub_pub::subtfCallback,this,_1));
  sub_octomap=nh_.subscriber<octomap_msgs::Octomap>(octomap_topic_name,1,boost::bind(&sub_pub
  if (mypub.init(wan_ip, static_cast<uint16_t>(port), false, whitelist))
  {
       ROS_INFO("init dds success!");  
  }
     Domain::stopAll();
}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"sub_pub");

    sub_pub sub_pub1;

    ros::spin();
	
    return 0;
}
