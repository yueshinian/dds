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
//C++ lib
#include <vector>
#include <string>
       
int main(int argc ,char **argv)
{
    ros::init(argc, argv, "tfBr");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    int x=0;
    int y=0;
    while(ros::ok())
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x,y,0.0) );
        tf::Quaternion q;
        q.setRPY(0,0,0.1);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera"));
        //ros::spinOnce();
        loop_rate.sleep();
        x++;
        y++;
    }

    return 0;
}

