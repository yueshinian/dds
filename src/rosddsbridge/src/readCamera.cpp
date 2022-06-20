#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <condition_variable>
#include <mutex>
using namespace std;
 
// OpenCV includes
#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#define LOG 0

int saveflag=0;
int count=0;
void video2image(ros::NodeHandle nh, std::string topicName, std::string url)
{
    cout<<"waiting front video"<<endl;
    /*设置图片节点*/
    image_transport::ImageTransport it(nh);
    /*设置图片的发布者*/
    image_transport::Publisher videoPub = it.advertise(topicName, 1);
    /*设置存放摄像头图像的变量*/    
    VideoCapture videoCapture;
    /*设置cvImage的智能指针*/
    cv_bridge::CvImagePtr frame;
    /*初始化CvImage智能指针，CvImage为Mat与ROS图像之间转换的载体*/
    frame = boost::make_shared<cv_bridge::CvImage>();
    /*设置ROS图片为BGR且每个像素点用1个字节来表示类似于CV_8U*/
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    videoCapture.open(url);
    while (!videoCapture.isOpened())
    {
        videoCapture.open(url);
        cout<<"open front camera failed!"<<endl;
        if(!ros::ok())
            break;
    }
    cout<<"OPENED FRONT VIDEO"<<endl;
    ros::Rate loop_rate(10);
    int count=0;
    while (ros::ok()) 
    {
        /*将摄像头获取到的图像存放在frame中的image*/
        videoCapture >> frame->image;
        /*判断是否获取到图像，若获取到图像，将其转化为ROS图片*/
        if (!(frame->image.empty()))
        {
            frame->header.stamp = ros::Time::now();
            videoPub.publish(frame->toImageMsg());
            if(LOG) ROS_INFO("PUB front video seq %d data size ",count++);
        }
        //loop_rate.sleep();
    }
    return;
}

void video2image2(ros::NodeHandle nh, std::string topicName, std::string url)
{
    cout<<"waiting back video"<<endl;
    /*设置图片节点*/
    image_transport::ImageTransport it(nh);
    /*设置图片的发布者*/
    image_transport::Publisher videoPub = it.advertise(topicName, 1);
    /*设置存放摄像头图像的变量*/    
    VideoCapture videoCapture;
    /*设置cvImage的智能指针*/
    cv_bridge::CvImagePtr frame;
    /*初始化CvImage智能指针，CvImage为Mat与ROS图像之间转换的载体*/
    frame = boost::make_shared<cv_bridge::CvImage>();
    /*设置ROS图片为BGR且每个像素点用1个字节来表示类似于CV_8U*/
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    videoCapture.open(url);
    while (!videoCapture.isOpened())
    {
        videoCapture.open(url);
        cout<<"open back camera failed!"<<endl;
        if(!ros::ok())
            break;
    }
    cout<<"OPENED BACK VIDEO"<<endl;
    ros::Rate loop_rate(10);
    int count=0;
    while (ros::ok()) 
    {
        /*将摄像头获取到的图像存放在frame中的image*/
        videoCapture >> frame->image;
        /*判断是否获取到图像，若获取到图像，将其转化为ROS图片*/
        if (!(frame->image.empty()))
        {
            frame->header.stamp = ros::Time::now();
            videoPub.publish(frame->toImageMsg());
            if(LOG) ROS_INFO("PUB backvideo seq %d data size ",count++);
        }

        if(saveflag==1)
{
	std::stringstream convert;
            convert<<"/home/hit/ddscomm/src/rosddsbridge/saveImage"<< count++<<".jpg";
sensor_msgs::ImagePtr msg=frame->toImageMsg();
std::cout<<"image data: "<<msg->data.size()<<std::endl;
	//cv::imshow("1",cv_bridge::toCvShare(msg)->image);
	//cvWaitKey(10000);
        cv::imwrite(convert.str(),cv_bridge::toCvShare(msg)->image);
	saveflag=0;
	std::cout<<convert.str()<<std::endl;
}
        //loop_rate.sleep();
    }
    return;
}

void cinchar()
{
	while(ros::ok())
{
	std::cout<<"input a char "<<std::endl;
        std::cin>>saveflag;
	std::cout<<"saveflag value: "<<saveflag<<std::endl;
if (saveflag==9){
break;
}
}
return;
}

int main( int argc, char** argv )
{
    
    ros::init(argc,argv,"readCamera");
    ros::NodeHandle nh;
    /*
    //设置图片节点
    image_transport::ImageTransport it(nh);
    //设置图片的发布者
    image_transport::Publisher frontVideo = it.advertise("/frontvideo", 1);
    image_transport::Publisher backVideo = it.advertise("/backvideo", 1);
    //设置存放摄像头图像的变量    
    VideoCapture frontCapture;
    VideoCapture backCapture;
    //设置cvImage的智能指针
    cv_bridge::CvImagePtr frontFrame;
    cv_bridge::CvImagePtr backFrame;
    //初始化CvImage智能指针，CvImage为Mat与ROS图像之间转换的载体
    frontFrame = boost::make_shared<cv_bridge::CvImage>();
    backFrame = boost::make_shared<cv_bridge::CvImage>();
    //设置ROS图片为BGR且每个像素点用1个字节来表示类似于CV_8U
    frontFrame->encoding = sensor_msgs::image_encodings::BGR8;
    backFrame->encoding = sensor_msgs::image_encodings::BGR8;
    */

    //readcamera *frontvideo = new readcamera(nh,"frontvideo",0);
    //readcamera *backvideo = new readcamera(nh,"backvideo","rtsp://192.168.1.100:554/user=admin&password=&channel=1&stream=0.sdp?");
    std::thread frontCamera(video2image,nh,"frontvideo","rtsp://192.168.1.100:554/user=admin&password=&channel=1&stream=0.sdp?");
    cout<<"frontCamera thread id "<<frontCamera.get_id()<<endl;
    ros::Duration(0.5).sleep();
    std::thread backCamera(video2image2,nh,"backvideo","rtsp://192.168.1.101:554/user=admin&password=&channel=1&stream=0.sdp?");
    cout<<"backCamera thread id "<<backCamera.get_id()<<endl;
    std::thread cinChar(cinchar);
    

    frontCamera.join();
    backCamera.join();
    cinChar.join();

    return 0;
 
}
