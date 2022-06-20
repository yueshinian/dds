#include <iostream>
#include <string>
#include <sstream>
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
 
int main( int argc, char** argv )
{
    int sample=6;
    switch(sample){
	case 0:
	{
            cout << "Sample 0, Mat zeros" << endl;
	    Mat m= Mat::zeros(5,5, CV_32F);
	    cout << m << endl;
            break;
	}
        case 1:
        {	
            cout << "Sample 0, Mat ones" << endl;
	    Mat m= Mat::ones(5,5, CV_32F);
	    cout << m << endl;
	    break;
	}
	case 2:
	{	
            cout << "Sample 0, Mat eye" << endl;
            Mat m= Mat::eye(5,5, CV_32F);
	    cout << m << endl;
 
	    Mat a= Mat::eye(Size(3,2), CV_32F);
	    Mat b= Mat::ones(Size(3,2), CV_32F);
	    Mat c= a+b;
	    Mat d= a-b;
            cout << "Sample 0, 矩阵元素和差" << endl;
	    cout << a << endl;
	    cout << b << endl;
	    cout << c << endl;
	    cout << d << endl;
	    break;
	}
	case 3:
	{	
            cout << "Sample 0, Mat operations:" << endl;
	    Mat m0= Mat::eye(3,3, CV_32F);
	    m0=m0+Mat::ones(3,3, CV_32F);
	    Mat m1= Mat::eye(2,3, CV_32F);
	    Mat m2= Mat::ones(3,2, CV_32F);
 
	    cout << "\nm0\n" << m0 << endl;
	    cout << "\nm1\n" << m1 << endl;
	    cout << "\nm2\n" << m2 << endl;
 
	    cout << "\nm1.*2\n" << m1*2 << endl;
	    cout << "\n(m1+2).*(m1+3)\n" << (m1+1).mul(m1+3) << endl;
	    cout << "\nm1*m2\n" << m1*m2 << endl;
	    cout << "\nt(m2)\n" << m2.t() << endl;
	    cout << "\ninv(m0)\n" << m0.inv() << endl;
	    break;
	}
	case 4:
	{
            Mat image= imread("/home/redwall/catkin_ws/src/redwall_arm_vision/src/lena.jpg", CV_LOAD_IMAGE_COLOR);
            int myRow=511;
            int myCol=511;
	    int val=*(image.data+myRow*image.cols*image.channels()+ myCol);
	    cout << "Pixel value: " << val << endl;
	    // 有imshow就会报段错误
	    // imshow("Lena", image);
	    break;
	}
	case 5:
	{
            Mat image= imread("/home/redwall/catkin_ws/src/redwall_arm_vision/src/lena.jpg", CV_LOAD_IMAGE_COLOR);
            int myRow=511;
	    int myCol=511;
	    int B=*(image.data+myRow*image.cols*image.channels()+ myCol + 0);
	    int G=*(image.data+myRow*image.cols*image.channels()+ myCol + 1);
	    int R=*(image.data+myRow*image.cols*image.channels()+ myCol + 2);
	    cout << "Pixel value (B,G,R): (" << B << "," << G << "," << R << ")" << endl;
	    break;
	}
	case 6:
	{
            ros::init(argc,argv,"image_color");
            ros::NodeHandle nh;    
            image_transport::ImageTransport it(nh);
            image_transport::Publisher pub = it.advertise("/image", 1);
            int count=0;
            /**************ROS与Opencv图像转换***********************/
            Mat image= imread("/home/yz/catkin_ws/ddscomm/src/rosddsbridge/config/earth.jpg", CV_LOAD_IMAGE_COLOR);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            ros::Rate loop_rate(10);
            while (nh.ok()) {
                pub.publish(msg);
                ROS_INFO("PUB IMAGE seq %d data size %d",count++,msg->data.size());
                ros::spinOnce();
                loop_rate.sleep();
            }
	}
 
        case 7:
        {
            ros::init(argc,argv,"image_gray");
            ros::NodeHandle nh;
            image_transport::ImageTransport it(nh);
            image_transport::Publisher pub = it.advertise("/image", 1);
            /**************ROS与Opencv图像转换***********************/
            Mat gray= imread("/catkin_ws/ddscomm/src/rosddsbridge/config/earth.jpg", CV_LOAD_IMAGE_GRAYSCALE);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
            ros::Rate loop_rate(1);
            while (nh.ok()) {
                pub.publish(msg);
                ROS_INFO("PUB IMAGE data size %d",msg->data.size());
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

         case 8:
        {
            ros::init(argc,argv,"image_become");
            ros::NodeHandle nh;
            image_transport::ImageTransport it(nh);
            image_transport::Publisher pub = it.advertise("/image", 1);
            /**************ROS与Opencv图像转换***********************/
            Mat image_= imread("/catkin_ws/ddscomm/src/rosddsbridge/config/earth.jpg", CV_LOAD_IMAGE_GRAYSCALE);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_).toImageMsg();
            ros::Rate loop_rate(0.2);
            int count=1;
            while (nh.ok()) {
            	if(count)
            	{
            	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
            	count--;
            }
            else
            {
            	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_).toImageMsg();
            	count++;
            }
                pub.publish(msg);
                ROS_INFO("PUB IMAGE data size %d",msg->data.size());
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        case 9:
        {
            ros::init(argc,argv,"video");
            ros::NodeHandle nh;
            /*设置图片节点*/
            image_transport::ImageTransport it(nh);
            /*设置图片的发布者*/
            image_transport::Publisher imgPub = it.advertise("/frontvideo", 1);
            /*设置存放摄像头图像的变量*/    
            VideoCapture frontCapture;
            /*设置cvImage的智能指针*/
            cv_bridge::CvImagePtr frame;
            /*初始化CvImage智能指针，CvImage为Mat与ROS图像之间转换的载体*/
            frame = boost::make_shared<cv_bridge::CvImage>();
            /*设置ROS图片为BGR且每个像素点用1个字节来表示类似于CV_8U*/
            frame->encoding = sensor_msgs::image_encodings::BGR8;
            frontCapture.open(0);   
            ros::Rate loop_rate(10);
            int count=1;

            while (nh.ok()) {
                /*将摄像头获取到的图像存放在frame中的image*/
                frontCapture >> frame->image;
                /*判断是否获取到图像，若获取到图像，将其转化为ROS图片*/
                if (!(frame->image.empty())){
                    frame->header.stamp = ros::Time::now();
                    imgPub.publish(frame->toImageMsg());
                }
                ROS_INFO("PUB IMAGE seq %d data size ",count++);
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    }
 
    return 0;
 
}
