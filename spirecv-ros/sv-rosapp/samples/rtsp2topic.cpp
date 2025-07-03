#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>


int resize_h(1080), resize_w(1920);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rstp2topic");
    ros::NodeHandle nh("~");
    //ros::Rate loop_rate(30); 
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/suav/pod/main_camera_images", 10);
    
    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open("rtsp://192.168.147.119/554");
    if (!cap.isOpened()) {
	    ROS_INFO("failed to open rtsp");
	    return -1;
    }

    while (ros::ok())
    {
        cap >> frame;       
        if (!frame.empty())
        {
            cv::resize(frame, frame, cv::Size(resize_w, resize_h));
            // 设置图像帧格式->bgr8
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            // 将图像通过话题发布出去
            image_pub.publish(msg);
        }
        ros::spinOnce();
        // 按照设定的帧率延时，ros::Rate loop_rate(30)
        //loop_rate.sleep();
    }

    cap.release();
}
