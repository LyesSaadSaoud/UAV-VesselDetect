#include <iostream>
#include <string>
#include <math.h>
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include <sv_nonfree.h>
#include <ros/ros.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

cv::Mat cam_image_copy;
cv::Mat img;
cv::Mat mask;
cv::Mat crop_mask;
cv::Mat binary_mask;
cv::Mat uint8_mask;

float angle;
bool get_image = false;

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_image;
    try
    {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    cam_image->image.copyTo(cam_image_copy);
        img = cam_image_copy.clone();
        get_image = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    return;
}

int main(int argc, char *argv[])
{
  	ros::init(argc, argv, "autofocus_object_detection");
  	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);
	// 更新频率为60HZ
	ros::Rate loop_rate(60);
	int uav_id = 1;
	std::string local_saving_path = "";

	nh.getParam("uav_id", uav_id);
	nh.getParam("local_saving_path", local_saving_path);
	image_transport::Subscriber image_subscriber;
	image_subscriber = it.subscribe("/usb_cam/image_raw", 30, cameraCallback);
	ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/tuav/spirecv/autofocus_object_detection", 1);

	// 实例化 通用目标 检测器类
	sv::CommonObjectDetector cod_init;
	sv::CommonObjectDetector cod_sub_region;
	sv::VideoStreamer streamer;
	streamer.setup(cv::Size(640, 480), 8554, 4, "/live");
	
	cod_init.loadCameraParams(sv::get_home() + "/SpireCV/calib_webcam_1280x720.yaml");
	cod_sub_region.loadCameraParams(sv::get_home() + "/SpireCV/calib_webcam_1280x720.yaml");

	cod_init.loadAlgorithmParams(sv::get_home() + "/SpireCV/sv_algorithm_params_mbzirc_1280.json");
	cod_sub_region.loadAlgorithmParams(sv::get_home() + "/SpireCV/sv_algorithm_params_mbzirc_640.json");

	sv::AutoFocusObjectDetector afod(&cod_init, &cod_sub_region);
	afod.loadCameraParams(sv::get_home() + "/SpireCV/calib_webcam_1280x720.yaml");
	afod.loadAlgorithmParams(sv::get_home() + "/SpireCV/sv_algorithm_params.json");

	int frame_id = 0;
	sv::VideoWriter vw;

	while (ros::ok())
	{
		if (get_image == true)
		{
			get_image = false;
			// 实例化Spirecv的 单帧检测结果 接口类 TargetsInFrame
			sv::TargetsInFrame tgts(frame_id++);
			cv::resize(img, img, cv::Size(afod.image_width, afod.image_height));
                       streamer.stream(img);
			// 执行通用目标检测
			afod.detect(img, tgts);

			if (local_saving_path.size() > 0)
			{
				if (frame_id ==1){
					vw.setup(local_saving_path, cv::Size(img.cols, img.rows), 10, true);
				}
				vw.write(img, tgts);
			}
			// 可视化检测结果，叠加到img上
			sv::drawTargetsInFrame(img, tgts);

			spirecv_msgs::TargetsInFrame ros_tgts;
			ros_tgts.header.frame_id = "frame";
			ros_tgts.header.stamp = ros::Time::now();
			ros_tgts.header.seq = 1;
			ros_tgts.frame_id = tgts.frame_id;
			ros_tgts.height = tgts.height;
			ros_tgts.width = tgts.width;
			ros_tgts.fps = tgts.fps;
			ros_tgts.fov_x = tgts.fov_x;
			ros_tgts.fov_y = tgts.fov_y;

			for (int i=0; i<tgts.targets.size(); i++)
			{
				cv::Mat mask = tgts.targets[i].getMask();

				if (!mask.empty())
				{
					crop_mask = mask(cv::Rect((tgts.targets[i].cx-tgts.targets[i].w/2)*tgts.width, (tgts.targets[i].cy-tgts.targets[i].h/2)*tgts.height, \
					tgts.targets[i].w * tgts.width, tgts.targets[i].h * tgts.height));
					int targetWidth = 100;  // 目标图像的宽度
					// 计算缩放比例
					float scaleX = static_cast<float>(targetWidth) / crop_mask.cols;
					// 计算目标尺寸
					int newWidth = static_cast<int>(crop_mask.cols * scaleX);
					int newHeight = static_cast<int>(crop_mask.rows * scaleX);
					cv::Size targetSize(newWidth, newHeight);
					cv::resize(crop_mask, crop_mask, targetSize);

					//cv::Mat result = crop_mask.clone();
					crop_mask.convertTo(uint8_mask, CV_8U, 255);
					cv::threshold(uint8_mask, binary_mask, 128, 255, cv::THRESH_BINARY);
					vector<vector<cv::Point>> contours;
					cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
					vector<cv::RotatedRect> rects(contours.size());
					for (size_t i = 0; i < contours.size(); i++)
					{
						rects[i] = minAreaRect(contours[i]);
						if (rects[i].size.width * rects[i].size.height < crop_mask.cols*crop_mask.rows*0.25)
						{
							continue;
						}
						if (rects[i].size.width > rects[i].size.height)
						{
							angle = rects[i].angle - 90.;
						}
						else
						{
							angle = rects[i].angle;
						}
						//printf("angle:%.3f\n", angle);
					}

					// for (size_t i = 0; i < contours.size(); i++)
					// {
					// 	cv::Point2f vertices[4];
					// 	rects[i].points(vertices);
					// 	for (int j = 0; j < 4; j++)
					// 	{
					// 		line(result, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255, 0, 0), 2);
					// 	}
					// }
					// cv::imshow("result", result);
					// cv::waitKey(10);

				}

				spirecv_msgs::Target ros_target;
				ros_target.cx = tgts.targets[i].cx;
				ros_target.cy = tgts.targets[i].cy;
				ros_target.w = tgts.targets[i].w;
				ros_target.h = tgts.targets[i].h;
				ros_target.angle = angle;

				ros_target.score = tgts.targets[i].score;
				ros_target.category = tgts.targets[i].category;
				ros_target.category_id = tgts.targets[i].category_id;

				ros_target.los_ax = tgts.targets[i].los_ax;
				ros_target.los_ay = tgts.targets[i].los_ay;

				ros_target.px = tgts.targets[i].px;
				ros_target.py = tgts.targets[i].py;
				ros_target.pz = tgts.targets[i].pz;
				ros_tgts.targets.push_back(ros_target);
			}
			spirecv_msg_pub.publish(ros_tgts);
                       //streamer.stream(img);
			cv::namedWindow("object_det", cv::WINDOW_KEEPRATIO);
			cv::resizeWindow("object_det", 640, 360);
			cv::imshow("object_det", img);
			cv::waitKey(10);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
