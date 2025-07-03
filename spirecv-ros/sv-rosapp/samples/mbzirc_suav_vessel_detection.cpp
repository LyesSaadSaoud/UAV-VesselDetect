#include <iostream>
#include <string>
#include <math.h>
#include <vector>
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include <sv_nonfree.h>
#include <ros/ros.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include "Interpreter.hpp"

using namespace std;
cv::Mat img;
cv::Mat crop_img;
bool get_image = false;
cv::Mat refer_img;
cv::Mat refer_img2;
cv::Mat img2;
float center;

std::string gstreamer_pipeline_G1(std::string web_cam_ip)
{
    return "rtspsrc location=rtsp://" + web_cam_ip + "/554?W=1920&H=1080&FPS=30&BR=4000000 latency=100 ! application/x-rtp,media=video ! rtph264depay ! parsebin ! nvv4l2decoder enable-max-performancegst=1 ! nvvidconv ! video/x-raw,format=(string)BGRx ! videoconvert ! appsink sync=false";
}


void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_image;
    try
    {
      cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img = cam_image->image.clone();
      img2 = cam_image->image.clone();
      //ROS_INFO("image width: %d ", img.rows);
      get_image = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    return;
}

/*float cosine_similarity(const std::vector<float>& vec1, const std::vector<float>& vec2)
{
  if (vec1.size() != vec2.size()) {
    std::cerr << "length of vecs is different!!!" << std::endl;
    return 0.0;
  }
  float sum = 0.0;
  for (size_t i = 0; i < vec1.size(); ++i) {
    float diff = vec1[i] - vec2[i];
    sum += diff * diff;
  }
  return std::sqrt(sum);
}*/

float cosine_similarity(const std::vector<float>& vec1, const std::vector<float>& vec2) {

     if (vec1.size() != vec2.size()) {
         std::cerr << "length of vecs is different!!!" << std::endl;
         return 0.0;
     }

     float dot_product = 0.0;
     float magnitude1 = 0.0;
     float magnitude2 = 0.0;

     for (size_t i = 0; i < vec1.size(); ++i) {
         dot_product += vec1[i] * vec2[i];
         magnitude1 += vec1[i] * vec1[i];
         magnitude2 += vec2[i] * vec2[i];
     }

     magnitude1 = std::sqrt(magnitude1);
     magnitude2 = std::sqrt(magnitude2);

     // 避免除以零错误
     if (magnitude1 == 0.0 || magnitude2 == 0.0) {
         std::cerr << "向量模为零" << std::endl;
         return 1.0;
     }

     // 计算余弦相似度
     return (1.0 - (dot_product / (magnitude1 * magnitude2)))*10;
 }

class Classifier {
public:
    Classifier(){
        targets.clear();
    }

    void set_refer(std::vector<float> referVec, std::vector<float> referVec2) {
        refer = referVec;
        refer2 = referVec2;

    }

    void new_target(std::vector<float> &t) {
        targets.push_back(t);
    }

    void update_target(std::vector<float> &t, int index) {
        assert(index >= 0 && index < targets.size());
        if (cosine_similarity(targets[index], refer) + cosine_similarity(targets[index], refer2) <= cosine_similarity(t, refer) + cosine_similarity(t, refer2)) {
            targets[index] = t;
        }
    }

    int new_vector(std::vector<float> &t, int trackID) {
        if (targets.empty()) {
            new_target(t);
            return targets.size() - 1;
        }
        else {
            int selected = -1;
	    	double simi = 100;
            printf("#%d to ", trackID);
            for (int i = 0; i < targets.size(); i++) {
            	double sim = cosine_similarity(t, targets[i]);
            	printf("T%d: %.2lf ", i, sim);
                if (sim < simi) {
                	simi = sim;
                  	selected = i;
                }
            }
			if (simi <= THRESHOLD) {
			    printf("\n#%d == T%d: %.2lf\n", trackID, selected, simi);
	            update_target(t, selected);
	            return selected;
			}
			else {
                printf("\n#%d new with %d frames\n", trackID, newCategoryCnts[trackID]);
				if (newCategoryCnts[trackID] >= newCategoryCntThreshold) {
					new_target(t);
					return targets.size() - 1;
				}
				else {
    				thisFrameTrackIDs.push_back(trackID);
					return 100;
				}
			}
        }
    }
    
    void newFrame() {
    	std::map<int, int> tmp;
    	for (auto &a: thisFrameTrackIDs) {
    		tmp[a] = newCategoryCnts[a] + 1;
    	}
        printf("\n");
    	newCategoryCnts = tmp;
    	thisFrameTrackIDs.clear();
    }

    size_t get_size() {
        return targets.size();
    }

    bool clear() {
        targets.clear();
        return targets.size() == 0;
    }

private:
    std::vector<std::vector<float>> targets;
    std::vector<float> refer;
    std::vector<float> refer2;
    float THRESHOLD = 1.0;
    std::map<int, int> newCategoryCnts;
    std::vector<int> thisFrameTrackIDs;
	const int newCategoryCntThreshold = 5;
} c;

void searchStateCallback(const std_msgs::Int8::ConstPtr & msg) {
    if (msg->data == 0) c.clear();

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mbzirc_suav_vessel_detection");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  // 更新频率为60HZ
  //ros::Rate loop_rate(60);
  std::string mode = "all";
  std::string video_decoder = "soft";
  std::string camera_params_yaml = sv::get_home() + "/spirecv-ros/sv-rosapp/camera_params/calib_webcam_1280x720.yaml";
  std::string local_saving_path = "";
  std::string local_saving_path_2 = "";
  bool show_image = true;
  bool save_video = true;

  nh.getParam("mode", mode);
  nh.getParam("video_decoder", video_decoder);
  nh.getParam("camera_parameters", camera_params_yaml);
  nh.getParam("local_saving_path", local_saving_path);
  nh.getParam("local_saving_path_2", local_saving_path_2);
  nh.getParam("show_image", show_image);
  nh.getParam("save_video", save_video);

  double windowWidth, windowHeight, windowX, windowY;
  nh.getParam("windowWidth", windowWidth);
  nh.getParam("windowHeight", windowHeight);
  nh.getParam("windowX", windowX);
  nh.getParam("windowY", windowY);
  printf("CAMERA_PARAMS: %s\n", camera_params_yaml.c_str());
  printf("LOCAL_SAVING_PATH: %s\n", local_saving_path.c_str());

  cv::VideoCapture cap;
  std::string web_cam_ip="192.168.2.119";
  std::string pipeline;
  image_transport::Subscriber image_subscriber;
  if (video_decoder == "hard"){
    pipeline = gstreamer_pipeline_G1(web_cam_ip);
    cap.open(pipeline, cv::CAP_GSTREAMER);
  }
  if (video_decoder == "soft"){
    image_subscriber = it.subscribe("/suav/pod/main_camera_images", 10, cameraCallback);
  }

  ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/suav/pod/vessel_det", 1);
  ros::Publisher spirecv_msg_pub_2 = nh.advertise<spirecv_msgs::TargetsInFrame>("/suav/pod/usv_detection", 1);
  auto searchStateSub = nh.subscribe("/suav/pod/searchState", 10, searchStateCallback);

  Interpreter itpt1;
  static float data0[3 * 224 * 224];
  static float data1[3 * 224 * 224];
  static float data2[3 * 224 * 224];
  static float prob0[256];
  static float prob1[256];
  static float prob2[256];
  std::vector<float> vec0;
  std::vector<float> vec1;
  std::vector<float> vec2;


  // 实例化 船检测器 类
  sv::CommonObjectDetector cod;
  cod.loadCameraParams(camera_params_yaml);
  cod.loadAlgorithmParams(sv::get_home() + "/spirecv-ros/sv-rosapp/algorithm_params/sv_algorithm_params_suav.json");
  sv::MultipleObjectTracker mot;
  mot.loadCameraParams(camera_params_yaml);
  mot.loadAlgorithmParams(sv::get_home() + "/spirecv-ros/sv-rosapp/algorithm_params/sv_algorithm_params_suav.json");
  mot.init(&cod);
  // 实例化 usv检测器 类
  sv::CommonObjectDetector cod_init;
  sv::CommonObjectDetector cod_sub_region;
  cod_init.loadCameraParams(sv::get_home() + "/spirecv-ros/sv-rosapp/camera_params/calib_webcam_1280x720.yaml");
  cod_sub_region.loadCameraParams(sv::get_home() + "/spirecv-ros/sv-rosapp/camera_params/calib_webcam_640x480.yaml");

  cod_init.loadAlgorithmParams(sv::get_home() + "/spirecv-ros/sv-rosapp/algorithm_params/sv_algorithm_params_usv_1280.json");
  cod_sub_region.loadAlgorithmParams(sv::get_home() + "/spirecv-ros/sv-rosapp/algorithm_params/sv_algorithm_params_usv_640.json");

  sv::AutoFocusObjectDetector afod(&cod_init, &cod_sub_region);
  afod.loadCameraParams(sv::get_home() + "/spirecv-ros/sv-rosapp/camera_params/calib_webcam_1280x720.yaml");
  afod.loadAlgorithmParams(sv::get_home() + "/spirecv-ros/sv-rosapp/algorithm_params/sv_algorithm_params_suav.json");

  refer_img = cv::imread(sv::get_home() + "/spirecv-ros/sv-rosapp/1.png");
  refer_img2 = cv::imread(sv::get_home() + "/spirecv-ros/sv-rosapp/default.jpeg");

  cv::resize(refer_img, refer_img, cv::Size(224, 224));
  cv::resize(refer_img2, refer_img2, cv::Size(224, 224));

  int index1 = 0;
  for (int c = 0; c < 3; ++c) {
    for (int h = 0; h < 224; ++h) {
      for (int w = 0; w < 224; ++w) {
        index1++;
        data1[index1] = static_cast<float>(refer_img.at<cv::Vec3b>(h, w)[c]) / 255.0;
        data2[index1] = static_cast<float>(refer_img2.at<cv::Vec3b>(h, w)[c]) / 255.0;
      }
    }
  }

  itpt1.doInference(data1, prob1, 1);
  itpt1.doInference(data2, prob2, 1);
  vec1.assign(prob1, prob1 + 256);
  vec2.assign(prob2, prob2 + 256);
  c.set_refer(vec1, vec2);

  int frame_id = 0;
  sv::VideoWriter vw;
  sv::VideoWriter vw_2;
  //sv::VideoStreamer streamer;
  //streamer.setup(cv::Size(640, 480), 8554, 2, "/live");

  while (ros::ok())
  {
    if (video_decoder == "hard"){
      cap >> img;
      img2 = img.clone();
    }

    //if (get_image == true)
    if (!img.empty() || get_image == true)
    {

      get_image = false;
      frame_id++;
      sv::TargetsInFrame tgts_all(frame_id);
      sv::TargetsInFrame tgts(frame_id);
      sv::TargetsInFrame tgts_usv(frame_id);
      //cv::resize(img, img, cv::Size(1280, 720));
      //cv::resize(img2, img2, cv::Size(1280, 720));
      if(mode == "vessel" or mode == "all")
      {
      	mot.track(img, tgts_all);
        tgts.frame_id = tgts_all.frame_id;
        tgts.height = tgts_all.height;
        tgts.width = tgts_all.width;
        tgts.fps = tgts_all.fps;
        tgts.fov_x = tgts_all.fov_x;
        tgts.fov_y = tgts_all.fov_y;
        for (int i=0; i<tgts_all.targets.size(); i++){
          if(tgts_all.targets[i].category == "boat"){
            tgts.targets.push_back(tgts_all.targets[i]);
          }
        }
        if (local_saving_path.size() > 0 && save_video){
          if (frame_id ==1){
            vw.setup(local_saving_path, cv::Size(img.cols, img.rows), 25, true);
          }
          vw.write(img, tgts);
        }

      }
      if(mode == "usv" or mode == "all")
      {
      	afod.detect(img, tgts_usv);
        if (local_saving_path_2.size() > 0 && save_video){
          if (frame_id ==1){
            vw_2.setup(local_saving_path_2, cv::Size(img.cols, img.rows), 25, true);
          }
          vw_2.write(img, tgts_usv);
        }

      }
      sv::drawTargetsInFrame(img, tgts);
      sv::drawTargetsInFrame(img, tgts_usv);
      // ROS_INFO("image2 width: %d ", img.rows);
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

      spirecv_msgs::TargetsInFrame ros_tgts_usv;
      ros_tgts_usv.header.frame_id = "frame";
      ros_tgts_usv.header.stamp = ros::Time::now();
      ros_tgts_usv.header.seq = 1;
      ros_tgts_usv.frame_id = tgts_usv.frame_id;
      ros_tgts_usv.height = tgts_usv.height;
      ros_tgts_usv.width = tgts_usv.width;
      ros_tgts_usv.fps = tgts_usv.fps;
      ros_tgts_usv.fov_x = tgts_usv.fov_x;
      ros_tgts_usv.fov_y = tgts_usv.fov_y;

      for (int i=0; i<tgts.targets.size(); i++)
      {
        crop_img = img2(cv::Rect((tgts.targets[i].cx-tgts.targets[i].w/2)*tgts.width+1., (tgts.targets[i].cy-tgts.targets[i].h/2)*tgts.height+1., \
          tgts.targets[i].w * tgts.width, tgts.targets[i].h * tgts.height));
        cv::namedWindow("cropped", cv::WINDOW_KEEPRATIO);
        cv::setWindowProperty("cropped", cv::WND_PROP_TOPMOST, 1);
        cv::resizeWindow("cropped", 224, 224);
        cv::moveWindow("cropped", windowX + windowWidth - 224, windowY + windowHeight + 100);
        cv::resize(crop_img, crop_img, cv::Size(224, 224));
        cv::imshow("cropped", crop_img);
        cv::waitKey(1);
        int index0 = 0;
        for (int c = 0; c < 3; ++c){
          for (int h = 0; h < 224; ++h){
            for (int w = 0; w < 224; ++w){
              data0[index0++] = static_cast<float>(crop_img.at<cv::Vec3b>(h, w)[c]) / 255.0;
            }
          }
        }
        itpt1.doInference(data0, prob0, 1);
        vec0.assign(prob0, prob0 + 256);
        float similarity01 = cosine_similarity(vec0, vec1);
        float similarity02 = cosine_similarity(vec0, vec2);
        //float similarity = (similarity01 + similarity02) * 0.5;
        float similarity = min(similarity01 , similarity02);
        printf("#\n similarity01: %.2lf, similarity02: %.2lf, similarity: %.2lf\n", similarity01, similarity02, similarity);
        center = tgts.targets[i].cx;
        spirecv_msgs::Target ros_target;

        double edgeThreshold = 0.01;
        if (
            tgts.targets[i].cx + tgts.targets[i].w / 2 < 1 - edgeThreshold &&
            tgts.targets[i].cx - tgts.targets[i].w / 2 > edgeThreshold &&
            tgts.targets[i].cy + tgts.targets[i].h / 2 < 1 - edgeThreshold &&
            tgts.targets[i].cy - tgts.targets[i].h / 2 > edgeThreshold
        ){
        	int trackedID = tgts.targets[i].tracked_id;
            int id_res = c.new_vector(vec0, trackedID);

            printf("#%d: T%d with %.2lf, total %ld\n", trackedID, id_res, similarity, c.get_size());
            ros_target.score = similarity;
            ros_target.category_id = id_res;
        }
        else {
            ros_target.score = 100;
            ros_target.category_id = 100;
        };


        ros_target.cx = tgts.targets[i].cx;
        ros_target.cy = tgts.targets[i].cy;// + tgts.targets[i].h / 2;
        ros_target.w = tgts.targets[i].w;
        ros_target.h = tgts.targets[i].h;
        ros_target.angle = 0.;

        //ros_target.score = tgts.targets[i].score;
        ros_target.category = tgts.targets[i].category;
        //ros_target.category_id = tgts.targets[i].category_id;
        ros_target.tracked_id = tgts.targets[i].tracked_id;
        ros_target.los_ax = tgts.targets[i].los_ax;
        ros_target.los_ay = tgts.targets[i].los_ay;

        ros_target.px = tgts.targets[i].px;
        ros_target.py = tgts.targets[i].py;
        ros_target.pz = tgts.targets[i].pz;
        ros_tgts.targets.push_back(ros_target);
      }
      c.newFrame();

      for (int i=0; i<tgts_usv.targets.size(); i++)
      	{
        	spirecv_msgs::Target ros_target_usv;
        	ros_target_usv.cx = tgts_usv.targets[i].cx;
        	ros_target_usv.cy = tgts_usv.targets[i].cy;// + tgts_usv.targets[i].h / 2;
		      ros_target_usv.w = tgts_usv.targets[i].w;
		      ros_target_usv.h = tgts_usv.targets[i].h;
		      ros_target_usv.angle = 0.;
          ros_target_usv.category = tgts_usv.targets[i].category;
          //ros_target.category_id = id_res;

          ros_target_usv.los_ax = tgts_usv.targets[i].los_ax;
          ros_target_usv.los_ay = tgts_usv.targets[i].los_ay;

          ros_target_usv.px = tgts_usv.targets[i].px;
          ros_target_usv.py = tgts_usv.targets[i].py;
          ros_target_usv.pz = tgts_usv.targets[i].pz;
          ros_tgts_usv.targets.push_back(ros_target_usv);

      }

      spirecv_msg_pub.publish(ros_tgts);
      spirecv_msg_pub_2.publish(ros_tgts_usv);
      //streamer.stream(img);

      if (show_image){
        cv::namedWindow("suav_vessel_det", cv::WINDOW_KEEPRATIO);
        cv::setWindowProperty("suav_vessel_det", cv::WND_PROP_TOPMOST, 1);
        cv::resizeWindow("suav_vessel_det", windowWidth, windowHeight);
        cv::moveWindow("suav_vessel_det", windowX, windowY);
        cv::line(img, cv::Point(img.cols / 2 - 50, img.rows / 2), cv::Point(img.cols / 2 + 50, img.rows / 2), cv::Scalar(0, 255, 0), 3);
        cv::line(img, cv::Point(img.cols / 2, img.rows / 2 - 50), cv::Point(img.cols / 2, img.rows / 2 + 50), cv::Scalar(0, 255, 0), 3);        
        cv::imshow("suav_vessel_det", img);
      }
      cv::waitKey(1);
    }
    ros::spinOnce();
  }

  return 0;
}
