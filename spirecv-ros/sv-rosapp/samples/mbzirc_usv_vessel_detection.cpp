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

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_image;
    try
    {
      cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      img = cam_image->image.clone();
      img2 = cam_image->image.clone();
      get_image = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    return;
}


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
     return 1.0 - (dot_product / (magnitude1 * magnitude2));
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
        if (cosine_similarity(targets[index], refer) + cosine_similarity(targets[index], refer2) >= cosine_similarity(t, refer) + cosine_similarity(t, refer2)) {
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
    float THRESHOLD = 0.1;
    std::map<int, int> newCategoryCnts;
    std::vector<int> thisFrameTrackIDs;
	const int newCategoryCntThreshold = 5;
} c;

void searchStateCallback(const std_msgs::Int8::ConstPtr & msg) {
    if (msg->data == 0) c.clear();

}
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mbzirc_usv_vessel_detection");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  std::string camera_params_yaml = sv::get_home() + "/spirecv-ros/sv-rosapp/camera_params/calib_webcam_1280x720.yaml";
  std::string local_saving_path = "";

  nh.getParam("camera_parameters", camera_params_yaml);
  nh.getParam("local_saving_path", local_saving_path);


  printf("CAMERA_PARAMS: %s\n", camera_params_yaml.c_str());
  printf("LOCAL_SAVING_PATH: %s\n", local_saving_path.c_str());
  image_transport::Subscriber image_subscriber;
  image_subscriber = it.subscribe("/usv/pod/main_camera_images", 10, cameraCallback);
  ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/usv/pod/vessel_det", 1);
  auto searchStateSub = nh.subscribe("/suav/pod/searchState", 10, searchStateCallback);

  Interpreter itpt1;
  static float data0[3 * 224 * 224];
  static float data1[3 * 224 * 224];
  static float data2[3 * 224 * 224];
  static float prob0[2048];
  static float prob1[2048];
  static float prob2[2048];
  std::vector<float> vec0;
  std::vector<float> vec1;
  std::vector<float> vec2;


  // 实例化 船检测器 类
  sv::CommonObjectDetector cod;
  cod.loadCameraParams(camera_params_yaml);
  cod.loadAlgorithmParams(sv::get_home() + "/spirecv-ros/sv-rosapp/algorithm_params/sv_algorithm_params_usv.json");
  sv::MultipleObjectTracker mot;
  mot.loadCameraParams(camera_params_yaml);
  mot.loadAlgorithmParams(sv::get_home() + "/spirecv-ros/sv-rosapp/algorithm_params/sv_algorithm_params_usv.json");
  mot.init(&cod);

  refer_img = cv::imread(sv::get_home() + "/spirecv-ros/sv-rosapp/refer.jpeg");
  refer_img2 = cv::imread(sv::get_home() + "/spirecv-ros/sv-rosapp/refer.jpeg");

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
  vec1.assign(prob1, prob1 + 2048);
  vec2.assign(prob2, prob2 + 2048);
  c.set_refer(vec1, vec2);

  int frame_id = 0;
  sv::VideoWriter vw;
  sv::VideoStreamer streamer;
  streamer.setup(cv::Size(640, 480), 8554, 2, "/live");

  while (ros::ok())
  {
    if (get_image == true)
    {
      get_image = false;
      frame_id++;
      sv::TargetsInFrame tgts_all(frame_id);
      sv::TargetsInFrame tgts(frame_id);
      cv::resize(img, img, cv::Size(1280, 720));
      cv::resize(img2, img2, cv::Size(1280, 720));

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
      if (local_saving_path.size() > 0 ){
        if (frame_id ==1){
          vw.setup(local_saving_path, cv::Size(img.cols, img.rows), 25, true);
        }
        vw.write(img, tgts);
      }
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
        crop_img = img2(cv::Rect((tgts.targets[i].cx-tgts.targets[i].w/2)*tgts.width+1., (tgts.targets[i].cy-tgts.targets[i].h/2)*tgts.height+1., \
          tgts.targets[i].w * tgts.width, tgts.targets[i].h * tgts.height));
        cv::resize(crop_img, crop_img, cv::Size(224, 224));

        int index0 = 0;
        for (int c = 0; c < 3; ++c){
          for (int h = 0; h < 224; ++h){
            for (int w = 0; w < 224; ++w){
              data0[index0++] = static_cast<float>(crop_img.at<cv::Vec3b>(h, w)[c]) / 255.0;
            }
          }
        }
        itpt1.doInference(data0, prob0, 1);
        vec0.assign(prob0, prob0 + 2048);
        float similarity01 = cosine_similarity(vec0, vec1);
        float similarity02 = cosine_similarity(vec0, vec2);
        float similarity = (similarity01 + similarity02) * 0.5;
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
        ros_target.cy = tgts.targets[i].cy + tgts.targets[i].h / 2;
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
      spirecv_msg_pub.publish(ros_tgts);
      streamer.stream(img);
      cv::namedWindow("usv_vessel_det", cv::WINDOW_KEEPRATIO);
      cv::setWindowProperty("usv_vessel_det", cv::WND_PROP_TOPMOST, 1);
      cv::resizeWindow("usv_vessel_det", 640, 360);
      cv::imshow("usv_vessel_det", img);

      cv::waitKey(1);
    }
    ros::spinOnce();
  }

  return 0;
}
