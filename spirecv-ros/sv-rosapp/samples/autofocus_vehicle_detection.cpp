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
#include "Interpreter.hpp"

using namespace std;

cv::Mat cam_image_copy;
cv::Mat img;
cv::Mat crop_img;
bool get_image = false;

cv::Mat img2;
cv::Mat refer_img;


void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_image;
    try
    {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    cam_image->image.copyTo(cam_image_copy);
        img = cam_image_copy.clone();
        //img2 = cam_image_copy.clone();//similarity based image
        get_image = true;
        //std::cout<<cam_image_copy.rows<<std::endl;
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

    float sum = 0.0;

    for (size_t i = 0; i < vec1.size(); ++i) {
        float diff = vec1[i] - vec2[i];
        sum += diff * diff;
    }

    return std::sqrt(sum);
}

// float cosine_similarity(const std::vector<float>& vec1, const std::vector<float>& vec2) {

//     if (vec1.size() != vec2.size()) {
//         std::cerr << "length of vecs is different!!!" << std::endl;
//         return 0.0;
//     }

//     float dot_product = 0.0;
//     float magnitude1 = 0.0;
//     float magnitude2 = 0.0;

//     for (size_t i = 0; i < vec1.size(); ++i) {
//         dot_product += vec1[i] * vec2[i];
//         magnitude1 += vec1[i] * vec1[i];
//         magnitude2 += vec2[i] * vec2[i];
//     }

//     magnitude1 = std::sqrt(magnitude1);
//     magnitude2 = std::sqrt(magnitude2);

//     // 避免除以零错误
//     if (magnitude1 == 0.0 || magnitude2 == 0.0) {
//         std::cerr << "向量模为零" << std::endl;
//         return 0.0;
//     }

//     // 计算余弦相似度
//     return dot_product / (magnitude1 * magnitude2);
// }

class Classifier {
public:
    Classifier() {
        targets.clear();
    }

    void new_target(std::vector<float> &t) {
        targets.push_back(t);
    }

    void update_target(std::vector<float> &t, int index) {
        assert(index >= 0 && index < targets.size());
        targets[index] = t;
    }

    int new_vector(std::vector<float> &t) {
        if (targets.empty()) {
            new_target(t);
            return targets.size() - 1;
        }
        else {
            int selected = -1;
	    	double simi = 100;
            for (int i = 0; i < targets.size(); i++) {
                std::cout << "dist: " << cosine_similarity(t, targets[i]) << std::endl;
                if (cosine_similarity(t, targets[i]) < simi) {
					simi = cosine_similarity(t, targets[i]);
                    selected = i;
                }
            }
			printf("Choose %d with %.2lf\n", selected, simi);
            if (selected >= 0 && simi <= THRESHOLD) {
                update_target(t, selected);
                return selected;
            }
            else {
                new_target(t);
                return targets.size() - 1;
            }
        }
    }

private:
    std::vector<std::vector<float>> targets;
    float THRESHOLD = 0.15;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "autofocus_vehicle_detection");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    // 更新频率为60HZ
    ros::Rate loop_rate(60);
    int uav_id = 1;
    std::string local_saving_path = "";

    nh.getParam("uav_id", uav_id);
    nh.getParam("local_saving_path", local_saving_path);
    image_transport::Subscriber image_subscriber;
    image_subscriber = it.subscribe("/usb_cam/image_raw", 10, cameraCallback);
    ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/suav/spirecv/autofocus_vehicle_detection", 1);

    Interpreter itpt1;
    static float data0[3 * 224 * 224];
    static float data1[3 * 224 * 224];
    static float prob0[1280];
    static float prob1[1280];
    std::vector<float> vec0;
    std::vector<float> vec1;

    // 实例化 通用目标 检测器类
    sv::CommonObjectDetector cod_init;
    sv::CommonObjectDetector cod_sub_region;

    cod_init.loadCameraParams(sv::get_home() + "/SpireCV/calib_webcam_1280x720.yaml");
    cod_sub_region.loadCameraParams(sv::get_home() + "/SpireCV/calib_webcam_1280x720.yaml");

    cod_init.loadAlgorithmParams(sv::get_home() + "/SpireCV/sv_algorithm_params_personvehicle_1280.json");
    cod_sub_region.loadAlgorithmParams(sv::get_home() + "/SpireCV/sv_algorithm_params_personvehicle_640.json");

    sv::AutoFocusObjectDetector afod(&cod_init, &cod_sub_region);
    afod.loadCameraParams(sv::get_home() + "/SpireCV/calib_webcam_1280x720.yaml");
    afod.loadAlgorithmParams(sv::get_home() + "/SpireCV/sv_algorithm_params.json");

    refer_img = cv::imread("/home/kk/Pictures/refer.png");
    cv::resize(refer_img, refer_img, cv::Size(224, 224));
    int index1 = 0;
    for (int c = 0; c < 3; ++c) {
        for (int h = 0; h < 224; ++h) {
            for (int w = 0; w < 224; ++w) {
                data1[index1++] = static_cast<float>(refer_img.at<cv::Vec3b>(h, w)[c]) / 255.0;
            }
        }
    }

    itpt1.doInference(data1, prob1, 1);
    //std::cout << "feature map size:"<< sizeof(prob1)/sizeof(prob1[0]) << std::endl;
    vec1.assign(prob1, prob1 + 1280);
    //std::cout << vec1.size() << std::endl;
    Classifier c;

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
            //cv::resize(img2, img2, cv::Size(afod.image_width, afod.image_height));

            // 执行通用目标检测
            afod.detect(img, tgts);

            if (local_saving_path.size() > 0)
            {
                if (frame_id ==1){
                    vw.setup(local_saving_path, cv::Size(img.cols, img.rows), 25, true);
                }
                vw.write(img, tgts);
            }
            // 可视化检测结果，叠加到img上
            //sv::drawTargetsInFrame(img, tgts);

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
                crop_img = img(cv::Rect((tgts.targets[i].cx-tgts.targets[i].w/2)*tgts.width, (tgts.targets[i].cy-tgts.targets[i].h/2)*tgts.height, \
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
                vec0.assign(prob0, prob0 + 1280);


                spirecv_msgs::Target ros_target;
                if (tgts.targets[i].cx + tgts.targets[i].w / 2 < 0.95 &&
                    tgts.targets[i].cx - tgts.targets[i].w / 2 > 0.05 &&
                    tgts.targets[i].cy + tgts.targets[i].h / 2 < 0.95 &&
                    tgts.targets[i].cy - tgts.targets[i].h / 2 > 0.05
                )
                {
                    std::cout << "----------------------------------------------" << std::endl;
                    float similarity01 = cosine_similarity(vec0, vec1);
                    int id_res = c.new_vector(vec0);
                    std::cout << "similarity: " << similarity01 << std::endl;
                    std::cout << "vehicle_id: " << id_res << std::endl;
                    std::cout << "----------------------------------------------" << std::endl;
                    ros_target.score = similarity01;
                    ros_target.category_id = id_res;
                }
                else {
                    ros_target.score = 100;
                    ros_target.category_id = 100;
                }

                ros_target.cx = tgts.targets[i].cx;
                ros_target.cy = tgts.targets[i].cy;
                ros_target.w = tgts.targets[i].w;
                ros_target.h = tgts.targets[i].h;
                ros_target.angle = 0.;

                //ros_target.score = tgts.targets[i].score;
                ros_target.category = tgts.targets[i].category;
                //ros_target.category_id = tgts.targets[i].category_id;

                ros_target.los_ax = tgts.targets[i].los_ax;
                ros_target.los_ay = tgts.targets[i].los_ay;

                ros_target.px = tgts.targets[i].px;
                ros_target.py = tgts.targets[i].py;
                ros_target.pz = tgts.targets[i].pz;
                ros_tgts.targets.push_back(ros_target);
            }
            spirecv_msg_pub.publish(ros_tgts);

            sv::drawTargetsInFrame(img, tgts);
            cv::namedWindow("vehicle_det", cv::WINDOW_KEEPRATIO);
			cv::resizeWindow("vehicle_det", 640, 480);
            cv::imshow("vehicle_det", img);
            cv::waitKey(10);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
