
// ros 相关
#include <nodelet/nodelet.h>
#include <ros/ros.h> 
#include <std_msgs/Float32.h> 
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "infer_nodelet/RadarTrackObjectProject.h"
#include "infer_nodelet/RadarTrackObjectProjectSingle.h"
#include "infer_nodelet/ImageDetectObject.h"
#include "infer_nodelet/ImageDetectObjectSingle.h"


// 检测跟踪相关
#include "detector.hpp"
#include <numeric>
#include <time.h>
#include <chrono>
#include <thread>
#include<vector>
#include <opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"

// 跟踪相关
// #include "tracker.h"
// #include "utils.h"
#include "safeq.hpp"
#include "bytetrack.h"
using namespace cv;


typedef struct InferParam{
    std::string camera_type;            // 远近枪标志
    std::string camera_direction;       // 上下行标志
    std::string receive_img_topic;      // 接收图像主题
    std::string receive_radar_topic;    // 接收雷达主题
    std::string publish_img_topic;      // 发布图像主题
    std::string publish_fps;            // 发布频率主题
    std::string publish_img_result;     // 发布图像检测数据主题

}InferParam;


// 颜色分类 & 车牌识别
typedef struct ModelInputData{
    cv::Mat im;
    int tracker_id;
}ModelInputData;

// 颜色分类结果
typedef struct VehicleColorResult{
    int vehicle_color;
    int tracker_id;
    float confidence;
}VehicleColorResult;

// 车牌识别结果
typedef struct PlateRecResult{
    std::string plate;
    float confidence;
    int tracker_id;
}PlateRecResult;

// 抛洒物检测
typedef struct AbandonInputData{
    cv::Mat im;
    std::vector<DetectorRetData>  data ;
}AbandonInputData;

typedef struct DetectOutputData{
    cv::Mat im;
    std::vector<DetectorRetData>  data ;
    std_msgs::Header header;

}DetectOutputData;

SafeQueue<sensor_msgs::ImageConstPtr, 2> imgQueue[6]; // 图像队列
SafeQueue<DetectOutputData, 2> imgRetQueue[6]; // 图像队列

SafeQueue<infer_nodelet::RadarTrackObjectProject::ConstPtr, 2> trackQueue[6];  // 雷达跟踪队列

// 可视化和车颜色一致
// 0，默认为绿色。 1：白，2：黑，3：红，4：黄，5：灰，6：蓝，7：绿，8：棕"
// 0，默认为绿色。 1：白，2：黑，3：红，4：黄，5：灰，6：蓝，7：绿，8：棕"
std::vector<cv::Scalar> colors = {cv::Scalar(0, 255, 0),cv::Scalar(255, 255, 255), cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 255), cv::Scalar(169, 169, 169), cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(50, 0, 50), cv::Scalar(255,0,0), cv::Scalar(0,0,0)};


// 输入的是抛洒物和原图，除以的是抛洒物的面积，不是并集.
float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1, float ymax1)
{
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) ;
    return u <= 0.f ? 0.f : (i / u);
}


void bytetrack_yaml_parse(const std::string& config_path,
                          bytetrack_params& params) {
  std::ifstream file(config_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open config file: " << config_path << std::endl;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::stringstream ss(line);
    std::string key, value;
    std::getline(ss, key, ':');
    std::getline(ss, value);
    key = key.substr(key.find_first_not_of(" \t\r\n"));
    value.erase(value.find_last_not_of(" \t\r\n") + 1);
    if (key == "CONF_THRE") {
      std::istringstream iss(value);
      iss >> params.conf_thresh;
    } else if (key == "NMS_THRE") {
      std::istringstream iss(value);
      iss >> params.nms_thresh;
    } else if (key == "TRACK_THRESH") {
      std::istringstream iss(value);
      iss >> params.track_thresh;
    } else if (key == "MATCH_THRESH") {
      std::istringstream iss(value);
      iss >> params.match_thresh;
    } else if (key == "TRACK_BUFFER") {
      std::istringstream iss(value);
      iss >> params.track_buffer;
    } else if (key == "FRAME_RATE") {
      std::istringstream iss(value);
      iss >> params.frame_rate;
    } else if (key == "MIN_BOX_AREA") {
      std::istringstream iss(value);
      iss >> params.min_box_area;
    }
  }
}
