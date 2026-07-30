
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
#include <memory>
#include<vector>
#include <opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"

// 跟踪相关+
#include "bytetrack.h"
#include "safeq.hpp"
#include "traffic_analyzer.h"
#if USE_SOPHON
#include "sophon/resnet_sophon.hpp"
#include "sophon/detector_sophon.hpp"
#endif
#include "shm/shmmem.h"
#include "shm/packet.h"
#if USE_NVIDIA
#include "nvidia/vehicle_color.hpp"
#endif
using namespace cv;


struct InferParam{
    std::string camera_type;            // 远近枪标志
    std::string camera_direction;        // 上下行标志
    std::string pole_name;               // 杆号
    std::string receive_img_topic;      //接收图像主题
    std::string publish_img_topic;      //发布图像主题
    std::string publish_img_result;     //发布图像检测数据主题
    std::string publish_fps;            //发布频率主题
    std::string receive_radar_topic;    //接收雷达主题
    std::string video_path;             //视频文件路径（test模式使用）
    std::string shm_name;               //共享内存名字（SHM模式）
    bool publish_img = true;             // Enable/disable image topic publishing
    bool draw_tracker = true;             // Enable/disable drawing on image

};


// 颜色分类 & 车牌识别
typedef struct ModelInputData{
    cv::Mat im;
    int tracker_id;
}ModelInputData;

// 颜色分类结果
 struct VehicleColorResult{
    int tracker_id;
    int vehicle_color;
    float confidence;
};

// 车牌识别结果
struct PlateRecResult{
    std::string plate;
    float confidence;
    int tracker_id;
};

// 抛洒物检测
struct AbandonInputData{
    cv::Mat im;
    std::vector<DetectorRetData>  data ;
};

#if USE_SOPHON
// Batch pipeline: 输入帧
struct BatchFrameData {
    cv::Mat mat;
    int camera_id;
    int frame_width;
    int frame_height;
    double img_time_sec = 0.0;
    double img_time_nsec = 0.0;
};

// Batch pipeline: 前向推理结果（供 post 线程消费）
struct InferResult {
    std::vector<bm_image> input_images;
    std::vector<bm_tensor_t> output_tensors;
    std::vector<std::pair<int, int>> txy_batch;
    std::vector<std::pair<float, float>> ratios_batch;
    std::vector<int> camera_ids;
    std::vector<cv::Mat> frames;
    std::vector<double> img_time_secs;
    std::vector<double> img_time_nsecs;
    int batch_size;
};

// 全局共享变量（声明为 inline，在头文件中定义）
inline std::unique_ptr<YoloV8_det> g_shared_detector;
inline SafeQueue<InferResult, 30> g_post_queue;

// 每个相机的推理结果（帧+检测结果），由 processResult 线程消费
struct CameraResult {
    cv::Mat frame;
    std::vector<DetectorRetData> detections;
    double img_time_sec = 0.0;
    double img_time_nsec = 0.0;
};
inline std::vector<SafeQueue<CameraResult, 3>> g_result_queues;

// 每个相机的原始帧队列（processRadarCamera 入队，batch_preprocess_thread 出队）
inline std::vector<SafeQueue<BatchFrameData, 3>> g_cam_frame_queues;
#endif

class InferDet {
public:
    void loadParam(image_transport::Publisher pub_img,
                   ros::Publisher pub_tracker,
                   ros::Publisher pub_fps,
                   const std::string camera_type,
                   const std::string camera_direction,
                   const std::string pole_name,
                   int vehicle_color_rate,
                   int abandon_rate,
                   bool publish_img = true,
                   bool draw_tracker = true);

    void load_det_model(std::string model_path, int mlu_infer_device, int num_class, int stride);
    void load_abandon_model(std::string model_path, int mlu_infer_device, int num_class, int stride);
#if USE_SOPHON || USE_NVIDIA
    void load_color_model(std::string model_path);
#endif
    void setWriteParam(std::string byte_track_config_file,
                       bool write_flag,
                       std::string write_path,
                       int min_points_len);
void setShmParam(const std::string& shm_name);

    void setCameraId(int id) { camera_id_ = id; }
    int getCameraId() const { return camera_id_; }

    int processRadarCamera(int index);
#if USE_SOPHON
    void processResult();
    void publishThread();
#endif
    void processHz();
#if USE_SOPHON || USE_NVIDIA
    void vehicleColor();
#endif
    void abandonDetect();
    void processVideoFile(std::string video_path, int index);
    TrafficAnalyzer traffic_analyzer;

private:
    image_transport::Publisher pub_img;
    ros::Publisher pub_tracker;
    ros::Publisher pub_fps;

    int camera_id_ = -1;
#if !USE_SOPHON
    Detector detector;
#endif
    Detector abandon_detector;
#if USE_SOPHON
    RESNET vehicle_color_detector;
#elif USE_NVIDIA
    VehicleColorDetector vehicle_color_detector;
#endif

#if USE_SOPHON || USE_NVIDIA
    SafeQueue<ModelInputData> vehicleColorQueue;
#endif
    SafeQueue<AbandonInputData> abandonRecQueue;
#if USE_SOPHON || USE_NVIDIA
    SafeQueue<VehicleColorResult> vehicleColorResultQueue;
#endif
    SafeQueue<DetectorRetDatas> abandonResultQueue;

    double img_time_sec = 0.0;
    double img_time_nsec = 0.0;
    double radar_time_sec = 0.0;
    double radar_time_nsec = 0.0;
    std::atomic<int> publish_hz{0};

    std::string camera_type;
    std::string camera_direction;
    std::string pole_name;
    int vehicle_color_rate = 10;
    int abandon_rate = 5;
    int min_points_len = 30;
    bool write_flag = false;
    std::string write_path = "/tmp/";
    std::string byte_track_config_file;
    bool publish_img = true;
    bool draw_tracker = true;
    bool use_shm = false;
    std::string shm_name;
    std::shared_ptr<ehawkeye::modules::units::shmmem> shm_reader;

    std::mutex mtx;

    cv::Mat img_src;

    // Publish slot: processResult writes latest frame+stracks, publishThread consumes
    struct PublishSlot {
        cv::Mat frame;
        std::vector<STrack> stracks;
        std::vector<DetectorRetData> abandon_results;
        bool ready = false;
        std::mutex mtx;
    };
    PublishSlot pub_slot_;

    struct CachedFrame {
        int frame_id;
        cv::Mat img;
    };
    std::deque<CachedFrame> frame_cache_;
    static const int MAX_CACHED_FRAMES = 30;
    int cache_interval_ = 15;

    void saveTrackMontage(int track_id, int class_id,
                          const std::vector<std::vector<float>>& track_points,
                          const std::string& save_dir,
                          const std::string& filename_prefix);
};

inline std::vector<SafeQueue<sensor_msgs::ImageConstPtr, 3>> imgQueue; // 图像队列
// SafeQueue<infer_nodelet::RadarTrackObjectProject, 10 > trackQueue;  // 雷达跟踪队列
inline std::vector<SafeQueue<infer_nodelet::RadarTrackObjectProject::ConstPtr, 3>> trackQueue;  // 雷达跟踪队列

// vehicle_color (模型输出 0-7): 0=白, 1=黑, 2=红, 3=黄, 4=灰, 5=蓝, 6=绿, 7=棕
inline std::vector<cv::Scalar> vehicle_colors = {
    cv::Scalar(255, 0, 255),  // 0: 默认
    cv::Scalar(255, 255, 255),  // 0: 白
    cv::Scalar(0, 0, 0),        // 1: 黑
    cv::Scalar(0, 0, 255),      // 2: 红
    cv::Scalar(0, 255, 255),    // 3: 黄
    cv::Scalar(128, 128, 128),  // 4: 灰
    cv::Scalar(255, 0, 0),      // 5: 蓝
    cv::Scalar(0, 255, 0),      // 6: 绿
    cv::Scalar(42, 42, 165),    // 7: 棕
};


// 输入的是抛洒物和原图，除以的是抛洒物的面积，不是并集.
inline float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1, float ymax1)
{
    float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    float i = w * h;
    float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) ;
    return u <= 0.f ? 0.f : (i / u);
}


inline void bytetrack_yaml_parse(const std::string& config_path,
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
    } else if (key == "TRACK_DEBUG") {
      std::istringstream iss(value);
      iss >> params.track_debug;
    }
  }
}

