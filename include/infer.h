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
#include "batch_pipeline_base.hpp"
#if USE_SOPHON
#include "sophon/resnet_sophon.hpp"
#include "sophon/sophon_pipeline.hpp"
#endif
#if USE_RKNN
#include "rknn/rknn_pipeline.hpp"
#endif
#if USE_NVIDIA
#include "nvidia/vehicle_color.hpp"
#include "nvidia/nvidia_pipeline.hpp"
#endif
#include "shm/shmmem.h"
#include "shm/packet.h"
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
    int camera_id;
}ModelInputData;

// 颜色分类结果
 struct VehicleColorResult{
    int tracker_id;
    int vehicle_color;
    float confidence;
    int camera_id;
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
    int camera_id;
};

// 全局 batch pipeline 接口（平台无关）
#if USE_SOPHON || USE_RKNN || USE_NVIDIA
inline std::unique_ptr<BatchPipeline> g_pipeline;
inline SafeQueue<InferResult, 2> g_post_queue;
inline std::vector<SafeQueue<CameraResult, 2>> g_result_queues;
// RKNN 统一队列：4路摄像头推帧，3个推理线程消费
inline SafeQueue<BatchFrameData, 30> g_infer_queue;
// Sophon 仍使用 per-camera 队列；容量 2 以吸收 pre_thread 攒批等待期间的输入帧，同时降低图像积压延迟
inline std::vector<SafeQueue<BatchFrameData, 2>> g_cam_frame_queues;
#endif

// per-camera ROI 配置（按 camera_id = infer_params 索引）。
// 仅 batch 模式 ROI 生成时使用；未被 camera_roi 配置覆盖的相机用默认值（回退到全局 ROI 参数）。
struct CameraRoiConfig {
    bool has_cfg = false;    // 该相机是否在 camera_roi 列表中显式配置
    bool enabled = true;
    float height_ratio = 0.5f;  // ROI 高度 = 原图高 * 比例
    float width_ratio = 0.5f;   // ROI 宽度 = 原图宽 * 比例
    float y_ratio = 0.0f;       // ROI 框左上角纵向偏移（原图高 * 比例，0=顶部）
    float x_ratio = 0.5f;       // ROI 框左上角横向偏移（原图宽 * 比例，0.5=居中）
};

#if USE_NVIDIA
// NVIDIA batch 模式开关（运行时配置）：false=单路同步，true=镜像 SOPHON 的 batch 流水线
inline bool g_nvidia_batch_mode = false;
// per-camera ROI 配置表（索引 = camera_id，在 infer_nodelet 构建 infer_params 后填充）
inline std::vector<CameraRoiConfig> g_camera_roi;
// 逐环节每路帧计数（用于定位帧在哪一环丢失）：input=进入相机队列, pre=pre_thread取到, post=分发到结果队列
inline std::vector<long long> g_frame_cnt_input;
inline std::vector<long long> g_frame_cnt_pre;
inline std::vector<long long> g_frame_cnt_post;
// ROI 调试：保存裁剪的 ROI 图与带 ROI 框的原图（仅 batch 模式 ROI 启用时使用）
inline bool g_roi_save_debug = false;
inline std::string g_roi_save_path = "/tmp/";
#endif

#if USE_SOPHON
// 全局共享的颜色分类与抛洒物模型（每路不再各自加载一份）
inline std::unique_ptr<RESNET> g_color_model;
inline SafeQueue<ModelInputData, 64> g_color_queue;
inline std::vector<SafeQueue<VehicleColorResult, 3>> g_color_result_queues;
inline std::unique_ptr<Detector> g_abandon_model;
inline SafeQueue<AbandonInputData, 16> g_abandon_queue;
inline std::vector<SafeQueue<DetectorRetDatas, 3>> g_abandon_result_queues;
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
                       bool save_img_flag,
                       std::string write_path,
                       int min_points_len);
    void setShmParam(const std::string& shm_name);
    void setRoiParams(bool roi_enabled, float roi_height_ratio, float roi_width_ratio,
                      float roi_y_ratio, float roi_x_ratio);
    void setProcessDelayPrintInterval(int interval_sec) { if (interval_sec > 0) process_delay_print_interval = interval_sec; }

    void setCameraId(int id) { camera_id_ = id; }
    int getCameraId() const { return camera_id_; }

    int processRadarCamera(int index);
    // 在图像上绘制 ROI 增强检测区域框（batch / 非 batch 路径共用）
    void drawRoiBox(cv::Mat& img);
#if USE_SOPHON || USE_RKNN || USE_NVIDIA
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
#if !USE_SOPHON && !USE_RKNN
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
    // 本机接收该帧的本地墙钟(ms)，用于在发布点计算处理延时（与数据源时钟无关）
    int64_t receive_local_ms = 0;
    int process_delay_print_interval = 20;  // 处理延时打印间隔(秒)
    long long last_delay_print_ms = 0;      // 上次打印处理延时的本地墙钟(ms)
    std::atomic<int> publish_hz{0};

    std::string camera_type;
    std::string camera_direction;
    std::string pole_name;
    int vehicle_color_rate = 10;
    int abandon_rate = 5;
    int min_points_len = 30;
    bool write_flag = false;
    bool save_img_flag = false;
    std::string write_path = "/tmp/";
    std::string byte_track_config_file;
    bool publish_img = true;
    bool draw_tracker = true;
    // ROI 增强可视化参数（绘制检测增强区域框）
    bool roi_enabled = false;
    float roi_height_ratio = 0.5f;
    float roi_width_ratio = 0.5f;
    float roi_y_ratio = 0.0f;
    float roi_x_ratio = 0.5f;
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

// 平台级流水线程（infer_pipeline_threads.cpp 中实现，由 nodelet 入口启动）
#if USE_RKNN
void rknn_infer_thread(int core_id);
#endif
#if USE_SOPHON || USE_NVIDIA
void batch_preprocess_thread(int batch_size, int max_full);
void batch_postprocess_thread();
#endif
#if USE_SOPHON
void color_infer_thread();
void abandon_infer_thread();
#endif

// ROS 订阅回调（infer_auxiliary.cpp 中实现）
void imgCallback(const sensor_msgs::ImageConstPtr& msg, int& index);
void trackCallback(const infer_nodelet::RadarTrackObjectProject::ConstPtr msg, int& index);
// 设置每相机杆号（诊断日志用，infer_auxiliary.cpp 中实现）
void setCamPoleName(int index, const std::string& pole_name);

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