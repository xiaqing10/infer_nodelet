#include "log_macros.h"
#include <infer.h>
#include <iostream>
#include <string.h>
#include "device_params_config.h"
#include <algorithm>
#include <map>
#include <sys/stat.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#if USE_NVIDIA
#include <cuda_runtime.h>
#endif
// 注册Nodelet
#include <pluginlib/class_list_macros.h>

using namespace std;

namespace infer_ns {
    class DataNodelet : public nodelet::Nodelet {
    public:
        DataNodelet() = default;

    private:
        std::vector<ros::Subscriber> sub_imgs;
        std::vector<ros::Subscriber> sub_radars;
        int next_thread_idx = 0;

        // Helper function to create topic names
        std::string makeTopic(const std::string& prefix,
                             const std::string& pole_name,
                             const std::string& direction,
                             const std::string& focal_type) {
            return "/" + pole_name + "/" + direction + "/" + focal_type + prefix;
        }

        // Unified topic generation
        InferParam createTopicParams(const CameraInfo& cam, const std::string& pole_name) {
            InferParam p;
            p.camera_type = cam.focal_type;
            p.camera_direction = cam.direction;
            p.pole_name = pole_name;
            p.receive_img_topic = makeTopic("_camera/image_raw", pole_name, cam.direction, cam.focal_type);
            p.publish_img_topic = makeTopic("_camera/image_detect", pole_name, cam.direction, cam.focal_type);
            p.publish_img_result = makeTopic("_camera/image_detect_object", pole_name, cam.direction, cam.focal_type);
            p.publish_fps = makeTopic("_camera/image_detect_object/fps_hz", pole_name, cam.direction, cam.focal_type);
            p.receive_radar_topic = makeTopic("/radar/track_object_project", pole_name, cam.direction, "");
            p.shm_name = "/_" + pole_name + "_" + cam.direction + "_" + cam.focal_type + "_camera_image_raw.decoder";
            return p;
        }

        // Load model paths with error handling
        std::vector<std::string> loadModelPaths(ros::NodeHandle& nh) {
            const std::vector<std::string> model_names = {
                "det_engine_name", "color_engine_name",
                "freid_engine_name", "abandon_engine_name"
            };

            std::vector<std::string> paths;
            for (const auto& name : model_names) {
                std::string path;
                if (nh.getParam(name, path)) {
                    paths.push_back(path);
                    LOG_INFO("Loaded model: %s = %s", name.c_str(), path.c_str());
                } else {
                    LOG_INFO("Missing parameter: %s", name.c_str());
                    paths.push_back("");
                }
            }
            return paths;
        }

        // Start inference threads for a single camera
        void startInferenceThread(InferParam param,
                                 const std::vector<std::string>& model_paths,
                                 int dev_id,
                                 int det_class,
                                 int det_stride,
                                 int abandon_class,
                                 int abandon_stride,
                                 int abandon_rate,
                                 int vechile_color_rate,
                                 const std::string& byte_track_config_file,
                                 bool write_flag,
                                 const std::string& write_path,
                                 int min_points_len,
                                 ros::NodeHandle& nh) {
            auto infer_node = std::make_shared<InferDet>();
            image_transport::ImageTransport it(nh);

            // Setup publishers
            auto pub_img = it.advertise(param.publish_img_topic, 1);
            auto pub_tracker = nh.advertise<infer_nodelet::ImageDetectObject>(
                param.publish_img_result,1
            );
            auto pub_fps = nh.advertise<std_msgs::Float32>(param.publish_fps,1);

            // Configure inference node
            infer_node->loadParam(pub_img, pub_tracker, pub_fps,
                                  param.camera_type, param.camera_direction,
                                  param.pole_name,
                                  vechile_color_rate, abandon_rate,
                                  param.publish_img,
                                  param.draw_tracker);
            infer_node->setWriteParam(byte_track_config_file, write_flag,
                                     write_path, min_points_len);
            bool use_shm = false;
            nh.param("use_shm", use_shm, false);
            if (use_shm) {
                infer_node->setShmParam(param.shm_name);
            }

            // Initialize TrafficAnalyzer
            {
                bool ta_enabled = false;
                nh.param("traffic_analyzer/enabled", ta_enabled, false);
                infer_node->traffic_analyzer.setEnabled(ta_enabled);
                if (ta_enabled) {
                    std::string templates_path, events_path;
                    int stationary_frames = 30;
                    float stationary_threshold = 0.005f;
                    nh.param("traffic_analyzer/templates_path", templates_path, std::string("/data/rvf/nfsroot/templates"));
                    nh.param("traffic_analyzer/events_path", events_path, std::string("/data/rvf/nfsroot/events"));
                    nh.param("traffic_analyzer/stationary_frames", stationary_frames, 30);
                    nh.param("traffic_analyzer/stationary_threshold", stationary_threshold, 0.005f);

                    mkdir(templates_path.c_str(), 0755);
                    mkdir(events_path.c_str(), 0755);

                    infer_node->traffic_analyzer.setConfig(
                        templates_path, events_path, 1920, 1080,
                        stationary_frames, stationary_threshold);
                    // Try loading existing templates; if none, learn from yesterday's tracks
                    {
                        std::string template_file = templates_path + "/" + param.camera_type + "_" + param.camera_direction + ".json";
                        struct stat st;
                        bool has_templates = (stat(template_file.c_str(), &st) == 0);
                        if (has_templates) {
                            infer_node->traffic_analyzer.loadTemplates(
                                param.camera_type, param.camera_direction);
                        } else {
                            // Learn from yesterday's tracks
                            auto now = std::chrono::system_clock::now();
                            std::time_t t = std::chrono::system_clock::to_time_t(now);
                            t -= 86400; // yesterday
                            std::stringstream day;
                            day << std::put_time(std::localtime(&t), "%Y-%m-%d");
                            std::string tracks_dir = write_path + day.str() + "/tracks/";
                            LOG_INFO("No templates for %s-%s, learning from %s",
                                     param.camera_type.c_str(), param.camera_direction.c_str(),
                                     tracks_dir.c_str());
                            infer_node->traffic_analyzer.learnTemplates(
                                tracks_dir, param.camera_type, param.camera_direction);
                            infer_node->traffic_analyzer.saveTemplates(
                                param.camera_type, param.camera_direction);
                        }
                    }

                    LOG_INFO("TrafficAnalyzer enabled for %s-%s: templates=%s events=%s",
                             param.camera_type.c_str(), param.camera_direction.c_str(),
                             templates_path.c_str(), events_path.c_str());
                }
            }
            //infer_node->loadModel(model_paths);
#if USE_SOPHON || USE_RKNN
            // 使用全局共享 pipeline，不再加载独立模型
            infer_node->setCameraId(next_thread_idx);
#else
            infer_node->load_det_model(model_paths[0],dev_id, det_class, det_stride);
#endif
            infer_node->load_abandon_model(model_paths[3],dev_id,abandon_class,abandon_stride);
#if USE_SOPHON || USE_NVIDIA
            infer_node->load_color_model(model_paths[1]);
#endif

            // Start processing threads
            int thread_idx = next_thread_idx++;
            // 确保全局队列足够大
            if (thread_idx >= (int)imgQueue.size()) {
                imgQueue.resize(thread_idx + 1);
                trackQueue.resize(thread_idx + 1);
            }
            auto spawn = [](auto func, auto obj, int idx) {
                std::thread(func, obj, idx).detach();
            };

            // Check if video mode
            if (!param.video_path.empty()) {
                LOG_INFO("Starting video file processing thread: %s", param.video_path.c_str());
                std::thread(&InferDet::processVideoFile, infer_node, param.video_path, thread_idx).detach();
            } else {
                spawn(&InferDet::processRadarCamera, infer_node, thread_idx);
            }
            std::thread(&InferDet::processHz, infer_node).detach();
#if USE_SOPHON || USE_RKNN
            std::thread(&InferDet::processResult, infer_node).detach();
            std::thread(&InferDet::publishThread, infer_node).detach();
#endif
#if USE_SOPHON || USE_NVIDIA
            std::thread(&InferDet::vehicleColor, infer_node).detach();
#endif
            std::thread(&InferDet::abandonDetect, infer_node).detach();

            // Create subscribers (only for non-video mode)
            if (param.video_path.empty()) {
                sub_radars.emplace_back(nh.subscribe<infer_nodelet::RadarTrackObjectProject>(
                    param.receive_radar_topic, 1,
                    boost::bind(&trackCallback, _1, thread_idx)
                ));
                if (!use_shm)
                    sub_imgs.emplace_back(nh.subscribe<sensor_msgs::Image>(
                        param.receive_img_topic, 1,
                        boost::bind(&imgCallback, _1, thread_idx)
                    ));
            }
        }

        struct CameraVisualConfig {
            std::string direction;
            std::string focal_type;
            bool publish_img = false;
            bool draw_tracker = false;
        };
        std::vector<CameraVisualConfig> camera_visual_config;

        void loadCameraVisualConfig(ros::NodeHandle& nh) {
            XmlRpc::XmlRpcValue vis_list;
            if (!nh.getParam("camera_visual", vis_list)) {
                LOG_INFO("No camera_visual config found, all visualization disabled");
                return;
            }
            if (vis_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                LOG_WARN("camera_visual must be a list");
                return;
            }
            for (int i = 0; i < vis_list.size(); i++) {
                CameraVisualConfig cfg;
                if (vis_list[i].hasMember("direction"))
                    cfg.direction = static_cast<std::string>(vis_list[i]["direction"]);
                if (vis_list[i].hasMember("focal_type"))
                    cfg.focal_type = static_cast<std::string>(vis_list[i]["focal_type"]);
                if (vis_list[i].hasMember("publish_img"))
                    cfg.publish_img = static_cast<bool>(vis_list[i]["publish_img"]);
                if (vis_list[i].hasMember("draw_tracker"))
                    cfg.draw_tracker = static_cast<bool>(vis_list[i]["draw_tracker"]);
                camera_visual_config.push_back(cfg);
                LOG_INFO("Camera visual config: %s-%s publish_img=%d draw_tracker=%d",
                         cfg.direction.c_str(), cfg.focal_type.c_str(), cfg.publish_img, cfg.draw_tracker);
            }
        }

        virtual void onInit() override {
            NODELET_INFO("Infer initialized");
            auto private_nh = getMTPrivateNodeHandle();
            auto nh = getMTNodeHandle();

            // Load configuration ----------------------------------------------
            bool test_flag = false;
            private_nh.getParam("test/test_flag", test_flag);

            std::vector<InferParam> infer_params;
            if (!test_flag) {
                DeviceParamsConfig cfg(nh);
                LOG_COUT("MEC Version: " << cfg.meta.version);

                loadCameraVisualConfig(private_nh);
                for (const auto& pole : cfg.mec_info.poles) {
                    LOG_INFO("Pole: %s (index:%d)", pole.pole_name.c_str(), pole.pole_index);
                    for (const auto& cam : pole.cameras) {
                        LOG_INFO("  Camera: %s-%s", cam.direction.c_str(), cam.focal_type.c_str());
                        InferParam p = createTopicParams(cam, pole.pole_name);
                        // Default: no visualization (performance mode)
                        p.publish_img = false;
                        p.draw_tracker = false;
                        // Apply per-camera visualization config
                        for (const auto& vis : camera_visual_config) {
                            if (vis.direction == cam.direction && vis.focal_type == cam.focal_type) {
                                p.publish_img = vis.publish_img;
                                p.draw_tracker = vis.draw_tracker;
                                LOG_INFO("  Visualization enabled for %s-%s: publish_img=%d draw_tracker=%d",
                                         cam.direction.c_str(), cam.focal_type.c_str(), p.publish_img, p.draw_tracker);
                                break;
                            }
                        }
                        infer_params.push_back(p);
                    }
                }
            } else {
                std::string video_path;
                private_nh.getParam("test/video_path", video_path);
                InferParam param;
                private_nh.getParam("test/camera_type", param.camera_type);
                private_nh.getParam("test/camera_direction", param.camera_direction);
                private_nh.getParam("test/publish_img_topic", param.publish_img_topic);
                private_nh.getParam("test/publish_img_result", param.publish_img_result);
                private_nh.getParam("test/publish_fps", param.publish_fps);
                private_nh.getParam("test/publish_img", param.publish_img);
                private_nh.getParam("test/draw_tracker", param.draw_tracker);
                private_nh.getParam("test/receive_radar_topic", param.receive_radar_topic);
                if (!video_path.empty()) {
                    LOG_INFO("RUNNING IN VIDEO FILE MODE: %s", video_path.c_str());
                    param.video_path = video_path;
                } else {
                    LOG_INFO("RUNNING IN TEST MODE (ROS topics)");
                    private_nh.getParam("test/receive_img_topic", param.receive_img_topic);
                }
                private_nh.getParam("test/shm_name", param.shm_name);
                infer_params.push_back(param);
            }
            LOG_INFO("Found %zu camera configurations", infer_params.size());

            // Load models & parameters ------------------------------------------
            auto model_paths = loadModelPaths(private_nh);
            int abandon_rate = 5, vechile_color_rate = 10, min_points_len = 30;
            std::string write_path = "/home/files/nfsroot/", byte_track_config_file = "";
            bool write_flag = false;

            private_nh.param("abandon_rate", abandon_rate, abandon_rate);
            private_nh.param("vechile_color_rate", vechile_color_rate, vechile_color_rate);
            private_nh.param("min_points_len", min_points_len, min_points_len);
            private_nh.param("write_path", write_path, write_path);
            private_nh.param("byte_track_config_file", byte_track_config_file, byte_track_config_file);
            private_nh.param("write_flag", write_flag, write_flag);

            XmlRpc::XmlRpcValue params;
            private_nh.getParam("model", params);

            // 检测MLU设备数量（通过设备文件，不调用cnrtInit避免与Net::init冲突）
#if USE_CAMBRICON
            int num_devices = 0;
            struct stat st;
            for (int i = 0; i < 8; i++) {
                if (stat(("/dev/cambricon_dev" + std::to_string(i)).c_str(), &st) == 0) {
                    num_devices++;
                } else {
                    break;
                }
            }
            if (num_devices == 0) num_devices = 1;
#elif USE_SOPHON
            int num_devices = 0;
            struct stat st;
            for (int i = 0; i < 8; i++) {
                if (stat(("/dev/bm" + std::to_string(i)).c_str(), &st) == 0) {
                    num_devices++;
                } else {
                    break;
                }
            }
            if (num_devices == 0) num_devices = 1;
#elif USE_ASCEND
            int num_devices = 0;
            struct stat st;
            for (int i = 0; i < 8; i++) {
                if (stat(("/dev/davinci" + std::to_string(i)).c_str(), &st) == 0) {
                    num_devices++;
                } else {
                    break;
                }
            }
            if (num_devices == 0) num_devices = 1;
#elif USE_NVIDIA
            int num_devices = 0;
            cudaError_t cuda_ret = cudaGetDeviceCount(&num_devices);
            if (cuda_ret != cudaSuccess) {
                num_devices = 1;
            }
            if (num_devices == 0) num_devices = 1;
#else
            int num_devices = 1;
#endif
            LOG_INFO("Detected %d MLU device(s), distributing %zu cameras round-robin",
                     num_devices, infer_params.size());

#if USE_SOPHON
            // 创建全局共享的 Sophon pipeline
            g_pipeline = std::make_unique<SophonPipeline>();
            g_pipeline->init(model_paths[0]);
            int batch_size = g_pipeline->getBatchSize();
            LOG_INFO("Global Sophon pipeline initialized, batch_size=%d", batch_size);

            // 初始化结果队列（每个相机一个）
            g_result_queues.resize(infer_params.size());
            g_cam_frame_queues.resize(infer_params.size());

            // 启动全局三段式流水线线程
            std::thread(batch_preprocess_thread, batch_size).detach();
            std::thread(batch_postprocess_thread).detach();
#elif USE_RKNN
            // 创建全局共享的 RKNN pipeline
            std::string det_model_type = "auto";
            if (params.size() > 0 && params[0].hasMember("det_model_type")) {
                det_model_type = std::string(params[0]["det_model_type"]);
            }
            auto rknn_pipeline = std::make_unique<RknnPipeline>();
            rknn_pipeline->init(model_paths[0], det_model_type);
            g_pipeline = std::move(rknn_pipeline);
            int batch_size = g_pipeline->getBatchSize();
            LOG_INFO("Global RKNN pipeline initialized, batch_size=%d model_type=%s",
                     batch_size, det_model_type.c_str());

            // 初始化结果队列（每个相机一个）
            g_result_queues.resize(infer_params.size());
            g_cam_frame_queues.resize(infer_params.size());

            // 启动3个独立推理线程，每个绑定一个NPU核心
            for (int core = 0; core < RKNN_NUM_CORES; core++) {
                std::thread(rknn_infer_thread, core).detach();
            }
            // 结果由 processResult 线程异步消费
#endif

            // Start inference threads -------------------------------------------
            for (size_t infer_index = 0; infer_index < infer_params.size(); infer_index++) {
                int dev_id = infer_index % num_devices;
                int det_class = 10;
                int det_stride = 3;
                int abandon_class = 3;
                int abandon_stride = 3;

                // 如果YAML中model列表有对应的配置项，读取非默认参数
                if (infer_index < static_cast<size_t>(params.size())) {
                    if (params[infer_index].hasMember("det_class"))
                        det_class = params[infer_index]["det_class"];
                    if (params[infer_index].hasMember("det_stride"))
                        det_stride = params[infer_index]["det_stride"];
                    if (params[infer_index].hasMember("abandon_class"))
                        abandon_class = params[infer_index]["abandon_class"];
                    if (params[infer_index].hasMember("abandon_stride"))
                        abandon_stride = params[infer_index]["abandon_stride"];
                }

                LOG_INFO("Camera[%zu] -> MLU device %d (det_class=%d det_stride=%d abandon_class=%d abandon_stride=%d)",
                         infer_index, dev_id, det_class, det_stride, abandon_class, abandon_stride);

                startInferenceThread(infer_params[infer_index], model_paths,dev_id, det_class,det_stride,abandon_class,abandon_stride ,abandon_rate,
                                    vechile_color_rate, byte_track_config_file,
                                    write_flag, write_path, min_points_len, private_nh);
            }
        }
    };
}

PLUGINLIB_EXPORT_CLASS(infer_ns::DataNodelet, nodelet::Nodelet)
