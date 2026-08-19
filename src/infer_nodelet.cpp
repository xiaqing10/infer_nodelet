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
                                 bool save_img_flag,
                                 const std::string& write_path,
                                 int min_points_len,
                                 bool roi_enabled,
                                 float roi_height_ratio,
                                 float roi_width_ratio,
                                 float roi_y_ratio,
                                 float roi_x_ratio,
                                 int process_delay_print_interval,
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
            infer_node->setWriteParam(byte_track_config_file, write_flag, save_img_flag,
                                     write_path, min_points_len);
            infer_node->setRoiParams(roi_enabled, roi_height_ratio, roi_width_ratio,
                                     roi_y_ratio, roi_x_ratio);
            infer_node->setProcessDelayPrintInterval(process_delay_print_interval);
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
#elif USE_NVIDIA
            if (g_nvidia_batch_mode) {
                // batch 模式：使用全局共享 pipeline，不再加载独立模型
                infer_node->setCameraId(next_thread_idx);
            } else {
                infer_node->load_det_model(model_paths[0],dev_id, det_class, det_stride);
            }
#else
            infer_node->load_det_model(model_paths[0],dev_id, det_class, det_stride);
#endif
#if USE_SOPHON
            // Sophon 下颜色分类与抛洒物模型为全局共享单实例，在 onInit 中一次性加载，
            // 这里不再为每路加载独立模型。
#else
            infer_node->load_abandon_model(model_paths[3],dev_id,abandon_class,abandon_stride);
#endif
#if (USE_SOPHON || USE_NVIDIA) && !USE_SOPHON
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
#elif USE_NVIDIA
            if (g_nvidia_batch_mode) {
                std::thread(&InferDet::processResult, infer_node).detach();
                std::thread(&InferDet::publishThread, infer_node).detach();
            }
#endif
#if (USE_SOPHON || USE_NVIDIA) && !USE_SOPHON
            std::thread(&InferDet::vehicleColor, infer_node).detach();
#endif
#if !USE_SOPHON
            std::thread(&InferDet::abandonDetect, infer_node).detach();
#endif

            // Create subscribers (only for non-video mode)
            if (param.video_path.empty()) {
                setCamPoleName(thread_idx, param.pole_name);
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

        struct CameraFilterConfig {
            std::string pole_name;
            std::string direction;
            std::string focal_type;
            bool enable = true;
        };
        std::vector<CameraFilterConfig> camera_filter_config;

        void loadCameraFilterConfig(ros::NodeHandle& nh) {
            XmlRpc::XmlRpcValue flt_list;
            if (!nh.getParam("camera_filter", flt_list)) {
                LOG_INFO("No camera_filter config found, all cameras enabled");
                return;
            }
            if (flt_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                LOG_WARN("camera_filter must be a list");
                return;
            }
            for (int i = 0; i < flt_list.size(); i++) {
                CameraFilterConfig cfg;
                if (flt_list[i].hasMember("pole_name"))
                    cfg.pole_name = static_cast<std::string>(flt_list[i]["pole_name"]);
                if (flt_list[i].hasMember("direction"))
                    cfg.direction = static_cast<std::string>(flt_list[i]["direction"]);
                if (flt_list[i].hasMember("focal_type"))
                    cfg.focal_type = static_cast<std::string>(flt_list[i]["focal_type"]);
                if (flt_list[i].hasMember("enable"))
                    cfg.enable = static_cast<bool>(flt_list[i]["enable"]);
                camera_filter_config.push_back(cfg);
                LOG_INFO("Camera filter config: %s-%s-%s enable=%d",
                         cfg.pole_name.c_str(), cfg.direction.c_str(), cfg.focal_type.c_str(), (int)cfg.enable);
            }
        }

        struct CameraRoiEntry {
            std::string pole_name;
            std::string direction;
            std::string focal_type;
            bool enabled = true;
            float height_ratio = 0.5f;
            float width_ratio = 0.5f;
            float y_ratio = 0.0f;
            float x_ratio = 0.5f;
        };
        std::vector<CameraRoiEntry> camera_roi_config;

        void loadCameraRoiConfig(ros::NodeHandle& nh) {
            XmlRpc::XmlRpcValue roi_list;
            if (!nh.getParam("camera_roi", roi_list)) {
                LOG_INFO("No camera_roi config found, all cameras use global ROI");
                return;
            }
            if (roi_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                LOG_WARN("camera_roi must be a list");
                return;
            }
            for (int i = 0; i < roi_list.size(); i++) {
                CameraRoiEntry e;
                if (roi_list[i].hasMember("pole_name"))
                    e.pole_name = static_cast<std::string>(roi_list[i]["pole_name"]);
                if (roi_list[i].hasMember("direction"))
                    e.direction = static_cast<std::string>(roi_list[i]["direction"]);
                if (roi_list[i].hasMember("focal_type"))
                    e.focal_type = static_cast<std::string>(roi_list[i]["focal_type"]);
                if (roi_list[i].hasMember("enabled"))
                    e.enabled = static_cast<bool>(roi_list[i]["enabled"]);
                if (roi_list[i].hasMember("height_ratio"))
                    e.height_ratio = static_cast<double>(roi_list[i]["height_ratio"]);
                if (roi_list[i].hasMember("width_ratio"))
                    e.width_ratio = static_cast<double>(roi_list[i]["width_ratio"]);
                if (roi_list[i].hasMember("y_ratio"))
                    e.y_ratio = static_cast<double>(roi_list[i]["y_ratio"]);
                if (roi_list[i].hasMember("x_ratio"))
                    e.x_ratio = static_cast<double>(roi_list[i]["x_ratio"]);
                camera_roi_config.push_back(e);
                LOG_INFO("Camera ROI config: %s-%s-%s enabled=%d h=%.2f w=%.2f y=%.2f x=%.2f",
                         e.pole_name.c_str(), e.direction.c_str(), e.focal_type.c_str(),
                         (int)e.enabled, e.height_ratio, e.width_ratio, e.y_ratio, e.x_ratio);
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
                loadCameraFilterConfig(private_nh);
                loadCameraRoiConfig(private_nh);
                for (const auto& pole : cfg.mec_info.poles) {
                    LOG_INFO("Pole: %s (index:%d)", pole.pole_name.c_str(), pole.pole_index);
                    for (const auto& cam : pole.cameras) {
                        LOG_INFO("  Camera: %s-%s", cam.direction.c_str(), cam.focal_type.c_str());
                        // 按 camera_filter 过滤：enable=false 的相机直接跳过，不创建推理线程/占用 batch 槽位
                        bool filtered_out = false;
                        for (const auto& flt : camera_filter_config) {
                            if (flt.pole_name == pole.pole_name &&
                                flt.direction == cam.direction &&
                                flt.focal_type == cam.focal_type &&
                                !flt.enable) {
                                LOG_INFO("  Camera %s-%s-%s filtered out (disabled)",
                                         pole.pole_name.c_str(), cam.direction.c_str(), cam.focal_type.c_str());
                                filtered_out = true;
                                break;
                            }
                        }
                        if (filtered_out) continue;
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
            bool save_img_flag = false;

            private_nh.param("abandon_rate", abandon_rate, abandon_rate);
            private_nh.param("vechile_color_rate", vechile_color_rate, vechile_color_rate);
            private_nh.param("min_points_len", min_points_len, min_points_len);
            private_nh.param("write_path", write_path, write_path);
            private_nh.param("byte_track_config_file", byte_track_config_file, byte_track_config_file);
            private_nh.param("write_flag", write_flag, write_flag);
            private_nh.param("save_img_flag", save_img_flag, save_img_flag);

            bool roi_enabled = true;
            float roi_height_ratio = 0.5f;
            float roi_width_ratio = 0.5f;
            float nms_iou_threshold = 0.5f;
            private_nh.param("roi_enabled", roi_enabled, roi_enabled);
            private_nh.param("roi_height_ratio", roi_height_ratio, roi_height_ratio);
            private_nh.param("roi_width_ratio", roi_width_ratio, roi_width_ratio);
            private_nh.param("nms_iou_threshold", nms_iou_threshold, nms_iou_threshold);

#if USE_NVIDIA
            // 填充 per-camera ROI 配置表（索引 = camera_id）。未被 camera_roi 显式覆盖的相机
            // 用全局 ROI 参数 + 默认位置（y=0 顶部, x=0.5 居中）。test 模式无 pole/方向信息，走默认。
            g_camera_roi.resize(infer_params.size());
            for (size_t i = 0; i < infer_params.size(); i++) {
                CameraRoiConfig rc;
                rc.enabled = roi_enabled;
                rc.height_ratio = roi_height_ratio;
                rc.width_ratio = roi_width_ratio;
                rc.y_ratio = 0.0f;
                rc.x_ratio = 0.5f;
                for (const auto& e : camera_roi_config) {
                    if (e.pole_name == infer_params[i].pole_name &&
                        e.direction == infer_params[i].camera_direction &&
                        e.focal_type == infer_params[i].camera_type) {
                        rc.has_cfg = true;
                        rc.enabled = e.enabled;
                        rc.height_ratio = e.height_ratio;
                        rc.width_ratio = e.width_ratio;
                        rc.y_ratio = e.y_ratio;
                        rc.x_ratio = e.x_ratio;
                        break;
                    }
                }
                g_camera_roi[i] = rc;
                LOG_INFO("Camera[%zu] ROI %s-%s-%s: cfg=%d enabled=%d h=%.2f w=%.2f y=%.2f x=%.2f",
                         i, infer_params[i].pole_name.c_str(),
                         infer_params[i].camera_direction.c_str(), infer_params[i].camera_type.c_str(),
                         (int)rc.has_cfg, (int)rc.enabled, rc.height_ratio, rc.width_ratio, rc.y_ratio, rc.x_ratio);
            }
#endif

            int process_delay_print_interval = 10;
            private_nh.param("process_delay_print_interval", process_delay_print_interval, process_delay_print_interval);

            // 批填充目标 max_full：-1 = 未配置，自动取实际启用相机数（infer_params.size()）
            int max_full = -1;
            private_nh.param("max_full", max_full, -1);

            // ROI 调试：保存裁剪的 ROI 图与带 ROI 框的原图（默认关，仅 batch 模式 ROI 启用时生效）
            bool roi_save_debug = false;
            private_nh.param("roi_save_debug", roi_save_debug, false);
            g_roi_save_debug = roi_save_debug;
            g_roi_save_path = write_path + "/roi_debug";

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
            private_nh.param("batch_mode", g_nvidia_batch_mode, false);
            LOG_INFO("NVIDIA batch_mode=%d", (int)g_nvidia_batch_mode);
            private_nh.param("debug_log", g_debug_log, false);
            LOG_INFO("debug_log=%d", (int)g_debug_log);
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
#if USE_NVIDIA
            g_frame_cnt_input.assign(infer_params.size(), 0);
            g_frame_cnt_pre.assign(infer_params.size(), 0);
            g_frame_cnt_post.assign(infer_params.size(), 0);
#endif
            g_color_result_queues.resize(infer_params.size());
            g_abandon_result_queues.resize(infer_params.size());

            // 全局共享的颜色分类与抛洒物模型：只加载一次，供所有相机复用
            g_color_model = std::make_unique<RESNET>();
            g_color_model->Init(model_paths[1]);
            LOG_INFO("Global color model initialized: %s", model_paths[1].c_str());
            g_abandon_model = std::make_unique<Detector>();
            g_abandon_model->init(model_paths[3], 0, 3, 3);
            LOG_INFO("Global abandon model initialized: %s", model_paths[3].c_str());

            // 启动全局三段式流水线线程 + 共享 worker
            int effective_max_full = (max_full > 0) ? max_full : (int)infer_params.size();
            std::thread(batch_preprocess_thread, batch_size, effective_max_full).detach();
            std::thread(batch_postprocess_thread).detach();
            std::thread(color_infer_thread).detach();
            std::thread(abandon_infer_thread).detach();
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
#elif USE_NVIDIA
            if (g_nvidia_batch_mode) {
                // 创建全局共享的 NVIDIA batch pipeline
                int det_class = 10;
                if (params.size() > 0 && params[0].hasMember("det_class"))
                    det_class = params[0]["det_class"];
                auto nvidia_pipeline = std::make_unique<NvidiaPipeline>();
                nvidia_pipeline->setNumLabels(det_class);
                nvidia_pipeline->setRoiEnabled(roi_enabled);
                nvidia_pipeline->setRoiHeightRatio(roi_height_ratio);
                nvidia_pipeline->setRoiWidthRatio(roi_width_ratio);
                nvidia_pipeline->setNmsIou(nms_iou_threshold);
                nvidia_pipeline->init(model_paths[0]);
                g_pipeline = std::move(nvidia_pipeline);
                int batch_size = g_pipeline->getBatchSize();
                LOG_INFO("Global NVIDIA batch pipeline initialized, batch_size=%d det_class=%d roi=%d roi_ratio=%.2f/%.2f nms_iou=%.2f",
                         batch_size, det_class, (int)roi_enabled, roi_height_ratio, roi_width_ratio, nms_iou_threshold);

                // 初始化结果队列（每个相机一个）
                g_result_queues.resize(infer_params.size());
                g_cam_frame_queues.resize(infer_params.size());
#if USE_NVIDIA
                g_frame_cnt_input.assign(infer_params.size(), 0);
                g_frame_cnt_pre.assign(infer_params.size(), 0);
                g_frame_cnt_post.assign(infer_params.size(), 0);
#endif

                // 启动全局三段式流水线线程
                // max_full：未配置(-1)时自动取实际启用相机数；已配置则用配置值（batch_pre 内做安全上限钳制）
                int effective_max_full = (max_full > 0) ? max_full : (int)infer_params.size();
                std::thread(batch_preprocess_thread, batch_size, effective_max_full).detach();
                std::thread(batch_postprocess_thread).detach();
            }
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

                // per-camera ROI：优先用 camera_roi 覆盖值（g_camera_roi），否则回退全局值，保证画框与 batch 抠图/检测一致
                bool cam_roi_enabled = roi_enabled;
                float cam_roi_h = roi_height_ratio;
                float cam_roi_w = roi_width_ratio;
                float cam_roi_y = 0.0f;
                float cam_roi_x = 0.5f;
#if USE_NVIDIA
                if (infer_index < g_camera_roi.size()) {
                    const CameraRoiConfig& rc = g_camera_roi[infer_index];
                    cam_roi_enabled = rc.enabled;
                    cam_roi_h = rc.height_ratio;
                    cam_roi_w = rc.width_ratio;
                    cam_roi_y = rc.y_ratio;
                    cam_roi_x = rc.x_ratio;
                }
#endif
                startInferenceThread(infer_params[infer_index], model_paths,dev_id, det_class,det_stride,abandon_class,abandon_stride ,abandon_rate,
                                    vechile_color_rate, byte_track_config_file,
                                    write_flag, save_img_flag, write_path, min_points_len,
                                    cam_roi_enabled, cam_roi_h, cam_roi_w, cam_roi_y, cam_roi_x, process_delay_print_interval, private_nh);
            }
        }
    };
}

PLUGINLIB_EXPORT_CLASS(infer_ns::DataNodelet, nodelet::Nodelet)
