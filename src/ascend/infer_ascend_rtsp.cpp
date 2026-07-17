#include "log_macros.h"
#include <infer.h>
#include <iostream>
#include <string.h>
#include "device_params_config.h"
#include <algorithm>
#include <map>
#include "img_score.hpp"
#include "traffic_analyzer.h"
#include <sys/stat.h>
#include <ctime>
#include <pluginlib/class_list_macros.h>

#include "acl/acl.h"
#include "AclLiteUtils.h"
#include "AclLiteImageProc.h"
#include "AclLiteModel.h"
#include "AclLiteVideoProc.h"
#include "AclLiteResource.h"

using namespace std;

void imgCallback(const sensor_msgs::ImageConstPtr& msg, int& index);
void trackCallback(const infer_nodelet::RadarTrackObjectProject::ConstPtr msg, int& index);

struct RtspStreamConfig {
    std::string url;
    std::string camera_type;
    std::string camera_direction;
};

namespace infer_ns {
    class DataNodeletRtsp : public nodelet::Nodelet {
    public:
        DataNodeletRtsp() = default;

    private:
        std::vector<ros::Subscriber> sub_radars;

        std::string makeTopic(const std::string& prefix,
                              const std::string& pole_name,
                              const std::string& direction,
                              const std::string& focal_type) {
            return "/" + pole_name + "/" + direction + "/" + focal_type + prefix;
        }

        InferParam createTopicParams(const std::string& pole_name,
                                     const std::string& direction,
                                     const std::string& focal_type) {
            InferParam p;
            p.camera_type = focal_type;
            p.camera_direction = direction;
            p.pole_name = pole_name;
            p.receive_img_topic = "";
            p.publish_img_topic = makeTopic("_camera/image_detect", pole_name, direction, focal_type);
            p.publish_img_result = makeTopic("_camera/image_detect_object", pole_name, direction, focal_type);
            p.publish_fps = makeTopic("_camera/image_detect_object/fps_hz", pole_name, direction, focal_type);
            p.receive_radar_topic = makeTopic("/radar/track_object_project", pole_name, direction, "");
            return p;
        }

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
                                  const std::string& pole_name,
                                  ros::NodeHandle& nh) {
            auto infer_node = std::make_shared<InferDet>();
            image_transport::ImageTransport it(nh);

            auto pub_img = it.advertise(param.publish_img_topic, 1);
            auto pub_tracker = nh.advertise<infer_nodelet::ImageDetectObject>(
                param.publish_img_result, 1
            );
            auto pub_fps = nh.advertise<std_msgs::Float32>(param.publish_fps, 1);

            infer_node->loadParam(pub_img, pub_tracker, pub_fps,
                                  param.camera_type, param.camera_direction,
                                  pole_name,
                                  vechile_color_rate, abandon_rate,
                                  param.publish_img,
                                  param.draw_tracker);
            infer_node->setWriteParam(byte_track_config_file, write_flag,
                                     write_path, min_points_len);

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
                    {
                        std::string template_file = templates_path + "/" + param.camera_type + "_" + param.camera_direction + ".json";
                        struct stat st;
                        bool has_templates = (stat(template_file.c_str(), &st) == 0);
                        if (has_templates) {
                            infer_node->traffic_analyzer.loadTemplates(
                                param.camera_type, param.camera_direction);
                        } else {
                            auto now = std::chrono::system_clock::now();
                            std::time_t t = std::chrono::system_clock::to_time_t(now);
                            t -= 86400;
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
                }
            }

            infer_node->load_det_model(model_paths[0], dev_id, det_class, det_stride);
            infer_node->load_abandon_model(model_paths[3], dev_id, abandon_class, abandon_stride);

            int thread_idx = sub_radars.size();
            auto spawn = [](auto func, auto obj, int idx) {
                std::thread(func, obj, idx).detach();
            };

            spawn(&InferDet::processRadarCamera, infer_node, thread_idx);
            std::thread(&InferDet::processHz, infer_node).detach();
            std::thread(&InferDet::abandonDetect, infer_node).detach();

            sub_radars.emplace_back(nh.subscribe<infer_nodelet::RadarTrackObjectProject>(
                param.receive_radar_topic, 1,
                boost::bind(&trackCallback, _1, thread_idx)
            ));
        }

        void rtspReadThread(std::string rtsp_url, int thread_idx, int device_id) {
            LOG_INFO("RTSP read thread started: %s -> imgQueue[%d]", rtsp_url.c_str(), thread_idx);

            AclLiteVideoProc cap(rtsp_url, device_id);
            if (!cap.IsOpened()) {
                LOG_ERROR("Failed to open RTSP stream: %s", rtsp_url.c_str());
                return;
            }
            LOG_INFO("RTSP stream opened: %s, width=%d, height=%d",
                     rtsp_url.c_str(), cap.Get(FRAME_WIDTH), cap.Get(FRAME_HEIGHT));

            AclLiteImageProc dvpp;
            AclLiteError ret = dvpp.Init();
            if (ret != ACLLITE_OK) {
                LOG_ERROR("DVPP init failed for RTSP stream: %s", rtsp_url.c_str());
                return;
            }

            aclrtRunMode runMode;
            aclrtGetRunMode(&runMode);

            while (ros::ok()) {
                ImageData image;
                ret = cap.Read(image);
                if (ret != ACLLITE_OK) {
                    LOG_WARN("RTSP read failed: %s, retrying...", rtsp_url.c_str());
                    usleep(100000);
                    continue;
                }

                ImageData hostImage;
                ret = CopyImageToLocal(hostImage, image, runMode);
                if (ret != ACLLITE_OK) {
                    LOG_WARN("CopyImageToLocal failed for stream: %s", rtsp_url.c_str());
                    continue;
                }

                cv::Mat yuv(hostImage.height * 3 / 2, hostImage.width, CV_8UC1, hostImage.data.get());
                cv::Mat bgr;
                cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                    std_msgs::Header(), "bgr8", bgr
                ).toImageMsg();
                imgQueue[thread_idx].Produce(std::move(msg));
            }

            cap.Close();
            dvpp.DestroyResource();
            LOG_INFO("RTSP read thread ended: %s", rtsp_url.c_str());
        }

        virtual void onInit() override {
            NODELET_INFO("DataNodeletRtsp initialized!");

            auto private_nh = getMTPrivateNodeHandle();
            auto nh = getMTNodeHandle();

            bool rtsp_enabled = false;
            private_nh.param("rtsp/enabled", rtsp_enabled, false);
            if (!rtsp_enabled) {
                LOG_INFO("RTSP mode disabled, exiting.");
                return;
            }

            std::vector<RtspStreamConfig> rtsp_streams;
            XmlRpc::XmlRpcValue rtsp_list;
            if (private_nh.getParam("rtsp/streams", rtsp_list)) {
                if (rtsp_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                    for (int i = 0; i < rtsp_list.size(); i++) {
                        RtspStreamConfig cfg;
                        cfg.url = static_cast<std::string>(rtsp_list[i]["url"]);
                        cfg.camera_type = static_cast<std::string>(rtsp_list[i]["camera_type"]);
                        cfg.camera_direction = static_cast<std::string>(rtsp_list[i]["camera_direction"]);
                        rtsp_streams.push_back(cfg);
                        LOG_INFO("RTSP stream[%d]: url=%s, type=%s, dir=%s",
                                 i, cfg.url.c_str(), cfg.camera_type.c_str(), cfg.camera_direction.c_str());
                    }
                }
            }

            LOG_INFO("Found %zu RTSP streams", rtsp_streams.size());

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

            XmlRpc::XmlRpcValue model_params;
            private_nh.getParam("model", model_params);

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
            LOG_INFO("Detected %d Ascend device(s), distributing %zu streams round-robin",
                     num_devices, rtsp_streams.size());

            for (size_t i = 0; i < rtsp_streams.size(); i++) {
                int dev_id = i % num_devices;
                int det_class = 10;
                int det_stride = 3;
                int abandon_class = 3;
                int abandon_stride = 3;

                if (i < model_params.size()) {
                    if (model_params[i].hasMember("det_class"))
                        det_class = model_params[i]["det_class"];
                    if (model_params[i].hasMember("det_stride"))
                        det_stride = model_params[i]["det_stride"];
                    if (model_params[i].hasMember("abandon_class"))
                        abandon_class = model_params[i]["abandon_class"];
                    if (model_params[i].hasMember("abandon_stride"))
                        abandon_stride = model_params[i]["abandon_stride"];
                }

                LOG_INFO("Stream[%zu] -> Ascend device %d", i, dev_id);

                InferParam param = createTopicParams("rtsp" + std::to_string(i),
                                                     rtsp_streams[i].camera_direction,
                                                     rtsp_streams[i].camera_type);

                startInferenceThread(param, model_paths, dev_id,
                                    det_class, det_stride,
                                    abandon_class, abandon_stride,
                                    abandon_rate,
                                    vechile_color_rate, byte_track_config_file,
                                    write_flag, write_path, min_points_len,
                                    "rtsp" + std::to_string(i),
                                    private_nh);

                std::thread(&DataNodeletRtsp::rtspReadThread, this,
                           rtsp_streams[i].url, i, dev_id).detach();
            }
        }
    };
}

PLUGINLIB_EXPORT_CLASS(infer_ns::DataNodeletRtsp, nodelet::Nodelet)