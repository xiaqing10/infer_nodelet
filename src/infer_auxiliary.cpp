#include "log_macros.h"
#include <infer.h>
#include <iostream>
#include <string.h>
#include <algorithm>
#include <map>
#include <sys/stat.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>

using namespace std;

void InferDet::loadParam(image_transport::Publisher _pub_img,
                         ros::Publisher _pub_tracker,
                         ros::Publisher _pub_fps,
                         string _camera_type,
                         string _camera_direction,
                         string _pole_name,
                         int _vehicle_color_rate,
                         int _abandon_rate,
                         bool _publish_img,
                         bool _draw_tracker)
    {
        pub_img = std::move(_pub_img);
        pub_tracker = std::move(_pub_tracker);
        pub_fps = std::move(_pub_fps);
        camera_type = std::move(_camera_type);
        camera_direction = std::move(_camera_direction);
        pole_name = std::move(_pole_name);
        vehicle_color_rate = _vehicle_color_rate;
        abandon_rate = _abandon_rate;
        publish_img = _publish_img;
        draw_tracker = _draw_tracker;

        LOG_INFO("Parameters loaded for %s-%s",
                camera_type.c_str(), camera_direction.c_str());
    }

void InferDet::load_det_model(std::string model_path, int mlu_infer_device, int num_class, int stride){
#if !USE_SOPHON && !USE_RKNN
    LOG_INFO("Loding... DETRECTOR Model %s", model_path.c_str());
    detector.init(model_path, mlu_infer_device, num_class, stride);
#else
    (void)model_path; (void)mlu_infer_device; (void)num_class; (void)stride;
#endif
}

void InferDet::load_abandon_model(std::string model_path, int mlu_infer_device, int num_class, int stride){
    abandon_detector.init(model_path, mlu_infer_device, num_class, stride);
    LOG_INFO("Load model success!!!");
}

#if USE_SOPHON || USE_NVIDIA
void InferDet::load_color_model(std::string model_path){
    LOG_INFO("Loding... COLOR Model %s", model_path.c_str());
#if USE_SOPHON
    vehicle_color_detector.Init(model_path);
#elif USE_NVIDIA
    vehicle_color_detector.init(model_path);
#endif
}
#endif

void InferDet::setWriteParam(std::string byte_track_config_file_, bool write_flag_, std::string write_path_, int min_points_len_){
    byte_track_config_file = byte_track_config_file_;
    write_flag = write_flag_;
    write_path = write_path_;
    if(min_points_len_ > 0 ) min_points_len = min_points_len_;

    if (write_flag && !write_path.empty()) {
        std::string cmd = "mkdir -p " + write_path;
        int ret = system(cmd.c_str());
        if (ret != 0) {
            LOG_WARN("Failed to create write_path: %s", write_path.c_str());
        }
    }
}

void InferDet::setShmParam(const std::string& shm_name_) {
#if USE_SHM
    use_shm = true;
    shm_name = shm_name_;
    shm_reader = std::make_shared<ehawkeye::modules::units::shmmem>(shm_name, 30, false);
    LOG_INFO("SHM reader created: %s", shm_name.c_str());
#else
    LOG_WARN("SHM not enabled (compile with -DUSE_SHM=ON)");
#endif
}

void InferDet::processHz() {
    std_msgs::Float32 hz_data;
    const double rate_duration = 5.0;
    auto last = std::chrono::steady_clock::now();

    while (ros::ok()) {
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds((long long)(rate_duration * 1000.0)));
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last).count();
            last = now;
            if (elapsed <= 0.0) elapsed = rate_duration;
            double current_fps = 0.0;

            {
                std::lock_guard<std::mutex> lock(mtx);
                current_fps = static_cast<double>(publish_hz) / elapsed;
                publish_hz = 0;
            }

            {
                hz_data.data = current_fps;
                pub_fps.publish(hz_data);
                LOG_INFO("%s %s %s publish_hz: %.2f",
                         pole_name.c_str(),
                         camera_direction.c_str(),
                         camera_type.c_str(),
                         current_fps);
            }
            if (!camera_direction.empty() && !camera_type.empty()) {
                //diagnostic.Pub//diagnosticData(
                //    ERROR_CODE_LEVEL::WARN,
                //    rvf::system::KeyCode::kNoInferData,
                //    "NoInferData-" + camera_direction + "-" + camera_type
                //);
            }

        } catch (const std::exception& e) {
            LOG_ERROR("Exception in process_hz: %s", e.what());
        }
    }
}

#if USE_SOPHON || USE_NVIDIA
void InferDet::vehicleColor(){
    ModelInputData input_data;
    while (ros::ok()) {
        if (!vehicleColorQueue.Consume(input_data)) {
            ros::Rate(2).sleep();
            continue;
        }
        const auto cls_ret = vehicle_color_detector.inference(input_data.im);
        vehicleColorResultQueue.Produce({
            .tracker_id = input_data.tracker_id,
            .vehicle_color = cls_ret.label,
            .confidence = cls_ret.confidence
        });
    }
}
#endif

void InferDet::abandonDetect() {
    LOG_INFO("Abandoned object detection thread started");
    AbandonInputData input_data;

    while (ros::ok()) {
        abandonRecQueue.ConsumeSync(input_data);

        cv::Mat img = input_data.im;
        auto results = abandon_detector.inference(img);

        std::vector<DetectorRetData> valid_results;
        valid_results.reserve(results.size());
        std::copy_if(results.begin(), results.end(), std::back_inserter(valid_results),
            [&img](const auto& obj) {
                return obj.xmin >= 0 && obj.ymin >= 0 &&
                       obj.xmax < img.cols && obj.ymax < img.rows;
            });

        std::vector<DetectorRetData> filtered;
        for (const auto& new_obj : valid_results) {
            bool match_found = std::any_of(input_data.data.begin(), input_data.data.end(),
                [&new_obj](const auto& existing_obj) {
                    return CalculateOverlap(
                        new_obj.xmin, new_obj.ymin, new_obj.xmax, new_obj.ymax,
                        existing_obj.xmin, existing_obj.ymin, existing_obj.xmax, existing_obj.ymax
                    ) > 0.1f;
                });

            if (!match_found) {
                filtered.push_back(new_obj);
            }
        }
        if (1) {
            DetectorRetDatas results_data{{std::move(filtered)}};
            abandonResultQueue.Produce(std::move(results_data));
        }
    }
}

void InferDet::saveTrackMontage(int track_id, int class_id,
                                const std::vector<std::vector<float>>& track_points,
                                const std::string& save_dir,
                                const std::string& filename_prefix) {
    if (track_points.empty()) return;

    // Build map: frame_id -> point data
    std::map<int, std::vector<float>> frame_to_point;
    for (auto& p : track_points) {
        int fid = static_cast<int>(p[5]);
        frame_to_point[fid] = p;
    }

    // Collect all trajectory points (center) for drawing trajectory lines
    std::vector<cv::Point2f> all_pts;
    std::map<int, cv::Point2f> frame_to_center;
    for (auto& p : track_points) {
        float cx = p[0] + p[2] / 2;
        float cy = p[1] + p[3] / 2;
        int fid = static_cast<int>(p[5]);
        cv::Point2f pt(cx, cy);
        all_pts.push_back(pt);
        frame_to_center[fid] = pt;
    }

    // Use the last up-to-30 cached frames (most recent first, then reverse)
    std::vector<CachedFrame> recent;
    for (auto it = frame_cache_.rbegin(); it != frame_cache_.rend() && (int)recent.size() < MAX_CACHED_FRAMES; ++it) {
        recent.push_back(*it);
    }
    std::reverse(recent.begin(), recent.end());

    if (recent.empty()) {
        return;
    }

    // Scale factor for cached images
    float scale_x = (float)recent[0].img.cols / img_src.cols;
    float scale_y = (float)recent[0].img.rows / img_src.rows;

    // Build panels: draw trajectory on each cached frame
    std::vector<cv::Mat> panels;
    for (auto& cf : recent) {
        cv::Mat panel = cf.img.clone();

        // Draw trajectory lines up to this frame
        cv::Point prev_pt(-1, -1);
        for (auto& p : track_points) {
            int fid = static_cast<int>(p[5]);
            if (fid > cf.frame_id) continue;
            float cx = p[0] + p[2] / 2;
            float cy = p[1] + p[3] / 2;
            cv::Point cur((int)(cx * scale_x), (int)(cy * scale_y));
            if (prev_pt.x >= 0) {
                cv::line(panel, prev_pt, cur, cv::Scalar(0, 255, 255), 2);
            }
            prev_pt = cur;
        }

        // Draw bounding box on this frame — find nearest point with fid <= cf.frame_id
        auto it = frame_to_point.upper_bound(cf.frame_id);
        if (it != frame_to_point.begin()) {
            --it;
            auto& p = it->second;
            int rx = (int)(p[0] * scale_x);
            int ry = (int)(p[1] * scale_y);
            int rw = (int)(p[2] * scale_x);
            int rh = (int)(p[3] * scale_y);
            cv::rectangle(panel, cv::Rect(rx, ry, rw, rh), cv::Scalar(0, 255, 255), 2);
        }

        // Draw start (green) and current (red) points
        auto first_it = frame_to_center.begin();
        if (first_it != frame_to_center.end()) {
            cv::Point start((int)(first_it->second.x * scale_x), (int)(first_it->second.y * scale_y));
            cv::circle(panel, start, 4, cv::Scalar(0, 255, 0), -1);
        }
        // Find nearest center point with fid <= cf.frame_id
        auto center_it = frame_to_center.upper_bound(cf.frame_id);
        if (center_it != frame_to_center.begin()) {
            --center_it;
            cv::Point cur((int)(center_it->second.x * scale_x), (int)(center_it->second.y * scale_y));
            cv::circle(panel, cur, 4, cv::Scalar(0, 0, 255), -1);
        }

        // Label: class_id_track_id and frame number
        std::string label = std::to_string(class_id) + "_" + std::to_string(track_id) + " f" + std::to_string(cf.frame_id);
        cv::putText(panel, label, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

        panels.push_back(panel);
    }

    // Layout: 5 columns x 6 rows = 30 slots
    const int COLS = 5;
    const int ROWS = 6;
    int panel_w = recent[0].img.cols;
    int panel_h = recent[0].img.rows;
    int montage_w = COLS * panel_w;
    int montage_h = ROWS * panel_h;

    cv::Mat montage(montage_h, montage_w, recent[0].img.type(), cv::Scalar(0, 0, 0));
    for (int i = 0; i < (int)panels.size() && i < COLS * ROWS; i++) {
        int row = i / COLS;
        int col = i % COLS;
        cv::Rect roi(col * panel_w, row * panel_h, panel_w, panel_h);
        panels[i].copyTo(montage(roi));
    }

    std::string montage_filename = save_dir + filename_prefix + "_montage.jpg";
    cv::imwrite(montage_filename, montage);

    // Also save the original single-frame trajectory image for backward compatibility
    if (!panels.empty()) {
        cv::Mat last_frame = panels.back().clone();
        std::string img_filename = save_dir + filename_prefix + ".jpg";
        cv::imwrite(img_filename, last_frame);
    }
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg, int& index){
    sensor_msgs::ImageConstPtr _msg = msg;
    imgQueue[index].Produce(std::move(_msg));
}

void trackCallback(const infer_nodelet::RadarTrackObjectProject::ConstPtr msg, int& index){
    infer_nodelet::RadarTrackObjectProject::ConstPtr _msg = msg;
    trackQueue[index].Produce(std::move(_msg));
}
