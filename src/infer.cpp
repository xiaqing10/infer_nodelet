#include "log_macros.h"
#include <infer.h>
#include <iostream>
#include<string.h>
//#include <vehicle_color.hpp>
#include "device_params_config.h"
#include <algorithm>
#include <map>
//#include "//diagnostic.hpp"
#include "img_score.hpp"  // 图像质量检测
#if USE_SOPHON
#include "sophon/ff_decode_sophon.hpp"
#endif
#if USE_SOPHON || USE_RKNN
#include "batch_pipeline_base.hpp"
#endif
#if USE_NVIDIA
#include <cuda_runtime.h>
#endif
#include <sys/stat.h>
#include <ctime>
#include "ffmpeg_video_reader.h"
// 注册Nodelet
#include <pluginlib/class_list_macros.h>
//#include <cnrt.h>
using namespace std;

#if USE_RKNN
static void rknn_infer_thread(int core_id) {
    LOG_INFO("[rknn_infer] thread started, core=%d", core_id);
    YoloV8_det& det = static_cast<RknnPipeline*>(g_pipeline.get())->getDetector(core_id);

    while (ros::ok()) {
        BatchFrameData fd;
        if (!g_infer_queue.ConsumeSync(fd)) {
            continue;
        }

        YoloV8BoxVec boxes;
        int ret = det.Detect(fd.mat, boxes);
        if (ret != 0) continue;

        CameraResult cr;
        cr.frame = std::move(fd.mat);
        cr.img_time_sec = fd.img_time_sec;
        cr.img_time_nsec = fd.img_time_nsec;
        for (auto& box : boxes) {
            DetectorRetData d;
            d.label = box.class_id + 1;
            d.confidence = box.score;
            d.xmin = (int)box.x1;
            d.ymin = (int)box.y1;
            d.xmax = (int)box.x2;
            d.ymax = (int)box.y2;
            cr.detections.push_back(d);
        }

        g_result_queues[fd.camera_id].Produce(std::move(cr));
    }
}
#elif USE_SOPHON
static void batch_preprocess_thread(int batch_size) {
    LOG_INFO("[batch_pre] thread started, batch_size=%d", batch_size);
    int num_cameras = g_cam_frame_queues.size();

    while (ros::ok()) {
        std::vector<BatchFrameData> batch_frames;

        for (int i = 0; i < num_cameras; i++) {
            BatchFrameData fd;
            if (g_cam_frame_queues[i].Consume(fd)) {
                batch_frames.push_back(std::move(fd));
            }
        }
        if (batch_frames.empty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

        InferResult ir;
        int ret = g_pipeline->preprocessAndInfer(batch_frames, ir);
        if (ret != 0) {
            LOG_ERROR("[batch_pre] preprocessAndInfer failed");
            continue;
        }

        g_post_queue.Produce(std::move(ir));
    }
}

static void batch_postprocess_thread() {
    LOG_INFO("[batch_post] thread started");
    while (ros::ok()) {
        InferResult ir;
        if (!g_post_queue.ConsumeSync(ir)) {
            continue;
        }

        for (int i = 0; i < ir.batch_size; i++) {
            int cam_id = ir.camera_ids[i];
            CameraResult cr;
            cr.frame = std::move(ir.frames[i]);
            cr.detections = std::move(ir.detections[i]);
            cr.img_time_sec = ir.img_time_secs[i];
            cr.img_time_nsec = ir.img_time_nsecs[i];
            g_result_queues[cam_id].Produce(std::move(cr));
        }
    }
}
#endif

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
        //diagnostic.init("Infer Model", "Infer Camera", rvf::system::ModuleCode::kInfer);
        
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

/*
void InferDet::loadModel(const std::vector<std::string>& model_paths) {
    // Validate model path count
    constexpr size_t REQUIRED_MODELS = 4;
    if (model_paths.size() < REQUIRED_MODELS) {
        LOG_ERROR("Insufficient model files (%zu < %zu)", model_paths.size(), REQUIRED_MODELS);
        //diagnostic.Pub//diagnosticData(ERROR_CODE_LEVEL::ERROR, 
                                    rvf::system::KeyCode::kModelLoadFailed, 
                                    "InsufficientModelFiles");
    }

    if (access(model_paths[0].c_str(), F_OK) != 0 || access(model_paths[1].c_str(), F_OK) != 0 || access(model_paths[3].c_str(), F_OK) != 0 ) {
        //diagnostic.Pub//diagnosticData(ERROR_CODE_LEVEL::ERROR, rvf::system::KeyCode::kModelLoadFailed, "DetectorModelFileNotExists"  );
        LOG_ERROR("Error: Detector model file %s not exists", model_paths[0].c_str());
    }

    LOG_INFO("Loading... Detector Model %s", model_paths[0].c_str());
    detector.init(model_paths[0]);
    
    LOG_INFO("Loding... COLOR Model %s",  model_paths[1].c_str());
#if USE_SOPHON
    vehicle_color_detector.Init(model_paths[1]);
#endif

    LOG_INFO("Loading... Abandon Model %s", model_paths[3].c_str());
    abandon_detector.init(model_paths[3], 3);

    LOG_INFO("Model loading completed");
}
*/


void  InferDet::load_det_model(std::string model_path, int mlu_infer_device, int num_class, int stride){
#if !USE_SOPHON && !USE_RKNN
    LOG_INFO("Loding... DETRECTOR Model %s", model_path.c_str());
    detector.init(model_path, mlu_infer_device, num_class, stride);
#else
    (void)model_path; (void)mlu_infer_device; (void)num_class; (void)stride;
#endif
}

void  InferDet::load_abandon_model(std::string model_path, int mlu_infer_device, int num_class, int stride){
    abandon_detector.init(model_path, mlu_infer_device, num_class, stride);
    LOG_INFO("Load model success!!!");
}

#if USE_SOPHON || USE_NVIDIA
void  InferDet::load_color_model(std::string model_path){
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

    // 确保保存路径存在
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
    // 统计周期 5 秒。用 steady_clock 测量实际经过时间，
    // 避免 ros::Rate 在 ROS 时间漂移时产生"双触发+跳帧"导致读数失真。
    const double rate_duration = 5.0;
    auto last = std::chrono::steady_clock::now();
    
    while (ros::ok()) {  // Ensure clean shutdown
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds((long long)(rate_duration * 1000.0)));
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last).count();
            last = now;
            if (elapsed <= 0.0) elapsed = rate_duration;
            double current_fps = 0.0;
            
            // Minimize lock duration
            {
                std::lock_guard<std::mutex> lock(mtx);
                current_fps = static_cast<double>(publish_hz) / elapsed;
                publish_hz = 0;
            }
            
            // Operations outside lock to reduce contention
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

    while (ros::ok()) {  // Add proper shutdown handling
        abandonRecQueue.ConsumeSync(input_data);


        cv::Mat img = input_data.im;
        auto results = abandon_detector.inference(img);

        // Precompute valid results once
        std::vector<DetectorRetData> valid_results;
        valid_results.reserve(results.size());
        std::copy_if(results.begin(), results.end(), std::back_inserter(valid_results),
            [&img](const auto& obj) {
                return obj.xmin >= 0 && obj.ymin >= 0 && 
                       obj.xmax < img.cols && obj.ymax < img.rows;
            });

        // Filter overlapping detections using lambda
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
        // 不为空也要发
        if (1) {
            DetectorRetDatas results_data{{std::move(filtered)}};
            abandonResultQueue.Produce(std::move(results_data));
        }
    }
}




void InferDet::processVideoFile(std::string video_path, int index) {
    LOG_INFO("processVideoFile thread running... video: %s", video_path.c_str());

    FFmpegVideoReader reader;
    if (!reader.open(video_path)) {
        LOG_ERROR("Failed to open video file: %s", video_path.c_str());
        return;
    }

    double fps = reader.getFPS();
    int total_frames = reader.getTotalFrames();
    LOG_INFO("Video fps: %.2f, total frames: %d, size: %dx%d", fps, total_frames, reader.getWidth(), reader.getHeight());
    cache_interval_ = std::max(1, (int)(fps + 0.5));

    bytetrack_params params;
    bytetrack_yaml_parse(byte_track_config_file, params);
    BYTETracker bytetrack(params, write_flag, write_path, min_points_len, camera_type, camera_direction);
    bytetrack.setTrackRemovedCallback(
        [this](int track_id, int class_id, const std::vector<std::vector<float>>& track_points) -> std::string {
            std::vector<TrackPoint> pts;
            for (auto& p : track_points) {
                if (img_src.cols == 0 || img_src.rows == 0) continue;
                float cx = (p[0] + p[2] / 2) / img_src.cols;
                float cy = (p[1] + p[3] / 2) / img_src.rows;
                if (!std::isfinite(cx) || !std::isfinite(cy)) continue;
                TrackPoint pt;
                pt.frame_id = static_cast<int>(p[5]);
                pt.x = cx;
                pt.y = cy;
                pt.class_id = static_cast<int>(p[4]);
                pts.push_back(pt);
            }
            TrafficEventType evt = traffic_analyzer.analyzeTrajectory(track_id, class_id, pts,
                                                camera_type, camera_direction, img_src);
            switch (evt) {
                case TrafficEventType::WRONG_WAY: return "wrong_way";
                case TrafficEventType::STATIONARY: return "stationary";
                default: return "";
            }
        });

    bytetrack.setSaveFrameCallback(
        [this](int track_id, int class_id, const std::vector<std::vector<float>>& track_points,
               const std::string& save_dir, const std::string& filename_prefix) {
            { FILE* dbg = fopen("/tmp/montage_debug.txt", "a"); if(dbg) { fprintf(dbg, "CB CALLED track_id=%d prefix=%s\n", track_id, filename_prefix.c_str()); fclose(dbg); } }
            saveTrackMontage(track_id, class_id, track_points, save_dir, filename_prefix);
        });

    int rec_index = 1;
    auto log_time = std::chrono::system_clock::now();
    std::vector<DetectorRetData> abandon_results;

    cv::Mat frame;
    int frame_count = 0;
    ros::Rate video_loop_rate(15);

    while (ros::ok()) {
        reader.read(frame);
        if (frame.empty()) {
            LOG_INFO("Video finished, total frames processed: %d", frame_count);
            break;
        }

        frame_count++;
        auto once_start_time = std::chrono::system_clock::now();

        frame.copyTo(img_src);

#if USE_RKNN
        {
            BatchFrameData bfd;
            bfd.mat = frame.clone();
            bfd.camera_id = camera_id_;
            bfd.frame_width = frame.cols;
            bfd.frame_height = frame.rows;
            g_infer_queue.Produce(std::move(bfd));
        }
#elif USE_SOPHON
        {
            BatchFrameData bfd;
            bfd.mat = frame.clone();
            bfd.camera_id = camera_id_;
            bfd.frame_width = frame.cols;
            bfd.frame_height = frame.rows;
            g_cam_frame_queues[camera_id_].Produce(std::move(bfd));
        }
#else
        std::vector<DetectorRetData> res = detector.inference(frame);

        for (auto& obj : res) {
            cv::Rect r01(obj.xmin, obj.ymin, obj.xmax - obj.xmin, obj.ymax - obj.ymin);
            cv::rectangle(img_src, r01, cv::Scalar(0, 0, 255), 3);
        }

        rec_index++;
        if (rec_index % abandon_rate == 0) {
            AbandonInputData input_abandon_data;
            input_abandon_data.im = frame.clone();
            input_abandon_data.data = res;
            abandonRecQueue.Produce(std::move(input_abandon_data));
        }

        // Cache resized frame every 1 second (for montage) — before update() so callback has current frame
        {
            int current_fid = bytetrack.getFrameId() + 1;  // update() will increment frame_id
            if (current_fid % cache_interval_ == 0) {
                cv::Mat small;
                cv::resize(img_src, small, cv::Size(), 0.33, 0.33);
                frame_cache_.push_back({current_fid, small.clone()});
                while (frame_cache_.size() > MAX_CACHED_FRAMES)
                    frame_cache_.pop_front();
            }
        }

        std::vector<STrack> output_stracks = bytetrack.update(res);

        infer_nodelet::ImageDetectObject tracker_msg;
        infer_nodelet::ImageDetectObjectSingle single_msg;
        int objects_number = output_stracks.size();

        for (auto bbox : output_stracks) {
            int _id = bbox.track_id;
            int x0 = std::max(0, (int)bbox.det_box[0]);
            int y0 = std::max(0, (int)bbox.det_box[1]);
            int w = std::max(0, (int)bbox.det_box[2]);
            int h = std::max(0, (int)bbox.det_box[3]);

            if (w <= 0 || h <= 0) continue;
            if (x0 < 0 || y0 < 0 || x0 > img_src.cols || x0 + w > img_src.cols || w < 0 || h < 0 || y0 > img_src.rows || y0 + h > img_src.rows) continue;

            if (draw_tracker) {
                int track_x = (int)(bbox.track_points[0][0] + bbox.track_points[0][2] / 2);
                int track_y = (int)(bbox.track_points[0][1] + bbox.track_points[0][3]);
                for (unsigned track_point_len = 1; track_point_len < bbox.track_points.size(); track_point_len++) {
                    auto _x = (int)(bbox.track_points[track_point_len][0] + bbox.track_points[track_point_len][2] / 2);
                    auto _y = (int)(bbox.track_points[track_point_len][1] + bbox.track_points[track_point_len][3]);
                    cv::line(img_src, cv::Point(track_x, track_y), cv::Point(_x, _y), vehicle_colors[bbox.vehicle_color], 3, 8);
                    track_x = _x;
                    track_y = _y;
                }
            }
            cv::Scalar color = vehicle_colors[bbox.vehicle_color];
            if (bbox.state == TrackState::Lost) {
                // Lost轨迹：虚线框 + 半透明文字
                for (int line_x = x0; line_x < x0 + w; line_x += 8) {
                    cv::line(img_src, cv::Point(line_x, y0), cv::Point(std::min(line_x + 4, x0 + w), y0), color, 2);
                    cv::line(img_src, cv::Point(line_x, y0 + h), cv::Point(std::min(line_x + 4, x0 + w), y0 + h), color, 2);
                }
                for (int line_y = y0; line_y < y0 + h; line_y += 8) {
                    cv::line(img_src, cv::Point(x0, line_y), cv::Point(x0, std::min(line_y + 4, y0 + h)), color, 2);
                    cv::line(img_src, cv::Point(x0 + w, line_y), cv::Point(x0 + w, std::min(line_y + 4, y0 + h)), color, 2);
                }
                cv::putText(img_src, std::to_string(_id) + "-" + std::to_string(bbox.class_id) + "(lost)",
                            cv::Point(x0, y0 - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            } else {
                cv::rectangle(img_src, cv::Rect(x0, y0, w, h), color, 2);
                cv::putText(img_src, std::to_string(_id) + "-" + std::to_string(bbox.class_id),
                            cv::Point(x0, y0 - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }

            single_msg.x_pixel_norm = std::clamp((float)x0 / img_src.cols, 0.0f, 1.0f);
            single_msg.y_pixel_norm = std::clamp((float)y0 / img_src.rows, 0.0f, 1.0f);
            single_msg.w_pixel_norm = std::clamp((float)w / img_src.cols, 0.0f, 1.0f);
            single_msg.h_pixel_norm = std::clamp((float)h / img_src.rows, 0.0f, 1.0f);
            single_msg.target_type = bbox.class_id;
            single_msg.id = _id;
            single_msg.color = bbox.vehicle_color;
            single_msg.plate_number = bbox.plate_number;
            single_msg.plate_confid = bbox.plate_confid;
            single_msg.plate_color = bbox.plate_color;
            tracker_msg.objects.push_back(single_msg);
        }

        // 添加抛洒物
        {
            DetectorRetDatas abandon_data;
            //abandon_results.clear();
            while (true) {
                auto _abandon = abandonResultQueue.Consume(abandon_data);
                if (!_abandon) break;
                else abandon_results = abandon_data.data;
            }

            for (unsigned _abandon_index = 0; _abandon_index < abandon_results.size(); _abandon_index++) {
                cv::Rect r = cv::Rect(abandon_results[_abandon_index].xmin, abandon_results[_abandon_index].ymin,
                                      abandon_results[_abandon_index].xmax - abandon_results[_abandon_index].xmin,
                                      abandon_results[_abandon_index].ymax - abandon_results[_abandon_index].ymin);

                single_msg.x_pixel_norm = std::clamp((float)abandon_results[_abandon_index].xmin / img_src.cols, 0.0f, 1.0f);
                single_msg.y_pixel_norm = std::clamp((float)abandon_results[_abandon_index].ymin / img_src.rows, 0.0f, 1.0f);
                single_msg.w_pixel_norm = std::clamp((float)(abandon_results[_abandon_index].xmax - abandon_results[_abandon_index].xmin) / img_src.cols, 0.0f, 1.0f);
                single_msg.h_pixel_norm = std::clamp((float)(abandon_results[_abandon_index].ymax - abandon_results[_abandon_index].ymin) / img_src.rows, 0.0f, 1.0f);

                if (abandon_results[_abandon_index].label == 1) {
                    single_msg.target_type = 33;
                    single_msg.id = 65536;
                    cv::putText(img_src, "c", cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
                } else if (abandon_results[_abandon_index].label == 2) {
                    single_msg.target_type = 35;
                    single_msg.id = 65537;
                    cv::putText(img_src, "b", cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
                } else if (abandon_results[_abandon_index].label == 3) {
                    single_msg.target_type = 21;
                    single_msg.id = 65538;
                    cv::putText(img_src, "d", cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
                }

                single_msg.color = 0;
                single_msg.plate_confid = 0;
                single_msg.plate_color = 0;
                tracker_msg.objects.push_back(single_msg);
                cv::Scalar color = vehicle_colors[0];
                cv::rectangle(img_src, r, color, 2);
                // cv::putText(img_src, "o", cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
                objects_number += 1;
            }
            }

        tracker_msg.header.seq = frame_count;
        tracker_msg.header.stamp.sec = (uint32_t)(frame_count / fps);
        tracker_msg.header.stamp.nsec = (uint32_t)((frame_count / fps - (uint32_t)(frame_count / fps)) * 1e9);
        tracker_msg.header.frame_id = "video";
        tracker_msg.frame_seq = frame_count;
        tracker_msg.objects_number = objects_number;

        if (publish_img) {
            cv_bridge::CvImage brigeImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_src);
            pub_img.publish(brigeImg.toImageMsg());
        }
        pub_tracker.publish(tracker_msg);

        publish_hz += 1;



        auto once_end_time = std::chrono::system_clock::now();
        if (once_end_time - log_time > std::chrono::milliseconds(20000)) {
            LOG_INFO("all infer consume: %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(once_end_time - once_start_time).count());
            log_time = once_end_time;
        }
#endif

        video_loop_rate.sleep();
    }

    reader.release();

    LOG_INFO("processVideoFile thread finished");
}

void InferDet::saveTrackMontage(int track_id, int class_id,
                                const std::vector<std::vector<float>>& track_points,
                                const std::string& save_dir,
                                const std::string& filename_prefix) {
    if (track_points.empty()) return;

    // Debug: write to a separate file
    { FILE* dbg = fopen("/tmp/montage_debug.txt", "a"); if(dbg) { fprintf(dbg, "START track_id=%d prefix=%s points=%zu cache=%zu img_src=%dx%d\n", track_id, filename_prefix.c_str(), track_points.size(), frame_cache_.size(), img_src.cols, img_src.rows); fclose(dbg); } }

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
        { FILE* dbg = fopen("/tmp/montage_debug.txt", "a"); if(dbg) { fprintf(dbg, "SKIP: no cached frames\n"); fclose(dbg); } }
        return;
    }

    { FILE* dbg = fopen("/tmp/montage_debug.txt", "a"); if(dbg) { fprintf(dbg, "recent=%zu scale=%.3fx%.3f\n", recent.size(), (float)recent[0].img.cols / img_src.cols, (float)recent[0].img.rows / img_src.rows); fclose(dbg); } }

    // Scale factor for cached images
    float scale_x = (float)recent[0].img.cols / img_src.cols;
    float scale_y = (float)recent[0].img.rows / img_src.rows;

    // Debug: check first few track points
    if (!track_points.empty()) {
        auto& first = track_points[0];
        auto& last = track_points.back();
        { FILE* dbg = fopen("/tmp/montage_debug.txt", "a"); if(dbg) { fprintf(dbg, "first: fid=%d cx=%.1f cy=%.1f last: fid=%d cx=%.1f cy=%.1f\n", (int)first[5], first[0]+first[2]/2, first[1]+first[3]/2, (int)last[5], last[0]+last[2]/2, last[1]+last[3]/2); fclose(dbg); } }
    }

    // Build panels: draw trajectory on each cached frame
    std::vector<cv::Mat> panels;
    for (auto& cf : recent) {
        cv::Mat panel = cf.img.clone();

        // Draw trajectory lines up to this frame
        cv::Point prev_pt(-1, -1);
        int line_count = 0;
        for (auto& p : track_points) {
            int fid = static_cast<int>(p[5]);
            if (fid > cf.frame_id) continue;
            float cx = p[0] + p[2] / 2;
            float cy = p[1] + p[3] / 2;
            cv::Point cur((int)(cx * scale_x), (int)(cy * scale_y));
            if (prev_pt.x >= 0) {
                cv::line(panel, prev_pt, cur, cv::Scalar(0, 255, 255), 2);
                line_count++;
            }
            prev_pt = cur;
        }
        if (line_count == 0) {
            { FILE* dbg = fopen("/tmp/montage_debug.txt", "a"); if(dbg) { fprintf(dbg, "WARN: no lines drawn for frame %d (track %d)\n", cf.frame_id, track_id); fclose(dbg); } }
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

    { FILE* dbg = fopen("/tmp/montage_debug.txt", "a"); if(dbg) { fprintf(dbg, "panels=%zu layout=5x6 panel=%dx%d\n", panels.size(), panel_w, panel_h); fclose(dbg); } }

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


int  InferDet::processRadarCamera(int index){

    LOG_INFO("processRadarCamera thread runing...");

    cv::Mat mat_receive;
    sensor_msgs::ImageConstPtr msg_img;
    infer_nodelet::RadarTrackObjectProject::ConstPtr  msg_track;

    // 如果是SHM模式，启动独立读取线程
#if USE_RKNN
    if (use_shm) {
        std::thread([this, index]() {
            while (ros::ok()) {
                void* data = nullptr;
                int size = 0;
                int result = shm_reader->nocopyRead((void**)&data, size);
                if (result >= 0 && data) {
                    auto* pkt = static_cast<ehawkeye::modules::common::packet*>(data);
                    cv::Mat mat = cv::Mat(pkt->height, pkt->width, CV_8UC3, pkt->data).clone();
                    BatchFrameData bfd;
                    bfd.mat = mat;
                    bfd.camera_id = camera_id_;
                    bfd.frame_width = mat.cols;
                    bfd.frame_height = mat.rows;
                    bfd.img_time_sec = (double)(pkt->dts / 1000000LL);
                    bfd.img_time_nsec = (double)((pkt->dts % 1000LL) * 1000000LL);
                    g_infer_queue.Produce(std::move(bfd));
                }
            }
        }).detach();
    }
#elif USE_SOPHON
    if (use_shm) {
        std::thread([this, index]() {
            while (ros::ok()) {
                void* data = nullptr;
                int size = 0;
                int result = shm_reader->nocopyRead((void**)&data, size);
                if (result >= 0 && data) {
                    auto* pkt = static_cast<ehawkeye::modules::common::packet*>(data);
                    cv::Mat mat = cv::Mat(pkt->height, pkt->width, CV_8UC3, pkt->data).clone();
                    BatchFrameData bfd;
                    bfd.mat = mat;
                    bfd.camera_id = camera_id_;
                    bfd.frame_width = mat.cols;
                    bfd.frame_height = mat.rows;
                    bfd.img_time_sec = (double)(pkt->dts / 1000000LL);
                    bfd.img_time_nsec = (double)((pkt->dts % 1000LL) * 1000000LL);
                    g_cam_frame_queues[camera_id_].Produce(std::move(bfd));
                }
            }
        }).detach();
    }
#endif

    // 跟踪部分
    bytetrack_params params;
    bytetrack_yaml_parse(byte_track_config_file, params);
    BYTETracker bytetrack(params, write_flag, write_path, min_points_len,  camera_type, camera_direction);
    bytetrack.setTrackRemovedCallback(
        [this](int track_id, int class_id, const std::vector<std::vector<float>>& track_points) -> std::string {
            std::vector<TrackPoint> pts;
            for (auto& p : track_points) {
                if (img_src.cols == 0 || img_src.rows == 0) continue;
                float cx = (p[0] + p[2] / 2) / img_src.cols;
                float cy = (p[1] + p[3] / 2) / img_src.rows;
                if (!std::isfinite(cx) || !std::isfinite(cy)) continue;
                TrackPoint pt;
                pt.frame_id = static_cast<int>(p[5]);
                pt.x = cx;
                pt.y = cy;
                pt.class_id = static_cast<int>(p[4]);
                pts.push_back(pt);
            }
            TrafficEventType evt = traffic_analyzer.analyzeTrajectory(track_id, class_id, pts,
                                                camera_type, camera_direction, img_src);
            switch (evt) {
                case TrafficEventType::WRONG_WAY: return "wrong_way";
                case TrafficEventType::STATIONARY: return "stationary";
                default: return "";
            }
        });

    bytetrack.setSaveFrameCallback(
        [this](int track_id, int class_id, const std::vector<std::vector<float>>& track_points,
               const std::string& save_dir, const std::string& filename_prefix) {
            { FILE* dbg = fopen("/tmp/montage_debug.txt", "a"); if(dbg) { fprintf(dbg, "CB CALLED track_id=%d prefix=%s\n", track_id, filename_prefix.c_str()); fclose(dbg); } }
            saveTrackMontage(track_id, class_id, track_points, save_dir, filename_prefix);
        });
    
    int rec_index = 1; // 其他目标无需要实时检测，计数器
    cache_interval_ = 15;  // ~1 second for 15fps cameras
    const int  DETECT_QUALITY_RATE  = 14 * 60;
    auto log_time = std::chrono::system_clock::now();  //    记录一次推理的开始时间
    std::vector<DetectorRetData> abandon_results;   // 主线程发的是这一段时间内检测的重复数据.
    bool print_diff_time = true;  // 是否打印时间差
    int lost_camera_count = 0;
    int lost_radar_count = 0;
    std::chrono::steady_clock::time_point camera_offline_start;
    bool camera_offline_reported = false;
    ros::Rate loop_rate(15);

    while(ros::ok()){
        auto once_start_time = std::chrono::system_clock::now();

if (use_shm)
        {
            // SHM 模式下帧由独立线程直接推入 g_cam_frame_queues，
            // 经 batch pipeline 处理后由 processResult 线程异步消费。
            // 主循环仅负责雷达数据读取和离线检测。
            // 休眠一小段时间避免空转
            usleep(10000);
            continue;
        } else
        {
            if(imgQueue[index].Consume(msg_img)){
            img_time_sec = msg_img -> header.stamp.sec;
            img_time_nsec = msg_img -> header.stamp.nsec;
            lost_camera_count = 0;
            camera_offline_reported = false;
        }
        else{
            usleep(3000);
            lost_camera_count ++;
            if(!camera_offline_reported){
                camera_offline_start = std::chrono::steady_clock::now();
                camera_offline_reported = true;
            }
            if(lost_camera_count % 600 == 0 && camera_direction != "" && camera_type != "" ){
                auto offline_sec = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - camera_offline_start).count();
                LOG_WARN("No Camera Data %s %s, offline for %lds", camera_direction.c_str(), camera_type.c_str(), offline_sec);
            }
            continue;
        }

        try {
                mat_receive =  cv_bridge::toCvShare(msg_img, "bgr8")->image ;
            }
        catch( cv_bridge::Exception& e )
            {
                LOG_ERROR( "Could not convert from '%s' to 'bgr8'.", msg_img->encoding.c_str() );
                mat_receive = cv::Mat();
            }
        }

        if (mat_receive.empty())
            {
                LOG_ERROR("Error: Could not parse image") ;
                //diagnostic.Pub//diagnosticData(ERROR_CODE_LEVEL::ERROR, rvf::system::KeyCode::kImgCodeError, "ImageDecodeError-" + camera_direction + "-" + camera_type);
                continue;
            }
else{
            // LOG_INFO("START TO INFER & TRACKER");
#if !USE_SOPHON && !USE_RKNN
            mat_receive.copyTo(img_src);
#endif
#if USE_RKNN
            // Unified queue: push frame to single queue, infer threads consume
            BatchFrameData bfd;
            bfd.mat = mat_receive.clone();
            bfd.camera_id = camera_id_;
            bfd.frame_width = mat_receive.cols;
            bfd.frame_height = mat_receive.rows;
            bfd.img_time_sec = img_time_sec;
            bfd.img_time_nsec = img_time_nsec;
            g_infer_queue.Produce(std::move(bfd));
            // 结果由 processResult 线程异步消费
            continue;
#elif USE_SOPHON
            std::vector<DetectorRetData>  res = detector.inference(mat_receive);
            // add seg draw 
            /*
            const std::vector<std::vector<unsigned int>> COLORS = {
            {0, 114, 189},   {217, 83, 25},   {237, 177, 32},  {126, 47, 142},  {119, 172, 48},  {77, 190, 238},
            {162, 20, 47},   {76, 76, 76},    {153, 153, 153}, {255, 0, 0},     {255, 128, 0},   {191, 191, 0},
            {0, 255, 0},     {0, 0, 255},     {170, 0, 255},   {85, 85, 0},     {85, 170, 0},    {85, 255, 0},
            {170, 85, 0},    {170, 170, 0},   {170, 255, 0},   {255, 85, 0},    {255, 170, 0},   {255, 255, 0},
            {0, 85, 128},    {0, 170, 128},   {0, 255, 128},   {85, 0, 128},    {85, 85, 128},   {85, 170, 128},
            {85, 255, 128},  {170, 0, 128},   {170, 85, 128},  {170, 170, 128}, {170, 255, 128}, {255, 0, 128},
            {255, 85, 128},  {255, 170, 128}, {255, 255, 128}, {0, 85, 255},    {0, 170, 255},   {0, 255, 255},
            {85, 0, 255},    {85, 85, 255},   {85, 170, 255},  {85, 255, 255},  {170, 0, 255},   {170, 85, 255},
            {170, 170, 255}, {170, 255, 255}, {255, 0, 255},   {255, 85, 255},  {255, 170, 255}, {85, 0, 0},
            {128, 0, 0},     {170, 0, 0},     {212, 0, 0},     {255, 0, 0},     {0, 43, 0},      {0, 85, 0},
            {0, 128, 0},     {0, 170, 0},     {0, 212, 0},     {0, 255, 0},     {0, 0, 43},      {0, 0, 85},
            {0, 0, 128},     {0, 0, 170},     {0, 0, 212},     {0, 0, 255},     {0, 0, 0},       {36, 36, 36},
            {73, 73, 73},    {109, 109, 109}, {146, 146, 146}, {182, 182, 182}, {219, 219, 219}, {0, 114, 189},
            {80, 183, 189},  {128, 128, 0}};

            const std::vector<std::vector<unsigned int>> MASK_COLORS = {
            {255, 56, 56},  {255, 157, 151}, {255, 112, 31}, {255, 178, 29}, {207, 210, 49},  {72, 249, 10}, {146, 204, 23},
            {61, 219, 134}, {26, 147, 52},   {0, 212, 187},  {44, 153, 168}, {0, 194, 255},   {52, 69, 147}, {100, 115, 255},
            {0, 24, 236},   {132, 56, 255},  {82, 0, 133},   {203, 56, 255}, {255, 149, 200}, {255, 55, 199}};
            cv::Mat mask = img_src.clone();
	    */
            for (auto& obj : res) {
                cv::Rect r01(obj.xmin, obj.ymin, obj.xmax-obj.xmin, obj.ymax-obj.ymin);
                cv::rectangle(img_src, r01,cv::Scalar(0, 0, 255) , 3);    
            }
            
            // 经测试抛洒物检测实时的话占用主线程资源，所以要跳帧.
            
            rec_index ++;
            if(rec_index % abandon_rate == 0 ){
                AbandonInputData input_abandon_data;
                input_abandon_data.im = mat_receive.clone();
                input_abandon_data.data = res;
                abandonRecQueue.Produce(std::move(input_abandon_data));
                // LOG_INFO("start to push abandon data ");
            }

            // 进行图像质量检测
            if (rec_index % DETECT_QUALITY_RATE == 0) {
            //runQualityChecks(img_src, camera_direction, camera_type);
            }

            // 画出雷达投影  
            if(1){
                auto t_ = trackQueue[index].Consume(msg_track);
                if(t_ ) {
                    radar_time_sec = msg_track->header.stamp.sec;
                    radar_time_nsec = msg_track->header.stamp.nsec;

                    lost_radar_count = 0;

                    int64_t time_diff = img_time_sec * 1000 + img_time_nsec / 1000000  - radar_time_sec * 1000 - radar_time_nsec/ 1000000;
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                    //  每分钟的打印下时间差
                    if (print_diff_time){
                        if(time_diff < -400 || time_diff > 400) {
                            LOG_WARN("Diff Time: %s %s  %ld img: %lf  radar: %lf ",camera_direction.c_str(), camera_type.c_str(),  time_diff, (img_time_sec * 1000 + img_time_nsec / 1000000 -ms ),(radar_time_sec * 1000 + radar_time_nsec/ 1000000 -ms ));
                            std::string code_str = "CameraRadarTimeDiff-" + camera_direction + "-" + camera_type + "-" + std::to_string(time_diff) + "ms";
                            //diagnostic.Pub//diagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kCameraRadarOversize, code_str);
                            }
                        
                        else{
                            LOG_INFO("Diff Time: %s %s  %ld img: %lf  radar: %lf ",camera_direction.c_str(), camera_type.c_str(), time_diff, (img_time_sec * 1000 + img_time_nsec / 1000000 -ms ),(radar_time_sec * 1000 + radar_time_nsec/ 1000000 -ms ));
                        }
                        print_diff_time = false;
                    }

                    for (unsigned int i =0 ;i < msg_track->objects.size(); i ++){
                        if(camera_type == "long"){
                            auto x = msg_track->objects[i].x_pixel_norm_long * img_src.cols;
                            auto y = msg_track->objects[i].y_pixel_norm_long * img_src.rows;

                            if (x > 0 && y > 0){
                                cv::circle(img_src,cv::Point (x, y), 11, cv::Scalar(0, 0, 255), -1);
                                // std::cout << " x & y: " << x <<" " << y << std::endl;
                            }
                        }
                        else{
                            auto x = msg_track->objects[i].x_pixel_norm_short * img_src.cols;
                            auto y = msg_track->objects[i].y_pixel_norm_short * img_src.rows;
                            if (x > 0 && y > 0){
                                cv::circle(img_src,cv::Point (x, y), 11, cv::Scalar(0, 0, 255), -1);
                                // std::cout << " x & y: " << x <<" " << y << std::endl;
                            }
                        }
                    }
                }

                else{
                    lost_radar_count ++;
                    if(lost_radar_count == 1000 && camera_direction != "" && camera_type != "" ){
                        LOG_WARN("No Radar Data %s %s %d", camera_direction.c_str(), camera_type.c_str(), lost_radar_count);
                        //diagnostic.Pub//diagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kNoRadarData, "NoRadarData");
                        lost_radar_count = 0;
                    }
                }
            }

            // Cache resized frame every 1 second (for montage) — before update() so callback has current frame
            {
                int current_fid = bytetrack.getFrameId() + 1;  // update() will increment frame_id
                if (current_fid % cache_interval_ == 0) {
                    cv::Mat small;
                    cv::resize(img_src, small, cv::Size(), 0.33, 0.33);
                    frame_cache_.push_back({current_fid, small.clone()});
                    while (frame_cache_.size() > MAX_CACHED_FRAMES)
                        frame_cache_.pop_front();
                }
            }

            std::vector<STrack> output_stracks = bytetrack.update(res); // 跟踪 目前不支持传输类被，类别默认使用第一帧.

#if USE_SOPHON || USE_NVIDIA
            while(true){
                VehicleColorResult vehicle_color_result;
                auto _c = vehicleColorResultQueue.Consume(vehicle_color_result);
                if(!_c) break;
                bytetrack.updateVehicleColor(vehicle_color_result.tracker_id, vehicle_color_result.vehicle_color, vehicle_color_result.confidence);
            }
#endif

            infer_nodelet::ImageDetectObject  tracker_msg;
            infer_nodelet::ImageDetectObjectSingle  single_msg;
            int objects_number = output_stracks.size();
            for (auto bbox : output_stracks) {
                int _id = bbox.track_id;
                int x0 = max(0, (int)bbox.det_box[0]);
                int y0 = max(0, (int)bbox.det_box[1]);
                int w = max(0, (int)bbox.det_box[2]);
                int h = max(0, (int)bbox.det_box[3]);

                if (w <= 0 || h <= 0) {
                    // LOG_WARN("Invalid bounding box: w=%d, h=%d", w, h);
                    continue;
                }
                cv::Rect r= cv::Rect(x0,y0,w,h);
                if (x0 < 0 || y0 < 0 || x0 > img_src.cols || x0 + w > img_src.cols || w < 0 || h < 0 || y0 > img_src.rows || y0 + h > img_src.rows){
                    // LOG_WARN("detector box : %d %d %d %d %d %d ", x0, y0, w, h, img_src.cols, img_src.rows );
                    continue;
                }
                // 画出每条轨迹
                if(draw_tracker){
                    int track_x = (int)(bbox.track_points[0][0] + bbox.track_points[0][2] / 2);
                    int track_y = (int)(bbox.track_points[0][1] + bbox.track_points[0][3]);
                    for(unsigned track_point_len = 1; track_point_len < bbox.track_points.size(); track_point_len++){
                        auto _x = (int)(bbox.track_points[track_point_len][0] + bbox.track_points[track_point_len][2] / 2);
                        auto _y = (int)(bbox.track_points[track_point_len][1] + bbox.track_points[track_point_len][3]);
                        cv::line(img_src, cv::Point(track_x, track_y), cv::Point(_x, _y), vehicle_colors[bbox.vehicle_color],3,8);
                        track_x = _x;
                        track_y = _y;
                    }
                }
                //if(bbox.class_id > 2 &&  r.area() > 2000 && rec_index  && !bbox.color_lock ) {
#if USE_SOPHON || USE_NVIDIA
                if(bbox.class_id > 2 && r.area() > 2000 && !bbox.color_lock ) {
                    cv::Mat crop_vehicle_img = mat_receive(cv::Rect(r.x, r.y, r.width, r.height));
                    ModelInputData input_vehicle_data;
                    input_vehicle_data.im = crop_vehicle_img.clone();
                    input_vehicle_data.tracker_id = _id;
                    vehicleColorQueue.Produce(std::move(input_vehicle_data));
                }
#endif

                // 单轨迹数据处理完毕封装发送数据
                cv::Scalar color = vehicle_colors[bbox.vehicle_color];
                if (bbox.state == TrackState::Lost) {
                    for (int line_x = r.x; line_x < r.x + r.width; line_x += 8) {
                        cv::line(img_src, cv::Point(line_x, r.y), cv::Point(std::min(line_x + 4, r.x + r.width), r.y), color, 2);
                        cv::line(img_src, cv::Point(line_x, r.y + r.height), cv::Point(std::min(line_x + 4, r.x + r.width), r.y + r.height), color, 2);
                    }
                    for (int line_y = r.y; line_y < r.y + r.height; line_y += 8) {
                        cv::line(img_src, cv::Point(r.x, line_y), cv::Point(r.x, std::min(line_y + 4, r.y + r.height)), color, 2);
                        cv::line(img_src, cv::Point(r.x + r.width, line_y), cv::Point(r.x + r.width, std::min(line_y + 4, r.y + r.height)), color, 2);
                    }
                    cv::putText(img_src, std::to_string(_id) + "-" + to_string(bbox.class_id) + "(lost)", cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                } else {
                    cv::rectangle(img_src, r, color, 2);    
                    cv::putText(img_src, std::to_string(_id) + "-" + to_string(bbox.class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                }
                
                single_msg.x_pixel_norm = std::clamp((float)r.x  / img_src.cols, float(0),float(1));
                single_msg.y_pixel_norm = std::clamp((float)r.y / img_src.rows,float(0),float(1));
                single_msg.w_pixel_norm = std::clamp((float)r.width  / img_src.cols, float(0),float(1));
                single_msg.h_pixel_norm = std::clamp((float)r.height / img_src.rows ,float(0),float(1));
                single_msg.target_type  = bbox.class_id;
                single_msg.id = _id;
                single_msg.color        = bbox.vehicle_color;
                single_msg.plate_number = bbox.plate_number;
                single_msg.plate_confid = bbox.plate_confid;
                single_msg.plate_color  = bbox.plate_color;
                tracker_msg.objects.push_back(single_msg);
            }

            // 添加抛洒物
            if(1){
                DetectorRetDatas abandon_data;
                // abandon_results.clear();
                while(true){
                    auto _abandon = abandonResultQueue.Consume(abandon_data);
                    if(!_abandon) break;
                    else abandon_results = abandon_data.data;  // 只有数据发过来了才更新，否则沿用.
                }


                for(unsigned _abandon_index = 0; _abandon_index < abandon_results.size(); _abandon_index ++){
                    cv::Rect r= cv::Rect(abandon_results[_abandon_index].xmin, abandon_results[_abandon_index].ymin, \
                        abandon_results[_abandon_index].xmax - abandon_results[_abandon_index].xmin, \
                            abandon_results[_abandon_index].ymax - abandon_results[_abandon_index].ymin);

                    single_msg.x_pixel_norm = std::clamp((float)abandon_results[_abandon_index].xmin / img_src.cols, float(0),float(1));
                    single_msg.y_pixel_norm = std::clamp((float)abandon_results[_abandon_index].ymin / img_src.rows,float(0),float(1));
                    single_msg.w_pixel_norm = std::clamp((float)(abandon_results[_abandon_index].xmax - abandon_results[_abandon_index].xmin)  / img_src.cols, float(0),float(1));
                    single_msg.h_pixel_norm = std::clamp((float)(abandon_results[_abandon_index].ymax - abandon_results[_abandon_index].ymin) / img_src.rows ,float(0),float(1));

                    /*11：动物，21：箱子，  22：轮胎，23：树枝，  31：施工车辆 ，32：施工人员 ，33：锥桶，  34：柱桶 ，35：路障， 36：路牌 */
                    if(abandon_results[_abandon_index].label == 1){
                        single_msg.target_type  = 33; 
                        single_msg.id = 65536;
                    }
                    else if(abandon_results[_abandon_index].label == 2){
                        single_msg.target_type  = 35; 
                        single_msg.id = 65537;
                    }
                    else if(abandon_results[_abandon_index].label == 3){
                        single_msg.target_type  = 21; 
                        single_msg.id = 65538;
                    }

                    single_msg.color        =  0;
                    single_msg.plate_confid =  0;
                    single_msg.plate_color  = 0;  // 目前暂无此功能
                    tracker_msg.objects.push_back(single_msg);
cv::Scalar color = vehicle_colors[0];
                    cv::rectangle(img_src, r, color, 2);
                    cv::putText(img_src, "other", cv::Point(r.x , r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
                    objects_number += 1;
                }
            }

            // 封装信息
            tracker_msg.header.seq = 1;
            tracker_msg.header.stamp.sec = img_time_sec;
            tracker_msg.header.stamp.nsec = img_time_nsec;
            tracker_msg.header.frame_id = "image";  
            tracker_msg.frame_seq += 1;
            tracker_msg.objects_number = objects_number;
            if (publish_img) {
                cv_bridge::CvImage brigeImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_src);
                pub_img.publish(brigeImg.toImageMsg());
            }
            pub_tracker.publish(tracker_msg);

            publish_hz+=1;

            // 20s打印一次推理时间以及雷达和摄像头时间差
            auto once_end_time = std::chrono::system_clock::now();
            if (once_end_time - log_time > std::chrono::milliseconds(20000)) {
                LOG_INFO("all infer consume: %ld ms" , std::chrono::duration_cast<std::chrono::milliseconds>(once_end_time - once_start_time).count());
                log_time = once_end_time;
                print_diff_time = true;
            }

            loop_rate.sleep();
#endif
        }
    }
return 0;
}

#if USE_SOPHON || USE_RKNN
void InferDet::processResult() {
    LOG_INFO("processResult thread running...");

    bytetrack_params params;
    bytetrack_yaml_parse(byte_track_config_file, params);
    BYTETracker bytetrack(params, write_flag, write_path, min_points_len, camera_type, camera_direction);
    bytetrack.setTrackRemovedCallback(
        [this](int track_id, int class_id, const std::vector<std::vector<float>>& track_points) -> std::string {
            std::vector<TrackPoint> pts;
            for (auto& p : track_points) {
                if (img_src.cols == 0 || img_src.rows == 0) continue;
                float cx = (p[0] + p[2] / 2) / img_src.cols;
                float cy = (p[1] + p[3] / 2) / img_src.rows;
                if (!std::isfinite(cx) || !std::isfinite(cy)) continue;
                TrackPoint pt;
                pt.frame_id = static_cast<int>(p[5]);
                pt.x = cx;
                pt.y = cy;
                pt.class_id = static_cast<int>(p[4]);
                pts.push_back(pt);
            }
            TrafficEventType evt = traffic_analyzer.analyzeTrajectory(track_id, class_id, pts,
                                                camera_type, camera_direction, img_src);
            switch (evt) {
                case TrafficEventType::WRONG_WAY: return "wrong_way";
                case TrafficEventType::STATIONARY: return "stationary";
                default: return "";
            }
        });

    bytetrack.setSaveFrameCallback(
        [this](int track_id, int class_id, const std::vector<std::vector<float>>& track_points,
               const std::string& save_dir, const std::string& filename_prefix) {
            saveTrackMontage(track_id, class_id, track_points, save_dir, filename_prefix);
        });

    int rec_index = 1;
    cache_interval_ = 15;
    const int DETECT_QUALITY_RATE = 14 * 60;
    auto log_time = std::chrono::system_clock::now();
    int local_frame_count = 0;
    std::vector<DetectorRetData> abandon_results;
    bool print_diff_time = true;
    int lost_radar_count = 0;
    ros::Rate loop_rate(15);

    while (ros::ok()) {
        // 从推理结果队列消费（帧+检测结果）
        CameraResult cr;
        g_result_queues[camera_id_].ConsumeSync(cr);
        auto t0 = std::chrono::system_clock::now();

        std::vector<DetectorRetData> res = std::move(cr.detections);
        img_src = cr.frame;  // 浅拷贝，不涉及内存拷贝
        img_time_sec = cr.img_time_sec;
        img_time_nsec = cr.img_time_nsec;

        // 抛洒物检测
        rec_index++;
        if (rec_index % abandon_rate == 0) {
            AbandonInputData input_abandon_data;
            input_abandon_data.im = img_src.clone();
            input_abandon_data.data = res;
            abandonRecQueue.Produce(std::move(input_abandon_data));
        }

        // 雷达数据
        {
            infer_nodelet::RadarTrackObjectProject::ConstPtr msg_track;
            auto t_ = trackQueue[camera_id_].Consume(msg_track);
            if (t_) {
                radar_time_sec = msg_track->header.stamp.sec;
                radar_time_nsec = msg_track->header.stamp.nsec;
                lost_radar_count = 0;

                int64_t time_diff = img_time_sec * 1000 + img_time_nsec / 1000000 - radar_time_sec * 1000 - radar_time_nsec / 1000000;
                if (print_diff_time) {
                    if (time_diff < -400 || time_diff > 400) {
                        LOG_WARN("Diff Time: %s %s  %ld", camera_direction.c_str(), camera_type.c_str(), time_diff);
                    } else {
                        LOG_INFO("Diff Time: %s %s  %ld", camera_direction.c_str(), camera_type.c_str(), time_diff);
                    }
                    print_diff_time = false;
                }
            } else {
                lost_radar_count++;
                if (lost_radar_count == 1000 && camera_direction != "" && camera_type != "") {
                    LOG_WARN("No Radar Data %s %s %d", camera_direction.c_str(), camera_type.c_str(), lost_radar_count);
                    lost_radar_count = 0;
                }
            }
        }

        // Cache frame for montage
        if (write_flag) {
            int current_fid = bytetrack.getFrameId() + 1;
            if (current_fid % cache_interval_ == 0) {
                cv::Mat small;
                cv::resize(img_src, small, cv::Size(), 0.33, 0.33);
                frame_cache_.push_back({current_fid, small.clone()});
                while (frame_cache_.size() > MAX_CACHED_FRAMES)
                    frame_cache_.pop_front();
            }
        }

        // ByteTrack 跟踪
        std::vector<STrack> output_stracks = bytetrack.update(res);

        // 车辆颜色
#if USE_SOPHON || USE_NVIDIA
        while (true) {
            VehicleColorResult vehicle_color_result;
            auto _c = vehicleColorResultQueue.Consume(vehicle_color_result);
            if (!_c) break;
            bytetrack.updateVehicleColor(vehicle_color_result.tracker_id, vehicle_color_result.vehicle_color, vehicle_color_result.confidence);
        }
#endif

        infer_nodelet::ImageDetectObject tracker_msg;
        infer_nodelet::ImageDetectObjectSingle single_msg;
        int objects_number = output_stracks.size();

        for (auto bbox : output_stracks) {
            int _id = bbox.track_id;
            int x0 = max(0, (int)bbox.det_box[0]);
            int y0 = max(0, (int)bbox.det_box[1]);
            int w = max(0, (int)bbox.det_box[2]);
            int h = max(0, (int)bbox.det_box[3]);

            if (w <= 0 || h <= 0) continue;
            cv::Rect r = cv::Rect(x0, y0, w, h);
            if (x0 < 0 || y0 < 0 || x0 > img_src.cols || x0 + w > img_src.cols || w < 0 || h < 0 || y0 > img_src.rows || y0 + h > img_src.rows)
                continue;

#if USE_SOPHON || USE_NVIDIA
            if (bbox.class_id > 2 && r.area() > 2000 && !bbox.color_lock) {
                cv::Mat crop_vehicle_img = img_src(cv::Rect(r.x, r.y, r.width, r.height));
                ModelInputData input_vehicle_data;
                input_vehicle_data.im = crop_vehicle_img.clone();
                input_vehicle_data.tracker_id = _id;
                vehicleColorQueue.Produce(std::move(input_vehicle_data));
            }
#endif

            single_msg.x_pixel_norm = std::clamp((float)r.x / img_src.cols, 0.0f, 1.0f);
            single_msg.y_pixel_norm = std::clamp((float)r.y / img_src.rows, 0.0f, 1.0f);
            single_msg.w_pixel_norm = std::clamp((float)r.width / img_src.cols, 0.0f, 1.0f);
            single_msg.h_pixel_norm = std::clamp((float)r.height / img_src.rows, 0.0f, 1.0f);
            single_msg.target_type = bbox.class_id;
            single_msg.id = _id;
            single_msg.color = bbox.vehicle_color;
            single_msg.plate_number = bbox.plate_number;
            single_msg.plate_confid = bbox.plate_confid;
            single_msg.plate_color = bbox.plate_color;
            tracker_msg.objects.push_back(single_msg);
        }

        // 添加抛洒物
        {
            DetectorRetDatas abandon_data;
            while (true) {
                auto _abandon = abandonResultQueue.Consume(abandon_data);
                if (!_abandon) break;
                else abandon_results = abandon_data.data;
            }

            for (unsigned _abandon_index = 0; _abandon_index < abandon_results.size(); _abandon_index++) {
                single_msg.x_pixel_norm = std::clamp((float)abandon_results[_abandon_index].xmin / img_src.cols, 0.0f, 1.0f);
                single_msg.y_pixel_norm = std::clamp((float)abandon_results[_abandon_index].ymin / img_src.rows, 0.0f, 1.0f);
                single_msg.w_pixel_norm = std::clamp((float)(abandon_results[_abandon_index].xmax - abandon_results[_abandon_index].xmin) / img_src.cols, 0.0f, 1.0f);
                single_msg.h_pixel_norm = std::clamp((float)(abandon_results[_abandon_index].ymax - abandon_results[_abandon_index].ymin) / img_src.rows, 0.0f, 1.0f);

                if (abandon_results[_abandon_index].label == 1) {
                    single_msg.target_type = 33;
                    single_msg.id = 65536;
                } else if (abandon_results[_abandon_index].label == 2) {
                    single_msg.target_type = 35;
                    single_msg.id = 65537;
                } else if (abandon_results[_abandon_index].label == 3) {
                    single_msg.target_type = 21;
                    single_msg.id = 65538;
                }

                single_msg.color = 0;
                single_msg.plate_confid = 0;
                single_msg.plate_color = 0;
                tracker_msg.objects.push_back(single_msg);
                objects_number += 1;
            }
        }

        tracker_msg.header.seq = 1;
        tracker_msg.header.stamp.sec = img_time_sec;
        tracker_msg.header.stamp.nsec = img_time_nsec;
        tracker_msg.header.frame_id = "image";
        tracker_msg.frame_seq += 1;
        tracker_msg.objects_number = objects_number;
        pub_tracker.publish(tracker_msg);

        // 跟踪数据每帧必发布，与图像同步；以跟踪数据发布为准统计每路实际帧率
        publish_hz += 1;

        // 图像发布：写入 publish slot，由独立的 publishThread 异步消费
        if (publish_img) {
            std::lock_guard<std::mutex> lock(pub_slot_.mtx);
            pub_slot_.frame = img_src.clone();
            pub_slot_.stracks = output_stracks;
            pub_slot_.abandon_results = abandon_results;
            pub_slot_.ready = true;
        }

        local_frame_count++;

        auto once_end_time = std::chrono::system_clock::now();
        if (once_end_time - log_time > std::chrono::milliseconds(20000)) {
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(once_end_time - log_time).count();
            double fps = local_frame_count * 1000.0 / dt;
            LOG_INFO("%s %s freq: %.1f fps", camera_direction.c_str(), camera_type.c_str(), fps);
            local_frame_count = 0;
            log_time = once_end_time;
            print_diff_time = true;
        }
    }
}

void InferDet::publishThread() {
    LOG_INFO("publishThread running...");
    while (ros::ok()) {
        cv::Mat frame;
        std::vector<STrack> stracks;
        std::vector<DetectorRetData> abandon_results;
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lock(pub_slot_.mtx);
            if (pub_slot_.ready) {
                frame = pub_slot_.frame.clone();
                stracks = pub_slot_.stracks;
                abandon_results = pub_slot_.abandon_results;
                pub_slot_.ready = false;
                has_data = true;
            }
        }
        if (!has_data) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        cv::Mat draw_img = frame.clone();

        // Draw tracker visualization
        if (draw_tracker) {
            for (auto& bbox : stracks) {
                int x0 = std::max(0, (int)bbox.det_box[0]);
                int y0 = std::max(0, (int)bbox.det_box[1]);
                int w = std::max(0, (int)bbox.det_box[2]);
                int h = std::max(0, (int)bbox.det_box[3]);
                if (w <= 0 || h <= 0) continue;
                cv::Rect r(x0, y0, w, h);
                if (x0 < 0 || y0 < 0 || x0 + w > draw_img.cols || y0 + h > draw_img.rows)
                    continue;

                if (draw_tracker) {
                    for (auto& tp : bbox.track_points) {
                        int track_x = (int)(tp[0] + tp[2] / 2);
                        int track_y = (int)(tp[1] + tp[3]);
                        cv::line(draw_img, cv::Point(track_x, track_y), cv::Point(track_x, track_y), vehicle_colors[bbox.vehicle_color], 3, 8);
                    }
                }

                cv::Scalar color = vehicle_colors[bbox.vehicle_color];
                if (bbox.state == TrackState::Lost) {
                    for (int line_x = r.x; line_x < r.x + r.width; line_x += 8) {
                        cv::line(draw_img, cv::Point(line_x, r.y), cv::Point(std::min(line_x + 4, r.x + r.width), r.y), color, 2);
                        cv::line(draw_img, cv::Point(line_x, r.y + r.height), cv::Point(std::min(line_x + 4, r.x + r.width), r.y + r.height), color, 2);
                    }
                    for (int line_y = r.y; line_y < r.y + r.height; line_y += 8) {
                        cv::line(draw_img, cv::Point(r.x, line_y), cv::Point(r.x, std::min(line_y + 4, r.y + r.height)), color, 2);
                        cv::line(draw_img, cv::Point(r.x + r.width, line_y), cv::Point(r.x + r.width, std::min(line_y + 4, r.y + r.height)), color, 2);
                    }
                    cv::putText(draw_img, std::to_string(bbox.track_id) + "-" + std::to_string(bbox.class_id) + "(lost)",
                                cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                } else {
                    cv::rectangle(draw_img, r, color, 2);
                    cv::putText(draw_img, std::to_string(bbox.track_id) + "-" + std::to_string(bbox.class_id),
                                cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                }
            }
        }

        // Draw abandon objects
        for (auto& ad : abandon_results) {
            cv::Rect r(ad.xmin, ad.ymin, ad.xmax - ad.xmin, ad.ymax - ad.ymin);
            cv::rectangle(draw_img, r, vehicle_colors[0], 2);
            const char* label = "?";
            if (ad.label == 1) label = "c";
            else if (ad.label == 2) label = "b";
            else if (ad.label == 3) label = "d";
            cv::putText(draw_img, label, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
        }

        cv_bridge::CvImage brigeImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_img);
        pub_img.publish(brigeImg.toImageMsg());
    }
}
#endif


// 图像质量检测
 /*
void InferDet::runQualityChecks(const cv::Mat& img, 
                                const std::string& direction, 
                                const std::string& type) {
    auto checkAndPublish = [&](std::function<double(cv::Mat)> detector, 
                              rvf::system::KeyCode code, 
                              const std::string& name,
                              double threshold = 0.8) {
        double score = detector(img.clone());
        LOG_INFO("%s %s DETECT %s %f", direction.c_str(), type.c_str(), name.c_str(), score);
        if (score > threshold) {
            //diagnostic.Pub//diagnosticData(WARN, code, name);
        }
    };

    checkAndPublish([this](cv::Mat img){ return signalDetect(img); }, 
                   rvf::system::KeyCode::kBlackScreen, "BlackScreen");
    checkAndPublish([this](cv::Mat img){ return blockDetect(img); }, 
                   rvf::system::KeyCode::kBlockScreen, "BlockScreen");
    checkAndPublish([this](cv::Mat img){ return snowNoiseDetect(img); }, 
                   rvf::system::KeyCode::kSnowScreen, "SnowScreen");
    checkAndPublish([this](cv::Mat img){ return brightnessDetect(img); },   
                   rvf::system::KeyCode::kBrightnessScreen, "BrightScreen");
    checkAndPublish([this](cv::Mat img){ return sharpnessDetect(img); },
                   rvf::system::KeyCode::kBlurScreen, "BlurScreen"); 
            
    // Day/Night detection
    bool day_or_night = DayOrNight(img.clone());
    LOG_INFO("%s %s %s", direction.c_str(), type.c_str(), 
            day_or_night ? "DAY" : "NIGHT");
}

*/
void imgCallback(const sensor_msgs::ImageConstPtr& msg,int& index){
    sensor_msgs::ImageConstPtr _msg = msg;
    imgQueue[index].Produce(std::move(_msg));
}

void trackCallback(const infer_nodelet::RadarTrackObjectProject::ConstPtr msg, int& index){
    infer_nodelet::RadarTrackObjectProject::ConstPtr _msg = msg;
    trackQueue[index].Produce(std::move(_msg));
}


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

