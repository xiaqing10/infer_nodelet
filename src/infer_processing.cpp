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
#include "ffmpeg_video_reader.h"
#if USE_SOPHON
#include "sophon/ff_decode_sophon.hpp"
#endif
#if USE_SOPHON || USE_RKNN
#include "batch_pipeline_base.hpp"
#endif
#if USE_NVIDIA
#include <cuda_runtime.h>
#endif

using namespace std;

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
    BYTETracker bytetrack(params, write_flag, save_img_flag, write_path, min_points_len, camera_type, camera_direction, pole_name);
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

#if USE_NVIDIA
        if (!g_nvidia_batch_mode) {
            frame.copyTo(img_src);
        }
#else
        frame.copyTo(img_src);
#endif

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
#if USE_NVIDIA
        if (g_nvidia_batch_mode) {
            BatchFrameData bfd;
            bfd.mat = frame.clone();
            bfd.camera_id = camera_id_;
            bfd.frame_width = frame.cols;
            bfd.frame_height = frame.rows;
            bfd.img_time_sec = img_time_sec;
            bfd.img_time_nsec = img_time_nsec;
            bfd.receive_local_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            g_cam_frame_queues[camera_id_].Produce(std::move(bfd));
            // 结果由 processResult 线程异步消费
            continue;
        }
#endif
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
            drawRoiBox(img_src);
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

int InferDet::processRadarCamera(int index){

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
                    bfd.img_time_sec = (double)(pkt->dts / 1000LL);
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
                    bfd.img_time_sec = (double)(pkt->dts / 1000LL);
                    bfd.img_time_nsec = (double)((pkt->dts % 1000LL) * 1000000LL);
                    g_cam_frame_queues[camera_id_].Produce(std::move(bfd));
                }
            }
        }).detach();
    }
#elif USE_NVIDIA
    if (use_shm && g_nvidia_batch_mode) {
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
                    bfd.img_time_sec = (double)(pkt->dts / 1000LL);
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
    BYTETracker bytetrack(params, write_flag, save_img_flag, write_path, min_points_len,  camera_type, camera_direction, pole_name);
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

#if !USE_RKNN
    int rec_index = 1; // 其他目标无需要实时检测，计数器
    cache_interval_ = 15;  // ~1 second for 15fps cameras
    const int  DETECT_QUALITY_RATE  = 14 * 60;
    auto log_time = std::chrono::system_clock::now();  //    记录一次推理的开始时间
    std::vector<DetectorRetData> abandon_results;   // 主线程发的是这一段时间内检测的重复数据.
    int lost_radar_count = 0;
    ros::Rate loop_rate(15);
#endif
    int lost_camera_count = 0;
    std::chrono::steady_clock::time_point camera_offline_start;
    bool camera_offline_reported = false;

    while(ros::ok()){
#if !USE_RKNN
        auto once_start_time = std::chrono::system_clock::now();
#endif
        int64_t receive_local_ms = 0;  // 本机接收该帧的本地墙钟(ms)，循环级作用域，供 produce 使用

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
            receive_local_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
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
                continue;
            }
else{
            // LOG_INFO("START TO INFER & TRACKER");
#if !USE_SOPHON && !USE_RKNN
#if USE_NVIDIA
            if (!g_nvidia_batch_mode) {
                mat_receive.copyTo(img_src);
            }
#else
            mat_receive.copyTo(img_src);
#endif
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
            bfd.receive_local_ms = receive_local_ms;
            g_infer_queue.Produce(std::move(bfd));
            // 结果由 processResult 线程异步消费
            continue;
#elif USE_SOPHON
            BatchFrameData bfd;
            bfd.mat = mat_receive.clone();
            bfd.camera_id = camera_id_;
            bfd.frame_width = mat_receive.cols;
            bfd.frame_height = mat_receive.rows;
            bfd.img_time_sec = img_time_sec;
            bfd.img_time_nsec = img_time_nsec;
            bfd.receive_local_ms = receive_local_ms;
            g_cam_frame_queues[camera_id_].Produce(std::move(bfd));
            // 结果由 processResult 线程异步消费
            continue;
#else
#if USE_NVIDIA
            if (g_nvidia_batch_mode) {
                BatchFrameData bfd;
                bfd.mat = mat_receive.clone();
                bfd.camera_id = camera_id_;
                bfd.frame_width = mat_receive.cols;
                bfd.frame_height = mat_receive.rows;
                bfd.img_time_sec = img_time_sec;
                bfd.img_time_nsec = img_time_nsec;
                bfd.receive_local_ms = receive_local_ms;
                g_frame_cnt_input[camera_id_]++;
                g_cam_frame_queues[camera_id_].Produce(std::move(bfd));
                // 结果由 processResult 线程异步消费
                continue;
            }
#endif
            std::vector<DetectorRetData>  res = detector.inference(mat_receive);
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
            }

            // 进行图像质量检测
            if (rec_index % DETECT_QUALITY_RATE == 0) {
            //runQualityChecks(img_src, camera_direction, camera_type);
            }

            // 画出雷达投影
            if(1){
                auto t_ = trackQueue[index].Consume(msg_track);
                if(t_ ) {
                    lost_radar_count = 0;

                    for (unsigned int i =0 ;i < msg_track->objects.size(); i ++){
                        if(camera_type == "long"){
                            auto x = msg_track->objects[i].x_pixel_norm_long * img_src.cols;
                            auto y = msg_track->objects[i].y_pixel_norm_long * img_src.rows;

                            if (x > 0 && y > 0){
                                cv::circle(img_src,cv::Point (x, y), 11, cv::Scalar(0, 0, 255), -1);
                            }
                        }
                        else{
                            auto x = msg_track->objects[i].x_pixel_norm_short * img_src.cols;
                            auto y = msg_track->objects[i].y_pixel_norm_short * img_src.rows;
                            if (x > 0 && y > 0){
                                cv::circle(img_src,cv::Point (x, y), 11, cv::Scalar(0, 0, 255), -1);
                            }
                        }
                    }
                }

                else{
                    lost_radar_count ++;
                    if(lost_radar_count == 1000 && camera_direction != "" && camera_type != "" ){
                        LOG_WARN("No Radar Data %s %s %d", camera_direction.c_str(), camera_type.c_str(), lost_radar_count);
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
                    continue;
                }
                cv::Rect r= cv::Rect(x0,y0,w,h);
                if (x0 < 0 || y0 < 0 || x0 > img_src.cols || x0 + w > img_src.cols || w < 0 || h < 0 || y0 > img_src.rows || y0 + h > img_src.rows){
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
                    const char* abandon_label = "?";
                    if(abandon_results[_abandon_index].label == 1){
                        single_msg.target_type  = 33;
                        single_msg.id = 65536;
                        abandon_label = "c";
                    }
                    else if(abandon_results[_abandon_index].label == 2){
                        single_msg.target_type  = 35;
                        single_msg.id = 65537;
                        abandon_label = "b";
                    }
                    else if(abandon_results[_abandon_index].label == 3){
                        single_msg.target_type  = 21;
                        single_msg.id = 65538;
                        abandon_label = "d";
                    }

                    single_msg.color        =  0;
                    single_msg.plate_confid =  0;
                    single_msg.plate_color  = 0;  // 目前暂无此功能
                    tracker_msg.objects.push_back(single_msg);
cv::Scalar color = vehicle_colors[0];
                    cv::rectangle(img_src, r, color, 2);
                    cv::putText(img_src, abandon_label, cv::Point(r.x , r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
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
                drawRoiBox(img_src);
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
            }

            loop_rate.sleep();
#endif
        }
    }
return 0;
}

#if USE_SOPHON || USE_RKNN || USE_NVIDIA
void InferDet::processResult() {
    LOG_INFO("processResult thread running...");

    bytetrack_params params;
    bytetrack_yaml_parse(byte_track_config_file, params);
    BYTETracker bytetrack(params, write_flag, save_img_flag, write_path, min_points_len, camera_type, camera_direction, pole_name);
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
    auto log_time = std::chrono::system_clock::now();
    int local_frame_count = 0;
    std::vector<DetectorRetData> abandon_results;
    int lost_radar_count = 0;
#if USE_NVIDIA
    // NVIDIA 发布链路分段计时：区分 Consume 等待 vs bytetrack.update vs 其余处理
    long long result_consume_ms = 0, result_bytetrack_ms = 0, result_other_ms = 0;
    // 诊断：逐帧时间戳间隔，判断帧到达是否均匀（区分数据源抖动 vs batch 帧间隔不均）
    long long prev_img_ms = -1;
    bool first_frame = true;
#endif

    while (ros::ok()) {
        // 从推理结果队列消费（帧+检测结果）
        CameraResult cr;
#if USE_NVIDIA
        auto t_result = std::chrono::steady_clock::now();
        long long result_consume_this = 0, result_bt_this = 0;
#endif
        g_result_queues[camera_id_].ConsumeSync(cr);
#if USE_NVIDIA
        result_consume_this = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t_result).count();
        result_consume_ms += result_consume_this;
#endif

        std::vector<DetectorRetData> res = std::move(cr.detections);
        img_src = cr.frame;  // 浅拷贝，不涉及内存拷贝
        img_time_sec = cr.img_time_sec;
        img_time_nsec = cr.img_time_nsec;
        receive_local_ms = cr.receive_local_ms;

#if USE_NVIDIA
        // 方案A 诊断可视化：仅由 ROI 增强检测出的框用绿色叠加，区分主图检测框。
        if (roi_enabled) {
            for (const auto& d : res) {
                if (!d.from_roi) continue;
                cv::Rect rr(d.xmin, d.ymin, d.xmax - d.xmin, d.ymax - d.ymin);
                if (rr.width <= 0 || rr.height <= 0) continue;
                if (rr.x < 0 || rr.y < 0 || rr.x + rr.width > img_src.cols || rr.y + rr.height > img_src.rows) continue;
                cv::rectangle(img_src, rr, cv::Scalar(0, 255, 0), 2);
            }
        }
#endif

#if USE_NVIDIA
        {
            // 诊断：打印本路相邻两帧的时间戳间隔与消费间隔，判断帧到达/处理是否均匀
            long long cur_ms = (long long)img_time_sec * 1000 + img_time_nsec / 1000000;
            long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (first_frame) {
                first_frame = false;
            } else {
                LOG_DEBUG("[frame-int] cam=%d img_gap=%lldms consume_this=%lldms",
                          camera_id_, cur_ms - prev_img_ms, result_consume_this);
            }
            prev_img_ms = cur_ms;
            (void)now_ms;
        }
#endif
        // 抛洒物检测
        rec_index++;
        if (rec_index % abandon_rate == 0) {
            AbandonInputData input_abandon_data;
            input_abandon_data.im = img_src.clone();
            input_abandon_data.data = res;
#if USE_SOPHON
            input_abandon_data.camera_id = camera_id_;
            g_abandon_queue.Produce(std::move(input_abandon_data));
#else
            abandonRecQueue.Produce(std::move(input_abandon_data));
#endif
        }

        // 雷达数据
        {
            infer_nodelet::RadarTrackObjectProject::ConstPtr msg_track;
            auto t_ = trackQueue[camera_id_].Consume(msg_track);
            if (t_) {
                lost_radar_count = 0;
            } else {
                lost_radar_count++;
                if (lost_radar_count == 1000 && camera_direction != "" && camera_type != "") {
                    LOG_WARN("No Radar Data %s %s %d", camera_direction.c_str(), camera_type.c_str(), lost_radar_count);
                    lost_radar_count = 0;
                }
            }
        }

        // Cache frame for montage
        if (save_img_flag) {
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
#if USE_NVIDIA
        auto t_bt = std::chrono::steady_clock::now();
#endif
        std::vector<STrack> output_stracks = bytetrack.update(res);
#if USE_NVIDIA
        result_bt_this = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t_bt).count();
        result_bytetrack_ms += result_bt_this;
#endif

        // 车辆颜色
#if USE_SOPHON
        while (true) {
            VehicleColorResult vehicle_color_result;
            auto _c = g_color_result_queues[camera_id_].Consume(vehicle_color_result);
            if (!_c) break;
            bytetrack.updateVehicleColor(vehicle_color_result.tracker_id, vehicle_color_result.vehicle_color, vehicle_color_result.confidence);
        }
#elif USE_NVIDIA
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

#if USE_SOPHON
            if (bbox.class_id > 2 && r.area() > 2000 && !bbox.color_lock) {
                cv::Mat crop_vehicle_img = img_src(cv::Rect(r.x, r.y, r.width, r.height));
                ModelInputData input_vehicle_data;
                input_vehicle_data.im = crop_vehicle_img.clone();
                input_vehicle_data.tracker_id = _id;
                input_vehicle_data.camera_id = camera_id_;
                g_color_queue.Produce(std::move(input_vehicle_data));
            }
#elif USE_NVIDIA
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
#if USE_SOPHON
                auto _abandon = g_abandon_result_queues[camera_id_].Consume(abandon_data);
#else
                auto _abandon = abandonResultQueue.Consume(abandon_data);
#endif
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
#if USE_NVIDIA
        // 处理延时诊断：发布时刻本地墙钟 - 接收时刻本地墙钟（两端同本地时钟，与数据源时钟无关）。
        // 反映图像从本节点接收到发布处理完成所经过的时间（含排队+推理+跟踪+画框）。
        if (receive_local_ms > 0) {
            long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if (last_delay_print_ms == 0 || now_ms - last_delay_print_ms >= (long long)process_delay_print_interval * 1000) {
                LOG_INFO("[proc-delay] pole=%s cam=%d delay=%lldms (recv_local=%lldms)", pole_name.c_str(), camera_id_, now_ms - receive_local_ms, receive_local_ms);
                last_delay_print_ms = now_ms;
            }
        }
#endif
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
#if USE_NVIDIA
        {
            long long cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_result).count();
            long long other_this = cycle_ms - result_consume_this - result_bt_this;
            if (other_this < 0) other_this = 0;
            result_other_ms += other_this;
        }
#endif

        auto once_end_time = std::chrono::system_clock::now();
        if (once_end_time - log_time > std::chrono::milliseconds(20000)) {
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(once_end_time - log_time).count();
            double fps = local_frame_count * 1000.0 / dt;
            LOG_INFO("%s %s freq: %.1f fps", camera_direction.c_str(), camera_type.c_str(), fps);
#if USE_NVIDIA
            if (local_frame_count > 0) {
                LOG_DEBUG("[nvidia-result] %s %s frames=%d consume=%.2fms bt=%.2fms other=%.2fms",
                         camera_direction.c_str(), camera_type.c_str(), local_frame_count,
                         (double)result_consume_ms / local_frame_count,
                         (double)result_bytetrack_ms / local_frame_count,
                         (double)(result_other_ms) / local_frame_count);
            }
#endif
            local_frame_count = 0;
#if USE_NVIDIA
            result_consume_ms = 0;
            result_bytetrack_ms = 0;
            result_other_ms = 0;
#endif
            log_time = once_end_time;
        }
    }
}

void InferDet::drawRoiBox(cv::Mat& img) {
    // Draw ROI 增强检测区域框（配合 ROI 抠图增强，仅 NVIDIA + publish_img 时随图像输出）
    // 尺寸与位置由 per-camera camera_roi 决定（y_ratio/x_ratio 为框左上角偏移），
    // 与 batch 抠图/检测使用的 g_camera_roi 保持一致。
#if USE_NVIDIA
    if (roi_enabled && img.cols > 0 && img.rows > 0) {
        int rw = (int)(img.cols * roi_width_ratio);
        int rh = (int)(img.rows * roi_height_ratio);
        if (rw >= 2 && rh >= 2) {
            int rx = std::max(0, std::min((int)(img.cols * roi_x_ratio), img.cols - rw));
            int ry = std::max(0, std::min((int)(img.rows * roi_y_ratio), img.rows - rh));
            cv::Rect roi_rect(rx, ry, rw, rh);
            roi_rect &= cv::Rect(0, 0, img.cols, img.rows);
            cv::rectangle(img, roi_rect, cv::Scalar(0, 255, 255), 2);
        }
    }
#endif
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

        drawRoiBox(draw_img);

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
