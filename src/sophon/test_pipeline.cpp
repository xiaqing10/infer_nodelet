#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdio>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <sstream>
#include <functional>
#include "detector_sophon.hpp"
#include "bytetrack.h"
#include "shm/shmmem.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "infer_nodelet/ImageDetectObject.h"
#include "infer_nodelet/ImageDetectObjectSingle.h"

struct alignas(1) Packet {
    int32_t magic;     int32_t size;
    int16_t pixfmt;    int16_t type;
    int32_t nalu;      int32_t width;
    int32_t height;    int64_t dts;
    char    reserved[32];
    char    data[0];
};

struct FrameData {
    cv::Mat mat;
    int camera_id;
    int frame_width;
    int frame_height;
};

struct InferResult {
    std::vector<bm_image> input_images;
    std::vector<bm_tensor_t> output_tensors;
    std::vector<std::pair<int, int>> txy_batch;
    std::vector<std::pair<float, float>> ratios_batch;
    std::vector<int> camera_ids;
    std::vector<cv::Mat> frames;
    int batch_size;
};

// Queue for images from each camera (pre -> forward)
struct CameraQueue {
    std::queue<FrameData> q;
    std::mutex mtx;
    std::condition_variable cv;
};

// Queue for inference results (forward -> post)
struct PostQueue {
    std::queue<InferResult> q;
    std::mutex mtx;
    std::condition_variable cv;
};

// Queue for per-camera detection results + frame (post -> track)
struct TrackData {
    cv::Mat frame;
    int frame_width;
    int frame_height;
    std::vector<DetectorRetData> detections;
};
struct TrackQueue {
    std::queue<TrackData> q;
    std::mutex mtx;
    std::condition_variable cv;
};

static YoloV8_det g_detector;
static const int BATCH_SIZE = 4;
static const int NUM_CAMERAS = 8;
static CameraQueue g_cam_queues[NUM_CAMERAS];
static PostQueue g_post_queue;
static TrackQueue g_track_queues[NUM_CAMERAS];
static std::atomic<int> g_frame_count{0};
static std::atomic<int> g_cam_frame_count[NUM_CAMERAS];
static std::atomic<int> g_cam_tracked_count[NUM_CAMERAS];
static std::atomic<bool> g_stop{false};

static std::vector<std::string> shm_names = {
    "/_k7_855_test_downstream_extra1_camera_image_raw.decoder",
    "/_k7_855_test_downstream_extra2_camera_image_raw.decoder",
    "/_k7_855_test_downstream_extra3_camera_image_raw.decoder",
    "/_k7_855_test_downstream_extra4_camera_image_raw.decoder",
    "/_k7_855_test_downstream_extra5_camera_image_raw.decoder",
    "/_k7_855_test_downstream_extra6_camera_image_raw.decoder",
    "/_k7_855_upstream_short_camera_image_raw.decoder",
    "/_k7_855_downstream_short_camera_image_raw.decoder",
};

static long long now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void bytetrack_yaml_parse(const std::string& config_path, bytetrack_params& params) {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        fprintf(stderr, "Failed to open config file: %s\n", config_path.c_str());
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string key, value;
        std::getline(ss, key, ':');
        std::getline(ss, value);
        key = key.substr(key.find_first_not_of(" \t\r\n"));
        value.erase(value.find_last_not_of(" \t\r\n") + 1);
        if (key == "CONF_THRE")       params.conf_thresh = std::stof(value);
        else if (key == "NMS_THRE")   params.nms_thresh = std::stof(value);
        else if (key == "TRACK_THRESH") params.track_thresh = std::stof(value);
        else if (key == "MATCH_THRESH") params.match_thresh = std::stof(value);
        else if (key == "MIN_BOX_AREA") params.min_box_area = std::stoi(value);
        else if (key == "FRAME_RATE")   params.frame_rate = std::stoi(value);
        else if (key == "TRACK_BUFFER") params.track_buffer = std::stoi(value);
        else if (key == "TRACK_DEBUG")  params.track_debug = std::stoi(value);
    }
}

// SHM reader thread: read frames from SHM and push to camera queue
void shm_reader_thread(int camera_id) {
    auto shm_reader = std::make_shared<ehawkeye::modules::units::shmmem>(shm_names[camera_id], 30, false);
    printf("[cam%d] SHM reader created: %s\n", camera_id, shm_names[camera_id].c_str());

    while (!g_stop) {
        void* data = nullptr;
        int size = 0;
        int ret = shm_reader->nocopyRead((void**)&data, size);
        if (ret < 0 || !data) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        auto* pkt = static_cast<Packet*>(data);

        FrameData fd;
        fd.mat = cv::Mat(pkt->height, pkt->width, CV_8UC3, pkt->data).clone();
        fd.camera_id = camera_id;
        fd.frame_width = pkt->width;
        fd.frame_height = pkt->height;

        {
            std::lock_guard<std::mutex> lock(g_cam_queues[camera_id].mtx);
            if (g_cam_queues[camera_id].q.size() >= 20) {
                g_cam_queues[camera_id].q.pop();
            }
            g_cam_queues[camera_id].q.push(std::move(fd));
        }
        g_cam_queues[camera_id].cv.notify_one();
    }
}

// Preprocess + Forward thread: collect batch from camera queues, infer, push to post queue
void infer_thread() {
    printf("[infer] thread started\n");
    int round_start = 0;

    while (!g_stop) {
        // Round-robin: start from where we left off to ensure fairness
        std::vector<FrameData> batch_frames;
        std::vector<int> batch_camera_ids;
        for (int i = 0; i < NUM_CAMERAS && (int)batch_frames.size() < BATCH_SIZE; i++) {
            int c = (round_start + i) % NUM_CAMERAS;
            FrameData fd;
            {
                std::unique_lock<std::mutex> lock(g_cam_queues[c].mtx);
                if (!g_cam_queues[c].q.empty()) {
                    fd = std::move(g_cam_queues[c].q.front());
                    g_cam_queues[c].q.pop();
                } else {
                    continue;
                }
            }
            fd.camera_id = c;
            batch_frames.push_back(std::move(fd));
            batch_camera_ids.push_back(c);
        }
        if (!batch_frames.empty()) {
            round_start = (batch_camera_ids.back() + 1) % NUM_CAMERAS;
        }

        if (batch_frames.empty()) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }

        int n = batch_frames.size();
        // Convert cv::Mat to bm_image
        std::vector<bm_image> batch_imgs(n);
        for (int i = 0; i < n; i++) {
            cv::bmcv::toBMI(batch_frames[i].mat, &batch_imgs[i]);
        }

        // Preprocess + Forward
        bm_tensor_t input_tensor;
        std::vector<bm_tensor_t> output_tensors;
        output_tensors.resize(g_detector.getOutputNum());
        std::vector<std::pair<int, int>> txy_batch;
        std::vector<std::pair<float, float>> ratios_batch;

        auto t0 = now_us();
        int ret = g_detector.pre_process(batch_imgs, input_tensor, txy_batch, ratios_batch);
        if (ret != 0) {
            printf("[infer] pre_process failed\n");
            for (auto& img : batch_imgs) bm_image_destroy(img);
            continue;
        }
        auto t1 = now_us();

        ret = g_detector.forward(input_tensor, output_tensors);
        if (ret != 0) {
            printf("[infer] forward failed\n");
            for (auto& img : batch_imgs) bm_image_destroy(img);
            continue;
        }
        auto t2 = now_us();

        printf("[infer] n=%d pre=%ldus fwd=%ldus\n", n, t1 - t0, t2 - t1);

        // Push to post queue
        InferResult ir;
        ir.input_images = std::move(batch_imgs);
        ir.output_tensors = std::move(output_tensors);
        ir.txy_batch = std::move(txy_batch);
        ir.ratios_batch = std::move(ratios_batch);
        ir.camera_ids = batch_camera_ids;
        ir.frames.resize(n);
        for (int i = 0; i < n; i++) {
            ir.frames[i] = batch_frames[i].mat.clone();
        }
        ir.batch_size = n;

        {
            std::lock_guard<std::mutex> lock(g_post_queue.mtx);
            g_post_queue.q.push(std::move(ir));
        }
        g_post_queue.cv.notify_one();
    }
}

// Post-process thread (1): take results from post queue, do post_process, dispatch to per-camera track queues
void post_thread() {
    printf("[post] thread started\n");

    while (!g_stop) {
        InferResult ir;
        {
            std::unique_lock<std::mutex> lock(g_post_queue.mtx);
            if (g_post_queue.q.empty()) {
                g_post_queue.cv.wait_for(lock, std::chrono::milliseconds(10));
                continue;
            }
            ir = std::move(g_post_queue.q.front());
            g_post_queue.q.pop();
        }

        std::vector<YoloV8BoxVec> boxes;
        auto t0 = now_us();
        int ret = g_detector.post_process(ir.input_images, ir.output_tensors,
                                                ir.txy_batch, ir.ratios_batch, boxes);
        auto t1 = now_us();
        if (ret != 0) {
            printf("[post] post_process failed\n");
            for (auto& img : ir.input_images) bm_image_destroy(img);
            continue;
        }

        int total_boxes = 0;
        for (int i = 0; i < ir.batch_size; i++) {
            int cam_id = ir.camera_ids[i];
            std::vector<DetectorRetData> det_results;
            for (auto& box : boxes[i]) {
                DetectorRetData d;
                d.label = box.class_id;
                d.confidence = box.score;
                d.xmin = (int)box.x1;
                d.ymin = (int)box.y1;
                d.xmax = (int)box.x2;
                d.ymax = (int)box.y2;
                det_results.push_back(d);
            }
            total_boxes += det_results.size();

            TrackData td;
            td.frame = ir.frames[i].clone();
            td.frame_width = td.frame.cols;
            td.frame_height = td.frame.rows;
            td.detections = std::move(det_results);

            {
                std::lock_guard<std::mutex> lock(g_track_queues[cam_id].mtx);
                if (g_track_queues[cam_id].q.size() >= 10) {
                    g_track_queues[cam_id].q.pop();
                }
                g_track_queues[cam_id].q.push(std::move(td));
            }
            g_track_queues[cam_id].cv.notify_one();

            g_cam_frame_count[cam_id].fetch_add(1);
        }

        auto t2 = now_us();
        if (total_boxes > 0) {
            printf("[post] batch=%d boxes=%d post=%ldus dispatch=%ldus\n",
                   ir.batch_size, total_boxes, t1 - t0, t2 - t1);
        }

        g_frame_count.fetch_add(ir.batch_size);

        for (auto& img : ir.input_images) {
            bm_image_destroy(img);
        }
    }
}

// ByteTrack thread (8): one per camera, consumes detection results + frame from g_track_queues
void track_thread(int camera_id, std::vector<bool>& img_pub_mask) {
    bool enable_img_pub = img_pub_mask[camera_id];
    printf("[track%d] thread started, img_pub=%d\n", camera_id, (int)enable_img_pub);

    // ROS publishers for this camera
    static std::vector<std::string> directions = {
        "downstream", "downstream", "downstream", "downstream", "downstream", "downstream", "upstream", "downstream"
    };
    static std::vector<std::string> focal_types = {
        "extra1", "extra2", "extra3", "extra4", "extra5", "extra6", "short", "short"
    };
    std::string pole = "k7_855_test";
    std::string dir = directions[camera_id];
    std::string ft = focal_types[camera_id];

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    auto pub_img = enable_img_pub ? it.advertise("/" + pole + "/" + dir + "/" + ft + "_camera/image_detect", 1) : image_transport::Publisher();
    auto pub_tracker = nh.advertise<infer_nodelet::ImageDetectObject>(
        "/" + pole + "/" + dir + "/" + ft + "_camera/image_detect_object", 1);

    bytetrack_params params;
    bytetrack_yaml_parse("/home/files/rvf/share/infer_nodelet/config/bytetrack.yaml", params);
    BYTETracker bytetrack(params, false, false, "", 30, "", "");

    int frame_seq = 0;

    while (!g_stop) {
        TrackData td;
        {
            std::unique_lock<std::mutex> lock(g_track_queues[camera_id].mtx);
            if (g_track_queues[camera_id].q.empty()) {
                g_track_queues[camera_id].cv.wait_for(lock, std::chrono::milliseconds(10));
                continue;
            }
            td = std::move(g_track_queues[camera_id].q.front());
            g_track_queues[camera_id].q.pop();
        }

        auto t0 = now_us();
        std::vector<STrack> output_stracks = bytetrack.update(td.detections);
        auto t1 = now_us();

        // Build ROS message
        infer_nodelet::ImageDetectObject tracker_msg;
        infer_nodelet::ImageDetectObjectSingle single_msg;
        int objects_number = output_stracks.size();

        for (auto& bbox : output_stracks) {
            int x0 = std::max(0, (int)bbox.det_box[0]);
            int y0 = std::max(0, (int)bbox.det_box[1]);
            int w = std::max(0, (int)bbox.det_box[2]);
            int h = std::max(0, (int)bbox.det_box[3]);
            if (w <= 0 || h <= 0) continue;

            single_msg.x_pixel_norm = std::clamp((float)x0 / td.frame_width, 0.0f, 1.0f);
            single_msg.y_pixel_norm = std::clamp((float)y0 / td.frame_height, 0.0f, 1.0f);
            single_msg.w_pixel_norm = std::clamp((float)w / td.frame_width, 0.0f, 1.0f);
            single_msg.h_pixel_norm = std::clamp((float)h / td.frame_height, 0.0f, 1.0f);
            single_msg.target_type = bbox.class_id;
            single_msg.id = bbox.track_id;
            single_msg.color = bbox.vehicle_color;
            single_msg.plate_number = bbox.plate_number;
            single_msg.plate_confid = bbox.plate_confid;
            single_msg.plate_color = bbox.plate_color;
            tracker_msg.objects.push_back(single_msg);
        }

        tracker_msg.header.seq = 1;
        tracker_msg.header.stamp = ros::Time::now();
        tracker_msg.header.frame_id = "image";
        tracker_msg.frame_seq = ++frame_seq;
        tracker_msg.objects_number = objects_number;

        pub_tracker.publish(tracker_msg);

        if (enable_img_pub) {
            // Draw detection boxes (green) and tracking boxes (red) on frame
            cv::Mat draw_frame = td.frame.clone();
            for (auto& bbox : output_stracks) {
                int x0 = std::max(0, (int)bbox.det_box[0]);
                int y0 = std::max(0, (int)bbox.det_box[1]);
                int w = std::max(0, (int)bbox.det_box[2]);
                int h = std::max(0, (int)bbox.det_box[3]);
                if (w <= 0 || h <= 0) continue;
                cv::rectangle(draw_frame, cv::Rect(x0, y0, w, h), cv::Scalar(0, 0, 255), 2);
                cv::putText(draw_frame, std::to_string(bbox.track_id) + "_" + std::to_string(bbox.class_id),
                            cv::Point(x0, y0 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            }

            cv_bridge::CvImage brigeImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_frame);
            pub_img.publish(brigeImg.toImageMsg());
        }

        ros::spinOnce();

        auto t2 = now_us();

        if (output_stracks.size() > 0) {
            printf("[track%d] tracked=%zu track=%ldus pub=%ldus\n",
                   camera_id, output_stracks.size(), t1 - t0, t2 - t1);
        }

        g_cam_tracked_count[camera_id].fetch_add(1);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_pipeline");

    std::string bmodel = "/home/files/rvf/share/infer_nodelet/model/kezhu-det-4b.bmodel";
    if (argc > 1) bmodel = argv[1];

    g_detector.Init(bmodel);
    printf("batch_size=%d\n", g_detector.batch_size);

    // Start SHM reader threads
    std::vector<std::thread> shm_threads;
    for (int i = 0; i < NUM_CAMERAS; i++) {
        shm_threads.emplace_back(shm_reader_thread, i);
    }

    // Start infer thread
    std::thread infer_th(infer_thread);

    // Start post-process thread (1)
    std::thread post_th(post_thread);

    // Start ByteTrack threads (one per camera)
    std::vector<bool> img_pub_mask(NUM_CAMERAS, false);
    bool enable_img_pub = true;
    const char* env = getenv("ENABLE_IMG_PUB");
    if (env) {
        std::string s(env);
        if (s == "none") {
            enable_img_pub = false;
        } else if (s == "all") {
            for (int i = 0; i < NUM_CAMERAS; i++) img_pub_mask[i] = true;
        } else {
            // comma-separated camera ids: "0,2,5"
            std::stringstream ss(s);
            std::string token;
            while (std::getline(ss, token, ',')) {
                int id = std::stoi(token);
                if (id >= 0 && id < NUM_CAMERAS) img_pub_mask[id] = true;
            }
        }
    } else {
        for (int i = 0; i < NUM_CAMERAS; i++) img_pub_mask[i] = true;
    }
    printf("ENABLE_IMG_PUB=%s", env ? env : "all");
    for (int i = 0; i < NUM_CAMERAS; i++) printf(" cam%d=%d", i, (int)img_pub_mask[i]);
    printf("\n");

    std::vector<std::thread> track_threads;
    for (int i = 0; i < NUM_CAMERAS; i++) {
        track_threads.emplace_back(track_thread, i, std::ref(img_pub_mask));
    }

    // Monitor and print per-camera fps every 5 seconds
    auto last_print = now_us();
    int last_count = 0;
    int last_cam_count[NUM_CAMERAS] = {0};
    int last_cam_tracked[NUM_CAMERAS] = {0};
for (int loop = 0; !g_stop; loop++) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        auto now = now_us();
        int count = g_frame_count.load();
        double fps = (count - last_count) / 5.0;
        printf("[stats] total_fps=%.1f", fps);
        for (int i = 0; i < NUM_CAMERAS; i++) {
            int cc = g_cam_frame_count[i].load();
            double cfps = (cc - last_cam_count[i]) / 5.0;
            int tc = g_cam_tracked_count[i].load();
            printf(" cam%d=%.1f(track=%.1f)", i, cfps, tc > 0 ? (tc - last_cam_tracked[i]) / 5.0 : 0.0);
            last_cam_count[i] = cc;
            last_cam_tracked[i] = tc;
        }
        printf("\n");
        last_print = now;
        last_count = count;
    }

    g_stop = true;
    for (auto& t : shm_threads) if (t.joinable()) t.join();
    infer_th.join();
    post_th.join();
    for (auto& t : track_threads) if (t.joinable()) t.join();

    return 0;
}