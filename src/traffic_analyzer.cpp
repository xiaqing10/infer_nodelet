#include "traffic_analyzer.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <dirent.h>
#include <sys/stat.h>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <functional>
#include <jsoncpp/json/json.h>

TrafficAnalyzer::TrafficAnalyzer() {}
TrafficAnalyzer::~TrafficAnalyzer() {}

void TrafficAnalyzer::setConfig(const std::string& templates_path,
                                const std::string& events_path,
                                int img_width, int img_height,
                                int stationary_frames,
                                float stationary_threshold) {
    templates_path_ = templates_path;
    events_path_ = events_path;
    img_width_ = img_width;
    img_height_ = img_height;
    stationary_frames_ = stationary_frames;
    stationary_threshold_ = stationary_threshold;
}

Trajectory TrafficAnalyzer::parseTrackFile(const std::string& filepath,
                                           const std::string& camera_type,
                                           const std::string& camera_direction) {
    Trajectory traj;
    traj.camera_type = camera_type;
    traj.camera_direction = camera_direction;

    std::ifstream ifs(filepath);
    if (!ifs.is_open()) return traj;

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.find("id:") == 0) {
            std::stringstream ss(line);
            std::string token;
            while (ss >> token) {
                if (token == "id:") ss >> traj.track_id;
                else if (token == "class:") ss >> traj.class_id;
            }
        } else if (line.find("frame_") == 0) {
            TrackPoint pt;
            std::stringstream ss(line);
            std::string token;
            std::string frame_str;
            ss >> frame_str;
            std::string frame_id_str = frame_str.substr(6, frame_str.size() - 7);
            pt.frame_id = std::stoi(frame_id_str);
            ss >> token; ss >> pt.x;
            ss >> token; ss >> pt.y;
            ss >> token; ss >> pt.class_id;
            pt.x /= img_width_;
            pt.y /= img_height_;
            traj.points.push_back(pt);
        }
    }
    return traj;
}

void TrafficAnalyzer::computeDirection(const std::vector<TrackPoint>& points,
                                       float& dir_x, float& dir_y) {
    dir_x = 0; dir_y = 0;
    if (points.size() < 2) return;
    dir_x = points.back().x - points.front().x;
    dir_y = points.back().y - points.front().y;
    float len = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    if (len > 0) { dir_x /= len; dir_y /= len; }
}

float TrafficAnalyzer::computeSpeed(const std::vector<TrackPoint>& points) {
    if (points.size() < 2) return 0;
    float total_dist = 0;
    for (size_t i = 1; i < points.size(); i++) {
        float dx = points[i].x - points[i-1].x;
        float dy = points[i].y - points[i-1].y;
        total_dist += std::sqrt(dx * dx + dy * dy);
    }
    return total_dist / (points.size() - 1);
}

void TrafficAnalyzer::clusterLanes(std::vector<Trajectory>& trajectories,
                                   std::vector<LaneTemplate>& lanes) {
    if (trajectories.empty()) return;

    const int NUM_LANES = 6;

    // Compute mean_x for each trajectory
    std::vector<std::pair<float, size_t>> x_means;
    for (size_t i = 0; i < trajectories.size(); i++) {
        float sum_x = 0;
        for (auto& p : trajectories[i].points) sum_x += p.x;
        x_means.push_back({sum_x / trajectories[i].points.size(), i});
    }

    // Find actual x range from data
    float data_x_min = 1.0f, data_x_max = 0.0f;
    for (auto& xm : x_means) {
        data_x_min = std::min(data_x_min, xm.first);
        data_x_max = std::max(data_x_max, xm.first);
    }
    if (data_x_max <= data_x_min) { data_x_min = 0; data_x_max = 1; }

    // Assign trajectories to lanes (equal-width bins)
    float bin_width = (data_x_max - data_x_min) / NUM_LANES;
    std::vector<int> cluster_ids(trajectories.size(), -1);
    for (auto& xm : x_means) {
        int lane = (int)((xm.first - data_x_min) / bin_width);
        if (lane < 0) lane = 0;
        if (lane >= NUM_LANES) lane = NUM_LANES - 1;
        cluster_ids[xm.second] = lane;
    }

    lanes.resize(NUM_LANES);

    for (int c = 0; c < NUM_LANES; c++) {
        LaneTemplate& lane = lanes[c];
        lane.lane_id = c;
        lane.sample_count = 0;

        float sum_dir_x = 0, sum_dir_y = 0;
        std::vector<float> all_speeds;
        float x_min = 1.0f, x_max = 0.0f;
        std::vector<std::vector<TrackPoint>> lane_trajs;

        for (size_t i = 0; i < trajectories.size(); i++) {
            if (cluster_ids[i] != c) continue;
            lane.sample_count++;
            float dir_x, dir_y;
            computeDirection(trajectories[i].points, dir_x, dir_y);
            sum_dir_x += dir_x; sum_dir_y += dir_y;
            float speed = computeSpeed(trajectories[i].points);
            all_speeds.push_back(speed);
            for (auto& p : trajectories[i].points) {
                x_min = std::min(x_min, p.x);
                x_max = std::max(x_max, p.x);
            }
            lane_trajs.push_back(trajectories[i].points);
        }

        lane.x_min = data_x_min + c * bin_width;
        lane.x_max = data_x_min + (c + 1) * bin_width;

        float dir_len = std::sqrt(sum_dir_x * sum_dir_x + sum_dir_y * sum_dir_y);
        if (dir_len > 0) { lane.direction_x = sum_dir_x / dir_len; lane.direction_y = sum_dir_y / dir_len; }

        if (!all_speeds.empty()) {
            float sum = 0;
            for (float s : all_speeds) sum += s;
            lane.mean_speed = sum / all_speeds.size();
            float var = 0;
            for (float s : all_speeds) var += (s - lane.mean_speed) * (s - lane.mean_speed);
            lane.std_speed = std::sqrt(var / all_speeds.size());
        }

        if (!lane_trajs.empty()) {
            const int SAMPLE_PTS = 20;
            lane.mean_path_x.resize(SAMPLE_PTS, 0);
            lane.mean_path_y.resize(SAMPLE_PTS, 0);
            std::vector<int> counts(SAMPLE_PTS, 0);

            for (auto& t : lane_trajs) {
                for (int si = 0; si < SAMPLE_PTS; si++) {
                    float frac = (float)si / (SAMPLE_PTS - 1);
                    float idx_f = frac * (t.size() - 1);
                    int idx = (int)idx_f;
                    float rem = idx_f - idx;
                    if (idx + 1 < (int)t.size()) {
                        lane.mean_path_x[si] += t[idx].x * (1 - rem) + t[idx + 1].x * rem;
                        lane.mean_path_y[si] += t[idx].y * (1 - rem) + t[idx + 1].y * rem;
                    } else {
                        lane.mean_path_x[si] += t[idx].x;
                        lane.mean_path_y[si] += t[idx].y;
                    }
                    counts[si]++;
                }
            }
            for (int si = 0; si < SAMPLE_PTS; si++) {
                if (counts[si] > 0) { lane.mean_path_x[si] /= counts[si]; lane.mean_path_y[si] /= counts[si]; }
            }
        }
    }
}

void TrafficAnalyzer::learnTemplates(const std::string& tracks_dir,
                                     const std::string& camera_type,
                                     const std::string& camera_direction) {
    // Recursively collect all .txt files under tracks_dir
    std::vector<std::string> txt_files;
    std::function<void(const std::string&)> collect_files = [&](const std::string& path) {
        DIR* d = opendir(path.c_str());
        if (!d) return;
        struct dirent* entry;
        while ((entry = readdir(d)) != nullptr) {
            std::string name(entry->d_name);
            if (name == "." || name == "..") continue;
            std::string full = path + "/" + name;
            if (entry->d_type == DT_DIR) {
                collect_files(full);
            } else if (entry->d_type == DT_REG && name.size() > 4 && name.substr(name.size() - 4) == ".txt") {
                txt_files.push_back(full);
            }
        }
        closedir(d);
    };
    collect_files(tracks_dir);

    if (txt_files.empty()) {
        std::cerr << "[TrafficAnalyzer] No valid trajectories in " << tracks_dir << std::endl;
        return;
    }

    std::vector<Trajectory> trajectories;
    for (auto& filepath : txt_files) {
        Trajectory traj = parseTrackFile(filepath, camera_type, camera_direction);
        if (traj.points.size() >= 10) trajectories.push_back(traj);
    }

    if (trajectories.empty()) {
        std::cerr << "[TrafficAnalyzer] No valid trajectories in " << tracks_dir << std::endl;
        return;
    }

    std::cout << "[TrafficAnalyzer] Loaded " << trajectories.size()
              << " trajectories for " << camera_type << "_" << camera_direction << std::endl;

    std::vector<LaneTemplate> lanes;
    clusterLanes(trajectories, lanes);

    std::lock_guard<std::mutex> lock(mtx_);
    bool found = false;
    for (auto& vt : view_templates_) {
        if (vt.camera_type == camera_type && vt.camera_direction == camera_direction) {
            vt.lanes = lanes; found = true; break;
        }
    }
    if (!found) {
        ViewTemplate vt;
        vt.camera_type = camera_type; vt.camera_direction = camera_direction;
        vt.lanes = lanes;
        view_templates_.push_back(vt);
    }

    std::cout << "[TrafficAnalyzer] Learned " << lanes.size()
              << " lanes for " << camera_type << "_" << camera_direction << std::endl;
}

void TrafficAnalyzer::saveTemplates(const std::string& camera_type,
                                    const std::string& camera_direction) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& vt : view_templates_) {
        if (vt.camera_type == camera_type && vt.camera_direction == camera_direction) {
            Json::Value root;
            root["camera_type"] = camera_type;
            root["camera_direction"] = camera_direction;
            root["num_lanes"] = (int)vt.lanes.size();

            for (size_t i = 0; i < vt.lanes.size(); i++) {
                Json::Value lane_json;
                lane_json["lane_id"] = vt.lanes[i].lane_id;
                lane_json["sample_count"] = vt.lanes[i].sample_count;
                lane_json["direction_x"] = vt.lanes[i].direction_x;
                lane_json["direction_y"] = vt.lanes[i].direction_y;
                lane_json["mean_speed"] = vt.lanes[i].mean_speed;
                lane_json["std_speed"] = vt.lanes[i].std_speed;
                lane_json["x_min"] = vt.lanes[i].x_min;
                lane_json["x_max"] = vt.lanes[i].x_max;

                Json::Value path_x(Json::arrayValue);
                Json::Value path_y(Json::arrayValue);
                for (auto v : vt.lanes[i].mean_path_x) path_x.append(v);
                for (auto v : vt.lanes[i].mean_path_y) path_y.append(v);
                lane_json["mean_path_x"] = path_x;
                lane_json["mean_path_y"] = path_y;

                root["lanes"].append(lane_json);
            }

            std::string filename = templates_path_ + "/" + camera_type + "_" + camera_direction + ".json";
            std::ofstream ofs(filename);
            if (ofs.is_open()) {
                ofs << root.toStyledString();
                ofs.close();
                std::cout << "[TrafficAnalyzer] Saved templates to " << filename << std::endl;
            }
            break;
        }
    }
}

void TrafficAnalyzer::loadTemplates(const std::string& camera_type,
                                    const std::string& camera_direction) {
    std::string filename = templates_path_ + "/" + camera_type + "_" + camera_direction + ".json";
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "[TrafficAnalyzer] No template file: " << filename << std::endl;
        return;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    if (!Json::parseFromStream(builder, ifs, &root, &errs)) {
        std::cerr << "[TrafficAnalyzer] Failed to parse " << filename << ": " << errs << std::endl;
        return;
    }

    ViewTemplate vt;
    vt.camera_type = root["camera_type"].asString();
    vt.camera_direction = root["camera_direction"].asString();

    Json::Value lanes_json = root["lanes"];
    for (auto& lane_json : lanes_json) {
        LaneTemplate lane;
        lane.lane_id = lane_json["lane_id"].asInt();
        lane.sample_count = lane_json["sample_count"].asInt();
        lane.direction_x = lane_json["direction_x"].asFloat();
        lane.direction_y = lane_json["direction_y"].asFloat();
        lane.mean_speed = lane_json["mean_speed"].asFloat();
        lane.std_speed = lane_json["std_speed"].asFloat();
        lane.x_min = lane_json["x_min"].asFloat();
        lane.x_max = lane_json["x_max"].asFloat();

        for (auto& v : lane_json["mean_path_x"]) lane.mean_path_x.push_back(v.asFloat());
        for (auto& v : lane_json["mean_path_y"]) lane.mean_path_y.push_back(v.asFloat());

        vt.lanes.push_back(lane);
    }

    std::lock_guard<std::mutex> lock(mtx_);
    bool found = false;
    for (auto& existing : view_templates_) {
        if (existing.camera_type == camera_type && existing.camera_direction == camera_direction) {
            existing = vt; found = true; break;
        }
    }
    if (!found) view_templates_.push_back(vt);

    std::cout << "[TrafficAnalyzer] Loaded " << vt.lanes.size()
              << " lanes from " << filename << std::endl;
}

int TrafficAnalyzer::findBestLane(const std::vector<TrackPoint>& points) {
    float mean_x = 0;
    for (auto& p : points) mean_x += p.x;
    mean_x /= points.size();
    if (!std::isfinite(mean_x)) {
        return -1;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& vt : view_templates_) {
        if (vt.camera_type == camera_type_ && vt.camera_direction == camera_direction_) {

            float best_score = -1e9;
            int best_lane = -1;
            for (size_t i = 0; i < vt.lanes.size(); i++) {
                auto& lane = vt.lanes[i];

                if (mean_x < lane.x_min || mean_x > lane.x_max) {
                    if (i == 0 && (std::isfinite(mean_x))) {
                        std::cout << "[TrafficAnalyzer] findBestLane: mean_x=" << mean_x
                                  << " (first lane x_min=" << lane.x_min << " x_max=" << lane.x_max << ")" << std::endl;
                    }
                    continue;
                }

                float dir_x, dir_y;
                computeDirection(points, dir_x, dir_y);
                float dot = dir_x * lane.direction_x + dir_y * lane.direction_y;
                float speed = computeSpeed(points);

                float dir_score = dot;
                float speed_score = 1.0f;
                if (lane.std_speed > 0.001f) {
                    float z = std::abs(speed - lane.mean_speed) / lane.std_speed;
                    speed_score = std::exp(-z * z / 2.0f);
                }

                float path_score = 0;
                if (!lane.mean_path_x.empty() && points.size() >= 5) {
                    const int SAMPLE_PTS = 20;
                    float sum_dist = 0;
                    int count = 0;
                    for (int si = 0; si < SAMPLE_PTS; si++) {
                        float frac = (float)si / (SAMPLE_PTS - 1);
                        float idx_f = frac * (points.size() - 1);
                        int idx = (int)idx_f;
                        float rem = idx_f - idx;
                        float px, py;
                        if (idx + 1 < (int)points.size()) {
                            px = points[idx].x * (1 - rem) + points[idx + 1].x * rem;
                            py = points[idx].y * (1 - rem) + points[idx + 1].y * rem;
                        } else {
                            px = points[idx].x; py = points[idx].y;
                        }
                        float dx = px - lane.mean_path_x[si];
                        float dy = py - lane.mean_path_y[si];
                        sum_dist += std::sqrt(dx * dx + dy * dy);
                        count++;
                    }
                    if (count > 0) path_score = -sum_dist / count;
                }

                float score = dir_score * 0.5f + speed_score * 0.2f + path_score * 0.3f;
                if (score > best_score) { best_score = score; best_lane = (int)i; }
            }
            std::cout << "[TrafficAnalyzer] findBestLane: best_lane=" << best_lane << " best_score=" << best_score << std::endl;
            return best_lane;
        }
    }
    std::cout << "[TrafficAnalyzer] findBestLane: no matching view for " << camera_type_ << "/" << camera_direction_
              << " (templates count=" << view_templates_.size() << ")" << std::endl;
    return -1;
}

bool TrafficAnalyzer::isWrongWay(const std::vector<TrackPoint>& points, int lane_id) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& vt : view_templates_) {
        if (vt.camera_type != camera_type_ || vt.camera_direction != camera_direction_) continue;
        if (lane_id < 0 || lane_id >= (int)vt.lanes.size()) return false;

        float dir_x, dir_y;
        computeDirection(points, dir_x, dir_y);
        float dot = dir_x * vt.lanes[lane_id].direction_x + dir_y * vt.lanes[lane_id].direction_y;
        return dot < -0.3f;
    }
    return false;
}

bool TrafficAnalyzer::isStationary(const std::vector<TrackPoint>& points) {
    if (points.size() < (size_t)stationary_frames_) return false;
    float total_dist = 0;
    size_t start = points.size() - stationary_frames_;
    for (size_t i = start + 1; i < points.size(); i++) {
        float dx = points[i].x - points[i-1].x;
        float dy = points[i].y - points[i-1].y;
        total_dist += std::sqrt(dx * dx + dy * dy);
    }
    return total_dist < stationary_threshold_;
}

TrafficEventType TrafficAnalyzer::analyzeTrajectory(int track_id, int class_id,
                           const std::vector<TrackPoint>& points,
                           const std::string& camera_type,
                           const std::string& camera_direction,
                           const cv::Mat& frame) {
    if (!enabled_) return TrafficEventType::NONE;
    camera_type_ = camera_type;
    camera_direction_ = camera_direction;

    int lane_id = findBestLane(points);
    if (lane_id < 0) {
        std::cout << "[TrafficAnalyzer] track_id=" << track_id << " class=" << class_id
                  << " camera=" << camera_type << "/" << camera_direction
                  << " points=" << points.size()
                  << " NO LANE MATCHED (templates=" << view_templates_.size() << ")" << std::endl;
        return TrafficEventType::NONE;
    }

    TrafficEvent event;
    event.track_id = track_id;
    event.class_id = class_id;
    event.camera_type = camera_type;
    event.camera_direction = camera_direction;
    event.lane_id = lane_id;
    event.timestamp = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    event.points = points;

    if (isWrongWay(points, lane_id)) {
        event.type = TrafficEventType::WRONG_WAY;
        std::cout << "[TrafficAnalyzer] track_id=" << track_id << " DETECTED: wrong_way lane=" << lane_id << std::endl;
    } else if (isStationary(points)) {
        event.type = TrafficEventType::STATIONARY;
        std::cout << "[TrafficAnalyzer] track_id=" << track_id << " DETECTED: stationary lane=" << lane_id << std::endl;
    } else {
        std::cout << "[TrafficAnalyzer] track_id=" << track_id << " normal (lane=" << lane_id << " no event)" << std::endl;
        return TrafficEventType::NONE;
    }

    saveEvent(event, frame);
    return event.type;
}

void TrafficAnalyzer::saveEvent(const TrafficEvent& event, const cv::Mat& frame) {
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    struct tm tm_buf;
    localtime_r(&tt, &tm_buf);
    std::stringstream ss;
    ss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    std::string time_str = ss.str();

    std::string event_id_str = event.camera_type + "_" + event.camera_direction
        + "_" + time_str + "_id" + std::to_string(event.track_id);
    std::string event_dir = events_path_ + "/" + event_id_str;
    mkdir(event_dir.c_str(), 0755);

    // Save JSON
    Json::Value root;
    root["event_id"] = event_id_str;
    root["track_id"] = event.track_id;
    root["class_id"] = event.class_id;
    root["camera_type"] = event.camera_type;
    root["camera_direction"] = event.camera_direction;
    root["lane_id"] = event.lane_id;
    root["timestamp"] = (Json::Value::Int64)event.timestamp;
    switch (event.type) {
        case TrafficEventType::WRONG_WAY: root["event_type"] = "wrong_way"; break;
        case TrafficEventType::STATIONARY: root["event_type"] = "stationary"; break;

        default: root["event_type"] = "unknown"; break;
    }

    Json::Value pts_json(Json::arrayValue);
    for (auto& p : event.points) {
        Json::Value pt;
        pt["frame"] = p.frame_id;
        pt["x"] = p.x;
        pt["y"] = p.y;
        pt["class"] = p.class_id;
        pts_json.append(pt);
    }
    root["trajectory"] = pts_json;

    std::string json_path = event_dir + "/event.json";
    std::ofstream ofs(json_path);
    if (ofs.is_open()) {
        ofs << root.toStyledString();
        ofs.close();
    }

    // Save JPG with trajectory drawn
    if (!frame.empty()) {
        cv::Mat vis = frame.clone();
        for (size_t i = 1; i < event.points.size(); i++) {
            cv::Point p1(event.points[i-1].x * img_width_, event.points[i-1].y * img_height_);
            cv::Point p2(event.points[i].x * img_width_, event.points[i].y * img_height_);
            cv::Scalar color;
            switch (event.type) {
                case TrafficEventType::WRONG_WAY: color = cv::Scalar(0, 0, 255); break;
                case TrafficEventType::STATIONARY: color = cv::Scalar(0, 255, 255); break;

                default: color = cv::Scalar(0, 255, 0); break;
            }
            cv::line(vis, p1, p2, color, 2);
        }
        std::string img_path = event_dir + "/trajectory.jpg";
        cv::imwrite(img_path, vis);
    }

    std::cout << "[TrafficAnalyzer] Event saved: " << event_id_str
              << " type=" << root["event_type"].asString() << std::endl;
}
