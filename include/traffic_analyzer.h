#ifndef TRAFFIC_ANALYZER_H
#define TRAFFIC_ANALYZER_H

#include <string>
#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "traffic_template.h"

class TrafficAnalyzer {
public:
    TrafficAnalyzer();
    ~TrafficAnalyzer();

    bool enabled() const { return enabled_; }
    void setEnabled(bool e) { enabled_ = e; }

    void setConfig(const std::string& templates_path,
                   const std::string& events_path,
                   int img_width, int img_height,
                   int stationary_frames = 30,
                   float stationary_threshold = 0.005f);

    void learnTemplates(const std::string& tracks_dir,
                        const std::string& camera_type,
                        const std::string& camera_direction);

    void saveTemplates(const std::string& camera_type,
                       const std::string& camera_direction);

    void loadTemplates(const std::string& camera_type,
                       const std::string& camera_direction);

    TrafficEventType analyzeTrajectory(int track_id, int class_id,
                           const std::vector<TrackPoint>& points,
                           const std::string& camera_type,
                           const std::string& camera_direction,
                           const cv::Mat& frame);

    void saveEvent(const TrafficEvent& event, const cv::Mat& frame);

private:
    Trajectory parseTrackFile(const std::string& filepath,
                              const std::string& camera_type,
                              const std::string& camera_direction);

    void clusterLanes(std::vector<Trajectory>& trajectories,
                      std::vector<LaneTemplate>& lanes);

    void computeDirection(const std::vector<TrackPoint>& points,
                          float& dir_x, float& dir_y);

    float computeSpeed(const std::vector<TrackPoint>& points);

    int findBestLane(const std::vector<TrackPoint>& points);
    bool isWrongWay(const std::vector<TrackPoint>& points, int lane_id);
    bool isStationary(const std::vector<TrackPoint>& points);


    std::string templates_path_;
    std::string events_path_;
    std::string camera_type_;
    std::string camera_direction_;
    int img_width_ = 1920;
    int img_height_ = 1080;
    int stationary_frames_ = 30;
    float stationary_threshold_ = 0.005f;

    bool enabled_ = false;
    std::vector<ViewTemplate> view_templates_;
    mutable std::mutex mtx_;
};

#endif
