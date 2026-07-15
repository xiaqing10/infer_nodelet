#ifndef TRAFFIC_TEMPLATE_H
#define TRAFFIC_TEMPLATE_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

struct TrackPoint {
    float x;
    float y;
    int class_id;
    int frame_id;
};

struct Trajectory {
    int track_id;
    int class_id;
    std::vector<TrackPoint> points;
    std::string camera_type;
    std::string camera_direction;
};

struct LaneTemplate {
    int lane_id;
    float x_min;
    float x_max;
    float direction_x;
    float direction_y;
    float mean_speed;
    float std_speed;
    std::vector<float> mean_path_x;
    std::vector<float> mean_path_y;
    int sample_count;
};

struct ViewTemplate {
    std::string camera_type;
    std::string camera_direction;
    std::vector<LaneTemplate> lanes;
};

enum class TrafficEventType {
    NONE = 0,
    WRONG_WAY = 1,
    STATIONARY = 2,
};

struct TrafficEvent {
    TrafficEventType type;
    int track_id;
    int class_id;
    int lane_id;
    std::string camera_type;
    std::string camera_direction;
    long timestamp;
    std::vector<TrackPoint> points;
};

#endif
