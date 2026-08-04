//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-DEMO is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//
#pragma once
#include "strack.h"
#include "detector.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <functional>
#include "unistd.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

struct bytetrack_params {
  // detector:
  float conf_thresh = 0.25f;
  float nms_thresh = 0.45f;
  // tracker:
  float track_thresh = 0.25f;
  float match_thresh = 0.7f;
  int min_box_area = 100;
  int frame_rate = 15;
  int track_buffer = 90;
  int track_debug = 0;
};

using TrackRemovedCallback = std::function<std::string(int track_id, int class_id,
                                                const std::vector<std::vector<float>>& track_points)>;

using SaveFrameCallback = std::function<void(int track_id, int class_id,
                                             const std::vector<std::vector<float>>& track_points,
                                             const std::string& save_dir,
                                             const std::string& filename_prefix)>;

class BYTETracker {
 public:
  BYTETracker(const bytetrack_params& params, bool write_flag, std::string write_path,int min_points_len,  std::string camera_type, std::string camera_direction);
  ~BYTETracker();

  vector<STrack> update(const vector<DetectorRetData>& objects);
#if USE_SOPHON
  TimeStamp* m_ts;
  void enableProfile(TimeStamp* ts);
#endif
  int updateVehicleColor(int xxTracker_id, int vehicle_color, float score);

  void setTrackRemovedCallback(TrackRemovedCallback cb) { track_removed_cb_ = cb; }
  void setSaveFrameCallback(SaveFrameCallback cb) { save_frame_cb_ = cb; }
  int getFrameId() const { return frame_id; }

 private:
  vector<STrack*> joint_stracks(vector<STrack*>& tlista,
                                vector<STrack>& tlistb);
  vector<STrack> joint_stracks(vector<STrack>& tlista, vector<STrack>& tlistb);

  vector<STrack> sub_stracks(vector<STrack>& tlista, vector<STrack>& tlistb);
  void remove_duplicate_stracks(vector<STrack>& resa, vector<STrack>& resb,
                                vector<STrack>& stracksa,
                                vector<STrack>& stracksb);

  void linear_assignment(vector<vector<float>>& cost_matrix,
                         int cost_matrix_size, int cost_matrix_size_size,
                         float thresh, vector<vector<int>>& matches,
                         vector<int>& unmatched_a, vector<int>& unmatched_b);
  vector<vector<float>> iou_distance(vector<STrack*>& atracks,
                                     vector<STrack>& btracks, int& dist_size,
                                     int& dist_size_size);
  vector<vector<float>> iou_distance(vector<STrack>& atracks,
                                     vector<STrack>& btracks);
  vector<vector<float>> ious(vector<vector<float>>& atlbrs,
                             vector<vector<float>>& btlbrs);

  double lapjv(const vector<vector<float>>& cost, vector<int>& rowsol,
               vector<int>& colsol, bool extend_cost = false,
               float cost_limit = LONG_MAX, bool return_cost = true);

 private:
  float track_thresh;
  float match_thresh;
  float high_thresh;
  int frame_rate;
  int track_buffer;
  int min_box_area;
  int frame_id;
  int max_time_lost;

  vector<STrack> tracked_stracks;
  vector<STrack> lost_stracks;
  vector<STrack> removed_stracks;
  KalmanFilter kalman_filter;

  int global_tracker_id = 0;
  bool write_flag = true;
  std::string write_path = "/home/files/nfsroot/";
  std::string camera_type = "";
  std::string camera_direction = "";
  std::string log_prefix = "[TRACK]";  // 日志前缀，在构造函数中拼接 camera_direction
  bool debug_enabled = false;
  int last_clean_day_ = -1;
  int min_points_len = 30;

  TrackRemovedCallback track_removed_cb_ = nullptr;
  SaveFrameCallback save_frame_cb_ = nullptr;
};
