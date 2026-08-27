#include <ros/ros.h>
//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-DEMO is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//
#include "bytetrack.h"
#include <dirent.h>
#include <cstring>
#include <fstream>
#include <ftw.h>
#include <cerrno>

std::string g_track_log_prefix = "[TRACK]";

BYTETracker::BYTETracker(const bytetrack_params& params, bool write_flag_, bool save_img_flag_, std::string write_path_, int min_points_len_,  std::string camera_type_, std::string camera_direction_, std::string pole_name_) {
  this->track_thresh = params.track_thresh;
  this->match_thresh = params.match_thresh;
  this->frame_rate = params.frame_rate;
  this->track_buffer = params.track_buffer;
  this->min_box_area = params.min_box_area;
  this->frame_id = 0;
  this->max_time_lost = int(this->frame_rate / 20.0 * this->track_buffer);

  this->write_flag = write_flag_;
  this->save_img_flag = save_img_flag_;
  this->write_path = write_path_;
  this->camera_type = camera_type_;
  this->camera_direction = camera_direction_;
  this->pole_name = pole_name_;
  g_track_log_prefix = "[TRACK:" + camera_type_ + ":" + camera_direction_ + "]";
  set_track_log_prefix(g_track_log_prefix);
  this->log_prefix = g_track_log_prefix;
  this->debug_enabled = (params.track_debug != 0);
  set_track_debug(this->debug_enabled);
  this->min_points_len = min_points_len_;
  cout << "Init tracker!" << endl;
}

int BYTETracker::updateVehicleColor(int xxTracker_id, int vehicle_color, float score) {
    for (size_t i = 0; i < this->tracked_stracks.size(); i++) {
        if (this->tracked_stracks[i].track_id == xxTracker_id) {
            if (score > 0.95)
                this->tracked_stracks[i].color_hits[vehicle_color] += 3;
            else
                this->tracked_stracks[i].color_hits[vehicle_color] += 1;

            auto _i = max_element(this->tracked_stracks[i].color_hits.begin(),
                                  this->tracked_stracks[i].color_hits.end(),
                                  [](std::pair<char, int> left, std::pair<char, int> right) {
                                      return left.second < right.second;
                                  });
            this->tracked_stracks[i].vehicle_color = _i->first;
            // 降低命中阈值以配合降低的颜色检测频率：仍保留小规模投票，
            // 不锁定首帧，但比原 >4 更快锁定，避免因采样稀疏导致长期无法锁定。
            if (this->tracked_stracks[i].color_hits[_i->first] > 1) {
                this->tracked_stracks[i].color_lock = true;
            }
            break;
        }
    }
    return 0;
}

#if USE_SOPHON
void BYTETracker::enableProfile(TimeStamp* ts) { m_ts = ts; }
#endif


// 递归创建目录（类似 mkdir -p）
static bool mkdir_p(const std::string& path) {
    if (path.empty()) return false;
    std::string p = path;
    if (p.back() == '/') p.pop_back();
    if (access(p.c_str(), F_OK) == 0) return true;
    size_t pos = p.find_last_of('/');
    if (pos != std::string::npos) {
        if (!mkdir_p(p.substr(0, pos))) return false;
    }
    return mkdir(p.c_str(), S_IRWXU) == 0 || errno == EEXIST;
}

// 清理超过3天的旧轨迹目录
static void clean_old_track_dirs(const std::string& base_path) {
    DIR* dir = opendir(base_path.c_str());
    if (!dir) return;

    auto now = std::chrono::system_clock::now();
    std::time_t now_t = std::chrono::system_clock::to_time_t(now);
    struct tm now_tm = *std::localtime(&now_t);
    const std::time_t FIVE_DAYS = 1 * 24 * 3600;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type != DT_DIR) continue;
        std::string name(entry->d_name);
        if (name == "." || name == "..") continue;

        struct tm tm = {};
        if (strptime(name.c_str(), "%Y-%m-%d", &tm) == nullptr) continue;
        std::time_t dir_t = std::mktime(&tm);

        if (now_t - dir_t > FIVE_DAYS) {
            std::string dir_path = base_path + "/" + name;
            ROS_INFO("CLEAN would remove %s, now_t=%ld dir_t=%ld diff=%ld FIVE_DAYS=%ld", dir_path.c_str(), (long)now_t, (long)dir_t, (long)(now_t - dir_t), (long)FIVE_DAYS);
            int ret = nftw(dir_path.c_str(), [](const char* fpath, const struct stat*, int, struct FTW*) -> int {
                return remove(fpath);
            }, 64, FTW_DEPTH | FTW_PHYS);
            ROS_INFO("CLEAN removed %s, nftw_ret=%d", dir_path.c_str(), ret);
        }
    }
    closedir(dir);
}

BYTETracker::~BYTETracker() {}

vector<STrack> BYTETracker::update(const vector<DetectorRetData>& objects) {
  ////////////////// Step 1: Get detections //////////////////
  this->frame_id++;
  vector<STrack> activated_stracks;
  vector<STrack> refind_stracks;
  vector<STrack> removed_stracks;
  vector<STrack> lost_stracks;
  vector<STrack> detections;
  vector<STrack> detections_low;

  vector<STrack> detections_cp;
  vector<STrack> tracked_stracks_swap;
  vector<STrack> resa, resb;
  vector<STrack> output_stracks;

  vector<STrack*> unconfirmed;
  vector<STrack*> tracked_stracks;

  vector<STrack*> r_tracked_stracks;

  if (objects.size() > 0) {
    for (int i = 0; i < objects.size(); i++) {
      vector<float> tlbr_;
      tlbr_.resize(4);
      tlbr_[0] = objects[i].xmin;
      tlbr_[1] = objects[i].ymin;
      tlbr_[2] = objects[i].xmax ;
      tlbr_[3] = objects[i].ymax;

      float score = objects[i].confidence;
      int class_id = objects[i].label;

      STrack strack(STrack::tlbr_to_tlwh(tlbr_), score, class_id);
      strack.both_roi = objects[i].both_roi;
      strack.from_roi = objects[i].from_roi;
      if (score >= track_thresh) {
        detections.push_back(strack);
      } else {
        detections_low.push_back(strack);
      }
    }
  }

  // 每天清理一次超过5天的旧轨迹目录
  if(this->write_flag){
      clean_old_track_dirs(this->write_path);
  }

  // 1正常的 激活 2 正常的 未激活  3 丢失的 激活 4 删除的 激活 5删除的 未激活
  // 新的目标是 state=0, is_activated=false

  //this->tracked_stracks 保存的是正常激活的tracker和新的tracker(未激活的)

  for (int i = 0; i < this->tracked_stracks.size(); i++) {
    if (!this->tracked_stracks[i].is_activated)
      unconfirmed.push_back(&this->tracked_stracks[i]); //新的轨迹，未激活
    else
      tracked_stracks.push_back(&this->tracked_stracks[i]); //激活的轨迹
  }

  if (debug_enabled) ROS_INFO("%s frame=%d dets=%d high=%d low=%d tracked=%d lost=%d unconfirmed=%d",
           log_prefix.c_str(),
           this->frame_id, (int)objects.size(), (int)detections.size(), (int)detections_low.size(),
           (int)tracked_stracks.size(), (int)this->lost_stracks.size(), (int)unconfirmed.size());

  ////////////////// Step 2: First association, with IoU //////////////////
  vector<STrack*> strack_pool;
  strack_pool = joint_stracks(tracked_stracks, this->lost_stracks);
  STrack::multi_predict(strack_pool, this->kalman_filter);

  vector<vector<float>> dists;
  int dist_size = 0, dist_size_size = 0;
  dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);

  vector<vector<int>> matches;
  vector<int> u_track, u_detection;
  linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches,
                    u_track, u_detection);

  if (debug_enabled) ROS_INFO("%s Stage1 pool=%d dets=%d matched=%d unmatched_track=%d unmatched_det=%d",
           log_prefix.c_str(),
           dist_size, dist_size_size, (int)matches.size(), (int)u_track.size(), (int)u_detection.size());

  for (int i = 0; i < matches.size(); i++) {
    STrack* track = strack_pool[matches[i][0]];
    STrack* det = &detections[matches[i][1]];

    // 位置偏差检查：预测框和检测框中心点偏差过大时拒绝匹配
    if (track->state == TrackState::Tracked) {
      float pred_cx = (track->tlbr[0] + track->tlbr[2]) / 2.0f;
      float pred_cy = (track->tlbr[1] + track->tlbr[3]) / 2.0f;
      float det_cx = (det->tlbr[0] + det->tlbr[2]) / 2.0f;
      float det_cy = (det->tlbr[1] + det->tlbr[3]) / 2.0f;
      float pred_w = track->tlbr[2] - track->tlbr[0];
      float pred_h = track->tlbr[3] - track->tlbr[1];
      float center_dist = std::sqrt((pred_cx - det_cx) * (pred_cx - det_cx) + (pred_cy - det_cy) * (pred_cy - det_cy));
      float max_dim = std::max(pred_w, pred_h);
      if (max_dim > 0 && center_dist > max_dim * 3.0f) {
        if (debug_enabled) ROS_INFO("%s Stage1 REJECT center_dev track_id=%d class=%d center_dist=%.1f max_dim=%.1f frame=%d",
                 log_prefix.c_str(),
                 track->track_id, track->class_id, center_dist, max_dim, this->frame_id);
        u_track.push_back(matches[i][0]);
        u_detection.push_back(matches[i][1]);
        continue;
      }
    }

    // 正常的激活的轨迹，直接用det更新
    if (track->state == TrackState::Tracked) {
      track->update(*det, this->frame_id);
      activated_stracks.push_back(*track);
    } else { //如果是丢失的轨迹，重新激活
      track->re_activate(*det, this->frame_id, false);
      refind_stracks.push_back(*track);
    }

    track->det_box = det->tlwh;
    track->both_roi = det->both_roi;
    track->from_roi = det->from_roi;
    // ========== 修复类别更新逻辑 ==========
    if (!track->class_lock) {
        bool should_update = false;

        if (det->score > 0.7) {
            should_update = true;
        }
        else if ((this->frame_id - track->start_frame) <= 5) {
            should_update = true;
        }
        else if (track->class_hits[track->class_id] < 3) {
            should_update = true;
        }

        if (should_update) {
            int weight = (det->score > 0.8) ? 2 : 1;
            track->class_hits[det->class_id] += weight;

            auto _i = max_element(track->class_hits.begin(), track->class_hits.end(),
                [](std::pair<char, int> left, std::pair<char, int> right) {
                    return left.second < right.second;
                });

            int current_best_count = _i->second;
            int current_class_count = track->class_hits[track->class_id];

            if (_i->first != track->class_id &&
                current_best_count >= current_class_count + 2) {
                track->class_id = _i->first;
            }

            if (track->class_hits[_i->first] > 8) {
                track->class_lock = true;
            }
        }
    }
    // ========== 修复结束 ==========

    if(track->track_points.size() < 500){
        std::vector<float> point_with_class = {track->tlwh[0], track->tlwh[1], track->tlwh[2], track->tlwh[3],
                                              static_cast<float>(det->class_id), static_cast<float>(this->frame_id),
                                              det->score, static_cast<float>(track->class_id)};
        track->track_points.push_back(point_with_class);
    }
    else{
        track->track_points.erase(track->track_points.begin());
        std::vector<float> point_with_class = {track->tlwh[0], track->tlwh[1], track->tlwh[2], track->tlwh[3],
                                              static_cast<float>(det->class_id), static_cast<float>(this->frame_id),
                                              det->score, static_cast<float>(track->class_id)};
        track->track_points.push_back(point_with_class);
    }
  }

  //第二次合并，低分框。 将上一轮没有匹配到的轨迹和低分框进行合并
  ////////////////// Step 3: Second association, using low score dets
  for (int i = 0; i < u_detection.size(); i++) {
    detections_cp.push_back(detections[u_detection[i]]);
  }
  detections.clear();
  detections.assign(detections_low.begin(), detections_low.end());

  for (int i = 0; i < u_track.size(); i++) {
    if (strack_pool[u_track[i]]->state == TrackState::Tracked) {
      r_tracked_stracks.push_back(strack_pool[u_track[i]]);
    }
  }

  dists.clear();
  dists =
      iou_distance(r_tracked_stracks, detections, dist_size, dist_size_size);

  matches.clear();
  u_track.clear();
  u_detection.clear();
  linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track,
                    u_detection);

  if (debug_enabled) ROS_INFO("%s Stage3 low_score tracks=%d dets=%d matched=%d unmatched_track=%d unmatched_det=%d",
           log_prefix.c_str(),
           dist_size, dist_size_size, (int)matches.size(), (int)u_track.size(), (int)u_detection.size());

  for (int i = 0; i < matches.size(); i++) {
    STrack* track = r_tracked_stracks[matches[i][0]];
    STrack* det = &detections[matches[i][1]];

    if (track->state == TrackState::Tracked) {
      track->update(*det, this->frame_id);
      activated_stracks.push_back(*track);
    } else {
      track->re_activate(*det, this->frame_id, false);
      refind_stracks.push_back(*track);
    }

    track->det_box = det->tlwh;
    track->both_roi = det->both_roi;
    track->from_roi = det->from_roi;
    if (!track->class_lock) {
        bool should_update = false;

        if (det->score > 0.7) {
            should_update = true;
        }
        else if ((this->frame_id - track->start_frame) <= 5) {
            should_update = true;
        }
        else if (track->class_hits[track->class_id] < 3) {
            should_update = true;
        }

        if (should_update) {
            int weight = (det->score > 0.8) ? 2 : 1;
            track->class_hits[det->class_id] += weight;

            auto _i = max_element(track->class_hits.begin(), track->class_hits.end(),
                [](std::pair<char, int> left, std::pair<char, int> right) {
                    return left.second < right.second;
                });

            int current_best_count = _i->second;
            int current_class_count = track->class_hits[track->class_id];

            if (_i->first != track->class_id &&
                current_best_count >= current_class_count + 2) {
                track->class_id = _i->first;
            }

            if (track->class_hits[_i->first] > 8) {
                track->class_lock = true;
            }
        }
    }

    if(track->track_points.size() < 500){
        std::vector<float> point_with_class = {track->tlwh[0], track->tlwh[1], track->tlwh[2], track->tlwh[3],
                                              static_cast<float>(det->class_id), static_cast<float>(this->frame_id),
                                              det->score, static_cast<float>(track->class_id)};
        track->track_points.push_back(point_with_class);
    }
    else{
        track->track_points.erase(track->track_points.begin());
        std::vector<float> point_with_class = {track->tlwh[0], track->tlwh[1], track->tlwh[2], track->tlwh[3],
                                              static_cast<float>(det->class_id), static_cast<float>(this->frame_id),
                                              det->score, static_cast<float>(track->class_id)};
        track->track_points.push_back(point_with_class);
    }
  }

  for (int i = 0; i < u_track.size(); i++) {
    STrack* track = r_tracked_stracks[u_track[i]];
    if (track->state != TrackState::Lost) {
      if (detections_low.size() != 0) {
        // 找最佳匹配的低分框
        float best_iou = 0;
        int best_idx = -1;
        for (int j = 0; j < detections.size(); j++) {
          float ix = std::max(track->tlbr[0], detections[j].tlbr[0]);
          float iy = std::max(track->tlbr[1], detections[j].tlbr[1]);
          float ix2 = std::min(track->tlbr[2], detections[j].tlbr[2]);
          float iy2 = std::min(track->tlbr[3], detections[j].tlbr[3]);
          if (ix2 <= ix || iy2 <= iy) continue;
          float iw = ix2 - ix;
          float ih = iy2 - iy;
          float inter = iw * ih;
          float track_area = (track->tlbr[2] - track->tlbr[0]) * (track->tlbr[3] - track->tlbr[1]);
          float det_area = (detections[j].tlbr[2] - detections[j].tlbr[0]) * (detections[j].tlbr[3] - detections[j].tlbr[1]);
          float union_area = track_area + det_area - inter;
          float iou_val = inter / union_area;
          if (iou_val > best_iou) { best_iou = iou_val; best_idx = j; }
        }
      }
      track->mark_lost();
      lost_stracks.push_back(*track);
    }
  }

  // Deal with unconfirmed tracks
  detections.clear();
  detections.assign(detections_cp.begin(), detections_cp.end());

  dists.clear();
  dists = iou_distance(unconfirmed, detections, dist_size, dist_size_size);

  matches.clear();
  vector<int> u_unconfirmed;
  u_detection.clear();
  linear_assignment(dists, dist_size, dist_size_size, 0.7, matches,
                    u_unconfirmed, u_detection);

  if (debug_enabled) ROS_INFO("%s Unconfirmed tracks=%d dets=%d matched=%d unmatched=%d",
           log_prefix.c_str(),
           dist_size, dist_size_size, (int)matches.size(), (int)u_unconfirmed.size());

  for (int i = 0; i < matches.size(); i++) {
    unconfirmed[matches[i][0]]->update(detections[matches[i][1]],
                                       this->frame_id);
    activated_stracks.push_back(*unconfirmed[matches[i][0]]);
  }

  for (int i = 0; i < u_unconfirmed.size(); i++) {
    STrack* track = unconfirmed[u_unconfirmed[i]];
    // 延迟激活窗口内：未匹配时直接移除，不做额外保留
    // （保留会导致 tracked_stracks 和 lost_stracks 重复的问题）
    track->mark_removed();
    removed_stracks.push_back(*track);
  }

  ////////////////// Step 4: Init new stracks //////////////////
  for (int i = 0; i < u_detection.size(); i++) {
    STrack* track = &detections[u_detection[i]];
    if (track->score < this->track_thresh) continue;
    track->activate(this->kalman_filter, this->frame_id, &global_tracker_id);
    activated_stracks.push_back(*track);
  }

  ////////////////// Step 5: Update state //////////////////
  for (int i = 0; i < this->lost_stracks.size(); i++) {
    int lost_frames = this->frame_id - this->lost_stracks[i].end_frame();
    int track_age = this->lost_stracks[i].frame_id - this->lost_stracks[i].start_frame;

    // 对长时间稳定跟踪的轨迹，给予更长的丢失容忍时间
    int effective_max_lost = this->max_time_lost;
    if (track_age > 100) {
      effective_max_lost = std::max(effective_max_lost, (int)(this->max_time_lost * 1.5));
    } else if (track_age > 50) {
      effective_max_lost = std::max(effective_max_lost, (int)(this->max_time_lost * 1.2));
    }

    if (lost_frames > effective_max_lost) {
      if (debug_enabled) ROS_INFO("%s LOST_TIMEOUT track_id=%d class=%d lost_frames=%d max=%d age=%d frame=%d",
               log_prefix.c_str(),
               this->lost_stracks[i].track_id, this->lost_stracks[i].class_id,
               lost_frames, effective_max_lost, track_age, this->frame_id);
      this->lost_stracks[i].mark_removed();
      removed_stracks.push_back(this->lost_stracks[i]);
    }
  }

  for (int i = 0; i < this->tracked_stracks.size(); i++) {
    if (this->tracked_stracks[i].state == TrackState::Tracked) {
      tracked_stracks_swap.push_back(this->tracked_stracks[i]);
    }
  }
  this->tracked_stracks.clear();
  this->tracked_stracks.assign(tracked_stracks_swap.begin(),
                               tracked_stracks_swap.end());

  this->tracked_stracks =
      joint_stracks(this->tracked_stracks, activated_stracks);

  this->tracked_stracks = joint_stracks(this->tracked_stracks, refind_stracks);

  this->lost_stracks = sub_stracks(this->lost_stracks, this->tracked_stracks);
  for (int i = 0; i < lost_stracks.size(); i++) {
    this->lost_stracks.push_back(lost_stracks[i]);
  }
  this->lost_stracks = sub_stracks(this->lost_stracks, removed_stracks);

  remove_duplicate_stracks(resa, resb, this->tracked_stracks,
                           this->lost_stracks);

  this->tracked_stracks.clear();
  this->tracked_stracks.assign(resa.begin(), resa.end());
  this->lost_stracks.clear();
  this->lost_stracks.assign(resb.begin(), resb.end());
  // 对丢失轨迹做一次predict，更新预测框位置
  {
    vector<STrack*> lost_ptrs;
    for (auto& t : this->lost_stracks) lost_ptrs.push_back(&t);
    STrack::multi_predict(lost_ptrs, this->kalman_filter);
    // predict只更新了mean/covariance，需要同步更新tlwh/tlbr
    for (auto& t : this->lost_stracks) {
      t.static_tlwh();
      t.static_tlbr();
    }
  }

  for (int i = 0; i < this->tracked_stracks.size(); i++) {
    if (this->tracked_stracks[i].is_activated &&
        this->tracked_stracks[i].tlwh[2] * this->tracked_stracks[i].tlwh[3] >
            this->min_box_area)
      output_stracks.push_back(this->tracked_stracks[i]);
  }

  for (int i = 0; i < removed_stracks.size(); i++) {
    if(removed_stracks[i].track_points.size() < this->min_points_len) {
      if (debug_enabled) ROS_INFO("%s SAVE_SKIP track_id=%d points=%zu min=%d tracklet=%d frame=%d",
               log_prefix.c_str(),
               removed_stracks[i].track_id, removed_stracks[i].track_points.size(),
               this->min_points_len, removed_stracks[i].tracklet_len, this->frame_id);
      continue;
    }

    if (debug_enabled) ROS_INFO("%s SAVE track_id=%d points=%zu tracklet=%d write_flag=%d frame=%d",
             log_prefix.c_str(),
             removed_stracks[i].track_id, removed_stracks[i].track_points.size(),
             removed_stracks[i].tracklet_len, this->write_flag, this->frame_id);
  if(this->write_flag){
      ofstream ofs;
      auto now = std::chrono::system_clock::now();
      std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::stringstream day, ss;
      struct tm tm_buf;
      localtime_r(&t, &tm_buf);
      day << std::put_time(&tm_buf, "%Y-%m-%d");
      ss << std::put_time(&tm_buf, "%Y%m%d%H%M%S");

      std::stringstream hour_ss;
      hour_ss << std::put_time(&tm_buf, "%H");
      std::string prefix = this->write_path + day.str() + "/tracks/" + hour_ss.str() + "/";
      mkdir_p(prefix);

      std::string time_str = ss.str();
      std::string filename = prefix + this->pole_name + "_" + this->camera_direction + "_" + this->camera_type + "_" + time_str + "_" + to_string(removed_stracks[i].class_id) + "_" + to_string(removed_stracks[i].track_id)+ ".txt";
      ofs.open(filename, ios::app);
            if (ofs.is_open()) {
          ofs << "id: " << removed_stracks[i].track_id << " "
          << "class: " << removed_stracks[i].class_id << " "
          << "score: " << removed_stracks[i].score << " "
          << "t: " << removed_stracks[i].tlwh[0] << " "
          << "l: " << removed_stracks[i].tlwh[1] << " "
          << "w: " << removed_stracks[i].tlwh[2] << " "
          << "h: " << removed_stracks[i].tlwh[3] << " "
          << "duration_frame: " << removed_stracks[i].end_frame() - removed_stracks[i].start_frame << " "
          << "color: " << removed_stracks[i].vehicle_color << " " << endl;
                ofs << "track_points: " << endl;
          for (auto p : removed_stracks[i].track_points) {
            float center_x = p[0] + p[2]/2;
            float center_y = p[1] + p[3]/2;
            int point_class = static_cast<int>(p[4]);
            int frame_id = static_cast<int>(p[5]);
            float point_score = p.size() > 6 ? p[6] : 0.0f;
            int track_class = p.size() > 7 ? static_cast<int>(p[7]) : point_class;
            ofs << "frame_" << frame_id << ": x: " << center_x << ", y: " << center_y
                << ", det_class: " << point_class << ", score: " << std::fixed << std::setprecision(4) << point_score
                << ", track_class: " << track_class << endl;
        }
      }
      ofs << endl;
      ofs.close();

      // Save corresponding image with trajectory drawn
      if (this->save_img_flag && save_frame_cb_) {
        save_frame_cb_(removed_stracks[i].track_id,
                       removed_stracks[i].class_id,
                       removed_stracks[i].track_points,
                       prefix,
                       this->pole_name + "_" + this->camera_direction + "_" + this->camera_type + "_" + time_str
                           + "_" + to_string(removed_stracks[i].class_id)
                           + "_" + to_string(removed_stracks[i].track_id));
      }

      if (track_removed_cb_) {
        std::string event_type_str = track_removed_cb_(removed_stracks[i].track_id,
                                                        removed_stracks[i].class_id,
                                                        removed_stracks[i].track_points);

        ofstream ofs_event;
        ofs_event.open(filename, ios::app);
        if (ofs_event.is_open()) {
          if (!event_type_str.empty()) {
            ofs_event << "event: " << event_type_str << endl;
            ofs_event.close();
            std::string new_filename = prefix + this->pole_name + "_" + this->camera_direction + "_" + this->camera_type + "_" + time_str + "_" + to_string(removed_stracks[i].class_id) + "_" + to_string(removed_stracks[i].track_id) + "_" + event_type_str + ".txt";
            rename(filename.c_str(), new_filename.c_str());
            filename = new_filename;
          } else {
            ofs_event << "event: normal" << endl;
            ofs_event.close();
          }
        }
      }
    } else if (track_removed_cb_) {
      track_removed_cb_(removed_stracks[i].track_id,
                        removed_stracks[i].class_id,
                        removed_stracks[i].track_points);
    }
  }
  this->removed_stracks.clear();
  return output_stracks;
}

vector<STrack*> BYTETracker::joint_stracks(vector<STrack*>& tlista,
                                           vector<STrack>& tlistb) {
  map<int, int> exists;
  vector<STrack*> res;
  for (int i = 0; i < tlista.size(); i++) {
    exists.insert(pair<int, int>(tlista[i]->track_id, 1));
    res.push_back(tlista[i]);
  }
  for (int i = 0; i < tlistb.size(); i++) {
    int tid = tlistb[i].track_id;
    if (!exists[tid] || exists.count(tid) == 0) {
      exists[tid] = 1;
      res.push_back(&tlistb[i]);
    }
  }
  return res;
}

vector<STrack> BYTETracker::joint_stracks(vector<STrack>& tlista,
                                          vector<STrack>& tlistb) {
  map<int, int> exists;
  vector<STrack> res;
  for (int i = 0; i < tlista.size(); i++) {
    exists.insert(pair<int, int>(tlista[i].track_id, 1));
    res.push_back(tlista[i]);
  }
  for (int i = 0; i < tlistb.size(); i++) {
    int tid = tlistb[i].track_id;
    if (!exists[tid] || exists.count(tid) == 0) {
      exists[tid] = 1;
      res.push_back(tlistb[i]);
    }
  }
  return res;
}

vector<STrack> BYTETracker::sub_stracks(vector<STrack>& tlista,
                                        vector<STrack>& tlistb) {
  map<int, STrack> stracks;
  for (int i = 0; i < tlista.size(); i++) {
    stracks.insert(pair<int, STrack>(tlista[i].track_id, tlista[i]));
  }
  for (int i = 0; i < tlistb.size(); i++) {
    int tid = tlistb[i].track_id;
    if (stracks.count(tid) != 0) {
      stracks.erase(tid);
    }
  }

  vector<STrack> res;
  std::map<int, STrack>::iterator it;
  for (it = stracks.begin(); it != stracks.end(); ++it) {
    res.push_back(it->second);
  }

  return res;
}

void BYTETracker::remove_duplicate_stracks(vector<STrack>& resa,
                                           vector<STrack>& resb,
                                           vector<STrack>& stracksa,
                                           vector<STrack>& stracksb) {
  vector<vector<float>> pdist = iou_distance(stracksa, stracksb);
  vector<pair<int, int>> pairs;
  for (int i = 0; i < pdist.size(); i++) {
    for (int j = 0; j < pdist[i].size(); j++) {
      // 只有时间上有重叠的轨迹才做重复判断
      // 避免将不同时间出现在同一位置的两个不同轨迹合并
      if (pdist[i][j] < 0.1 && stracksa[i].class_id == stracksb[j].class_id) {
        int end_a = stracksa[i].frame_id;
        int start_b = stracksb[j].start_frame;
        int end_b = stracksb[j].frame_id;
        int start_a = stracksa[i].start_frame;
        // 两个轨迹的帧范围必须有交集
        if (end_a >= start_b && end_b >= start_a) {
          pairs.push_back(pair<int, int>(i, j));
        }
      }
    }
  }

  vector<int> dupa, dupb;
  for (int i = 0; i < pairs.size(); i++) {
    int timep = stracksa[pairs[i].first].frame_id -
                stracksa[pairs[i].first].start_frame;
    int timeq = stracksb[pairs[i].second].frame_id -
                stracksb[pairs[i].second].start_frame;
    if (timep > timeq)
      dupb.push_back(pairs[i].second);
    else
      dupa.push_back(pairs[i].first);
  }

  for (int i = 0; i < stracksa.size(); i++) {
    vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
    if (iter == dupa.end()) {
      resa.push_back(stracksa[i]);
    }
  }

  for (int i = 0; i < stracksb.size(); i++) {
    vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
    if (iter == dupb.end()) {
      resb.push_back(stracksb[i]);
    }
  }
}

void BYTETracker::linear_assignment(vector<vector<float>>& cost_matrix,
                                    int cost_matrix_size,
                                    int cost_matrix_size_size, float thresh,
                                    vector<vector<int>>& matches,
                                    vector<int>& unmatched_a,
                                    vector<int>& unmatched_b) {
  if (cost_matrix.size() == 0) {
    for (int i = 0; i < cost_matrix_size; i++) {
      unmatched_a.push_back(i);
    }
    for (int i = 0; i < cost_matrix_size_size; i++) {
      unmatched_b.push_back(i);
    }
    return;
  }
  vector<int> rowsol;
  vector<int> colsol;
  float c = lapjv(cost_matrix, rowsol, colsol, true, thresh);
  for (int i = 0; i < rowsol.size(); i++) {
    if (rowsol[i] >= 0) {
      vector<int> match;
      match.push_back(i);
      match.push_back(rowsol[i]);
      matches.push_back(match);
    } else {
      unmatched_a.push_back(i);
    }
  }
  for (int i = 0; i < colsol.size(); i++) {
    if (colsol[i] < 0) {
      unmatched_b.push_back(i);
    }
  }
}

vector<vector<float>> BYTETracker::ious(vector<vector<float>>& atlbrs,
                                        vector<vector<float>>& btlbrs) {
  vector<vector<float>> ious;
  if (atlbrs.size() * btlbrs.size() == 0) return ious;

  ious.resize(atlbrs.size());
  for (int i = 0; i < ious.size(); i++) {
    ious[i].resize(btlbrs.size());
  }

  // bbox_ious
  for (int k = 0; k < btlbrs.size(); k++) {
    vector<float> ious_tmp;
    float box_area =
        (btlbrs[k][2] - btlbrs[k][0] + 1) * (btlbrs[k][3] - btlbrs[k][1] + 1);
    for (int n = 0; n < atlbrs.size(); n++) {
      float iw =
          min(atlbrs[n][2], btlbrs[k][2]) - max(atlbrs[n][0], btlbrs[k][0]) + 1;
      if (iw > 0) {
        float ih = min(atlbrs[n][3], btlbrs[k][3]) -
                   max(atlbrs[n][1], btlbrs[k][1]) + 1;
        if (ih > 0) {
          float ua = (atlbrs[n][2] - atlbrs[n][0] + 1) *
                         (atlbrs[n][3] - atlbrs[n][1] + 1) +
                     box_area - iw * ih;
          ious[n][k] = iw * ih / ua;
        } else {
          ious[n][k] = 0.0;
        }
      } else {
        ious[n][k] = 0.0;
      }
    }
  }

  return ious;
}

vector<vector<float>> BYTETracker::iou_distance(vector<STrack*>& atracks,
                                                vector<STrack>& btracks,
                                                int& dist_size,
                                                int& dist_size_size) {
  vector<vector<float>> cost_matrix;
  if (atracks.size() * btracks.size() == 0) {
    dist_size = atracks.size();
    dist_size_size = btracks.size();
    return cost_matrix;
  }
  vector<vector<float>> atlbrs, btlbrs;
  for (int i = 0; i < atracks.size(); i++) {
    atlbrs.push_back(atracks[i]->tlbr);
  }
  for (int i = 0; i < btracks.size(); i++) {
    btlbrs.push_back(btracks[i].tlbr);
  }

  dist_size = atracks.size();
  dist_size_size = btracks.size();

  vector<vector<float>> _ious = ious(atlbrs, btlbrs);

  for (int i = 0; i < _ious.size(); i++) {
    vector<float> _iou;
    for (int j = 0; j < _ious[i].size(); j++) {
      float iou = 1 - _ious[i][j];

      // 类别惩罚：不同类别时增加距离，降低跨类别匹配概率
      if (atracks[i]->class_id != btracks[j].class_id) {
        if (atracks[i]->class_lock || atracks[i]->class_hits[atracks[i]->class_id] >= 5) {
          iou += 0.3;
        } else {
          iou += 0.1;
        }
      }

      // 尺寸比例惩罚
      float track_w = atracks[i]->tlwh[2];
      float track_h = atracks[i]->tlwh[3];
      float det_w = btracks[j].tlwh[2];
      float det_h = btracks[j].tlwh[3];
      float track_area = track_w * track_h;
      float det_area = det_w * det_h;
      if (track_area > 0 && det_area > 0) {
        float ratio = std::max(track_area, det_area) / std::min(track_area, det_area);
        if (ratio > 6.0f) {
          iou += 0.3;
        } else if (ratio > 3.0f) {
          iou += 0.1;
        }
      }

      // 对 Lost 状态的轨迹略微放宽匹配要求（已通过最大位移检查）
      if (atracks[i]->state == TrackState::Lost) {
        iou -= 0.05;
      }

      // 检查tlbr合法性：左>右 或 上>下 说明Kalman预测发散，禁止匹配
      if (atlbrs[i][2] <= atlbrs[i][0] || atlbrs[i][3] <= atlbrs[i][1] || atlbrs[i][0] < 0 || atlbrs[i][1] < 0) {
        iou = 1.0;
      }



      // 当IoU很低时，用中心点距离辅助匹配（仅近端大目标，避免远端误匹配）
      float track_area_check = track_w * track_h;
      if (_ious[i][j] < 0.1 && track_area_check > 20000.0f && atracks[i]->state != TrackState::Lost) {
        float track_cx = (atlbrs[i][0] + atlbrs[i][2]) / 2.0f;
        float track_cy = (atlbrs[i][1] + atlbrs[i][3]) / 2.0f;
        float det_cx = (btlbrs[j][0] + btlbrs[j][2]) / 2.0f;
        float det_cy = (btlbrs[j][1] + btlbrs[j][3]) / 2.0f;
        float center_dist = std::sqrt((track_cx - det_cx) * (track_cx - det_cx) + (track_cy - det_cy) * (track_cy - det_cy));
        float track_diag = std::sqrt(track_w * track_w + track_h * track_h);
        float det_diag = std::sqrt(det_w * det_w + det_h * det_h);
        float max_diag = std::max(track_diag, det_diag);
        // 如果中心点距离小于最大框对角线的一半，降低距离值
        if (max_diag > 0 && center_dist < max_diag * 0.5f) {
          float center_iou = 1.0f - center_dist / (max_diag * 0.5f);
          iou = std::min(iou, 1.0f - center_iou * 0.5f);
        }
      }

      _iou.push_back(iou);
    }
    cost_matrix.push_back(_iou);
  }

  return cost_matrix;
}

vector<vector<float>> BYTETracker::iou_distance(vector<STrack>& atracks,
                                                vector<STrack>& btracks) {
  vector<vector<float>> atlbrs, btlbrs;
  for (int i = 0; i < atracks.size(); i++) {
    atlbrs.push_back(atracks[i].tlbr);
  }
  for (int i = 0; i < btracks.size(); i++) {
    btlbrs.push_back(btracks[i].tlbr);
  }

  vector<vector<float>> _ious = ious(atlbrs, btlbrs);
  vector<vector<float>> cost_matrix;
  for (int i = 0; i < _ious.size(); i++) {
    vector<float> _iou;
    for (int j = 0; j < _ious[i].size(); j++) {
      float iou = 1 - _ious[i][j];

      if (atracks[i].class_id != btracks[j].class_id) {
        if (atracks[i].class_lock || atracks[i].class_hits[atracks[i].class_id] >= 5) {
          iou += 0.3;
        } else {
          iou += 0.1;
        }
      }

      float track_area = atracks[i].tlwh[2] * atracks[i].tlwh[3];
      float det_area = btracks[j].tlwh[2] * btracks[j].tlwh[3];
      if (track_area > 0 && det_area > 0) {
        float ratio = std::max(track_area, det_area) / std::min(track_area, det_area);
        if (ratio > 4.0f) {
          iou += 0.5;
        } else if (ratio > 2.0f) {
          iou += 0.2;
        }
      }

      _iou.push_back(iou);
    }
    cost_matrix.push_back(_iou);
  }

  return cost_matrix;
}

double BYTETracker::lapjv(const vector<vector<float>>& cost,
                          vector<int>& rowsol, vector<int>& colsol,
                          bool extend_cost, float cost_limit,
                          bool return_cost) {
  vector<vector<float>> cost_c;
  cost_c.assign(cost.begin(), cost.end());

  vector<vector<float>> cost_c_extended;

  int n_rows = cost.size();
  int n_cols = cost[0].size();
  rowsol.resize(n_rows);
  colsol.resize(n_cols);

  int n = 0;
  if (n_rows == n_cols) {
    n = n_rows;
  } else {
    if (!extend_cost) {
      cout << "set extend_cost=True" << endl;
      return 1;
    }
  }
  if (extend_cost || cost_limit < LONG_MAX) {
    n = n_rows + n_cols;
    cost_c_extended.resize(n);

    for (int i = 0; i < cost_c_extended.size(); i++)
      cost_c_extended[i].resize(n);

    if (cost_limit < LONG_MAX) {
      for (int i = 0; i < cost_c_extended.size(); i++) {
        for (int j = 0; j < cost_c_extended[i].size(); j++) {
          cost_c_extended[i][j] = cost_limit / 2.0;
        }
      }
    } else {
      float cost_max = -1;
      for (int i = 0; i < cost_c.size(); i++) {
        for (int j = 0; j < cost_c[i].size(); j++) {
          if (cost_c[i][j] > cost_max) cost_max = cost_c[i][j];
        }
      }
      for (int i = 0; i < cost_c_extended.size(); i++) {
        for (int j = 0; j < cost_c_extended[i].size(); j++) {
          cost_c_extended[i][j] = cost_max + 1;
        }
      }
    }
    for (int i = n_rows; i < cost_c_extended.size(); i++) {
      for (int j = n_cols; j < cost_c_extended[i].size(); j++) {
        cost_c_extended[i][j] = 0;
      }
    }
    for (int i = 0; i < n_rows; i++) {
      for (int j = 0; j < n_cols; j++) {
        cost_c_extended[i][j] = cost_c[i][j];
      }
    }

    cost_c.clear();
    cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
  }
  double** cost_ptr;
  cost_ptr = new double*[n];
  for (int i = 0; i < n; i++) cost_ptr[i] = new double[n];

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      cost_ptr[i][j] = cost_c[i][j];
    }
  }

  int* x_c = new int[n];
  int* y_c = new int[n];

  int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
  if (ret != 0) {
    cout << "[BYTETRACK] lapjv_internal failed, ret=" << ret << ", falling back to greedy matching" << endl;
    for (int i = 0; i < n; i++) { delete[] cost_ptr[i]; }
    delete[] cost_ptr;
    delete[] x_c;
    delete[] y_c;
    return 1;
  }

  double opt = 0.0;
  if (n != n_rows) {
    for (int i = 0; i < n; i++) {
      if (x_c[i] >= n_cols) x_c[i] = -1;
      if (y_c[i] >= n_rows) y_c[i] = -1;
    }
    for (int i = 0; i < n_rows; i++) {
      rowsol[i] = x_c[i];
    }
    for (int i = 0; i < n_cols; i++) {
      colsol[i] = y_c[i];
    }

    if (return_cost) {
      for (int i = 0; i < rowsol.size(); i++) {
        if (rowsol[i] != -1) {
          // cout << i << "\t" << rowsol[i] << "\t" << cost_ptr[i][rowsol[i]] <<
          // endl;
          opt += cost_ptr[i][rowsol[i]];
        }
      }
    }
  } else if (return_cost) {
    for (int i = 0; i < rowsol.size(); i++) {
      opt += cost_ptr[i][rowsol[i]];
    }
  }
  for (int i = 0; i < n; i++) {
    delete[] cost_ptr[i];
  }
  delete[] cost_ptr;
  delete[] x_c;
  delete[] y_c;

  return opt;
}



