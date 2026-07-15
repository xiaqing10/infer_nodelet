//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-DEMO is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//
#include "strack.h"
#include <ros/ros.h>

static std::string s_track_log_prefix = "[TRACK]";
static bool s_track_debug = false;

const char* get_track_log_prefix() { return s_track_log_prefix.c_str(); }

void set_track_log_prefix(const std::string& prefix) { s_track_log_prefix = prefix; }

bool get_track_debug() { return s_track_debug; }

void set_track_debug(bool enabled) { s_track_debug = enabled; }

STrack::STrack(vector<float> tlwh_, float score, int class_id) {
  _tlwh.resize(4);
  _tlwh.assign(tlwh_.begin(), tlwh_.end());

  is_activated = false;
  track_id = 0;
  state = TrackState::New;

  tlwh.resize(4);
  tlbr.resize(4);

  static_tlwh();
  static_tlbr();
  frame_id = 0;
  tracklet_len = 0;
  this->score = score;
  this->class_id = class_id;
  start_frame = 0;
}

STrack::~STrack() {}

void STrack::activate(KalmanFilter& kalman_filter, int frame_id, int *global_tracker_id) {
  this->kalman_filter = kalman_filter;
  // this->track_id = this->next_id();
  this->track_id = *global_tracker_id;
  if(*global_tracker_id < 65535) {
    *global_tracker_id += 1;}
  else{
    *global_tracker_id = 0;
  }

  vector<float> _tlwh_tmp(4);
  _tlwh_tmp[0] = this->_tlwh[0];
  _tlwh_tmp[1] = this->_tlwh[1];
  _tlwh_tmp[2] = this->_tlwh[2];
  _tlwh_tmp[3] = this->_tlwh[3];
  vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
  DETECTBOX xyah_box;
  xyah_box[0] = xyah[0];
  xyah_box[1] = xyah[1];
  xyah_box[2] = xyah[2];
  xyah_box[3] = xyah[3];
  auto mc = this->kalman_filter.initiate(xyah_box);
  this->mean = mc.first;
  this->covariance = mc.second;

  static_tlwh();
  static_tlbr();

  this->tracklet_len = 0;
  this->state = TrackState::Tracked;
  if (frame_id == 1) {
    this->is_activated = true;
  }
  // this->is_activated = true;
  this->frame_id = frame_id;
  this->start_frame = frame_id;

  if (get_track_debug()) ROS_INFO("%s NEW track_id=%d class=%d tlwh=[%.1f,%.1f,%.1f,%.1f] a=%.3f h=%.1f score=%.2f frame=%d",
           get_track_log_prefix(),
           this->track_id, this->class_id, _tlwh_tmp[0], _tlwh_tmp[1], _tlwh_tmp[2], _tlwh_tmp[3],
           xyah[2], xyah[3], this->score, frame_id);
}

void STrack::re_activate(STrack& new_track, int frame_id, bool new_id) {
  vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
  DETECTBOX xyah_box;
  xyah_box[0] = xyah[0];
  xyah_box[1] = xyah[1];
  xyah_box[2] = xyah[2];
  xyah_box[3] = xyah[3];
  auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
  this->mean = mc.first;
  this->covariance = mc.second;

  static_tlwh();
  static_tlbr();

  this->tracklet_len = 0;
  this->state = TrackState::Tracked;
  this->is_activated = true;
  this->frame_id = frame_id;
  this->score = new_track.score;
  if (new_id) this->track_id = next_id();
}

void STrack::update(STrack& new_track, int frame_id) {
  this->frame_id = frame_id;
  this->tracklet_len++;

  float new_area = new_track.tlwh[2] * new_track.tlwh[3];
  bool was_activated = this->is_activated;

  // 延迟激活：连续匹配 K 帧且面积不再明显增长后才正式激活
  // 避免目标从画面边缘刚出现、形状不完整时污染 Kalman 状态
  if (!this->is_activated) {
    const float area_growth_ratio = (this->last_area > 0) ? (new_area / this->last_area) : 1.0f;
    const int required_frames = 3;  // 需要连续匹配 3 帧
    const float max_growth = 1.3f;  // 面积增长不超过 30% 视为稳定

    if (area_growth_ratio < max_growth) {
      this->activate_frame_count++;
    } else {
      this->activate_frame_count = 0;
    }

    if (get_track_debug()) ROS_INFO("%s PENDING track_id=%d class=%d area=%.0f->%.0f ratio=%.2f stable_cnt=%d/%d frame=%d",
             get_track_log_prefix(),
             this->track_id, this->class_id, this->last_area, new_area, area_growth_ratio,
             this->activate_frame_count, required_frames, frame_id);

    if (this->activate_frame_count >= required_frames) {
      this->is_activated = true;
    }
  }
  this->last_area = new_area;

  if (!was_activated && this->is_activated) {
    // 正式激活：用当前帧的检测框重新 initiate Kalman，丢弃延迟期间的污染状态
    vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
    DETECTBOX xyah_box;
    xyah_box[0] = xyah[0];
    xyah_box[1] = xyah[1];
    xyah_box[2] = xyah[2];
    xyah_box[3] = xyah[3];
    auto mc = this->kalman_filter.initiate(xyah_box);
    this->mean = mc.first;
    this->covariance = mc.second;

    if (get_track_debug()) ROS_INFO("%s ACTIVATED track_id=%d class=%d frame=%d area=%.0f aspect=%.3f",
             get_track_log_prefix(),
             this->track_id, this->class_id, frame_id, new_area,
             new_track.tlwh[2] / (new_track.tlwh[3] > 0 ? new_track.tlwh[3] : 1));
    static_tlwh();
  } else if (this->is_activated) {
    // 已激活轨迹：正常 Kalman update
    vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
    DETECTBOX xyah_box;
    xyah_box[0] = xyah[0];
    xyah_box[1] = xyah[1];
    xyah_box[2] = xyah[2];
    xyah_box[3] = xyah[3];

    auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
    this->mean = mc.first;
    this->covariance = mc.second;
    static_tlwh();
  } else {
    // 延迟激活期间（!is_activated）：不更新 Kalman，但用检测框更新 tlwh/tlbr
    // 确保下一帧匹配时使用的是最新检测位置，而不是初始位置
    this->tlwh[0] = new_track.tlwh[0];
    this->tlwh[1] = new_track.tlwh[1];
    this->tlwh[2] = new_track.tlwh[2];
    this->tlwh[3] = new_track.tlwh[3];
  }

  static_tlbr();

  this->state = TrackState::Tracked;

  this->score = new_track.score;
}

void STrack::static_tlwh() {
  if (this->state == TrackState::New) {
    tlwh[0] = _tlwh[0];
    tlwh[1] = _tlwh[1];
    tlwh[2] = _tlwh[2];
    tlwh[3] = _tlwh[3];
    return;
  }

  tlwh[0] = mean[0];
  tlwh[1] = mean[1];
  tlwh[2] = mean[2];
  tlwh[3] = mean[3];

  tlwh[2] *= tlwh[3];
  tlwh[0] -= tlwh[2] / 2;
  tlwh[1] -= tlwh[3] / 2;
}

void STrack::static_tlbr() {
  tlbr.clear();
  tlbr.assign(tlwh.begin(), tlwh.end());
  tlbr[2] += tlbr[0];
  tlbr[3] += tlbr[1];
}

vector<float> STrack::tlwh_to_xyah(vector<float> tlwh_tmp) {
  vector<float> tlwh_output = tlwh_tmp;
  tlwh_output[0] += tlwh_output[2] / 2;
  tlwh_output[1] += tlwh_output[3] / 2;
  tlwh_output[2] /= tlwh_output[3];
  return tlwh_output;
}

vector<float> STrack::to_xyah() { return tlwh_to_xyah(tlwh); }

vector<float> STrack::tlbr_to_tlwh(vector<float>& tlbr) {
  tlbr[2] -= tlbr[0];
  tlbr[3] -= tlbr[1];
  return tlbr;
}

void STrack::mark_lost() {
  state = TrackState::Lost;
  if (get_track_debug()) ROS_INFO("%s LOST track_id=%d class=%d frame=%d tracklet=%d tlwh=[%.1f,%.1f,%.1f,%.1f]",
           get_track_log_prefix(),
           this->track_id, this->class_id, this->frame_id, this->tracklet_len,
           this->tlwh[0], this->tlwh[1], this->tlwh[2], this->tlwh[3]);
}

void STrack::mark_removed() {
  state = TrackState::Removed;
  if (get_track_debug()) ROS_INFO("%s REMOVED track_id=%d class=%d frame=%d tracklet=%d activated=%d",
           get_track_log_prefix(),
           this->track_id, this->class_id, this->frame_id, this->tracklet_len,
           this->is_activated ? 1 : 0);
}

int STrack::next_id() {
  static int _count = 0;
  _count++;
  return _count;
}

int STrack::end_frame() { return this->frame_id; }

void STrack::multi_predict(vector<STrack*>& stracks,
                           KalmanFilter& kalman_filter) {
  for (int i = 0; i < stracks.size(); i++) {
    // 保持速度，让丢失轨迹继续预测移动
    // if (stracks[i]->state != TrackState::Tracked) {
    //   stracks[i]->mean[7] = 0;
    // }
    kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
  }
}