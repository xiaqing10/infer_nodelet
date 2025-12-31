//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-DEMO is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//
#include "bytetrack.h"
#include <fstream>

BYTETracker::BYTETracker(const bytetrack_params& params, bool write_flag_, std::string write_path_, int min_points_len_,  std::string camera_type_, std::string camera_direction_) {
  this->track_thresh = params.track_thresh;
  this->match_thresh = params.match_thresh;
  this->frame_rate = params.frame_rate;
  this->track_buffer = params.track_buffer;
  this->min_box_area = params.min_box_area;
  this->frame_id = 0;
  this->max_time_lost = int(this->frame_rate / 20.0 * this->track_buffer);

  this->write_flag = write_flag_;
  this->write_path = write_path_;
  this->camera_type = camera_type_;
  this->camera_direction = camera_direction_;
  this->min_points_len = min_points_len_;
  cout << "Init tracker!" << endl;
}

void BYTETracker::enableProfile(TimeStamp* ts) { m_ts = ts; }

BYTETracker::~BYTETracker() {}

// 更新车辆颜色，粗暴点，找出现次数最多的
int BYTETracker::updateVehicleColor(int xxTracker_id, int vehicle_color, float score){

    for(auto i = 0 ; i < this->tracked_stracks.size(); i++){

        if (this->tracked_stracks[i].track_id == xxTracker_id )  // 如果匹配到颜色ID，则更新map
        {
            auto key = vehicle_color;
            if (score > 0.95)
              this->tracked_stracks[i].color_hits[vehicle_color] += 3;
            else
              this->tracked_stracks[i].color_hits[vehicle_color] += 1;

            auto _i= max_element(this->tracked_stracks[i].color_hits.begin(),this->tracked_stracks[i].color_hits.end(),[](std::pair<char, int> left, std::pair<char,int> right) { return left.second < right.second; });
                  // 取最大值
            this->tracked_stracks[i].vehicle_color = _i->first;
            // 颜色如果累计够了，直接锁.
            if(this->tracked_stracks[i].color_hits[_i->first] > 4) {
              this->tracked_stracks[i].color_lock = true;
            }
            break;
        }
    }
   return 0;
}


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
  vector<STrack*> strack_pool;
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
      if (score >= track_thresh) {
        detections.push_back(strack);
      } else {
        detections_low.push_back(strack);
      }
    }
  }

  // 1正常的 激活 2 正常的 未激活  3 丢失的 激活 4 删除的 激活 5删除的 未激活
  // 新的目标是 state=0, is_activated=false

  //this->tracked_stracks 保存的是正常激活的tracker和新的tracker(未激活的)

  // unconfirmed 表示上一帧没有被确认的轨迹
  // tracked_stracks 表示上一帧已经被确认的轨迹
  // Add newly detected tracklets to tracked_stracks
  for (int i = 0; i < this->tracked_stracks.size(); i++) {
    if (!this->tracked_stracks[i].is_activated)
      unconfirmed.push_back(&this->tracked_stracks[i]); //新的轨迹，未激活
    else
      tracked_stracks.push_back(&this->tracked_stracks[i]); //激活的轨迹
  }

  ////////////////// Step 2: First association, with IoU //////////////////
  strack_pool = joint_stracks(tracked_stracks, this->lost_stracks);  // 激活的轨迹，丢失的轨迹都与高分匹配
  STrack::multi_predict(strack_pool, this->kalman_filter);

  vector<vector<float>> dists;
  int dist_size = 0, dist_size_size = 0;
  dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);  //上一帧的轨迹和高分框iou合并

  vector<vector<int>> matches;
  vector<int> u_track, u_detection;
  linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches,
                    u_track, u_detection);
  for (int i = 0; i < matches.size(); i++) {
    STrack* track = strack_pool[matches[i][0]];
    STrack* det = &detections[matches[i][1]];

    // 正常的激活的轨迹，直接用det更新
    if (track->state == TrackState::Tracked) {
      track->update(*det, this->frame_id);
      activated_stracks.push_back(*track);
    } else { //如果是丢失的轨迹，重新激活
      track->re_activate(*det, this->frame_id, false);
      refind_stracks.push_back(*track);
    }

    track->det_box = det->tlwh;
    // ========== 修复类别更新逻辑 ==========
    if (!track->class_lock) {
        // 只在高置信度或轨迹刚开始时更新类别
        bool should_update = false;

        // 条件1: 检测置信度高
        if (det->score > 0.7) {
            should_update = true;
        }
        // 条件2: 轨迹刚开始（前几帧）
        else if ((this->frame_id - track->start_frame) <= 5) {
            should_update = true;
        }
        // 条件3: 当前轨迹的类别计数很少，说明还不稳定
        else if (track->class_hits[track->class_id] < 3) {
            should_update = true;
        }

        if (should_update) {
            // 加权更新：置信度高的检测结果权重更大
            int weight = (det->score > 0.8) ? 2 : 1;
            track->class_hits[det->class_id] += weight;

            // 重新计算当前最优类别
            auto _i = max_element(track->class_hits.begin(), track->class_hits.end(),
                [](std::pair<char, int> left, std::pair<char, int> right) {
                    return left.second < right.second;
                });

            // 只有当新类别明显占优时才更新
            int current_best_count = _i->second;
            int current_class_count = track->class_hits[track->class_id];

            if (_i->first != track->class_id &&
                current_best_count >= current_class_count + 2) {
                track->class_id = _i->first;
            }

            // 提高锁定阈值，确保类别稳定
            if (track->class_hits[_i->first] > 8) {
                track->class_lock = true;
            }
        }
    }
    // ========== 修复结束 ==========


    // 在第一个匹配过程中（高分检测框）
    if(track->track_points.size() < 500){
        // 修改为存储包含类别的信息
        std::vector<float> point_with_class = {track->tlwh[0], track->tlwh[1], track->tlwh[2], track->tlwh[3],
                                              static_cast<float>(det->class_id), static_cast<float>(this->frame_id)};
        track->track_points.push_back(point_with_class);
    }
    else{
        track->track_points.erase(track->track_points.begin());
        std::vector<float> point_with_class = {track->tlwh[0], track->tlwh[1], track->tlwh[2], track->tlwh[3],
                                              static_cast<float>(det->class_id), static_cast<float>(this->frame_id)};
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
    if (strack_pool[u_track[i]]->state == TrackState::Tracked) {  //如果那个轨迹是激活的，而不是lost的，那么就放入r_tracked_stracks中
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
    // ========== 修复类别更新逻辑 ==========
    if (!track->class_lock) {
        // 只在高置信度或轨迹刚开始时更新类别
        bool should_update = false;

        // 条件1: 检测置信度高
        if (det->score > 0.7) {
            should_update = true;
        }
        // 条件2: 轨迹刚开始（前几帧）
        else if ((this->frame_id - track->start_frame) <= 5) {
            should_update = true;
        }
        // 条件3: 当前轨迹的类别计数很少，说明还不稳定
        else if (track->class_hits[track->class_id] < 3) {
            should_update = true;
        }

        if (should_update) {
            // 加权更新：置信度高的检测结果权重更大
            int weight = (det->score > 0.8) ? 2 : 1;
            track->class_hits[det->class_id] += weight;

            // 重新计算当前最优类别
            auto _i = max_element(track->class_hits.begin(), track->class_hits.end(),
                [](std::pair<char, int> left, std::pair<char, int> right) {
                    return left.second < right.second;
                });

            // 只有当新类别明显占优时才更新
            int current_best_count = _i->second;
            int current_class_count = track->class_hits[track->class_id];

            if (_i->first != track->class_id &&
                current_best_count >= current_class_count + 2) {
                track->class_id = _i->first;
            }

            // 提高锁定阈值，确保类别稳定
            if (track->class_hits[_i->first] > 8) {
                track->class_lock = true;
            }
        }
    }
    // ========== 修复结束 ==========

    // 在第二个匹配过程中（低分检测框）做同样的修改
    if(track->track_points.size() < 500){
        std::vector<float> point_with_class = {track->tlwh[0], track->tlwh[1], track->tlwh[2], track->tlwh[3],
                                              static_cast<float>(det->class_id), static_cast<float>(this->frame_id)};
        track->track_points.push_back(point_with_class);
    }
    else{
        track->track_points.erase(track->track_points.begin());
        std::vector<float> point_with_class = {track->tlwh[0], track->tlwh[1], track->tlwh[2], track->tlwh[3],
                                              static_cast<float>(det->class_id), static_cast<float>(this->frame_id)};
        track->track_points.push_back(point_with_class);
    }
  }
  // 以上第二次合并完毕。 但是依然还有没有合并上的正常的和丢失的轨迹，这些轨迹全部改为lost状态，并添加到lost_stracks中
  for (int i = 0; i < u_track.size(); i++) {
    STrack* track = r_tracked_stracks[u_track[i]];
    if (track->state != TrackState::Lost) {
      track->mark_lost();
      lost_stracks.push_back(*track);
    }
  }

  // Deal with unconfirmed tracks, usually tracks with only one beginning frame
  // 用高分检测框与上一帧没有被激活的迹进行匹配，就是初始化轨迹
  detections.clear();
  detections.assign(detections_cp.begin(), detections_cp.end());

  // 以上，已经将之前确认的轨迹进行合并完毕了，开始考虑上一帧没有被确认的轨迹.
  // 这些轨迹要么是新出现的，要么是被遮挡住了. 匹配的是依旧是高分检测框. @https://github.com/FoundationVision/ByteTrack/issues/395

  dists.clear();
  dists = iou_distance(unconfirmed, detections, dist_size, dist_size_size);

  matches.clear();
  vector<int> u_unconfirmed;
  u_detection.clear();
  linear_assignment(dists, dist_size, dist_size_size, 0.7, matches,
                    u_unconfirmed, u_detection);

  // 如果匹配上了，进行更新
  for (int i = 0; i < matches.size(); i++) {
    unconfirmed[matches[i][0]]->update(detections[matches[i][1]],
                                       this->frame_id);
    activated_stracks.push_back(*unconfirmed[matches[i][0]]);
  }

  // 将没有被激活的轨迹标记为删除，因为激活的条件是连续两帧被匹配上，断了就直接丢弃了。
  for (int i = 0; i < u_unconfirmed.size(); i++) {
    STrack* track = unconfirmed[u_unconfirmed[i]];
    track->mark_removed();
    removed_stracks.push_back(*track);
  }

  ////////////////// Step 4: Init new stracks //////////////////
  // 剩下了一些高分的检测框，没有被匹配上的，那么就进行初始化，为新的未激活的轨迹
  for (int i = 0; i < u_detection.size(); i++) {
    STrack* track = &detections[u_detection[i]];
    if (track->score < this->track_thresh) continue;
    track->activate(this->kalman_filter, this->frame_id, &global_tracker_id);
    activated_stracks.push_back(*track);
  }
  ////////////////// Step 5: Update state //////////////////
  //将this->lost_stracks中满足删除条件的轨迹标记为removed，并放入removed_stracks。 @2
  for (int i = 0; i < this->lost_stracks.size(); i++) {
    if (this->frame_id - this->lost_stracks[i].end_frame() >
        this->max_time_lost) {
      this->lost_stracks[i].mark_removed();
      removed_stracks.push_back(this->lost_stracks[i]);
    }
  }

  // 更新this->tracked_stracks，把状态为Tracked的轨迹留下
  for (int i = 0; i < this->tracked_stracks.size(); i++) {
    if (this->tracked_stracks[i].state == TrackState::Tracked) {
      tracked_stracks_swap.push_back(this->tracked_stracks[i]);
    }
  }
  this->tracked_stracks.clear();
  this->tracked_stracks.assign(tracked_stracks_swap.begin(),
                               tracked_stracks_swap.end());


  // 将本次更新的激活的轨迹和重新找回的轨迹加入到this->tracked_stracks中
  this->tracked_stracks =
      joint_stracks(this->tracked_stracks, activated_stracks);

  this->tracked_stracks = joint_stracks(this->tracked_stracks, refind_stracks);

  // 已经被跟踪的轨迹删除，没有被跟踪的轨迹加上，已经被删除的轨迹删除 = this->lost_stracks;
  this->lost_stracks = sub_stracks(this->lost_stracks, this->tracked_stracks);
  for (int i = 0; i < lost_stracks.size(); i++) {
    this->lost_stracks.push_back(lost_stracks[i]);
  }
  this->lost_stracks = sub_stracks(this->lost_stracks, removed_stracks);

  //移除重复轨迹
  remove_duplicate_stracks(resa, resb, this->tracked_stracks,
                           this->lost_stracks);

  this->tracked_stracks.clear();
  this->tracked_stracks.assign(resa.begin(), resa.end());
  this->lost_stracks.clear();
  this->lost_stracks.assign(resb.begin(), resb.end());
  for (int i = 0; i < this->tracked_stracks.size(); i++) {
    if(
        this->tracked_stracks[i].tlwh[2] * this->tracked_stracks[i].tlwh[3] >
            this->min_box_area)
      output_stracks.push_back(this->tracked_stracks[i]);
  }

  for (int i = 0; i < removed_stracks.size(); i++) {
    if(removed_stracks[i].track_points.size() < this->min_points_len) { //如果轨迹很短就忽略
      continue;
    }

    if(this->write_flag){
      ofstream ofs;
      auto now = std::chrono::system_clock::now();
      std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::stringstream day, ss;
      day << std::put_time(std::localtime(&t), "%Y-%m-%d");
      ss << std::put_time(std::localtime(&t), "%Y%m%d%H%M%S");

      // 判断写入的路径是否存在
      std::string prefix = this->write_path;
      if(access(prefix.c_str(), 0) == -1)       mkdir(prefix.c_str(),S_IRWXU);
      prefix = this->write_path + day.str();
      if(access(prefix.c_str(), 0) == -1)       mkdir(prefix.c_str(),S_IRWXU);
      prefix = this->write_path + day.str() + "/tracks/";
      if(access(prefix.c_str(), 0) == -1)       mkdir(prefix.c_str(),S_IRWXU);

      // 写文件
      std::string filename = prefix + ss.str() + "-" + to_string(removed_stracks[i].class_id) + "-" + to_string(removed_stracks[i].track_id)+ ".txt";
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
          // for (auto p : removed_stracks[i].track_points) {
          //   ofs << "x: " << p[0] + p[2]/2 << ", y: " << p[1] + p[3]/2 << endl; // 记录中心点
          // }
          for (auto p : removed_stracks[i].track_points) {
            // 现在 p 包含6个元素: [x, y, w, h, class_id, frame_id]
            float center_x = p[0] + p[2]/2;
            float center_y = p[1] + p[3]/2;
            int point_class = static_cast<int>(p[4]);
            int frame_id = static_cast<int>(p[5]);

            ofs << "frame_" << frame_id << ": x: " << center_x << ", y: " << center_y
                << ", class: " << point_class << endl;
        }
      }
      ofs << endl;
      ofs.close();
    }
  } // end write

  this->removed_stracks.clear();
  // for (int i = 0; i < removed_stracks.size(); i++) {
  //   this->removed_stracks.push_back(removed_stracks[i]);
  // }

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
      if (pdist[i][j] < 0.15) {
        pairs.push_back(pair<int, int>(i, j));
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
      _iou.push_back(1 - _ious[i][j]);
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
      _iou.push_back(1 - _ious[i][j]);
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
      system("pause");
      exit(0);
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
  cost_ptr = new double*[sizeof(double*) * n];
  for (int i = 0; i < n; i++) cost_ptr[i] = new double[sizeof(double) * n];

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      cost_ptr[i][j] = cost_c[i][j];
    }
  }

  int* x_c = new int[sizeof(int) * n];
  int* y_c = new int[sizeof(int) * n];

  int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
  if (ret != 0) {
    cout << "Calculate Wrong!" << endl;
    system("pause");
    exit(0);
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


