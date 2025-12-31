//===----------------------------------------------------------------------===//
//
// Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
//
// SOPHON-DEMO is licensed under the 2-Clause BSD License except for the
// third-party components.
//
//===----------------------------------------------------------------------===//
#pragma once

#include <opencv2/opencv.hpp>
#include "kalmanfilter.h"
#include "lapjv.h"

using namespace std;

enum TrackState
{
	New = 0,
	Tracked,
	Lost,
	Removed
};

class STrack
{
public:
	STrack(vector<float> tlwh_, float score, int class_id);
	~STrack();

	vector<float> static tlbr_to_tlwh(vector<float> &tlbr);
	void static multi_predict(vector<STrack *> &stracks, KalmanFilter &kalman_filter);
	void static_tlwh();
	void static_tlbr();
	vector<float> tlwh_to_xyah(vector<float> tlwh_tmp);
	vector<float> to_xyah();
	void mark_lost();
	void mark_removed();
	int next_id();
	int end_frame();

	void activate(KalmanFilter &kalman_filter, int frame_id, int * global_tracker_id);
	void re_activate(STrack &new_track, int frame_id, bool new_id = false);
	void update(STrack &new_track, int frame_id);

public:
	bool is_activated;
	int track_id;
	int state;

	vector<float> _tlwh;
	vector<float> tlwh;
	vector<float> tlbr;
	vector<float> det_box = {0, 0, 0, 0}; // detection box in tlwh format
	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN mean;
	KAL_COVA covariance;
	float score;
	std::map<int, int> class_hits; //缓存的class_hits  <id, num>
    std::map<int, int> color_hits; //缓存的class_hits
	
	int  class_id = 0; // 分类也要更新，防止远端检测错误 
	bool class_lock = false;

	bool color_lock = false; 
	int vehicle_color = 0; // 颜色也要更新，防止部分颜色错误

	std::string plate_number = "";
	int plate_color = 0;
	float plate_confid = 0;

	std::vector<vector<float>> track_points;  // 轨迹点



private:
	KalmanFilter kalman_filter;
};