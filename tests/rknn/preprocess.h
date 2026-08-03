#ifndef _RKNN_PRE_PROCESS_H_
#define _RKNN_PRE_PROCESS_H_

#include <opencv2/opencv.hpp>
#include "postprocess.h"

void letterbox(const cv::Mat &image, cv::Mat &padded_image, BOX_RECT &pads,
               const float scale, const cv::Size &target_size,
               const cv::Scalar &pad_color = cv::Scalar(114, 114, 114));

#endif