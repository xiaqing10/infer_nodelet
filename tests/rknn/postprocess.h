#ifndef _RKNN_POST_PROCESS_H_
#define _RKNN_POST_PROCESS_H_

#include <stdint.h>
#include <vector>

#define OBJ_CLASS_NUM 10
#define PROP_BOX_SIZE  (5 + OBJ_CLASS_NUM)
#define OBJ_NUMB_MAX_SIZE 256

typedef struct _BOX_RECT {
    int left;
    int right;
    int top;
    int bottom;
} BOX_RECT;

typedef struct _detect_result_t {
    int class_id;
    float prop;
    BOX_RECT box;
    char name[24];
} detect_result_t;

typedef struct _detect_result_group_t {
    int count;
    detect_result_t results[OBJ_NUMB_MAX_SIZE];
} detect_result_group_t;

int post_process(int8_t *input0, int8_t *input1, int8_t *input2,
                 int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold,
                 BOX_RECT pads, float scale_w, float scale_h,
                 std::vector<int32_t> &qnt_zps,
                 std::vector<float> &qnt_scales,
                 detect_result_group_t *group);

void deinitPostProcess();

#endif