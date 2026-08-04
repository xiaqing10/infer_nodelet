#include "rknn_api.h"
rknn_context ctx;
rknn_core_mask core_mask = RKNN_NPU_CORE_0_1_2;
// 加载模型
void* model;
int model_len = load_model("model.rknn", &model);
if (rknn_init(&ctx, model, model_len, RKNN_FLAG_MEM_ALLOC_OUTSIDE, NULL) != RKNN_SUCC) {
   printf("Init failed\n");
   return;
}
// 设置核心掩码
if (rknn_set_core_mask(ctx, core_mask) != RKNN_SUCC) {
   printf("Set core mask failed, fallback to single core\n");
}
