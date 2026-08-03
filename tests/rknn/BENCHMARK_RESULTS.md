# RKNN Benchmark Results

## Hardware
- **Platform**: Rockchip RK3588 (NPU)
- **SDK**: rknn-toolkit 1.4.0 (driver 0.9.8)

## Model
- **Model**: `detect.rknn_0625` (YOLOv5, 3 outputs)
- **Input**: 640x640 uint8 NHWC
- **Classes**: 10

## Performance (200 iterations, after 20 warmup)

| Metric | Time |
|--------|------|
| End-to-end (preprocess + inference + postprocess) | **21.25 ms** → **47.1 FPS** |
| Inference only (rknn_run) | **20.15 ms** → **49.6 FPS** |

## Analysis
- Preprocess + postprocess overhead: ~1.1 ms (5.5% of total)
- Inference is the dominant cost (~95%)
- Single-image throughput is **~47 FPS**; batch-4 would be ~4x higher