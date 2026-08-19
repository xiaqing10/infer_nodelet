#include "detector_nvidia.hpp"
#include "log_macros.h"
#include <cstring>

YoloV8_det::~YoloV8_det()
{
    if (context) {
        context->destroy();
        context = nullptr;
    }
    if (engine) {
        engine->destroy();
        engine = nullptr;
    }
    if (runtime) {
        runtime->destroy();
        runtime = nullptr;
    }
    if (stream) {
        cudaStreamDestroy(stream);
        stream = nullptr;
    }
    for (auto& ptr : device_ptrs) {
        CHECK(cudaFree(ptr));
    }
    for (auto& ptr : host_ptrs) {
        CHECK(cudaFreeHost(ptr));
    }
    device_ptrs.clear();
    host_ptrs.clear();
}

int YoloV8_det::Init(const std::string& engine_file_path, int num_labels)
{
    if (is_initialized_) {
        std::cout << "[WARN] NVIDIA detector already initialized" << std::endl;
        return 0;
    }

    model_num_labels = num_labels;

    std::ifstream file(engine_file_path, std::ios::binary);
    assert(file.good());
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    char* trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
    initLibNvInferPlugins(&gLogger, "");
    runtime = nvinfer1::createInferRuntime(gLogger);
    assert(runtime != nullptr);

    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    delete[] trtModelStream;
    context = engine->createExecutionContext();

    assert(context != nullptr);
    cudaStreamCreate(&stream);

    num_bindings = engine->getNbBindings();

    for (int i = 0; i < num_bindings; ++i) {
        Binding        binding;
        nvinfer1::Dims dims;
        nvinfer1::DataType dtype = engine->getBindingDataType(i);
        std::string        name  = engine->getBindingName(i);
        binding.name  = name;
        binding.dsize = type_to_size(dtype);
        bool IsInput = engine->bindingIsInput(i);
        if (IsInput) {
            num_inputs += 1;
            dims = engine->getProfileDimensions(i, 0, nvinfer1::OptProfileSelector::kMAX);
            context->setBindingDimensions(i, dims);
            binding.size = get_size_by_dims(dims);
            binding.dims = dims;
            input_bindings.push_back(binding);
        }
        else {
            dims = context->getBindingDimensions(i);
            binding.size = get_size_by_dims(dims);
            binding.dims = dims;
            output_bindings.push_back(binding);
            num_outputs += 1;
        }
    }

    make_pipe(true);

    is_initialized_ = true;
    std::cout << "[INFO] NVIDIA YOLOv8 model initialized, num_labels=" << model_num_labels << std::endl;
    return 0;
}

void YoloV8_det::make_pipe(bool warmup)
{
    for (auto& bindings : input_bindings) {
        void* d_ptr;
        CHECK(cudaMalloc(&d_ptr, bindings.size * bindings.dsize));
        device_ptrs.push_back(d_ptr);
    }

    for (auto& bindings : output_bindings) {
        void * d_ptr, *h_ptr;
        size_t size = bindings.size * bindings.dsize;
        CHECK(cudaMalloc(&d_ptr, size));
        CHECK(cudaHostAlloc(&h_ptr, size, 0));
        device_ptrs.push_back(d_ptr);
        host_ptrs.push_back(h_ptr);
    }

    if (warmup) {
        for (int i = 0; i < 1; i++) {
            for (auto& bindings : input_bindings) {
                size_t size  = bindings.size * bindings.dsize;
                void*  h_ptr = malloc(size);
                memset(h_ptr, 0, size);
                CHECK(cudaMemcpyAsync(device_ptrs[0], h_ptr, size, cudaMemcpyHostToDevice, stream));
                free(h_ptr);
            }
            infer();
        }
        printf("model warmup done \n");
    }
}

void YoloV8_det::letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size)
{
    const float inp_h  = size.height;
    const float inp_w  = size.width;
    float       height = image.rows;
    float       width  = image.cols;

    float r    = std::min(inp_h / height, inp_w / width);
    int   padw = std::round(width * r);
    int   padh = std::round(height * r);

    cv::Mat tmp;
    if ((int)width != padw || (int)height != padh) {
        cv::resize(image, tmp, cv::Size(padw, padh));
    }
    else {
        tmp = image.clone();
    }

    float dw = inp_w - padw;
    float dh = inp_h - padh;

    dw /= 2.0f;
    dh /= 2.0f;
    int top    = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left   = int(std::round(dw - 0.1f));
    int right  = int(std::round(dw + 0.1f));

    cv::copyMakeBorder(tmp, tmp, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});

    out.create({1, 3, (int)inp_h, (int)inp_w}, CV_32F);

    std::vector<cv::Mat> channels;
    cv::split(tmp, channels);

    cv::Mat c0((int)inp_h, (int)inp_w, CV_32F, (float*)out.data);
    cv::Mat c1((int)inp_h, (int)inp_w, CV_32F, (float*)out.data + (int)inp_h * (int)inp_w);
    cv::Mat c2((int)inp_h, (int)inp_w, CV_32F, (float*)out.data + (int)inp_h * (int)inp_w * 2);

    channels[0].convertTo(c2, CV_32F, 1 / 255.f);
    channels[1].convertTo(c1, CV_32F, 1 / 255.f);
    channels[2].convertTo(c0, CV_32F, 1 / 255.f);

    pparam.ratio  = 1 / r;
    pparam.dw     = dw;
    pparam.dh     = dh;
    pparam.height = height;
    pparam.width  = width;
}

void YoloV8_det::copy_from_Mat(const cv::Mat& image, cv::Size& size)
{
    cv::Mat  nchw;
    auto&    in_binding = input_bindings[0];
    int      width      = in_binding.dims.d[3];
    int      height     = in_binding.dims.d[2];
    cv::Size sz{width, height};
    letterbox(image, nchw, sz);

    CHECK(cudaMemcpyAsync(
        device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, stream));

    context->setBindingDimensions(0, nvinfer1::Dims{4, {1, 3, height, width}});
}

void YoloV8_det::infer()
{
    context->enqueueV2(device_ptrs.data(), stream, nullptr);
    for (int i = 0; i < num_outputs; i++) {
        size_t osize = output_bindings[i].size * output_bindings[i].dsize;
        CHECK(cudaMemcpyAsync(
            host_ptrs[i], device_ptrs[i + num_inputs], osize, cudaMemcpyDeviceToHost, stream));
    }
    cudaStreamSynchronize(stream);
}

void YoloV8_det::postprocess(YoloV8BoxVec& boxes, float score_thres, float iou_thres, int topk)
{
    boxes.clear();
    int num_channels = output_bindings[0].dims.d[1];
    int num_anchors  = output_bindings[0].dims.d[2];

    auto& dw     = pparam.dw;
    auto& dh     = pparam.dh;
    auto& width  = pparam.width;
    auto& height = pparam.height;
    auto& ratio  = pparam.ratio;

    std::vector<cv::Rect> bboxes;
    std::vector<float>    scores;
    std::vector<int>      labels;
    std::vector<int>      indices;

    cv::Mat output = cv::Mat(num_channels, num_anchors, CV_32F, static_cast<float*>(host_ptrs[0]));
    output         = output.t();
    for (int i = 0; i < num_anchors; i++) {
        auto  row_ptr    = output.row(i).ptr<float>();
        auto  bboxes_ptr = row_ptr;
        auto  scores_ptr = row_ptr + 4;
        auto  max_s_ptr  = std::max_element(scores_ptr, scores_ptr + model_num_labels);
        float score      = *max_s_ptr;
        if (score > score_thres) {
            float x = *bboxes_ptr++ - dw;
            float y = *bboxes_ptr++ - dh;
            float w = *bboxes_ptr++;
            float h = *bboxes_ptr;

            float x0 = clamp((x - 0.5f * w) * ratio, 0.f, width);
            float y0 = clamp((y - 0.5f * h) * ratio, 0.f, height);
            float x1 = clamp((x + 0.5f * w) * ratio, 0.f, width);
            float y1 = clamp((y + 0.5f * h) * ratio, 0.f, height);

            int              label = max_s_ptr - scores_ptr;
            cv::Rect_<float> bbox;
            bbox.x      = x0;
            bbox.y      = y0;
            bbox.width  = x1 - x0;
            bbox.height = y1 - y0;

            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
        }
    }

    cv::dnn::NMSBoxes(bboxes, scores, score_thres, iou_thres, indices);

    int cnt = 0;
    for (auto& i : indices) {
        if (cnt >= topk) {
            break;
        }
        YoloV8Box box;
        box.x1 = bboxes[i].x;
        box.y1 = bboxes[i].y;
        box.x2 = bboxes[i].x + bboxes[i].width;
        box.y2 = bboxes[i].y + bboxes[i].height;
        box.score = scores[i];
        box.class_id = labels[i];
        boxes.push_back(box);
        cnt += 1;
    }
}

int YoloV8_det::Detect(const cv::Mat& image, YoloV8BoxVec& boxes, int index)
{
    (void)index;
    if (!is_initialized_) {
        std::cerr << "[ERROR] NVIDIA detector not initialized" << std::endl;
        return -1;
    }

    cv::Size size = cv::Size{640, 640};
    copy_from_Mat(image, size);
    infer();
    postprocess(boxes);

    return 0;
}

int YoloV8_det::pre_process(const std::vector<cv::Mat>& images)
{
    if (!is_initialized_) {
        std::cerr << "[ERROR] NVIDIA detector not initialized" << std::endl;
        return -1;
    }
    int n = (int)images.size();
    if (n == 0) {
        return -1;
    }

    auto& in_binding = input_bindings[0];
    int width  = in_binding.dims.d[3];
    int height = in_binding.dims.d[2];
    cv::Size sz{width, height};
    int per_img = 3 * width * height;

    auto t_letterbox0 = std::chrono::steady_clock::now();
    std::vector<float> host_buf((size_t)n * per_img);
    pparam_batch_.resize(n);

    for (int i = 0; i < n; i++) {
        cv::Mat nchw;
        letterbox(images[i], nchw, sz);
        pparam_batch_[i] = pparam;  // 保存当前帧的 letterbox 参数
        memcpy(host_buf.data() + (size_t)i * per_img, nchw.ptr<float>(), (size_t)per_img * sizeof(float));
    }
    auto t_copy0 = std::chrono::steady_clock::now();

    CHECK(cudaMemcpyAsync(
        device_ptrs[0], host_buf.data(), host_buf.size() * sizeof(float), cudaMemcpyHostToDevice, stream));
    context->setBindingDimensions(0, nvinfer1::Dims{4, {n, 3, height, width}});
    auto t_h2d0 = std::chrono::steady_clock::now();

    static int pre_cnt = 0;
    static double pre_letterbox_ms = 0.0, pre_h2d_ms = 0.0;
    pre_cnt++;
    pre_letterbox_ms += std::chrono::duration<double, std::milli>(t_copy0 - t_letterbox0).count();
    pre_h2d_ms += std::chrono::duration<double, std::milli>(t_h2d0 - t_copy0).count();
    if (pre_cnt % 20 == 1) {
        LOG_DEBUG("[nvidia-pre] batch=%d  letterbox+pack=%.2fms  h2d=%.2fms  total=%.2fms  (avg over %d)",
                 n, pre_letterbox_ms / pre_cnt, pre_h2d_ms / pre_cnt,
                 (pre_letterbox_ms + pre_h2d_ms) / pre_cnt, pre_cnt);
        pre_letterbox_ms = pre_h2d_ms = 0.0;
    }

    return 0;
}

int YoloV8_det::forward()
{
    if (!is_initialized_) {
        std::cerr << "[ERROR] NVIDIA detector not initialized" << std::endl;
        return -1;
    }
    int cur_batch = (int)pparam_batch_.size();

    auto t0 = std::chrono::steady_clock::now();
    context->enqueueV2(device_ptrs.data(), stream, nullptr);
    auto t1 = std::chrono::steady_clock::now();

    for (int i = 0; i < num_outputs; i++) {
        size_t osize = output_bindings[i].size * output_bindings[i].dsize;
        CHECK(cudaMemcpyAsync(
            host_ptrs[i], device_ptrs[i + num_inputs], osize, cudaMemcpyDeviceToHost, stream));
    }
    auto t2 = std::chrono::steady_clock::now();

    cudaStreamSynchronize(stream);
    auto t3 = std::chrono::steady_clock::now();

    double enq = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double d2h = std::chrono::duration<double, std::milli>(t2 - t1).count();
    double sync = std::chrono::duration<double, std::milli>(t3 - t2).count();

    static int log_cnt = 0;
    log_cnt++;
    if (log_cnt % 20 == 1) {
        LOG_DEBUG("[nvidia-forward] batch=%d  enqueue=%.2fms  d2h_copy=%.2fms  sync_wait=%.2fms  total=%.2fms",
                 cur_batch, enq, d2h, sync, enq + d2h + sync);
    }
    return 0;
}

void YoloV8_det::export_batch_data(std::vector<PreParam>& pparams, std::vector<float>& output)
{
    pparams = pparam_batch_;
    if (!output_bindings.empty()) {
        size_t elems = output_bindings[0].size;
        const float* src = static_cast<const float*>(host_ptrs[0]);
        output.assign(src, src + elems);
    }
    else {
        output.clear();
    }
}

int YoloV8_det::post_process(const std::vector<PreParam>& pparams, const std::vector<float>& output,
                             std::vector<YoloV8BoxVec>& boxes, float score_thres, float iou_thres, int topk)
{
    if (!is_initialized_) {
        std::cerr << "[ERROR] NVIDIA detector not initialized" << std::endl;
        return -1;
    }
    int n = (int)pparams.size();
    boxes.assign(n, YoloV8BoxVec());

    int num_channels = output_bindings[0].dims.d[1];
    int num_anchors  = output_bindings[0].dims.d[2];
    int batch_dim    = output_bindings[0].dims.d[0];

    auto t_decode0 = std::chrono::steady_clock::now();
    static double s_nms_ms = 0.0;  // 本 batch 内 NMS 累计耗时（进程级静态，跨 batch 自清零）
    // 解码整批输出，按帧切片
    for (int b = 0; b < n; b++) {
        if (b >= batch_dim) break;

        auto& dw     = pparams[b].dw;
        auto& dh     = pparams[b].dh;
        auto& width  = pparams[b].width;
        auto& height = pparams[b].height;
        auto& ratio  = pparams[b].ratio;

        std::vector<cv::Rect> bboxes;
        std::vector<float>    scores;
        std::vector<int>      labels;
        std::vector<int>      indices;

        const float* frame_base = output.data() + (size_t)b * num_channels * num_anchors;
        cv::Mat out_mat = cv::Mat(num_channels, num_anchors, CV_32F, (void*)frame_base);
        out_mat = out_mat.t();
        for (int i = 0; i < num_anchors; i++) {
            auto  row_ptr    = out_mat.row(i).ptr<float>();
            auto  bboxes_ptr = row_ptr;
            auto  scores_ptr = row_ptr + 4;
            auto  max_s_ptr  = std::max_element(scores_ptr, scores_ptr + model_num_labels);
            float score      = *max_s_ptr;
            if (score > score_thres) {
                float x = *bboxes_ptr++ - dw;
                float y = *bboxes_ptr++ - dh;
                float w = *bboxes_ptr++;
                float h = *bboxes_ptr;

                float x0 = clamp((x - 0.5f * w) * ratio, 0.f, width);
                float y0 = clamp((y - 0.5f * h) * ratio, 0.f, height);
                float x1 = clamp((x + 0.5f * w) * ratio, 0.f, width);
                float y1 = clamp((y + 0.5f * h) * ratio, 0.f, height);

                int              label = max_s_ptr - scores_ptr;
                cv::Rect_<float> bb;
                bb.x      = x0;
                bb.y      = y0;
                bb.width  = x1 - x0;
                bb.height = y1 - y0;

                bboxes.push_back(bb);
                labels.push_back(label);
                scores.push_back(score);
            }
        }

        auto t_nms_b = std::chrono::steady_clock::now();
        cv::dnn::NMSBoxes(bboxes, scores, score_thres, iou_thres, indices);
        s_nms_ms += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_nms_b).count();

        int cnt = 0;
        for (auto& i : indices) {
            if (cnt >= topk) break;
            YoloV8Box box;
            box.x1 = bboxes[i].x;
            box.y1 = bboxes[i].y;
            box.x2 = bboxes[i].x + bboxes[i].width;
            box.y2 = bboxes[i].y + bboxes[i].height;
            box.score = scores[i];
            box.class_id = labels[i];
            boxes[b].push_back(box);
            cnt += 1;
        }
    }

    auto t_end_post = std::chrono::steady_clock::now();

    static int post_cnt = 0;
    static double post_decode_ms = 0.0, post_nms_ms = 0.0, post_total_ms = 0.0;
    post_cnt++;
    double this_decode = std::chrono::duration<double, std::milli>(t_end_post - t_decode0).count() - s_nms_ms;
    post_decode_ms += this_decode;
    post_nms_ms += s_nms_ms;
    post_total_ms += std::chrono::duration<double, std::milli>(t_end_post - t_decode0).count();
    s_nms_ms = 0.0;
    if (post_cnt % 20 == 1) {
        LOG_DEBUG("[nvidia-post] batch=%d  decode=%.2fms  nms=%.2fms  total=%.2fms  (avg over %d)",
                 n, post_decode_ms / post_cnt, post_nms_ms / post_cnt,
                 post_total_ms / post_cnt, post_cnt);
        post_decode_ms = post_nms_ms = post_total_ms = 0.0;
    }

    return 0;
}

int YoloV8_det::getBatchSize() const
{
    if (input_bindings.empty()) {
        return 1;
    }
    // 输入 binding 的 dims.d[0] 为 profile MAX batch
    int batch = input_bindings[0].dims.d[0];
    return batch > 0 ? batch : 1;
}

void YoloV8_det::draw_result(cv::Mat& img, YoloV8BoxVec& result)
{
    const std::vector<std::vector<int>> colors = {
        {255, 0, 0}, {255, 85, 0}, {255, 170, 0}, {255, 255, 0},
        {170, 255, 0}, {85, 255, 0}, {0, 255, 0}, {0, 255, 85},
        {0, 255, 170}, {0, 255, 255}, {0, 170, 255}, {0, 85, 255},
        {0, 0, 255}, {85, 0, 255}, {170, 0, 255}, {255, 0, 255},
        {255, 0, 170}, {255, 0, 85}};

    for (const auto& box : result) {
        int left = box.x1;
        int top = box.y1;
        int right = box.x2;
        int bottom = box.y2;

        int color_idx = box.class_id % colors.size();
        cv::Scalar color(colors[color_idx][0], colors[color_idx][1], colors[color_idx][2]);

        cv::rectangle(img, cv::Point(left, top), cv::Point(right, bottom), color, 2);

        std::string label = "class_" + std::to_string(box.class_id);
        label += ": " + std::to_string(box.score).substr(0, 4);

        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

        cv::rectangle(img,
                      cv::Point(left, top - text_size.height - 5),
                      cv::Point(left + text_size.width, top),
                      color, cv::FILLED);

        cv::putText(img, label,
                    cv::Point(left, top - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}