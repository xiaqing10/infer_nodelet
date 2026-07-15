#include "detector_nvidia.hpp"

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