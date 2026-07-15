#include "detector.hpp"

#if USE_CAMBRICON
using namespace cn;
using namespace std;

void Detector::init(string model, int mlu_infer_device, int _num_class, int _stride){
    net.init(model, mlu_infer_device);
    num_cls = _num_class;
    feature_stride = _stride;
}

void Detector::yolov8_nms(std::vector<std::vector<float>>& boxes) {
    int length = boxes.size();
    int index = length - 1;
    sort(boxes.begin(), boxes.end(), [](const std::vector<float> &a, const std::vector<float> &b) {return a[1]<b[1];});
    std::vector<float> areas(length);

    for (int i=0; i<length; i++) {
        float width = boxes[i][4] - boxes[i][2];
        float height = boxes[i][5] - boxes[i][3];
        areas[i] = width * height;
    }

    while (index > 0) {
        int i = 0;
        while (i < index) {
            float left = std::max(boxes[index][2], boxes[i][2]);
            float top = std::max(boxes[index][3], boxes[i][3]);
            float right = std::min(boxes[index][4], boxes[i][4]);
            float bottom = std::min(boxes[index][5], boxes[i][5]);
            float overlap_area = std::max(0.0f, right - left) * std::max(0.0f, bottom - top);
            float union_area = areas[index] + areas[i] - overlap_area;
            if (overlap_area / union_area > nms_thred) {
                areas.erase(areas.begin() + i);
                boxes.erase(boxes.begin() + i);
                index--;
            }
            else {i++;}
        }
        index--;
    }
}

float Detector::letterbox(
    const cv::Mat& image,
    cv::Mat& out_image,
    const cv::Size& new_shape,
    const cv::Scalar& color
)
{
    cv::Size shape = image.size();
    float r = std::min((float)new_shape.height / (float)shape.height, (float)new_shape.width / (float)shape.width);
    int newUnpad[2]{(int)std::round((float)shape.width * r), (int)std::round((float)shape.height * r) };
    cv::Mat tmp;
    if (shape.width != newUnpad[0] || shape.height != newUnpad[1]) {
        cv::resize(image, tmp, cv::Size(newUnpad[0], newUnpad[1]));
    }
    else {
        tmp = image.clone();
    }
    float dw = new_shape.width - newUnpad[0];
    float dh = new_shape.height - newUnpad[1];
    dw /= 2.0f;
    dh /= 2.0f;
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(tmp, out_image, top, bottom, left, right, cv::BORDER_CONSTANT, color);
    return 1.0f / r;
}

std::vector<DetectorRetData> Detector::inference(cv::Mat &img) {
    std::vector<DetectorRetData> infer_result;
    int h = net.input_dims(1);
    int w = net.input_dims(2);
    float *input = (float*)malloc(h * w * net.input_dims(3)*sizeof(float));
    float *box_output = (float*)malloc(net.output_dims(1,0)*net.output_dims(3,0)*sizeof(float));
    float *cls_output = (float*)malloc(net.output_dims(1,1)*net.output_dims(3,1)*sizeof(float));

    float img_w = img.cols;
    float img_h = img.rows;
    cv::Mat img_resize;

    float scale = letterbox(img, img_resize, cv::Size(w, h), cv::Scalar(114, 114, 114));

    cv::cvtColor(img_resize, img_resize, cv::COLOR_BGR2RGB);
    cv::Mat img_cvt(h,w,CV_32FC3,input);
    img_resize.convertTo(img_cvt,CV_32F);
    img_cvt /= 255.0;
    CN_CHECK(net.input_from(input));
    CN_CHECK(net.infer());
    CN_CHECK(net.output_to(box_output, 0));
    CN_CHECK(net.output_to(cls_output, 1));
    std::vector<int> stride;
    int net_grid_w = 0, net_grid_h = 0;
    if (feature_stride == 4)
        stride = {4, 8, 16, 32};
    else stride = { 8, 16, 32};

    std::map<int, std::vector<POINT_2D_S>> centers_points;
    std::vector<std::vector<float>> tmp_result;
    float* box_output_copy = box_output;
    float* cls_output_copy = cls_output;
    for (int n=0; n<3; n++) {
        if (box_output_copy != nullptr && cls_output_copy != nullptr) {
            box_output_copy += net_grid_w * net_grid_h * 4;
            cls_output_copy += net_grid_w * net_grid_h * num_cls;
            net_grid_h = net_grid_w = 640/stride[n];
            for (int k = 0; k < net_grid_h; ++k) {
                for (int j = 0; j < net_grid_w; ++j) {
                    POINT_2D_S yolox_center;
                    yolox_center.cx = j + 0.5;
                    yolox_center.cy = k + 0.5;
                    centers_points[stride[n]].push_back(yolox_center);
                }
            }
            std::vector<POINT_2D_S> all_points = centers_points[stride[n]];
            for (int i = 0; i < net_grid_w * net_grid_h; ++i) {
                const float* temp_d = box_output_copy + i * 4;
                const float* temp_c = cls_output_copy + i * num_cls;

                int topClass = 0;
                for (int j = 0; j < num_cls; ++j) {
                    if (temp_c[j] > temp_c[topClass]) {
                        topClass = j;
                    }
                }
                if (temp_c[topClass] < conf_thred) continue;

                float sub_x = all_points[i].cx - temp_d[0];
                float sub_y = all_points[i].cy - temp_d[1];
                float add_w = temp_d[2] + all_points[i].cx;
                float add_h = temp_d[3] + all_points[i].cy;

                float x_center = ((sub_x + add_w) * 0.5f) * stride[n];
                float y_center = ((sub_y + add_h) * 0.5f) * stride[n];
                float w = (add_w - sub_x) * stride[n];
                float h = (add_h - sub_y) * stride[n];

                if (w < 5.f || h < 5.f) continue;

                std::vector<float> single_result;
                single_result.push_back(topClass);
                single_result.push_back(temp_c[topClass]);
                float xmin = std::max((x_center - w * 0.5f), 0.f);
                float ymin = std::max((y_center - h * 0.5f), 0.f);
                float xmax = std::min((x_center + w * 0.5f), float(640));
                float ymax = std::min((y_center + h * 0.5f), float(640));
                single_result.push_back(xmin);
                single_result.push_back(ymin);
                single_result.push_back(xmax);
                single_result.push_back(ymax);

                tmp_result.push_back(single_result);
            }
        }
    }

    if (tmp_result.size() != 0) {
        yolov8_nms(tmp_result);
        int x_offset = (w * scale - img_w) / 2;
        int y_offset = (h * scale - img_h) / 2;
        for (unsigned int i=0; i<tmp_result.size(); i++){
            DetectorRetData single_det_result;

            single_det_result.label = tmp_result[i][0] += 1;
            single_det_result.confidence = tmp_result[i][1];
            single_det_result.xmin  = tmp_result[i][2] * scale - x_offset;
            single_det_result.ymin  = tmp_result[i][3] * scale - y_offset;
            single_det_result.xmax = tmp_result[i][4] * scale - x_offset;
            single_det_result.ymax = tmp_result[i][5] * scale - y_offset;

            infer_result.push_back(single_det_result);
        }
    }

    free(input);
    free(box_output);
    free(cls_output);

    return infer_result;
}

#elif USE_SOPHON

void Detector::init(std::string model, int device_id, int num_class, int stride) {
    (void)device_id;
    (void)num_class;
    (void)stride;
    yolo_det.Init(model);
}

std::vector<DetectorRetData> Detector::inference(cv::Mat &img) {
    std::vector<DetectorRetData> infer_result;
    int batch_size = yolo_det.batch_size;
    std::vector<bm_image> batch_imgs;
    std::vector<YoloV8BoxVec> boxes;

    bm_image bmimg;
    bm_image_create(yolo_det.handle, img.rows, img.cols, FORMAT_BGR_PACKED, DATA_TYPE_EXT_1N_BYTE, &bmimg);
    bm_image_alloc_dev_mem(bmimg, BMCV_IMAGE_FOR_IN);
    void *buffer[1] = {static_cast<void*>(img.data)};
    bm_image_copy_host_to_device(bmimg, buffer);
    batch_imgs.push_back(bmimg);
    yolo_det.Detect(batch_imgs, boxes, 0);

    for (int i = 0; i < batch_size; i++) {
        for (size_t j = 0; j < boxes[i].size(); j++) {
            YoloV8Box box = boxes[i][j];
            DetectorRetData d;
            d.label = box.class_id;
            d.confidence = box.score;
            d.xmin = (int)box.x1;
            d.ymin = (int)box.y1;
            d.xmax = (int)box.x2;
            d.ymax = (int)box.y2;
            if (d.xmin < 0 || d.ymin < 0 || d.xmax > img.cols || d.ymax > img.rows) continue;
            infer_result.push_back(d);
        }
    }
    bm_image_destroy(batch_imgs[0]);

    return infer_result;
}

#elif USE_ASCEND

void Detector::init(std::string model, int device_id, int num_class, int stride) {
    (void)num_class;
    (void)stride;
    yolo_det.Init(model, device_id);
}

std::vector<DetectorRetData> Detector::inference(cv::Mat &img) {
    std::vector<DetectorRetData> infer_result;
    YoloV8BoxVec boxes;
    yolo_det.Detect(img, boxes, 0);
    for (size_t j = 0; j < boxes.size(); j++) {
        YoloV8Box box = boxes[j];
        DetectorRetData d;
        d.label = box.class_id;
        d.confidence = box.score;
        d.xmin = (int)box.x1;
        d.ymin = (int)box.y1;
        d.xmax = (int)box.x2;
        d.ymax = (int)box.y2;
        if (d.xmin < 0 || d.ymin < 0 || d.xmax > img.cols || d.ymax > img.rows) continue;
        infer_result.push_back(d);
    }
    return infer_result;
}

#elif USE_NVIDIA

void Detector::init(std::string model, int device_id, int num_class, int stride) {
    (void)device_id;
    (void)stride;
    yolo_det.Init(model, num_class);
}

std::vector<DetectorRetData> Detector::inference(cv::Mat &img) {
    std::vector<DetectorRetData> infer_result;
    YoloV8BoxVec boxes;
    yolo_det.Detect(img, boxes, 0);
    for (size_t j = 0; j < boxes.size(); j++) {
        YoloV8Box box = boxes[j];
        DetectorRetData d;
        d.label = box.class_id + 1;
        d.confidence = box.score;
        d.xmin = (int)box.x1;
        d.ymin = (int)box.y1;
        d.xmax = (int)box.x2;
        d.ymax = (int)box.y2;
        if (d.xmin < 0 || d.ymin < 0 || d.xmax > img.cols || d.ymax > img.rows) continue;
        infer_result.push_back(d);
    }
    return infer_result;
}

#endif
