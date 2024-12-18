//
// Created by xuezhiyuan on 2024/11/29.
//
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "live.h"
#include "../android_log.h"
#define TAG "Live"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__)

Live::Live() {
    thread_num_ = 2;

    option_.lightmode = true;
    option_.num_threads = thread_num_;
}

Live::~Live() {
    for (int i = 0; i < nets_.size(); ++i) {
        nets_[i]->clear();
        delete nets_[i];
    }
    nets_.clear();
}

int Live::LoadModel(AAssetManager *assetManager, std::vector<ModelConfig> &configs) {
    configs_ = configs;
    model_num_ = static_cast<int>(configs_.size());
    for (int i = 0; i < model_num_; ++i) {
        ncnn::Net *net = new ncnn::Net();
        net->opt = option_;
        std::string param = "live/" + configs_[i].name + ".param";
        std::string model = "live/" + configs_[i].name + ".bin";
        int ret = net->load_param(assetManager, param.c_str());
        if (ret != 0) {
            LOG_ERR("LiveBody load param failed.");
            return -2 * (i) - 1;
        }

        ret = net->load_model(assetManager, model.c_str());
        if (ret != 0) {
            LOG_ERR("LiveBody load model failed.");
            return -2 * (i + 1);
        }
        nets_.emplace_back(net);
    }
    return 0;
}

float Live::Detect(cv::Mat &src, FaceBox &box) {
    auto start = std::chrono::high_resolution_clock::now();

    float confidence = 0.f;
    for (int i = 0; i < model_num_; i++) {
        cv::Mat roi;
        if(configs_[i].org_resize) {
            cv::resize(src, roi, cv::Size(configs_[i].width, configs_[i].height));
        } else {
            cv::Rect rect = CalculateBox(box, src.cols, src.rows, configs_[i]);
            // roi resize
            cv::resize(src(rect), roi, cv::Size(configs_[i].width, configs_[i].height));
        }

        ncnn::Mat in = ncnn::Mat::from_pixels(roi.data, ncnn::Mat::PIXEL_BGR, roi.cols, roi.rows);

        // inference
        ncnn::Extractor extractor = nets_[i]->create_extractor();
        extractor.set_light_mode(true);
        extractor.set_num_threads(thread_num_);

        extractor.input(net_input_name_.c_str(), in);
        ncnn::Mat out;
        extractor.extract(net_output_name_.c_str(), out);

        confidence += out.row(0)[1];
    }
    confidence /= model_num_;

    box.confidence = confidence;

    auto detectEnd = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(detectEnd - start);
    long long duration_ms = duration.count() / 1000;

//
//    cv::Mat faceMat;
//    // 检查box的坐标是否在src图像的边界内
////    int x1 = std::max(0, box.x1); // 确保x1不小于0
////    int y1 = std::max(0, box.y1); // 确保y1不小于0
////    int x2 = std::min(box.x2, src.cols - 1); // 确保x2不大于src的宽度
////    int y2 = std::min(box.y2, src.rows - 1); // 确保y2不大于src的高度
//
//    if (box.x1 >= 0 && box.y1 >= 0 && box.x2 <= src.cols - 1 && box.y2 <= src.rows - 1) {
//        int x1 = box.x1; // 确保x1不小于0
//        int y1 = box.y1; // 确保y1不小于0
//        int x2 = box.x2; // 确保x2不大于src的宽度
//        int y2 = box.y2; // 确保y2不大于src的高度
//        // 确保裁剪区域是有效的
//        if (x1 < x2 && y1 < y2) {
//            faceMat = src(cv::Rect(box.x1, box.y1, box.x2 - box.x1, box.y2 - box.y1));
//            cv::resize(faceMat, faceMat, cv::Size(224, 224));
//            cv::imwrite("/sdcard/DCIM/Camera/face.jpg", faceMat);
//        }
//    }

    LOGD("ncnn Detect confidence = %f", confidence);
    LOGD("ncnn Detect cost = %lld ms ", duration_ms);
    return confidence;
}

cv::Rect Live::CalculateBox(FaceBox &box, int w, int h, ModelConfig &config) {
    int x = static_cast<int>(box.x1);
    int y = static_cast<int>(box.y1);
    int box_width = static_cast<int>(box.x2 - box.x1 + 1);
    int box_height = static_cast<int>(box.y2 - box.y1 + 1);

    int shift_x = static_cast<int>(box_width * config.shift_x);
    int shift_y = static_cast<int>(box_height * config.shift_y);

    float scale = std::min(
            config.scale,
            std::min((w - 1) / (float) box_width, (h - 1) / (float) box_height)
    );

    int box_center_x = box_width / 2 + x;
    int box_center_y = box_height / 2 + y;

    int new_width = static_cast<int>(box_width * scale);
    int new_height = static_cast<int>(box_height * scale);

    int left_top_x = box_center_x - new_width / 2 + shift_x;
    int left_top_y = box_center_y - new_height / 2 + shift_y;
    int right_bottom_x = box_center_x + new_width / 2 + shift_x;
    int right_bottom_y = box_center_y + new_height / 2 + shift_y;

    if (left_top_x < 0) {
        right_bottom_x -= left_top_x;
        left_top_x = 0;
    }

    if (left_top_y < 0) {
        right_bottom_y -= left_top_y;
        left_top_y = 0;
    }

    if (right_bottom_x >= w) {
        int s = right_bottom_x - w + 1;
        left_top_x -= s;
        right_bottom_x -= s;
    }

    if (right_bottom_y >= h) {
        int s = right_bottom_y - h + 1;
        left_top_y -= s;
        right_bottom_y -= s;
    }

    return cv::Rect(left_top_x, left_top_y, new_width, new_height);
}
