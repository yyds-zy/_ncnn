//
// Created by xuezhiyuan on 2024/11/29.
//
#include "age_detector.h"
#include <ncnn/cpu.h>

AgeDetector::AgeDetector() : thread_num_(2) {
    option_.lightmode = true;
    option_.num_threads = thread_num_;
}

AgeDetector::~AgeDetector() {
    net_.clear();
}

int AgeDetector::LoadModel(AAssetManager* assetManager) {
    //net_.opt = option_;
    int ret = net_.load_param(assetManager, "age/age_v2.param");
    if(ret != 0) {
        LOG_ERR("FaceDetector load param failed. %d", ret);
        return -1;
    }

    ret = net_.load_model(assetManager, "age/age_v2.bin");
    if(ret != 0) {
        LOG_ERR("FaceDetector load model failed. %d", ret);
        return -2;
    }
    return 0;
}

int AgeDetector::DetectAge(ncnn::Mat &img_) {
    LOG_ERR("FaceDetector DetectAge 1.");
    ncnn_img = img_;
    //ncnn_img.substract_mean_normalize(age_mean_vals,age_norm_vals);
    //LOG_ERR("FaceDetector DetectAge 2.");
    // 设置输入
    ncnn::Extractor ex = net_.create_extractor();
    ex.input("input.1", ncnn_img);
    LOG_ERR("FaceDetector DetectAge 3.");
    // 执行推理
    ncnn::Mat out;
    ex.extract("664", out);
    LOG_ERR("FaceDetector DetectAge 4.");
    const float* data = out.row(0);
    LOG_ERR("FaceDetector DetectAge 5.");
    int age = 0;
    if (data != nullptr) {
        age = static_cast<int>(data[0]);
        LOG_ERR("FaceDetector xuezhiyuan  age  %d  ", age);
    }
    return age;
}