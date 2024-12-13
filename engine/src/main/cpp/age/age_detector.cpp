//
// Created by xuezhiyuan on 2024/11/29.
//
#include "age_detector.h"

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
    ncnn_img = img_;
    // 设置输入
    ncnn::Extractor ex = net_.create_extractor();
    ex.input("input.1", ncnn_img);
    // 执行推理
    ncnn::Mat out;
    ex.extract("664", out);
    const float* data = out.row(0);
    float result = data[0];
    LOG_ERR("FaceDetector xuezhiyuan  age  %f  ", result);
    int age = 0;
    if (data != nullptr) {
        age = static_cast<int>(result);
        LOG_ERR("FaceDetector xuezhiyuan  age  %d  ", age);
    }
    return age;
}