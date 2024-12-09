//
// Created by xuezhiyuan on 2024/11/29.
//

#ifndef LIVEBODYEXAMPLE_FACE_DETECTOR_H
#define LIVEBODYEXAMPLE_FACE_DETECTOR_H

#include <opencv2/core/mat.hpp>
#include "../include/ncnn/net.h"
#include "../definition.h"
#include "../android_log.h"


class FaceDetector {
public:
    FaceDetector();

    ~FaceDetector();

    void SetMinFaceSize(int size);

    int LoadModel(AAssetManager* assetManager);

    int Detect(cv::Mat& src, std::vector<FaceBox>& boxes);

private:
    ncnn::Net net_;
    int input_size_ = 192;
    const std::string net_input_name_ = "data";
    const std::string net_output_name_ = "detection_out";
    ncnn::Option option_;
    float threshold_;
    int thread_num_;
    const float mean_val_[3] = {104.f, 117.f, 123.f};
    int min_face_size_;
};



#endif //LIVEBODYEXAMPLE_FACE_DETECTOR_H
