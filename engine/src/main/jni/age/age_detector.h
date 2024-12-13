//
// Created by xuezhiyuan on 2024/11/29.
//
#ifndef DETECT_AGE_DETECTOR_H
#define DETECT_AGE_DETECTOR_H

#include <opencv2/imgproc.hpp>
#include "net.h"
#include "../definition.h"
#include "../android_log.h"


class AgeDetector{
public:
    AgeDetector();

    ~AgeDetector();

    int LoadModel(AAssetManager* assetManager);

    int DetectAge(ncnn::Mat& src);

private:
    ncnn::Net net_;
    ncnn::Mat ncnn_img;
    const float age_mean_vals[3] = {123.675, 116.28, 103.53};
    const float age_norm_vals[3] = {1.0 / 58.395, 1.0 / 57.12, 1.0 / 57.375};

    ncnn::Option option_;
    int thread_num_;
};


#endif //DETECT_AGE_DETECTOR_H
