//
// Created by xuezhiyuan on 2024/11/29.
//
#include "jni_long_field.h"
#include "definition.h"
#include "age/age_detector.h"
#include "img_process.h"
#include <android/asset_manager_jni.h>
#include <android/log.h>
#include <opencv2/opencv.hpp>

#define TAG "age_detector_jni.cpp"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__)

using namespace cv;

JniLongField age_detector_field("nativeHandler");

AgeDetector* get_age_detector(JNIEnv* env, jobject instance) {
    AgeDetector* const detector =
            reinterpret_cast<AgeDetector*>(age_detector_field.get(env, instance));
    return detector;
}

void set_age_detector(JNIEnv* env, jobject instance, AgeDetector* detector) {
    age_detector_field.set(env, instance, reinterpret_cast<intptr_t>(detector));
}

extern "C" {

JNIEXPORT jlong JNICALL
AGE_DETECTOR_METHOD(allocate)(JNIEnv *env, jobject instance);


JNIEXPORT void JNICALL
AGE_DETECTOR_METHOD(deallocate)(JNIEnv *env, jobject instance);

JNIEXPORT jint JNICALL
AGE_DETECTOR_METHOD(nativeLoadModel)(JNIEnv *env, jobject instance, jobject assets_manager);


JNIEXPORT jint JNICALL
AGE_DETECTOR_METHOD(nativeDetectBitmap)(JNIEnv *env, jobject instance, jobject bitmap);

}


JNIEXPORT jlong JNICALL
AGE_DETECTOR_METHOD(allocate)(JNIEnv *env, jobject instance) {
    auto * const detector = new AgeDetector();
    set_age_detector(env, instance, detector);
    return reinterpret_cast<intptr_t> (detector);
}

JNIEXPORT void JNICALL
AGE_DETECTOR_METHOD(deallocate)(JNIEnv *env, jobject instance) {
    delete get_age_detector(env, instance);
    set_age_detector(env, instance, nullptr);
}



JNIEXPORT jint JNICALL
AGE_DETECTOR_METHOD(nativeLoadModel)(JNIEnv *env, jobject instance, jobject assets_manager) {
    LOGD("AgeDetector, ageDetector =  --------------------------- LoadModel  ------------------------------------- ");
    AAssetManager* mgr = AAssetManager_fromJava(env, assets_manager);
    return get_age_detector(env, instance)->LoadModel(mgr);
}

JNIEXPORT jint JNICALL
AGE_DETECTOR_METHOD(nativeDetectBitmap)(JNIEnv *env, jobject instance, jobject bitmap) {
    LOGD("AgeDetector, ageDetector =  --------------------------- nativeDetectBitmap  ------------------------------------- ");
    //int ret = ConvertBitmap2Mat(env, bitmap, img);
    cv::Mat img = cv::imread("/sdcard/DCIM/36.jpg");
    if(img.empty()) {
        LOGD("AgeDetector, img.empty() --------------------- ");
        return 0;
    }

    cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

    // 调整图像大小到 (224, 224)
    cv::Mat resized_image;
    cv::resize(img, resized_image, cv::Size(224, 224));
    cv::imwrite("/sdcard/DCIM/37.jpg", resized_image);

    // 将 OpenCV Mat 转换为 [0, 1] 范围的浮点数数组
    resized_image.convertTo(resized_image, CV_32F, 1.0 / 255);

    // 创建 ncnn::Mat 并填充数据
    int channels = resized_image.channels();
    int height = resized_image.rows;
    int width = resized_image.cols;

    ncnn::Mat in(224, 224, 3);
    for (int c = 0; c < channels; ++c) {
        float* ptr = in.channel(c);
        for (int h = 0; h < height; ++h) {
            for (int w = 0; w < width; ++w) {
                ptr[h * width + w] = resized_image.at<cv::Vec3f>(h, w)[c];
            }
        }
    }
//
    // 添加批次维度
    ncnn::Mat in_with_batch = in.clone();
    //in_with_batch.reshape(224,224,3,1);

    //ncnn::Mat in = ncnn::Mat::from_pixels(resized_image.data, ncnn::Mat::PIXEL_BGR, width, height, 1);

    //ncnn::Mat in = ncnn::Mat::from_pixels(img.data, ncnn::Mat::PIXEL_BGR, img.cols,img.rows);
    LOGD("AgeDetector, ageDetector = --------------------------- img.cols = %d  img.rows = %d ", resized_image.cols, resized_image.rows);
    auto detectAgeStart = std::chrono::high_resolution_clock::now();
    int age = get_age_detector(env, instance)->DetectAge(in_with_batch);
    auto detectAgeEnd = std::chrono::high_resolution_clock::now();
    auto age_duration = std::chrono::duration_cast<std::chrono::microseconds>(detectAgeEnd - detectAgeStart);
    long long age_duration_ms = age_duration.count() / 1000;
    LOGD("AgeDetector ageDetector cost = %lld ms ", age_duration_ms);
    return age;
}
