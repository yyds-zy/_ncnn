//
// Created by xuezhiyuan on 2024/11/29.
//
#include <iostream>
#include "jni_long_field.h"
#include "definition.h"
#include "img_process.h"
#include <android/asset_manager_jni.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include "face/mtcnn.h"

#define TAG "key_point_detector_jni"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__)

JniLongField key_point_detector_field("nativeHandler");

MTCNN* get_key_point_detector(JNIEnv* env, jobject instance) {
    MTCNN* const mtcnn =
            reinterpret_cast<MTCNN*>(key_point_detector_field.get(env, instance));
    return mtcnn;
}

void set_key_point_detector(JNIEnv* env, jobject instance, MTCNN* mtcnn) {
    key_point_detector_field.set(env, instance, reinterpret_cast<intptr_t>(mtcnn));
}


extern "C" {

JNIEXPORT jlong JNICALL
MTCNN_DETECTOR_METHOD(allocate)(JNIEnv *env, jobject instance);


JNIEXPORT void JNICALL
MTCNN_DETECTOR_METHOD(deallocate)(JNIEnv *env, jobject instance);

JNIEXPORT jint JNICALL
MTCNN_DETECTOR_METHOD(nativeLoadModel)(JNIEnv *env, jobject instance, jobject assets_manager);


JNIEXPORT jobject JNICALL
MTCNN_DETECTOR_METHOD(nativeDetectBitmap)(JNIEnv *env, jobject instance, jobject bitmap);


JNIEXPORT jobject JNICALL
MTCNN_DETECTOR_METHOD(nativeDetectYuv)(JNIEnv *env, jobject instance, jbyteArray yuv,
                                      jint preview_width, jint preview_height,jint orientation);

}

jobject ConvertFaceBoxVector2ListTmp(JNIEnv *env, std::vector<FaceBox>& boxes) {
    jclass list_clz = env->FindClass(JAVA_ARRAY_LIST_CLASSPATH);

    jmethodID init_method = env->GetMethodID(list_clz, "<init>", "()V");
    jmethodID add_method = env->GetMethodID(list_clz, "add", "(Ljava/lang/Object;)Z");

    jobject list = env->NewObject(list_clz, init_method);
    env->DeleteLocalRef(list_clz);

    jclass face_clz = env->FindClass(ANDROID_FACE_BOX_CLASSPATH);

    jmethodID face_init_method = env->GetMethodID(face_clz, "<init>", "(IIIIF)V");

    for (auto& box : boxes) {
        int left = static_cast<int>(box.x1);
        int top = static_cast<int>(box.y1);
        int right = static_cast<int>(box.x2);
        int bottom = static_cast<int>(box.y2);


        jobject face = env->NewObject(face_clz, face_init_method, left, top, right, bottom, 0.f);
        env->CallBooleanMethod(list, add_method, face);

        env->DeleteLocalRef(face);
    }

    env->DeleteLocalRef(face_clz);
    return list;
}


JNIEXPORT jlong JNICALL
MTCNN_DETECTOR_METHOD(allocate)(JNIEnv *env, jobject instance) {
    auto * const mtcnn = new MTCNN();
    set_key_point_detector(env, instance, mtcnn);
    return reinterpret_cast<intptr_t> (mtcnn);
}

JNIEXPORT void JNICALL
MTCNN_DETECTOR_METHOD(deallocate)(JNIEnv *env, jobject instance) {
    delete get_key_point_detector(env, instance);
    set_key_point_detector(env, instance, nullptr);
}



JNIEXPORT jint JNICALL
MTCNN_DETECTOR_METHOD(nativeLoadModel)(JNIEnv *env, jobject instance, jobject assets_manager) {
    AAssetManager* mgr = AAssetManager_fromJava(env, assets_manager);
    return get_key_point_detector(env, instance)->init(mgr);
}

JNIEXPORT jobject JNICALL
MTCNN_DETECTOR_METHOD(nativeDetectBitmap)(JNIEnv *env, jobject instance, jobject bitmap) {
    cv::Mat img;
    int ret = ConvertBitmap2Mat(env, bitmap, img);
    if(ret != 0)
        return nullptr;

    std::vector<FaceBox> boxes;
    std::vector<Bbox> faces;
    ncnn::Mat in = ncnn::Mat::from_pixels(img.data, ncnn::Mat::PIXEL_BGR2RGB, img.cols, img.rows);
    // mtcnn 检测人脸关键点信息
    get_key_point_detector(env, instance)->detect(in, faces);
    for (const Bbox &face: faces) {
        FaceBox box;
        box.confidence = face.score;
        box.x1 = face.x1;
        box.y1 = face.y1;
        box.x2 = face.x2;
        box.y2 = face.y2;
        boxes.push_back(box);
    }

    AndroidBitmap_unlockPixels(env, bitmap);

    if(boxes.empty()) return nullptr;

    return ConvertFaceBoxVector2ListTmp(env, boxes);
}

JNIEXPORT jobject JNICALL
MTCNN_DETECTOR_METHOD(nativeDetectYuv)(JNIEnv *env, jobject instance, jbyteArray yuv,
        jint preview_width, jint preview_height,jint orientation) {
    jbyte *yuv_ = env->GetByteArrayElements(yuv, nullptr);

    cv::Mat bgr;
    Yuv420sp2bgr(reinterpret_cast<unsigned char *>(yuv_), preview_width, preview_height,
                 orientation, bgr);

    std::vector<FaceBox> boxes;

    std::vector<Bbox> faces;
    ncnn::Mat in = ncnn::Mat::from_pixels(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, bgr.cols, bgr.rows);

    auto face_start = std::chrono::high_resolution_clock::now();
    // TODO  mtcnn 检测人脸关键点信息  开始计时
    get_key_point_detector(env, instance)->detectMaxFace(in, faces);
    // TODO mtcnn 结束人脸关键点检测计时
    auto face_end = std::chrono::high_resolution_clock::now();
    auto face_cost = std::chrono::duration_cast<std::chrono::microseconds>(face_end - face_start);
    long long duration_ms = face_cost.count() / 1000;
    std::cout << "xuezhiyuan face detection duration: " << duration_ms << " ms" << std::endl;

    LOGD("face num %d ", faces.size());
    // 得到List<FaceBox>  boxes
    for (const Bbox &face: faces) {
        FaceBox box;
        box.confidence = face.score;
        box.x1 = face.x1;
        std::cout << "xuezhiyuan face detection x1: " << face.x1 << std::endl;
        LOGD("face x1 %d ", face.x1);
        box.y1 = face.y1;
        LOGD("face y1 %d ", face.y1);
        box.x2 = face.x2;
        LOGD("face x2 %d ", face.x2);
        box.y2 = face.y2;
        LOGD("face y2 %d ", face.y2);
        boxes.push_back(box);
    }
    env->ReleaseByteArrayElements(yuv, yuv_, 0);

    return ConvertFaceBoxVector2ListTmp(env, boxes);
}