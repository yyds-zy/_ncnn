#include "jni_long_field.h"
#include "definition.h"
#include "mtcnn/mtcnn.h"
#include "img_process.h"
#include <android/asset_manager_jni.h>
#include <android/log.h>

#define TAG "mtcnn_detector_jni.cpp"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__)

using namespace cv;



JniLongField age_detector_field("nativeHandler");

MTCNN* get_age_detector(JNIEnv* env, jobject instance) {
    MTCNN* const detector =
            reinterpret_cast<MTCNN*>(age_detector_field.get(env, instance));
    return detector;
}

void set_age_detector(JNIEnv* env, jobject instance, MTCNN* detector) {
    age_detector_field.set(env, instance, reinterpret_cast<intptr_t>(detector));
}

extern "C" {

JNIEXPORT jlong JNICALL
AGE_DETECTOR_METHOD(allocate)(JNIEnv *env, jobject instance);


JNIEXPORT void JNICALL
AGE_DETECTOR_METHOD(deallocate)(JNIEnv *env, jobject instance);

JNIEXPORT void JNICALL
AGE_DETECTOR_METHOD(nativeLoadModel)(JNIEnv *env, jobject instance, jobject assets_manager);


JNIEXPORT void JNICALL
AGE_DETECTOR_METHOD(nativeDetectBitmap)(JNIEnv *env, jobject instance, jobject bitmap);

}


JNIEXPORT jlong JNICALL
AGE_DETECTOR_METHOD(allocate)(JNIEnv *env, jobject instance) {
    auto * const detector = new MTCNN();
    set_age_detector(env, instance, detector);
    return reinterpret_cast<intptr_t> (detector);
}

JNIEXPORT void JNICALL
AGE_DETECTOR_METHOD(deallocate)(JNIEnv *env, jobject instance) {
    delete get_age_detector(env, instance);
    set_age_detector(env, instance, nullptr);
}



JNIEXPORT void JNICALL
AGE_DETECTOR_METHOD(nativeLoadModel)(JNIEnv *env, jobject instance, jobject assets_manager) {
    LOGD("MTCNN, mtcnn =  --------------------------- LoadModel  ------------------------------------- ");

    AAssetManager* mgr = AAssetManager_fromJava(env, assets_manager);
    get_age_detector(env, instance)->LoadModel(mgr);
    get_age_detector(env, instance)->SetMinFace(64);
}

JNIEXPORT void JNICALL
AGE_DETECTOR_METHOD(nativeDetectBitmap)(JNIEnv *env, jobject instance, jobject bitmap) {
    LOGD("MTCNN, mtcnn =  --------------------------- nativeDetectBitmap  ------------------------------------- ");
    cv::Mat img;
    int ret = ConvertBitmap2Mat(env, bitmap, img);
    std::vector<Bbox> faces;
    ncnn::Mat in = ncnn::Mat::from_pixels(img.data, ncnn::Mat::PIXEL_BGR, img.cols,img.rows);
    LOGD("MTCNN, mtcnn = --------------------------- img.cols = %d  img.rows = %d ", img.cols, img.rows);
    get_age_detector(env, instance)->Detect(in, faces);
    int32_t num_face = static_cast<int32_t>(faces.size());

    if (num_face > 0) {
        LOGD("MTCNN, mtcnn = --------------------------- num_facing = %d ", num_face);
    }
}
