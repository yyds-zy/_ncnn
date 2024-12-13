//
// Created by xuezhiyuan on 2024/11/29.
//
#include "jni_long_field.h"
#include "definition.h"
#include "age/age_detector.h"
#include "img_process.h"
#include <android/asset_manager_jni.h>
#include <android/log.h>

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
    return 0;
}
