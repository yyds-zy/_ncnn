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
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
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
MTCNN_DETECTOR_METHOD(nativeLoadModel)(JNIEnv *env, jobject instance, jobject assets_manager,
                                       jobject configs);


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

    jmethodID face_init_method = env->GetMethodID(face_clz, "<init>", "(IIIIFI)V");

    for (auto& box : boxes) {
        int left = static_cast<int>(box.x1);
        int top = static_cast<int>(box.y1);
        int right = static_cast<int>(box.x2);
        int bottom = static_cast<int>(box.y2);

        jobject face = env->NewObject(face_clz, face_init_method, left, top, right, bottom, box.confidence, box.age);
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

void ConvertAndroidConfig2NativeConfigTmp(JNIEnv *env,jobject model_configs, std::vector<ModelConfig>& modelConfigs) {
    modelConfigs.clear();

    jclass list_clz = env->GetObjectClass(model_configs);
    jmethodID list_size = env->GetMethodID(list_clz, "size", "()I");
    jmethodID list_get = env->GetMethodID(list_clz, "get", "(I)Ljava/lang/Object;");

    env->DeleteLocalRef(list_clz);

    int len = env->CallIntMethod(model_configs, list_size);
    for(int i = 0; i < len; i++) {
        jobject config = env->CallObjectMethod(model_configs, list_get, i);
        jclass config_clz = env->GetObjectClass(config);
        jfieldID config_name         = env->GetFieldID(config_clz, "name" ,"Ljava/lang/String;");
        jfieldID config_width        = env->GetFieldID(config_clz, "width", "I");
        jfieldID config_height       = env->GetFieldID(config_clz, "height", "I");
        jfieldID config_scale        = env->GetFieldID(config_clz, "scale", "F");
        jfieldID config_shift_x      = env->GetFieldID(config_clz, "shift_x", "F");
        jfieldID config_shift_y      = env->GetFieldID(config_clz, "shift_y", "F");
        jfieldID config_org_resize   = env->GetFieldID(config_clz, "org_resize", "Z");

        env->DeleteLocalRef(config_clz);

        ModelConfig modelConfig;
        modelConfig.width       = env->GetIntField(config, config_width);
        modelConfig.height      = env->GetIntField(config, config_height);
        modelConfig.scale       = env->GetFloatField(config, config_scale);
        modelConfig.shift_x     = env->GetFloatField(config, config_shift_x);
        modelConfig.shift_y     = env->GetFloatField(config, config_shift_y);
        modelConfig.org_resize  = env->GetBooleanField(config, config_org_resize);
        jstring model_name_jstr   = static_cast<jstring>(env->GetObjectField(config, config_name));
        const char *name = env->GetStringUTFChars(model_name_jstr, 0);

        std::string nameStr(name);
        modelConfig.name = nameStr;
        modelConfigs.push_back(modelConfig);

        env->ReleaseStringUTFChars(model_name_jstr, name);

    }
}


JNIEXPORT jint JNICALL
MTCNN_DETECTOR_METHOD(nativeLoadModel)(JNIEnv *env, jobject instance, jobject assets_manager,
                                       jobject configs) {
    std::vector<ModelConfig> model_configs;
    ConvertAndroidConfig2NativeConfigTmp(env, configs, model_configs);

    AAssetManager* mgr = AAssetManager_fromJava(env, assets_manager);
    return get_key_point_detector(env, instance)->init(mgr, model_configs);
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

    // TODO 检测人脸关键点  STEP 1
    get_key_point_detector(env, instance)->detectMaxFace(in, faces);

    int32_t num_face = static_cast<int32_t>(faces.size());
    if (num_face > 0) {
        // 得到List<FaceBox>  boxes
        for (const Bbox &face: faces) {
            FaceBox box;
            box.x1 = face.x1;
            box.y1 = face.y1;
            box.x2 = face.x2;
            box.y2 = face.y2;

            // TODO 检测活体  STEP 2
            float confidence = get_key_point_detector(env, instance)->detectLive(bgr, box);
            box.confidence = confidence;
            LOGD("current live confidence is, confidence = %f", confidence);

            // TODO 年龄检测预处理  STEP 3
            int out_size = 15;
            int *faceInfo = new int[out_size];
            faceInfo[0] = num_face;
            faceInfo[1] = faces[0].x1;//left
            faceInfo[2] = faces[0].y1;//top
            faceInfo[3] = faces[0].x2;//right
            faceInfo[4] = faces[0].y2;//bottom
            for (int j = 0; j < 10; j++) {
                faceInfo[5 + j] = static_cast<int>(faces[0].ppoint[j]);  // keypoint location
            }
            vector<Point2f> dst;
            dst.push_back(Point2f(faceInfo[5], faceInfo[10]));
            dst.push_back(Point2f(faceInfo[6], faceInfo[11]));
            dst.push_back(Point2f(faceInfo[7], faceInfo[12]));
            dst.push_back(Point2f(faceInfo[8], faceInfo[13]));
            dst.push_back(Point2f(faceInfo[9], faceInfo[14]));

            const vector<cv::Point2f> src = {
                    {30.2946f, 51.6963f},
                    {65.5318f, 51.5014f},
                    {48.0252f, 71.7366f},
                    {33.5493f, 92.3655f},
                    {62.7299f, 92.2041f}
            };
            // 估计相似变换
            cv::Mat M = cv::estimateAffinePartial2D(dst, src);

            cv::Mat warped;
            cv::warpAffine(bgr, warped, M, cv::Size(120, 100), cv::INTER_LINEAR,  cv::BORDER_CONSTANT, cv::Scalar());
            cv::cvtColor(warped, warped, cv::COLOR_BGR2RGB);
            cv::resize(warped, warped, cv::Size(224, 224));


            warped.convertTo(warped, CV_32F, 1.0 / 255);

            // TODO 通道映射
            int channels = warped.channels();
            int height = warped.rows;
            int width = warped.cols;

            auto face_start = std::chrono::high_resolution_clock::now();
            ncnn::Mat in(224, 224, 3);
            for (int c = 0; c < channels; ++c) {
                float* ptr = in.channel(c);
                for (int h = 0; h < height; ++h) {
                    for (int w = 0; w < width; ++w) {
                        ptr[h * width + w] = warped.at<cv::Vec3f>(h, w)[c];
                    }
                }
            }
            auto face_end = std::chrono::high_resolution_clock::now();
            auto face_cost = std::chrono::duration_cast<std::chrono::microseconds>(face_end - face_start);
            float duration_ms = face_cost.count() / 1000.0;

            LOGD("current pre process age is cost time, cost = %f", duration_ms);

            //ncnn::Mat in = ncnn::Mat::from_pixels(warped.data, ncnn::Mat::PIXEL_RGB, warped.cols, warped.rows);

            //ncnn::Mat in_with_batch = in.clone();
            in.reshape(224,224,3,1);

            // TODO 年龄检测  STEP 3
            int age = get_key_point_detector(env, instance)->detectAge(in);
            LOGD("current people age is, age = %d", age);
            box.age = age;
            boxes.push_back(box);
        }
    }
    env->ReleaseByteArrayElements(yuv, yuv_, 0);

    return ConvertFaceBoxVector2ListTmp(env, boxes);
}