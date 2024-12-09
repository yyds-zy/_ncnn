//
// Created by xuezhiyuan on 2024/11/29.
//
#include <iostream>
#include "jni_long_field.h"
#include "definition.h"
#include "img_process.h"
#include <android/asset_manager_jni.h>
#include <vector>
#include <eigen-3.3.9/Eigen/Dense>
#include <eigen-3.3.9/Eigen/Geometry>
#include <eigen-3.3.9/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
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
//
//// 定义仿射变换估计函数
Eigen::Affine2f estimateAffinePartial2DTemp(const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst) {
    if (src.size() != dst.size() || src.empty() || src.size() < 3) {
        throw std::invalid_argument("Source and destination point sets must be of the same size and contain at least 3 points.");
    }

    Eigen::Matrix2f H = Eigen::Matrix2f::Zero(); // 使用单精度浮点数
    Eigen::Vector2f src_mean, dst_mean;
    src_mean.setZero();
    dst_mean.setZero();

    // 计算均值
    for (size_t i = 0; i < src.size(); ++i) {
        src_mean += Eigen::Vector2f(src[i].x, src[i].y);
        dst_mean += Eigen::Vector2f(dst[i].x, dst[i].y);
    }
    src_mean /= src.size();
    dst_mean /= src.size();

    // 构建矩阵H
    for (size_t i = 0; i < src.size(); ++i) {
        Eigen::Vector2f src_centered(src[i].x - src_mean[0], src[i].y - src_mean[1]);
        Eigen::Vector2f dst_centered(dst[i].x - dst_mean[0], dst[i].y - dst_mean[1]);
        H += src_centered * dst_centered.transpose();
    }

    // 使用奇异值分解(SVD)求解H
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2f U = svd.matrixU();
    Eigen::Matrix2f V = svd.matrixV();

    // 计算仿射变换矩阵
    Eigen::Affine2f affine_transform = Eigen::Affine2f::Identity();
    Eigen::Matrix2f rotation = U * V.transpose();

    // 确保行列式为正，以保持方向一致
    if (rotation.determinant() < 0) {
        U.col(1) = -U.col(1);
        rotation = U * V.transpose();
    }

    affine_transform.linear() = rotation;
    Eigen::Vector2f translation = dst_mean - rotation * src_mean;
    affine_transform.translation() = translation;

    return affine_transform;
}

Eigen::Matrix<float, 2, 3> estimateAffinePartial2DTmp(const std::vector<Eigen::Vector2f>& dst, const std::vector<Eigen::Vector2f>& src) {
    Eigen::MatrixXf A(src.size() * 2, 6);
    Eigen::VectorXf B(src.size() * 2);
    Eigen::Matrix<float, 2, 3> M;

    for (size_t i = 0; i < src.size(); ++i) {
        const Eigen::Vector2f& s = src[i];
        const Eigen::Vector2f& d = dst[i];
        A(2 * i, 0) = s[0];
        A(2 * i, 1) = s[1];
        A(2 * i, 2) = 1;
        A(2 * i, 3) = 0;
        A(2 * i, 4) = 0;
        A(2 * i, 5) = 0;
        B(2 * i) = d[0];

        A(2 * i + 1, 0) = 0;
        A(2 * i + 1, 1) = 0;
        A(2 * i + 1, 2) = 0;
        A(2 * i + 1, 3) = s[0];
        A(2 * i + 1, 4) = s[1];
        A(2 * i + 1, 5) = 1;
        B(2 * i + 1) = d[1];
    }

    Eigen::VectorXf x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    M << x[0], x[1], x[2],
            x[3], x[4], x[5];

    return M;
}

cv::Mat affineFaceTransformation(const std::vector<Point2f>& srcPoints, const std::vector<Point2f>& dstPoints) {
    // 检查关键点数量是否正确
    if (srcPoints.size() != 5 || dstPoints.size() != 5) {
        throw std::runtime_error("需要5个源关键点和5个目标关键点");
    }

    // 选择三个关键点来计算仿射变换矩阵
    // 这里我们选择了前三个点，但你可以根据需要选择任意三个点
    std::vector<Point2f> srcPointsSelected = {srcPoints[0], srcPoints[1], srcPoints[2]};
    std::vector<Point2f> dstPointsSelected = {dstPoints[0], dstPoints[1], dstPoints[2]};

    // 创建仿射变换矩阵
    cv::Mat transformationMatrix = cv::getAffineTransform(
            cv::Mat(srcPointsSelected).reshape(2, 3), // 将std::vector<Point2f>转换为cv::Mat
            cv::Mat(dstPointsSelected).reshape(2, 3)
    );

    return transformationMatrix;
}

Eigen::Matrix2f affineFaceTransformationEigen(const std::vector<Point2f>& srcPoints, const std::vector<Point2f>& dstPoints) {
    // 检查关键点数量是否正确
    if (srcPoints.size() != 5 || dstPoints.size() != 5) {
        throw std::runtime_error("需要5个源关键点和5个目标关键点");
    }

    // 将关键点转换为Eigen::Vector2f
    Eigen::MatrixXf src(2, 5);
    Eigen::MatrixXf dst(2, 5);
    for (int i = 0; i < 5; ++i) {
        src(0, i) = srcPoints[i].x;
        src(1, i) = srcPoints[i].y;
        dst(0, i) = dstPoints[i].x;
        dst(1, i) = dstPoints[i].y;
    }

    // 使用Eigen的函数计算仿射变换矩阵
    Eigen::Matrix2f tform = Eigen::umeyama(src, dst, true);

    return tform;
}


std::vector<Point2f> applyAffineTransform(const std::vector<Point2f>& points, const Eigen::Matrix2f& tform) {
    std::vector<Point2f> transformedPoints;
    for (const auto& p : points) {
        Eigen::Vector2f point(p.x, p.y);
        Eigen::Vector2f transformedPoint = tform * point;
        transformedPoints.push_back({transformedPoint(0), transformedPoint(1)});
    }
    return transformedPoints;
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
    get_key_point_detector(env, instance)->detectMaxFace(in, faces);

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
    // LOGD("xuezhiyuan mtcnn detectFace cost = %lld ms ", duration_ms);

    std::cout << "xuezhiyuan face detection duration: " << duration_ms << " ms" << std::endl;
    //cv::imwrite("/sdcard/DCIM/Camera/bgr.jpg", bgr);


    // 得到List<FaceBox>  boxes
    for (const Bbox &face: faces) {
        FaceBox box;
        box.confidence = face.score;
        box.x1 = face.x1;
        box.y1 = face.y1;
        box.x2 = face.x2;
        box.y2 = face.y2;
        boxes.push_back(box);
    }
    cv::Mat faceROI;
    if (faces.size() > 0) {
        const FaceBox &box = boxes[0];
        faceROI = bgr(cv::Rect(box.x1, box.y1, box.x2 - box.x1, box.y2 - box.y1)); // 裁剪图像
        cv::resize(faceROI, faceROI, cv::Size(224, 244));
//        std::string filename = "/sdcard/DCIM/face.jpg"; // 构造文件名
//        cv::imwrite(filename, faceROI); // 保存裁剪后的图像
//        cv::resize(faceROI, faceROI, cv::Size(120, 100));
//        cv::imwrite("/sdcard/DCIM/Camera/face-128.jpg", faceROI); // 保存裁剪后的图像


    } else {
        faceROI = bgr;
    }

    int32_t num_face = static_cast<int32_t>(faces.size());
    if (num_face > 0) {
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

        // 将 cv::Point2f 类型的向量转换为 Eigen::Vector2f 类型的向量
//        std::vector<Eigen::Vector2f> dst_eigen;
//        std::vector<Eigen::Vector2f> src_eigen;
//        for (const auto& point : dst) {
//            dst_eigen.push_back(Eigen::Vector2f(point.x, point.y));
//        }
//        for (const auto& point : src) {
//            src_eigen.push_back(Eigen::Vector2f(point.x, point.y));
//        }

        //Eigen::Affine2f M = estimateAffinePartial2DTmp(src, dst);
        //Eigen::Matrix<float, 2, 3> M = estimateAffinePartial2DTmp(dst_eigen, src_eigen);
        //cv::Mat cvMat = cv::Mat(2, 3, CV_32F, M.data());
        //cv::Mat cvMat = affineFaceTransformation(src,dst);
        //Eigen::Matrix2f M = affineFaceTransformationEigen(src,dst);
        //cv::Mat cvMat = cv::Mat(2, 3, CV_32F, M.data());
        //std::vector<Point2f> transformedPoints = applyAffineTransform(srcPoints, tform);

        //cv::Mat cvMat = cv::getAffineTransform(src, dst);
//        std::vector<float> values;
//        values.reserve(6); // 预留空间，避免多次分配
//        values.push_back(M(0, 0)); // 旋转和缩放
//        values.push_back(M(0, 1)); // 旋转和缩放
//        values.push_back(M(0, 2)); // 平移
//        values.push_back(M(1, 0)); // 旋转和缩放
//        values.push_back(M(1, 1)); // 旋转和缩放
//        values.push_back(M(1, 2)); // 平移
//
//        // 3. 创建 cv::Mat，并用 vector 赋值
//        cv::Mat cvMat(2, 3, CV_32F, values.data());

        // 应用仿射变换
//        cv::Mat warped;
//        cv::warpAffine(faceROI, warped, cvMat, cv::Size(120, 100), cv::INTER_LINEAR,  cv::BORDER_CONSTANT, cv::Scalar());
//        cv::resize(warped, warped, cv::Size(224, 244));
//        cv::imwrite("/sdcard/DCIM/Camera/warped.jpg", warped);

//        Eigen::Matrix<float, 2, 3> M = estimateAffinePartial2DTmp(dst_eigen, src_eigen);
//
//        // 将Eigen矩阵转换为cv::Mat对象
//        cv::Mat cvMat = cv::Mat(2, 3, CV_32F, M.data());
//        cv::imwrite("/sdcard/DCIM/Camera/cvMat.jpg", cvMat);
//
//        cv::Mat warped;
//        cv::warpAffine(bgr, warped, cvMat, cv::Size(120, 100), cv::INTER_LINEAR,  cv::BORDER_CONSTANT, cv::Scalar());
//        cv::resize(warped, warped, cv::Size(256, 256));
//        cv::imwrite("/sdcard/DCIM/Camera/warped.jpg", warped);
    }

    env->ReleaseByteArrayElements(yuv, yuv_, 0);

    return ConvertFaceBoxVector2ListTmp(env, boxes);
}



//    // 存储人脸关键点信息
//    int32_t num_face = static_cast<int32_t>(faces.size());
//    if (num_face > 0) {
//        int out_size = 15;
//        int *faceInfo = new int[out_size];
//        faceInfo[0] = num_face;
//        faceInfo[1] = faces[0].x1;//left
//        faceInfo[2] = faces[0].y1;//top
//        faceInfo[3] = faces[0].x2;//right
//        faceInfo[4] = faces[0].y2;//bottom
//        for (int j = 0; j < 10; j++) {
//            faceInfo[5 + j] = static_cast<int>(faces[0].ppoint[j]);  // keypoint location
//        }
//
//        vector<Point2f> dst;
//        dst.push_back(Point2f(faceInfo[5], faceInfo[10]));
//        dst.push_back(Point2f(faceInfo[6], faceInfo[11]));
//        dst.push_back(Point2f(faceInfo[7], faceInfo[12]));
//        dst.push_back(Point2f(faceInfo[8], faceInfo[13]));
//        dst.push_back(Point2f(faceInfo[9], faceInfo[14]));
//
////        // 已经通过ncnn  人脸检测模型  得到人脸关键点
////        for (const auto &point: dst) {
////            LOGD("xuezhiyuan Point: (x=%f y=%f) ", point.x, point.y);
////        }
//        get_key_point_detector(env, instance)->drawDetection(rgb, faces);
//        cv::imwrite("/sdcard/DCIM/Camera/rgb.jpg", rgb);
//        const vector<cv::Point2f> src = {
//                {30.2946f, 51.6963f},
//                {65.5318f, 51.5014f},
//                {48.0252f, 71.7366f},
//                {33.5493f, 92.3655f},
//                {62.7299f, 92.2041f}
//        };
//
//        // 估计相似变换
//        //cv::Mat M = cv::estimateAffinePartial2D(dst, src);
//        Eigen::Affine2f M = estimateAffinePartial2D(src, dst);
//
//        std::vector<float> values;
//        values.reserve(6); // 预留空间，避免多次分配
//        values.push_back(M(0, 0)); // 旋转和缩放
//        values.push_back(M(0, 1)); // 旋转和缩放
//        values.push_back(M(0, 2)); // 平移
//        values.push_back(M(1, 0)); // 旋转和缩放
//        values.push_back(M(1, 1)); // 旋转和缩放
//        values.push_back(M(1, 2)); // 平移
//
//        // 3. 创建 cv::Mat，并用 vector 赋值
//        cv::Mat cvMat(2, 3, CV_32F, values.data());
//
//        // 应用仿射变换
//        cv::Mat warped;
//        cv::warpAffine(rgb, warped, cvMat, cv::Size(preview_width, preview_height), cv::INTER_LINEAR,  cv::BORDER_CONSTANT, cv::Scalar());
//        cv::imwrite("/sdcard/DCIM/Camera/warped.jpg", warped);
        //ncnn::Mat ageMat = ncnn::Mat::from_pixels(warped.data, ncnn::Mat::PIXEL_BGR, warped.cols, warped.rows);
        //delete[] faceInfo;
//    }

//    env->ReleaseByteArrayElements(yuv, yuv_, 0);
//    return ConvertFaceBoxVector2ListTmp(env, boxes);
//}


