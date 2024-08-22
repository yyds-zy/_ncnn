#ifndef __MTCNN_NCNN_H__
#define __MTCNN_NCNN_H__
#include "../include/ncnn/net.h"
#include <string>
#include <vector>
#include <time.h>
#include <algorithm>
#include <map>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../android_log.h"

using namespace std;
using namespace cv;
struct Bbox
{
    float score;
    int x1;
    int y1;
    int x2;
    int y2;
    float area;
    float ppoint[10];
    float regreCoord[4];
};

class MTCNN {
public:
    MTCNN() {};

    ~MTCNN();

    int LoadModel(AAssetManager* assetManager);

    void SetMinFace(int minSize);

    void Detect(ncnn::Mat& img_, std::vector<Bbox>& finalBbox);

    void detectMaxFace(ncnn::Mat& img_, std::vector<Bbox>& finalBbox);
    void drawDetection(cv::Mat& img_, std::vector<Bbox> &finalBbox);
    void release();
    int detectAge(ncnn::Mat& img_);
private:
    void generateBbox(ncnn::Mat score, ncnn::Mat location, vector<Bbox>& boundingBox_, float scale);
    void nmsTwoBoxs(vector<Bbox> &boundingBox_, vector<Bbox> &previousBox_, const float overlap_threshold, string modelname = "Union");
    void nms(vector<Bbox> &boundingBox_, const float overlap_threshold, string modelname="Union");
    void refine(vector<Bbox> &vecBbox, const int &height, const int &width, bool square);
    void extractMaxFace(vector<Bbox> &boundingBox_);
    int postProcess(ncnn::Mat& output);
    void softMax(float* input, int length, float* output);

    void PNet(float scale);
    void PNet();
    void RNet();
    void ONet();

    ncnn::Net Pnet, Rnet, Onet, AgeNet;
    ncnn::Mat img;
    ncnn::Option option_;

    const float nms_threshold[3] = {0.5f, 0.7f, 0.7f};
    const float mean_vals[3] = {127.5f, 127.5f, 127.5f};
    const float norm_vals[3] = {0.0078125f, 0.0078125f, 0.0078125f};
    const int MIN_DET_SIZE = 12;
    std::vector<Bbox> firstPreviousBbox_, secondPreviousBbox_, thirdPrevioussBbox_;
    std::vector<Bbox> firstBbox_, secondBbox_,thirdBbox_;
    int img_w, img_h;
    ncnn::Mat ncnn_img;

    const float age_mean_vals[3] = {123.675, 116.28, 103.53};
    const float age_norm_vals[3] = {1.0 / 58.395, 1.0 / 57.12, 1.0 / 57.375};


private:
    const float threshold[3] = { 0.6f, 0.7f, 0.9f };
    int minsize = 64;
    const float pre_facetor = 0.85f;
    bool binited = false;


    float threshold_;
    const float mean_val_[3] = {104.f, 117.f, 123.f};
    int thread_num_;
};


#endif //__MTCNN_NCNN_H__