//
// Created by xuezhiyuan on 2024/11/29.
//
#ifndef __MTCNN_NCNN_H__
#define __MTCNN_NCNN_H__
#include "net.h"
#include <string>
#include <vector>
#include <time.h>
#include <algorithm>
#include <map>
#include <iostream>
#include "../definition.h"
#include <opencv2/imgproc.hpp>

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
    bool init(AAssetManager* assetManager, std::vector<ModelConfig> &configs);
    MTCNN() {};
    ~MTCNN();
    void SetMinFace(int minSize);
    void detect(ncnn::Mat& img_, std::vector<Bbox>& finalBbox);
    void detectMaxFace(ncnn::Mat& img_, std::vector<Bbox>& finalBbox);
    void drawDetection(cv::Mat& img_, std::vector<Bbox> &finalBbox);
    int detectAge(ncnn::Mat& src);
    float detectLive(cv::Mat &src, FaceBox &box);
    void release();
private:
    cv::Rect CalculateBox(FaceBox &box, int w, int h, ModelConfig &config);
    void generateBbox(ncnn::Mat score, ncnn::Mat location, vector<Bbox>& boundingBox_, float scale);
    void nmsTwoBoxs(vector<Bbox> &boundingBox_, vector<Bbox> &previousBox_, const float overlap_threshold, string modelname = "Union");
    void nms(vector<Bbox> &boundingBox_, const float overlap_threshold, string modelname="Union");
    void refine(vector<Bbox> &vecBbox, const int &height, const int &width, bool square);
    void extractMaxFace(vector<Bbox> &boundingBox_);

    void PNet(float scale);
    void PNet();
    void RNet();
    void ONet();

    ncnn::Net Pnet, Rnet, Onet, Anet;
    ncnn::Mat img;

    const float nms_threshold[3] = {0.5f, 0.7f, 0.7f};
    const float mean_vals[3] = {127.5f, 127.5f, 127.5f};
    const float norm_vals[3] = {0.0078125f, 0.0078125f, 0.0078125f};
    const int MIN_DET_SIZE = 12;
    std::vector<Bbox> firstPreviousBbox_, secondPreviousBbox_, thirdPrevioussBbox_;
    std::vector<Bbox> firstBbox_, secondBbox_,thirdBbox_;
    int img_w, img_h;
    ncnn::Mat ncnn_img;

    int model_num_;
    std::vector<ncnn::Net *> nets_;
    std::vector<ModelConfig> configs_;
    const std::string net_input_name_ = "data";
    const std::string net_output_name_ = "softmax";


private:
    const float threshold[3] = { 0.6f, 0.7f, 0.9f };
    int minsize = 160;
    const float pre_facetor = 0.85f;

    bool binited = false;
};


#endif //__MTCNN_NCNN_H__