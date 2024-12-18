//
// Created by xuezhiyuan on 2024/11/29.
//
#include "mtcnn.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../android_log.h"

#define TAG "mtcnn_cpp"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG,__VA_ARGS__)
using namespace cv;

static unsigned long get_current_time(void) {
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return (tv.tv_sec * 1000000 + tv.tv_usec);
}

bool cmpScore(Bbox lsh, Bbox rsh) {
    if (lsh.score < rsh.score)
        return true;
    else
        return false;
}

bool cmpArea(Bbox lsh, Bbox rsh) {
    if (lsh.area < rsh.area)
        return false;
    else
        return true;
}

bool MTCNN::init(AAssetManager* assetManager, std::vector<ModelConfig> &configs) {
    LOGD("MTCNN, mtcnn =%s =%d", "model loading", binited);
    if (binited)
        release();
    Pnet.load_param(assetManager, "face/det1.param");
    Pnet.load_model(assetManager, "face/det1.bin");
    Rnet.load_param(assetManager, "face/det2.param");
    Rnet.load_model(assetManager, "face/det2.bin");
    Onet.load_param(assetManager, "face/det3.param");
    Onet.load_model(assetManager, "face/det3.bin");
    LOGD("MTCNN, mtcnn =%s", "model load suceess");
    Anet.load_param(assetManager, "age/age_v2.param");
    Anet.load_model(assetManager, "age/age_v2.bin");


    configs_ = configs;
    model_num_ = static_cast<int>(configs_.size());
    for (int i = 0; i < model_num_; ++i) {
        ncnn::Net *net = new ncnn::Net();
        std::string param = "live/" + configs_[i].name + ".param";
        std::string model = "live/" + configs_[i].name + ".bin";
        int ret = net->load_param(assetManager, param.c_str());
        if (ret != 0) {
            LOG_ERR("LiveBody load param failed.");
            return -2 * (i) - 1;
        }

        ret = net->load_model(assetManager, model.c_str());
        if (ret != 0) {
            LOG_ERR("LiveBody load model failed.");
            return -2 * (i + 1);
        }
        nets_.emplace_back(net);
    }

    binited = true;
    return !binited;
}

MTCNN::~MTCNN() {
    if (binited)
        release();
}

void MTCNN::release() {
    Pnet.clear();
    Rnet.clear();
    Onet.clear();
    Anet.clear();

    for (int i = 0; i < nets_.size(); ++i) {
        nets_[i]->clear();
        delete nets_[i];
    }
    nets_.clear();
}

void MTCNN::SetMinFace(int minSize) {
    minsize = minSize;
}

void MTCNN::generateBbox(ncnn::Mat score, ncnn::Mat location, std::vector<Bbox> &boundingBox_,
                         float scale) {
    const int stride = 2;
    const int cellsize = 12;
    //score p
    float *p = score.channel(1);//score.data + score.cstep;
    //float *plocal = location.data;
    Bbox bbox;
    float inv_scale = 1.0f / scale;
    for (int row = 0; row < score.h; row++) {
        for (int col = 0; col < score.w; col++) {
            if (*p > threshold[0]) {
                bbox.score = *p;
                bbox.x1 = round((stride * col + 1) * inv_scale);
                bbox.y1 = round((stride * row + 1) * inv_scale);
                bbox.x2 = round((stride * col + 1 + cellsize) * inv_scale);
                bbox.y2 = round((stride * row + 1 + cellsize) * inv_scale);
                bbox.area = (bbox.x2 - bbox.x1) * (bbox.y2 - bbox.y1);
                const int index = row * score.w + col;
                for (int channel = 0; channel < 4; channel++) {
                    bbox.regreCoord[channel] = location.channel(channel)[index];
                }
                boundingBox_.push_back(bbox);
            }
            p++;
            //plocal++;
        }
    }
}


void MTCNN::nmsTwoBoxs(vector<Bbox> &boundingBox_, vector<Bbox> &previousBox_,
                       const float overlap_threshold, string modelname) {
    if (boundingBox_.empty()) {
        return;
    }
    sort(boundingBox_.begin(), boundingBox_.end(), cmpScore);
    float IOU = 0;
    float maxX = 0;
    float maxY = 0;
    float minX = 0;
    float minY = 0;
    //std::cout << boundingBox_.size() << " ";
    for (std::vector<Bbox>::iterator ity = previousBox_.begin(); ity != previousBox_.end(); ity++) {
        for (std::vector<Bbox>::iterator itx = boundingBox_.begin(); itx != boundingBox_.end();) {
            int i = itx - boundingBox_.begin();
            int j = ity - previousBox_.begin();
            maxX = std::max(boundingBox_.at(i).x1, previousBox_.at(j).x1);
            maxY = std::max(boundingBox_.at(i).y1, previousBox_.at(j).y1);
            minX = std::min(boundingBox_.at(i).x2, previousBox_.at(j).x2);
            minY = std::min(boundingBox_.at(i).y2, previousBox_.at(j).y2);
            //maxX1 and maxY1 reuse
            maxX = ((minX - maxX + 1) > 0) ? (minX - maxX + 1) : 0;
            maxY = ((minY - maxY + 1) > 0) ? (minY - maxY + 1) : 0;
            //IOU reuse for the area of two bbox
            IOU = maxX * maxY;
            if (!modelname.compare("Union"))
                IOU = IOU / (boundingBox_.at(i).area + previousBox_.at(j).area - IOU);
            else if (!modelname.compare("Min")) {
                IOU = IOU /
                      ((boundingBox_.at(i).area < previousBox_.at(j).area) ? boundingBox_.at(i).area
                                                                           : previousBox_.at(
                                      j).area);
            }
            if (IOU > overlap_threshold && boundingBox_.at(i).score > previousBox_.at(j).score) {
                //if (IOU > overlap_threshold) {
                itx = boundingBox_.erase(itx);
            } else {
                itx++;
            }
        }
    }
    //std::cout << boundingBox_.size() << std::endl;
}

void MTCNN::nms(std::vector<Bbox> &boundingBox_, const float overlap_threshold, string modelname) {
    if (boundingBox_.empty()) {
        return;
    }
    sort(boundingBox_.begin(), boundingBox_.end(), cmpScore);
    float IOU = 0;
    float maxX = 0;
    float maxY = 0;
    float minX = 0;
    float minY = 0;
    std::vector<int> vPick;
    int nPick = 0;
    std::multimap<float, int> vScores;
    const int num_boxes = boundingBox_.size();
    vPick.resize(num_boxes);
    for (int i = 0; i < num_boxes; ++i) {
        vScores.insert(std::pair<float, int>(boundingBox_[i].score, i));
    }
    while (vScores.size() > 0) {
        int last = vScores.rbegin()->second;
        vPick[nPick] = last;
        nPick += 1;
        for (std::multimap<float, int>::iterator it = vScores.begin(); it != vScores.end();) {
            int it_idx = it->second;
            maxX = std::max(boundingBox_.at(it_idx).x1, boundingBox_.at(last).x1);
            maxY = std::max(boundingBox_.at(it_idx).y1, boundingBox_.at(last).y1);
            minX = std::min(boundingBox_.at(it_idx).x2, boundingBox_.at(last).x2);
            minY = std::min(boundingBox_.at(it_idx).y2, boundingBox_.at(last).y2);
            //maxX1 and maxY1 reuse
            maxX = ((minX - maxX + 1) > 0) ? (minX - maxX + 1) : 0;
            maxY = ((minY - maxY + 1) > 0) ? (minY - maxY + 1) : 0;
            //IOU reuse for the area of two bbox
            IOU = maxX * maxY;
            if (!modelname.compare("Union"))
                IOU = IOU / (boundingBox_.at(it_idx).area + boundingBox_.at(last).area - IOU);
            else if (!modelname.compare("Min")) {
                IOU = IOU / ((boundingBox_.at(it_idx).area < boundingBox_.at(last).area)
                             ? boundingBox_.at(it_idx).area : boundingBox_.at(last).area);
            }
            if (IOU > overlap_threshold) {
                it = vScores.erase(it);
            } else {
                it++;
            }
        }
    }

    vPick.resize(nPick);
    std::vector<Bbox> tmp_;
    tmp_.resize(nPick);
    for (int i = 0; i < nPick; i++) {
        tmp_[i] = boundingBox_[vPick[i]];
    }
    boundingBox_ = tmp_;
}

void MTCNN::refine(vector<Bbox> &vecBbox, const int &height, const int &width, bool square) {
    if (vecBbox.empty()) {
        cout << "Bbox is empty!!" << endl;
        return;
    }
    float bbw = 0, bbh = 0, maxSide = 0;
    float h = 0, w = 0;
    float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    for (vector<Bbox>::iterator it = vecBbox.begin(); it != vecBbox.end(); it++) {
        bbw = (*it).x2 - (*it).x1 + 1;
        bbh = (*it).y2 - (*it).y1 + 1;
        x1 = (*it).x1 + (*it).regreCoord[0] * bbw;
        y1 = (*it).y1 + (*it).regreCoord[1] * bbh;
        x2 = (*it).x2 + (*it).regreCoord[2] * bbw;
        y2 = (*it).y2 + (*it).regreCoord[3] * bbh;


        if (square) {
            w = x2 - x1 + 1;
            h = y2 - y1 + 1;
            maxSide = (h > w) ? h : w;
            x1 = x1 + w * 0.5 - maxSide * 0.5;
            y1 = y1 + h * 0.5 - maxSide * 0.5;
            (*it).x2 = round(x1 + maxSide - 1);
            (*it).y2 = round(y1 + maxSide - 1);
            (*it).x1 = round(x1);
            (*it).y1 = round(y1);
        }

        //boundary check
        if ((*it).x1 < 0)(*it).x1 = 0;
        if ((*it).y1 < 0)(*it).y1 = 0;
        if ((*it).x2 > width)(*it).x2 = width - 1;
        if ((*it).y2 > height)(*it).y2 = height - 1;

        it->area = (it->x2 - it->x1) * (it->y2 - it->y1);
    }
}

void MTCNN::extractMaxFace(vector<Bbox> &boundingBox_) {
    if (boundingBox_.empty()) {
        return;
    }
    sort(boundingBox_.begin(), boundingBox_.end(), cmpArea);
    for (std::vector<Bbox>::iterator itx = boundingBox_.begin() + 1; itx != boundingBox_.end();) {
        itx = boundingBox_.erase(itx);
    }
}

void MTCNN::PNet(float scale) {
    //first stage
    int hs = (int) ceil(img_h * scale);
    int ws = (int) ceil(img_w * scale);
    ncnn::Mat in;
    resize_bilinear(img, in, ws, hs);
    ncnn::Extractor ex = Pnet.create_extractor();
    ex.set_light_mode(true);
    ex.set_num_threads(4);
    ex.input("data", in);
    ncnn::Mat score_, location_;
    ex.extract("prob1", score_);
    ex.extract("conv4-2", location_);
    std::vector<Bbox> boundingBox_;

    generateBbox(score_, location_, boundingBox_, scale);
    nms(boundingBox_, nms_threshold[0]);

    firstBbox_.insert(firstBbox_.end(), boundingBox_.begin(), boundingBox_.end());
    boundingBox_.clear();
}


void MTCNN::PNet() {
    firstBbox_.clear();
    float minl = img_w < img_h ? img_w : img_h;
    float m = (float) MIN_DET_SIZE / minsize;
    minl *= m;
    float factor = pre_facetor;
    vector<float> scales_;
    while (minl > MIN_DET_SIZE) {
        scales_.push_back(m);
        minl *= factor;
        m = m * factor;
    }
    for (size_t i = 0; i < scales_.size(); i++) {
        int hs = (int) ceil(img_h * scales_[i]);
        int ws = (int) ceil(img_w * scales_[i]);
        ncnn::Mat in;
        resize_bilinear(img, in, ws, hs);
        ncnn::Extractor ex = Pnet.create_extractor();
        ex.set_num_threads(4);
        ex.set_light_mode(true);
        ex.input("data", in);
        ncnn::Mat score_, location_;
        ex.extract("prob1", score_);
        ex.extract("conv4-2", location_);
        std::vector<Bbox> boundingBox_;
        generateBbox(score_, location_, boundingBox_, scales_[i]);
        nms(boundingBox_, nms_threshold[0]);
        firstBbox_.insert(firstBbox_.end(), boundingBox_.begin(), boundingBox_.end());
        boundingBox_.clear();
    }
}

void MTCNN::RNet() {
    secondBbox_.clear();
    int count = 0;
    for (vector<Bbox>::iterator it = firstBbox_.begin(); it != firstBbox_.end(); it++) {
        ncnn::Mat tempIm;
        copy_cut_border(img, tempIm, (*it).y1, img_h - (*it).y2, (*it).x1, img_w - (*it).x2);
        ncnn::Mat in;
        resize_bilinear(tempIm, in, 24, 24);
        ncnn::Extractor ex = Rnet.create_extractor();
        ex.set_num_threads(4);
        ex.set_light_mode(true);
        ex.input("data", in);
        ncnn::Mat score, bbox;
        ex.extract("prob1", score);
        ex.extract("conv5-2", bbox);
        if ((float) score[1] > threshold[1]) {
            for (int channel = 0; channel < 4; channel++) {
                it->regreCoord[channel] = (float) bbox[channel];//*(bbox.data+channel*bbox.cstep);
            }
            it->area = (it->x2 - it->x1) * (it->y2 - it->y1);
            it->score = score.channel(1)[0];//*(score.data+score.cstep);
            secondBbox_.push_back(*it);
        }
    }
}

void MTCNN::ONet() {
    thirdBbox_.clear();
    for (vector<Bbox>::iterator it = secondBbox_.begin(); it != secondBbox_.end(); it++) {
        ncnn::Mat tempIm;
        copy_cut_border(img, tempIm, (*it).y1, img_h - (*it).y2, (*it).x1, img_w - (*it).x2);
        ncnn::Mat in;
        resize_bilinear(tempIm, in, 48, 48);
        ncnn::Extractor ex = Onet.create_extractor();
        ex.set_num_threads(4);
        ex.set_light_mode(true);
        ex.input("data", in);
        ncnn::Mat score, bbox, keyPoint;
        ex.extract("prob1", score);
        ex.extract("conv6-2", bbox);
        ex.extract("conv6-3", keyPoint);
        if ((float) score[1] > threshold[2]) {
            for (int channel = 0; channel < 4; channel++) {
                it->regreCoord[channel] = (float) bbox[channel];
            }
            it->area = (it->x2 - it->x1) * (it->y2 - it->y1);
            it->score = score.channel(1)[0];
            for (int num = 0; num < 5; num++) {
                (it->ppoint)[num] = it->x1 + (it->x2 - it->x1) * keyPoint[num];
                (it->ppoint)[num + 5] = it->y1 + (it->y2 - it->y1) * keyPoint[num + 5];
            }

            thirdBbox_.push_back(*it);
        }
    }
}

void MTCNN::detect(ncnn::Mat &img_, std::vector<Bbox> &finalBbox_) {
    img = img_;
    img_w = img.w;
    img_h = img.h;
    img.substract_mean_normalize(mean_vals, norm_vals);

    PNet();
    //the first stage's nms
    if (firstBbox_.size() < 1) return;
    nms(firstBbox_, nms_threshold[0]);
    refine(firstBbox_, img_h, img_w, true);
    //printf("firstBbox_.size()=%d\n", firstBbox_.size());


    //second stage
    RNet();
    //printf("secondBbox_.size()=%d\n", secondBbox_.size());
    if (secondBbox_.size() < 1) return;
    nms(secondBbox_, nms_threshold[1]);
    refine(secondBbox_, img_h, img_w, true);

    //third stage
    ONet();
    //printf("thirdBbox_.size()=%d\n", thirdBbox_.size());
    if (thirdBbox_.size() < 1) return;
    refine(thirdBbox_, img_h, img_w, true);
    nms(thirdBbox_, nms_threshold[2], "Min");
    finalBbox_ = thirdBbox_;
}


void MTCNN::detectMaxFace(ncnn::Mat &img_, std::vector<Bbox> &finalBbox) {
    firstPreviousBbox_.clear();
    secondPreviousBbox_.clear();
    thirdPrevioussBbox_.clear();
    firstBbox_.clear();
    secondBbox_.clear();
    thirdBbox_.clear();

    //norm
    img = img_;
    img_w = img.w;
    img_h = img.h;
    img.substract_mean_normalize(mean_vals, norm_vals);

    //pyramid size
    float minl = img_w < img_h ? img_w : img_h;
    float m = (float) MIN_DET_SIZE / minsize;
    minl *= m;
    float factor = pre_facetor;
    vector<float> scales_;
    while (minl > MIN_DET_SIZE) {
        scales_.push_back(m);
        minl *= factor;
        m = m * factor;
    }
    sort(scales_.begin(), scales_.end());
    //printf("scales_.size()=%d\n", scales_.size());

    unsigned long start_time = get_current_time();
    //Change the sampling process.
    for (size_t i = 0; i < scales_.size(); i++) {

        if (get_current_time() - start_time > 1000000) { // 1秒 = 1,000,000 微秒
            // 如果超时，则中断后续处理
            LOGD("Detection timeout after 1 second");
            finalBbox.clear(); // 清除最终的Bbox
            return; // 直接返回，不再继续处理
        }

        //first stage
        PNet(scales_[i]);
        nms(firstBbox_, nms_threshold[0]);
        nmsTwoBoxs(firstBbox_, firstPreviousBbox_, nms_threshold[0]);
        if (firstBbox_.size() < 1) {
            firstBbox_.clear();
            continue;
        }
        firstPreviousBbox_.insert(firstPreviousBbox_.end(), firstBbox_.begin(), firstBbox_.end());
        refine(firstBbox_, img_h, img_w, true);
        //printf("firstBbox_.size()=%d\n", firstBbox_.size());

        //second stage
        RNet();
        nms(secondBbox_, nms_threshold[1]);
        nmsTwoBoxs(secondBbox_, secondPreviousBbox_, nms_threshold[0]);
        secondPreviousBbox_.insert(secondPreviousBbox_.end(), secondBbox_.begin(),
                                   secondBbox_.end());
        if (secondBbox_.size() < 1) {
            firstBbox_.clear();
            secondBbox_.clear();
            continue;
        }
        refine(secondBbox_, img_h, img_w, true);
        //printf("secondBbox_.size()=%d\n", secondBbox_.size());

        //third stage
        ONet();
        //printf("thirdBbox_.size()=%d\n", thirdBbox_.size());
        if (thirdBbox_.size() < 1) {
            firstBbox_.clear();
            secondBbox_.clear();
            thirdBbox_.clear();
            continue;
        }
        refine(thirdBbox_, img_h, img_w, true);
        nms(thirdBbox_, nms_threshold[2], "Min");

        if (thirdBbox_.size() > 0) {
            extractMaxFace(thirdBbox_);
            finalBbox = thirdBbox_;//if largest face size is similar,.
            break;
        }
    }
}



int MTCNN::detectAge(ncnn::Mat &img_) {
    ncnn_img = img_;
    // 设置输入  年龄检测模型 2.0
    ncnn::Extractor ex = Anet.create_extractor();
    ex.input("input.1", ncnn_img);
    // 执行推理
    ncnn::Mat out;
    ex.extract("664", out);
    const float* data = out.row(0);
    float result = data[0];
    int age = 0;
    if (data != nullptr) {
        age = static_cast<int>(result);
        LOG_ERR("FaceDetector xuezhiyuan  age  %d  ", age);
    }
    return age;
}

float MTCNN::detectLive(cv::Mat &src, FaceBox &box) {
    auto start = std::chrono::high_resolution_clock::now();

    float confidence = 0.f;
    for (int i = 0; i < model_num_; i++) {
        cv::Mat roi;
        if(configs_[i].org_resize) {
            cv::resize(src, roi, cv::Size(configs_[i].width, configs_[i].height));
        } else {
            cv::Rect rect = CalculateBox(box, src.cols, src.rows, configs_[i]);
            // roi resize
            cv::resize(src(rect), roi, cv::Size(configs_[i].width, configs_[i].height));
        }

        ncnn::Mat in = ncnn::Mat::from_pixels(roi.data, ncnn::Mat::PIXEL_BGR, roi.cols, roi.rows);

        // inference
        ncnn::Extractor extractor = nets_[i]->create_extractor();
        extractor.set_light_mode(true);
        extractor.set_num_threads(2);

        extractor.input(net_input_name_.c_str(), in);
        ncnn::Mat out;
        extractor.extract(net_output_name_.c_str(), out);

        confidence += out.row(0)[1];
    }
    confidence /= model_num_;

    box.confidence = confidence;

    auto detectEnd = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(detectEnd - start);
    long long duration_ms = duration.count() / 1000;
    LOGD("ncnn Detect confidence = %f", confidence);
    LOGD("ncnn Detect cost = %lld ms ", duration_ms);
    return confidence;
}

cv::Rect MTCNN::CalculateBox(FaceBox &box, int w, int h, ModelConfig &config) {
    int x = static_cast<int>(box.x1);
    int y = static_cast<int>(box.y1);
    int box_width = static_cast<int>(box.x2 - box.x1 + 1);
    int box_height = static_cast<int>(box.y2 - box.y1 + 1);

    int shift_x = static_cast<int>(box_width * config.shift_x);
    int shift_y = static_cast<int>(box_height * config.shift_y);

    float scale = std::min(
            config.scale,
            std::min((w - 1) / (float) box_width, (h - 1) / (float) box_height)
    );

    int box_center_x = box_width / 2 + x;
    int box_center_y = box_height / 2 + y;

    int new_width = static_cast<int>(box_width * scale);
    int new_height = static_cast<int>(box_height * scale);

    int left_top_x = box_center_x - new_width / 2 + shift_x;
    int left_top_y = box_center_y - new_height / 2 + shift_y;
    int right_bottom_x = box_center_x + new_width / 2 + shift_x;
    int right_bottom_y = box_center_y + new_height / 2 + shift_y;

    if (left_top_x < 0) {
        right_bottom_x -= left_top_x;
        left_top_x = 0;
    }

    if (left_top_y < 0) {
        right_bottom_y -= left_top_y;
        left_top_y = 0;
    }

    if (right_bottom_x >= w) {
        int s = right_bottom_x - w + 1;
        left_top_x -= s;
        right_bottom_x -= s;
    }

    if (right_bottom_y >= h) {
        int s = right_bottom_y - h + 1;
        left_top_y -= s;
        right_bottom_y -= s;
    }

    return cv::Rect(left_top_x, left_top_y, new_width, new_height);
}
