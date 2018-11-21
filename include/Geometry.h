/**
* This file is part of DynaSLAM.
* Copyright (C) 2018 Berta Bescos <bbescos at unizar dot es> (University of Zaragoza) 萨拉戈萨大学
* For more information see <https://github.com/bertabescos/DynaSLAM>.
* 几何学 几何模型修正 深度数据关键点区域增长更新
*/

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include "Frame.h"

#define MAX_DB_SIZE 20// 数据库 20类物体  voc数据集???
#define MAX_REF_FRAMES 5
#define ELEM_INITIAL_MAP 5
#define MIN_DEPTH_THRESHOLD 0.2

namespace DynaSLAM
{

class Geometry
{
private:

    class DynKeyPoint // 关键点
    {
    public:
        cv::Point2i mPoint;     // 2d像素点
        int mRefFrameLabel;// 标签
    };

    class DataBase// 数据库
    {
    public:
        vector<ORB_SLAM2::Frame> mvDataBase = vector<ORB_SLAM2::Frame>(MAX_DB_SIZE); 
        int mIni=0;
        int mFin=0;
        int mNumElem = 0;
        bool IsFull();
        void InsertFrame2DB(const ORB_SLAM2::Frame &currentFrame);// 向 数据库插入关键帧 ====
    };
// 提取关键点================
    vector<DynKeyPoint> ExtractDynPoints(const vector<ORB_SLAM2::Frame> &vRefFrames, 
                                                                                    const ORB_SLAM2::Frame &currentFrame);
// 获取参考帧================
    vector<ORB_SLAM2::Frame> GetRefFrames(const ORB_SLAM2::Frame &currentFrame);
// 结合 掩码mask=============
    void CombineMasks(const ORB_SLAM2::Frame &currentFrame, cv::Mat &mask);
// 填充 ====================？
    void FillRGBD(const ORB_SLAM2::Frame &currentFrame,
                                cv::Mat &mask,cv::Mat &imGray,cv::Mat &imDepth);
    void FillRGBD(const ORB_SLAM2::Frame &currentFrame,
                                cv::Mat &mask,cv::Mat &imGray,cv::Mat &imDepth,cv::Mat &imRGB);// 多一张彩色图==

// 利用深度数据对关键点进行区域增长更新================
    cv::Mat DepthRegionGrowing(const vector<DynKeyPoint> &vDynPoints,
                                                                const cv::Mat &imDepth);

    bool isRotationMatrix(const cv::Mat &R);
    cv::Mat rotm2euler(const cv::Mat &R);// 旋转矩阵 转 欧拉角

// 阈值区域增长======
    cv::Mat RegionGrowing(const cv::Mat &Image,
                                                   int &x,int &y,const float &threshold);

    cv::Mat RegionGrowingGaps(const cv::Mat &Image, int &x, int &y);

    int mnRefFrames;

    int mDmax;

    float mDepthThreshold;// 深度阈值

    float mSegThreshold; // 分割阈值

    double mSizeThreshold;

    float mVarThreshold;

    double mParallaxThreshold;

    DataBase mDB;

    cv::Mat vAllPixels;

    bool IsInFrame(const float &x, const float &y, 
                                     const ORB_SLAM2::Frame &Frame);

    bool IsInImage(const float &x, const float &y, const cv::Mat image);
// 非空 候选者======
    void GetClosestNonEmptyCoordinates(const cv::Mat &mask, 
                                                                                   const int &x, const int &y, int &_x, int &_y);

public:
    Geometry();
    ~Geometry() = default;
// 几何模型修正 ====================
    void GeometricModelCorrection(const ORB_SLAM2::Frame &currentFrame, 
                                                                       cv::Mat &imDepth, cv::Mat &mask);
// 画框 ========?
    void InpaintFrames(const ORB_SLAM2::Frame &currentFrame, 
                                             cv::Mat &imGray, cv::Mat &imDepth, cv::Mat &imRGB, cv::Mat &mask);
// 更新数据库=====
    void GeometricModelUpdateDB(const ORB_SLAM2::Frame &mCurrentFrame);
};


}
#endif // GEOMETRY_H
