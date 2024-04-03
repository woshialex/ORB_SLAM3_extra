#ifndef DENSEMAPPING_H
#define DENSEMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "Settings.h"
#include <mutex>

// octomap
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
// pcl
#include <pcl/io/pcd_io.h>// 读写
#include <pcl/common/transforms.h>// 点云坐标变换
#include <pcl/point_types.h>      // 点类型
#include <pcl/filters/voxel_grid.h>// 体素格滤波
#include <pcl/filters/passthrough.h>//  直通滤波
#include <pcl/sample_consensus/method_types.h>// 采样一致性，采样方法
#include <pcl/sample_consensus/model_types.h>// 模型
#include <pcl/segmentation/sac_segmentation.h>// 采样一致性分割
#include <pcl/filters/extract_indices.h>// 提取点晕索引

namespace ORB_SLAM3
{

class System;
class DenseMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DenseMapping(System* pSys, Atlas* pAtlas, Settings* settings_):
        mpSys(pSys),
        mpAtlas(pAtlas),
        settings(settings_),
        m_octree(new octomap::OcTree(settings->gridSize())){
            m_octree->setProbHit(0.7);
            m_octree->setProbMiss(0.4);
            m_octree->setClampingThresMax(0.95);
            m_octree->setClampingThresMin(0.05);
            prevKFpose = Sophus::SE3f::transX(10000.0);
    }
    void Run();
    void InsertKeyFrame(KeyFrame* pKF){
        //avoid inserting too many keyframes so can run faster
        auto currPose = pKF->GetPoseInverse();
        //difference between currPose and prevKFpose
        auto diff = currPose.inverse() * prevKFpose;
        double t = pKF->mTimeStamp;
        double dt = t - mTimeStamp;
        if(dt<0.01)return;
        if (diff.translation().norm() < 0.40 && diff.rotationMatrix().eulerAngles(0, 1, 2).norm() < 3.14/8 && dt<0.1)return;
        {
            unique_lock<mutex> lock(mMutexQueue);
            if(pKF->mnId!=0)
                mKeyFrameQueue.push_back(pKF);
        }
        prevKFpose = currPose;
        mTimeStamp = t;
    }

    bool CheckNewKeyFrame(){
        unique_lock<mutex> lock(mMutexQueue);
        if (mKeyFrameQueue.size()>3){
            printf("Dense Mapping lag behind!!, queue size = %d\n", mKeyFrameQueue.size());
        }
        return !mKeyFrameQueue.empty();
    }

    void EmptyQueue(){
        unique_lock<mutex> lock(mMutexQueue);
        mKeyFrameQueue.clear();
    }

    void RequestCloseLoop(){
        unique_lock<mutex> lock(mMutexCloseLoop);
        mbCloseLoop = true;
    }

    void LoopClosed(){
        unique_lock<mutex> lock(mMutexCloseLoop);
        mbCloseLoop = false;
    }

    bool isCloseLoop(){
        unique_lock<mutex> lock(mMutexCloseLoop);
        return mbCloseLoop;
    }

    ~DenseMapping(){
        delete m_octree;
    }

    auto getOctreeCopy() { 
        //return a copy of the octree
        unique_lock<mutex> lock(mMutexOctree);
        return octomap::OcTree(*m_octree);
    }
    octomap::point3d getKFGlobalPC(KeyFrame* pKF, octomap::Pointcloud& ground, octomap::Pointcloud& nonground);
    void InsertScan(const octomap::point3d& sensorOrigin, const octomap::Pointcloud& ground, const octomap::Pointcloud& nonground);

private:
    System* mpSys;
    Atlas* mpAtlas;
    Settings* settings;
    octomap::OcTree *m_octree;
    bool mbCloseLoop = false;
    std::mutex mMutexQueue, mMutexCloseLoop;
    std::list<KeyFrame*> mKeyFrameQueue;
    std::mutex mMutexOctree;
    Sophus::SE3f prevKFpose;
    double mTimeStamp = 0;
};

}

#endif