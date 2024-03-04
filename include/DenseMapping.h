#ifndef DENSEMAPPING_H
#define DENSEMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
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
    DenseMapping(System* pSys, Atlas* pAtlas, float resolution):
        mpSys(pSys),
        mpAtlas(pAtlas),
        m_octree(new octomap::OcTree(resolution)){
            m_octree->setProbHit(0.7);
            m_octree->setProbMiss(0.4);
            m_octree->setClampingThresMax(0.95);
            m_octree->setClampingThresMin(0.05);
    }
    void Run();
    void InsertKeyFrame(KeyFrame* pKF){
        unique_lock<mutex> lock(mMutexQueue);
        if(pKF->mnId!=0)
            mKeyFrameQueue.push_back(pKF);
    }

    bool CheckNewKeyFrame(){
        unique_lock<mutex> lock(mMutexQueue);
        return !mKeyFrameQueue.empty();
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

    auto getOctree() { return m_octree; }
    octomap::Pointcloud getKFGlobalPC(KeyFrame* pKF);

private:
    System* mpSys;
    Atlas* mpAtlas;
    octomap::OcTree *m_octree;
    bool mbCloseLoop = false;
    std::mutex mMutexQueue, mMutexCloseLoop;
    std::list<KeyFrame*> mKeyFrameQueue;
};

}

#endif