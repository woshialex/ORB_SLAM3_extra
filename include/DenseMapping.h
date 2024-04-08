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

using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;

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
            m_treeDepth = m_octree->getTreeDepth();
            m_maxTreeDepth = m_treeDepth;
            prevKFpose = Sophus::SE3f::transX(10000.0);
            m_maxRange = settings->maxRange();
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

    octomap::point3d getKFGlobalPC(KeyFrame *pKF, PCLPointCloud &ground, PCLPointCloud &nonground);
    void InsertScan(const octomap::point3d &sensorOrigin, const PCLPointCloud &ground, const PCLPointCloud &nonground);
    void FilterGroundPlane( const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const;
    /// Test if key is within update area of map (2D, ignores height)
    inline bool isInUpdateBBX(const octomap::OcTree::iterator &it) const
    {
        // 2^(tree_depth-depth) voxels wide:
        unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
        octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
        return (key[0] + voxelWidth >= m_updateBBXMin[0] && key[1] + voxelWidth >= m_updateBBXMin[1] && key[0] <= m_updateBBXMax[0] && key[1] <= m_updateBBXMax[1]);
    }

    inline static void updateMinKey(const octomap::OcTreeKey &in,
                                    octomap::OcTreeKey &min)
    {
        for (unsigned i = 0; i < 3; ++i)
        {
            min[i] = std::min(in[i], min[i]);
        }
    };

    inline static void updateMaxKey(const octomap::OcTreeKey &in,
                                    octomap::OcTreeKey &max)
    {
        for (unsigned i = 0; i < 3; ++i)
        {
            max[i] = std::max(in[i], max[i]);
        }
    };

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

    octomap::KeyRay m_keyRay;  // temp storage for ray casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    double m_maxRange;

};

}

#endif