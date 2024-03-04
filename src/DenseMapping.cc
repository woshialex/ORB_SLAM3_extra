#include "DenseMapping.h"

namespace ORB_SLAM3
{

octomap::Pointcloud DenseMapping::getKFGlobalPC(KeyFrame* pKF) {
    pKF->SetNotErase();
    auto pc = pKF->GetPointCloud();
    // 转换到世界坐标下====
    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(pKF->GetPose());
    pcl::PointCloud<pcl::PointXYZ> temp;
    pcl::transformPointCloud(*pc, temp, T.inverse().matrix());
    pKF->SetErase();
    // todo: make it more efficient, avoid multiple loops
    // Convert pcl::PointCloud to octomap::Pointcloud
    octomap::Pointcloud octomapCloud;
    for (const auto& point : temp.points) {
        octomapCloud.push_back(point.x, point.y, point.z);
    }
    return octomapCloud;
}

void DenseMapping::Run(){
    KeyFrame* pKF = NULL;
    octomap::point3d origin = octomap::point3d(0,0,0);
    while(1){
        if(CheckNewKeyFrame()){
            {
                unique_lock<mutex> lock(mMutexQueue);
                pKF = mKeyFrameQueue.front();
                mKeyFrameQueue.pop_front();
            }
            auto octomapCloud = getKFGlobalPC(pKF);
            m_octree->insertPointCloud(octomapCloud, origin);
            m_octree->updateInnerOccupancy();//todo: check here
        }
        //todo, add if isNewMap octree.clear

        if(isCloseLoop()){
            m_octree->clear();
            vector<KeyFrame*> vKFs = mpAtlas->GetCurrentMap()->GetAllKeyFrames();
            for (auto pFK: vKFs) {
                auto octomapCloud = getKFGlobalPC(pFK);
                m_octree->insertPointCloud(octomapCloud, origin);
                m_octree->updateInnerOccupancy();
            }
            LoopClosed();
        }
        usleep(5000);
    }
}

}
