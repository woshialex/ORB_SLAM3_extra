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

    // split ground and nonground points
    // 随机采样一致性 模型分割=====
    if(false)//crash
    {
        pcl::PointCloud<pcl::PointXYZ> ground, nonground;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 模型系数
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // 点云索引

        pcl::SACSegmentation<pcl::PointCloud<pcl::PointXYZ>::PointType> seg; // 分割
        seg.setOptimizeCoefficients(true);                                      // 优化系数
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);                    // 分割 平面
        seg.setMethodType(pcl::SAC_RANSAC);                                     // 随机采样一致性 分割
        seg.setMaxIterations(200);                                              // 迭代 200次
        seg.setDistanceThreshold(0.04);                                         // 距离阈值
        seg.setAxis(Eigen::Vector3f(0, 1, 0));                                  // xz 平面中的 平面点云 y方向轴====
        seg.setEpsAngle(0.5);

        pcl::PointCloud<pcl::PointXYZ> cloud_filtered(temp); // 分割前的点云===
        pcl::ExtractIndices<pcl::PointCloud<pcl::PointXYZ>::PointType> extract;
        bool groundPlaneFound = false;                          // 地 平面找到 标志
        while (cloud_filtered.size() > 10 && !groundPlaneFound) // 地平面未找到
        {
            seg.setInputCloud(cloud_filtered.makeShared()); // 分割器输入点云
            seg.segment(*inliers, *coefficients);           // 分割
            if (inliers->indices.size() == 0)
            {
                break;
            }
            extract.setInputCloud(cloud_filtered.makeShared()); // 点云提取器
            extract.setIndices(inliers);

            // a*X + b*Y + c*Z + d = 0;
            if (std::abs(coefficients->values.at(3)) > 0.07) // 系数什么含义??
            {

                printf("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f \n",
                       inliers->indices.size(),
                       cloud_filtered.size(),
                       coefficients->values.at(0),
                       coefficients->values.at(1),
                       coefficients->values.at(2),
                       coefficients->values.at(3));

                extract.setNegative(false);
                extract.filter(ground); // 提取 平面上的点 地面点============
                // remove ground points from full pointcloud:
                // workaround for PCL bug:
                if (inliers->indices.size() != cloud_filtered.size())
                {
                    extract.setNegative(true);
                    pcl::PointCloud<pcl::PointXYZ> cloud_out;
                    extract.filter(cloud_out);
                    nonground += cloud_out; // 无地面的点云
                    cloud_filtered = cloud_out;
                }

                groundPlaneFound = true;
            }
            else
            {
                printf("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f \n",
                       inliers->indices.size(),
                       cloud_filtered.size(),
                       coefficients->values.at(0),
                       coefficients->values.at(1),
                       coefficients->values.at(2),
                       coefficients->values.at(3));

                pcl::PointCloud<pcl::PointXYZ> cloud_out;
                extract.setNegative(false);
                extract.filter(cloud_out);
                nonground += cloud_out; // 未找到平面，
                if (inliers->indices.size() != cloud_filtered.size())
                {
                    extract.setNegative(true);
                    cloud_out.points.clear();
                    extract.filter(cloud_out);
                    cloud_filtered = cloud_out;
                }
                else
                {
                    cloud_filtered.points.clear();
                }
            }

        } // while

        if (!groundPlaneFound)nonground = temp;
        // todo: make it more efficient, avoid multiple loops
        // Convert pcl::PointCloud to octomap::Pointcloud
        octomap::Pointcloud octomapCloud;
        for (const auto& point : nonground.points) {
            octomapCloud.push_back(point.x, point.y, point.z);
        }
        return octomapCloud;
    }else{
    // 这里可以简单剔除掉 y轴方向 > 机器人高度的 点，加速去除平面=======
        octomap::Pointcloud octomapCloud;
        for (const auto& point : temp.points) {
            if (point.y<-1.5 + 0.1)continue;//tmp
            octomapCloud.push_back(point.x, point.y, point.z);
        }
        return octomapCloud;
    }

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
