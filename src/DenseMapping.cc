#include "DenseMapping.h"

namespace ORB_SLAM3
{

octomap::point3d DenseMapping::getKFGlobalPC(KeyFrame* pKF, octomap::Pointcloud& ground, octomap::Pointcloud& nonground){
    ground.clear();
    nonground.clear();
    pKF->SetNotErase();
    auto pc = pKF->GetPointCloud();
    // 转换到世界坐标下====
    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(pKF->GetPoseInverse());
    pcl::PointCloud<pcl::PointXYZ> temp;
    pcl::transformPointCloud(*pc, temp, T.matrix());
    octomap::point3d sensorOrigin = octomap::point3d(T(0,3), T(1,3), T(2,3));
    //auto Tw = T.inverse();
    //octomap::point3d sensorOrigin = octomap::point3d(Tw(0,3), Tw(1,3), Tw(2,3));
    pKF->SetErase();

    // split ground and nonground points
    // 随机采样一致性 模型分割=====
    if(false)//crash
    {
        pcl::PointCloud<pcl::PointXYZ> ground_, nonground_;
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
                extract.filter(ground_); // 提取 平面上的点 地面点============
                // remove ground points from full pointcloud:
                // workaround for PCL bug:
                if (inliers->indices.size() != cloud_filtered.size())
                {
                    extract.setNegative(true);
                    pcl::PointCloud<pcl::PointXYZ> cloud_out;
                    extract.filter(cloud_out);
                    nonground_ += cloud_out; // 无地面的点云
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
                nonground_ += cloud_out; // 未找到平面，
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

        if (!groundPlaneFound)nonground_ = temp;
        // todo: make it more efficient, avoid multiple loops
        // Convert pcl::PointCloud to octomap::Pointcloud
        for (const auto& point : nonground_.points) {
            nonground.push_back(point.x, point.y, point.z);
        }
        if(groundPlaneFound)
            for (const auto& point : ground_.points) {
                ground.push_back(point.x, point.y, point.z);
            }
    }else{
    // 这里可以简单剔除掉 y轴方向 > 机器人高度的 点，加速去除平面=======
        bool map_2d = settings->map2D();
        for (const auto& point : temp.points) {
            auto y = point.y;
            if (y<-0.1)continue;//too high
            float agent_height = 1.5;
            if (y>agent_height - 0.1){//ground
                if(map_2d)y=0.1;
                else y = agent_height;
                ground.push_back(point.x, y, point.z);
                // nonground.push_back(point.x, y, point.z);
                //ground and nonground need to be pushed into octomap at the same time
            }else{
                if(map_2d)y=0.0;
                nonground.push_back(point.x, y, point.z);
            }
        }
        //todo: speedup
        if(map_2d){
            // 体素格滤波======
        }
    }
    return sensorOrigin;
}

void DenseMapping::Run(){
    KeyFrame* pKF = NULL;
    octomap::Pointcloud ground, nonground;
    while(1){
        if(isCloseLoop()){
            std::cout<< " closing loop, rebuild octree" << std::endl;
            //todo: add timing of this block  
            {
                unique_lock<mutex> lock(mMutexOctree);
                m_octree->clear();
            }
            vector<KeyFrame*> vKFs = mpAtlas->GetCurrentMap()->GetAllKeyFrames();
            {
                unique_lock<mutex> lock(mMutexQueue);
                mKeyFrameQueue.clear();
                int N = vKFs.size();
                //todo: don't clear and rebuild the updated frames only!!!
                int step = N/30;//max downsampled to 30 frames total
                for(int i=0; i<N; i+=step){
                    mKeyFrameQueue.push_back(vKFs[i]);
                }
            }
            LoopClosed();
        }

        if(CheckNewKeyFrame()){
            {
                unique_lock<mutex> lock(mMutexQueue);
                pKF = mKeyFrameQueue.front();
                mKeyFrameQueue.pop_front();
            }
            auto origin = getKFGlobalPC(pKF, ground, nonground);
            //lock updating m_octree
            unique_lock<mutex> lock(mMutexOctree);
            // m_octree->insertPointCloud(ground, origin);
            m_octree->insertPointCloud(nonground, origin);
            // m_octree->updateInnerOccupancy();//for ColorTree
        }
        //todo, add if isNewMap octree.clear

        usleep(2000);
    }
}

void DenseMapping::InsertScan(const octomap::point3d& sensorOrigin, const octomap::Pointcloud& ground, const octomap::Pointcloud& nonground)
{
    return;

//     // 坐标转换到 key???
//     if(!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)||
//         !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
//      {
//             printf("coulde not generate key for origin\n");
//      }

//      octomap::KeySet free_cells, occupied_cells;// 空闲格子，占有格子

// // 每一个 地面 点云=======================
//      for(auto p:ground.points)
//      {  
//         octomap::point3d point(p.x, p.y, p.z);
//         // only clear space (ground points)
//         if(m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
//         {
//              free_cells.insert(m_keyRay.begin(), m_keyRay.end()); // 地面为空闲格子======
//              m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);//颜色
//         }
//         octomap::OcTreeKey endKey;
//         if(m_octree->coordToKeyChecked(point, endKey))
//         {
//               updateMinKey(endKey, m_updateBBXMin);
//               updateMaxKey(endKey, m_updateBBXMax);
//          }
//         else
//         {
//               printf("could not generator key for endpoint");
//         }
//      }

// // 无地面点云====================================
// // all other points : free on ray, occupied on endpoings:
//      for(auto p:nonground.points)
//      {
//          octomap::point3d point(p.x, p.y, p.z);
//          //free cell
//          if(m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
//          {
//             // free_cells.insert(m_keyRay.begin(),m_keyRay.end()); // 非空闲
//          }
//          //occupided endpoint
//          octomap::OcTreeKey key;
//          if(m_octree->coordToKeyChecked(point, key))
//          {
//              occupied_cells.insert(key); // 占有格子======
//              updateMinKey(key, m_updateBBXMin);
//              updateMaxKey(key, m_updateBBXMax);
//              m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);
//          }

//      }

//    //  pcl::PointCloud<pcl::PointXYZRGB>observation;

// // 空闲格子====
//      for(octomap::KeySet::iterator it = free_cells.begin(),
//                                    end= free_cells.end();
//                                    it!=end; ++it)
//      {   
//          if(occupied_cells.find(*it) == occupied_cells.end())// 占有格子未找到====
//          {
//              m_octree->updateNode(*it, false);// 空闲格子====
//          }
//      }
// // 占有格子====
//      for(octomap::KeySet::iterator it = occupied_cells.begin(),
//                                    end= occupied_cells.end();
//                                    it!=end; ++it)
//      {   
//          m_octree->updateNode(*it, true);// 占有格子====
//      }

//      m_octree->prune();
}



}
