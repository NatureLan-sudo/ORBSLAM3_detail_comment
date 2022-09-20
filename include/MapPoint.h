/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"

#include "SerializationUtils.h"

#include <opencv2/core/core.hpp>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;
        // Variables used by the tracking
        //ar & mTrackProjX;
        //ar & mTrackProjY;
        //ar & mTrackDepth;
        //ar & mTrackDepthR;
        //ar & mTrackProjXR;
        //ar & mTrackProjYR;
        //ar & mbTrackInView;
        //ar & mbTrackInViewR;
        //ar & mnTrackScaleLevel;
        //ar & mnTrackScaleLevelR;
        //ar & mTrackViewCos;
        //ar & mTrackViewCosR;
        //ar & mnTrackReferenceForFrame;
        //ar & mnLastFrameSeen;

        // Variables used by local mapping
        //ar & mnBALocalForKF;
        //ar & mnFuseCandidateForKF;

        // Variables used by loop closing and merging
        //ar & mnLoopPointForKF;
        //ar & mnCorrectedByKF;
        //ar & mnCorrectedReference;
        //serializeMatrix(ar,mPosGBA,version);
        //ar & mnBAGlobalForKF;
        //ar & mnBALocalForMerge;
        //serializeMatrix(ar,mPosMerge,version);
        //serializeMatrix(ar,mNormalVectorMerge,version);

        // Protected variables
        ar & boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
        ar & boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
        //ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
        //ar & mObservations;
        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar,mDescriptor,version);
        ar & mBackupRefKFId;
        //ar & mnVisible;
        //ar & mnFound;

        ar & mbBad;
        ar & mBackupReplacedId;

        ar & mfMinDistance;
        ar & mfMaxDistance;

    }


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint();

    MapPoint(const Eigen::Vector3f &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap);
    MapPoint(const Eigen::Vector3f &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const Eigen::Vector3f &Pos);
    Eigen::Vector3f GetWorldPos();

    Eigen::Vector3f GetNormal();
    void SetNormalVector(const Eigen::Vector3f& normal);

    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,std::tuple<int,int>> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,int idx);
    void EraseObservation(KeyFrame* pKF);

    std::tuple<int,int> GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void PrintObservations();

    void PreSave(set<KeyFrame*>& spKF,set<MapPoint*>& spMP);
    void PostLoad(map<long unsigned int, KeyFrame*>& mpKFid, map<long unsigned int, MapPoint*>& mpMPid);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    Eigen::Vector3f mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    Eigen::Vector3f mPosMerge;
    Eigen::Vector3f mNormalVectorMerge;


    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;
    KeyFrame* mpHostKF;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

protected:    

     // Position in absolute coordinates 世界坐标系下的坐标
     Eigen::Vector3f mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     // 能观测到该地图点的关键帧以及其在关键帧中相关联的索引值
     std::map<KeyFrame*,std::tuple<int,int> > mObservations;
     // For save relation without pointer, this is necessary for save/load function
     std::map<long unsigned int, int> mBackupObservationsId1;
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     //  该MapPoint的平均观测方向
    // 用于判断点是否在可视范围内
     Eigen::Vector3f mNormalVector;

     // Best descriptor to fast matching
     // 最佳的描述子，用于快速实现匹配
     cv::Mat mDescriptor;

     // Reference KeyFrame
     // 参考关键帧，通常情况下MapPoint的参考关键帧就是创建该MapPoint的那个关键帧
     KeyFrame* mpRefKF;
     long unsigned int mBackupRefKFId;

     // Tracking counters
     // 地图点被跟踪到的次数
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     long long int mBackupReplacedId;

     // Scale invariance distances
     // 根据ORB特征点的尺度不变性获取到的可以观察到这个地图点的最大、最小距离
     float mfMinDistance;
     float mfMaxDistance;
    
     // 地图点所属于的地图
     Map* mpMap;

     // Mutex
     // 对当前点位姿操作时候的互斥量
     std::mutex mMutexPos;
     // 对当前点特征操作时候的互斥量
     std::mutex mMutexFeatures;
     // 对当前点地图操作时候的胡吃两？
     std::mutex mMutexMap;

};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
