---
title: ORB-SLAM2跟踪模块【2/4】：估计初始位姿
date: 2017-05-14
tags:
  - SLAM
  - ORB-SLAM2
---
上一次主要说明了ORB-SLAM2跟踪中提取ORB特征的过程，跟踪的主要接口是`Tracking`类中的`cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);`函数，其对输入图像进行预处理，提取ORB特征，并且调用`Track();`函数。`Track()`是`Tracking`类的protected函数。

`Track()`函数首先检测系统是否初始化完成，若完成，则进行下一步工作——估计当前帧的初始位姿。
<!--more-->ORB-SLAM支持两个模式，一个是正常的SLAM模式，一个是Localization模式，区别在于是否开启了Local Mapping，Local Mapping会改变地图，所以Localization关闭了Local Mapping。在SLAM模式下，ORB-SLAM2估计初始位姿的方式有两种：通过**从前一帧估计**和**在跟踪丢失情况下通过全局重定位估计**:

```c++
// Tracking.cc  line:302~321
if(mState==OK)
{
  // Local Mapping might have changed some MapPoints tracked in last frame
  CheckReplacedInLastFrame();

  if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
  {
    bOK = TrackReferenceKeyFrame();
  }
  else
  {
    bOK = TrackWithMotionModel();
    if(!bOK)
      bOK = TrackReferenceKeyFrame();
  }
}
else
{
  bOK = Relocalization();
}
```

## 从前一帧进行初始位姿的估计

在上一帧成功跟踪后，ORB-SLAM2会**假设相机为恒速运动模型**来估计当前帧的相机位姿，并且在当前帧中搜索前一帧观察到的地图点时，可以根据该运动来缩小搜索范围。使用`Tracking`类的protected函数`bool TrackWithMotionModel();`:

```c++
bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;
  
    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);
    
    // 省略
    ...
}
```

这个模型假设相机处于匀速运动，那么就可以用上一帧的位姿和速度来估计当前帧的位姿：`mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);`，其中`mTcw`是相机的SE(3)姿态，`mVelocity`是速度模型矩阵。作为备忘，注释解释ORB-SLAM2中关于位姿的设置：

```c++
// Frame.cc  line:255~268
void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    // mTcw是SE（3）矩阵, c:camera; w:world
    // 旋转矩阵,world2camera Rotation Matrix
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    // 旋转矩阵取转置
    mRwc = mRcw.t();
    // 平移矩阵
    mtcw = mTcw.rowRange(0,3).col(3);
    // 相机中心相当于mRcw.t()*(-mtcw)=mRwc*mtwc
    mOw = -mRcw.t()*mtcw;
}
```

估计完初始化位姿后，调用函数`int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);`，将上一帧跟踪到的地图点投影到当前帧，然后寻找匹配点，如果匹配点过少，则增大搜索范围。寻找到足够匹配点后，利用匹配点优化估计的初始位姿。

如果根据运动模型得到的匹配点数目少于20，就会**利用关键帧进行跟踪**，即与关键帧进行匹配。使用`Tracking`类的protected函数`bool TrackReferenceKeyFrame();`：

```c++
bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);
    
    // 省略
    ...

}
```
其首先计算当前帧的Bow，然后利用Bow来进行匹配：`int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);`，之后将当前帧的初始位置设置为上一帧的位姿，并利用匹配点进行位姿优化。

## 通过全局重定位进行初始位姿的估计

当利用恒速运动模型和用关键帧进行跟踪时，匹配点都小于设定的阈值，那么说明跟踪失败，需要全局重定位才能继续跟踪，使用`Tracking`类的protected函数`bool Relocalization();`。

```c++
bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    // 省略
    ...

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }
  
    // 省略
    ...
      
    
}
```

首先计算当前的Bow，然后查询关键帧进行匹配，匹配点对不小于15的话，该关键帧作为候选，然后根据ORB匹配点计算关键帧对应的地图点。之后使用PnP算法，利用5 Ransac迭代计算相机位姿并进行优化。如果有足够inlier，那么跟踪继续进行。

通过以上过程，我们可以对每一帧图像估计出其初始的位姿，然后就可以进行跟踪的下一步工作：局部地图跟踪。

在Localization模式下，可能出现当前帧与地图点没有匹配的情况，这个时候只能通过运动模型方法来进行初始位姿的估计，相当于一个视觉里程计。其他时候也是通过运动模型和关键帧来估计初始位姿。