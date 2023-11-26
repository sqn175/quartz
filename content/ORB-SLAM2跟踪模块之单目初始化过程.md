---
title: ORB-SLAM2跟踪模块【3/4】：单目初始化过程
date: 2017-09-02
tags:
  - SLAM
  - ORB-SLAM2
---
在ORB-SLAM2进行局部地图跟踪前，需要先进行初始化，初始化包括估计相机初始帧位姿，新建地图，新建关键帧等。
初始化代码如下：

```c++
  // Tracking.cc line 279~290
  if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
```

我们对单目初始化进行分析，单目初始化函数为`void Tracking::MonocularInitialization()`。

单目初始化是试图从初始的两帧单目图像中，对SLAM系统进行初始化，以便之后进行跟踪。

## 寻找匹配点

对前两帧图像帧提取完ORB特征后，对特征点进行匹配：

```c++
// Tracking.cc line:600 function: void Tracking::MonocularInitialization()
int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
```

**条件** ：检测特征匹配对个数是否大于等于100，若小于100，则初始化失败。

## 从匹配点中恢复初始相机位姿

在得到超过100个匹配点对后，运用RANSAC 8点法同时计算单应矩阵和基础矩阵，并选择最合适的结果作为相机的初始位姿。

（1） 计算单应矩阵`cv::Mat Hn = ComputeH21(vPn1i,vPn2i);`，并进行评分`currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);`，分别通过单应矩阵将第一帧的特征点投影到第二帧，以及将第二帧的特征点投影到第一帧，计算两次重投影误差，叠加作为评分SH。

（2） 计算基础矩阵`cv::Mat Fn = ComputeF21(vPn1i,vPn2i);`，并进行评分`currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);`，两次重投影误差叠加作为评分SF。

（3） 计算评分比：` float RH = SH/(SH+SF);`，如果RH>0.4，选择单应矩阵来恢复相机位姿，否则选择基础矩阵来恢复相机位姿。

（4） 恢复相机位姿

从基础矩阵和相机内参可以得到本质矩阵：

```c++
// Initializer.cc line:479 function: bool Initializer::ReconstructF()    
// Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;
```

从本质矩阵E21分解会得到4组可能的R，t，需要对这些解进行检查调用`CheckRT()`.

检查1：根据R,t对特征点进行三角测量得到对应的三维地图点，检查地图点各个坐标值是否是有限，如果是无限的，标记该特征点为无效；

检查2：计算视差，即三维地图点与两帧相机中心连线的夹角的cos值需要小于0.99998，且地图点的深度必须大于等于0；

检查3：根据R,t计算双向重投影误差，需要小于预先设定的阈值

统计满足以上三步检查的特征点个数得到nGood，对应4组R，t得到4个nGood.

**条件** ：某个nGood大于阈值`int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);`且与其他nGood有较大区别时，同时有较好视差时，选择该nGood对应的R,t作为最终结果，否则初始化失败。

## 创建初始地图

如果恢复相机初始位姿成功，那么我们能得到前两帧图像的位姿，以及三角测量后的3维地图点。之后进行如下操作：

```c++
// Tracking.cc line:614 function:Tracking::MonocularInitialization()   
if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
{
  for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
  {
    if(mvIniMatches[i]>=0 && !vbTriangulated[i])
    {
      mvIniMatches[i]=-1;
      nmatches--;
    }
  }

  // Set Frame Poses
  mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
  cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
  Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
  tcw.copyTo(Tcw.rowRange(0,3).col(3));
  mCurrentFrame.SetPose(Tcw);

  CreateInitialMapMonocular();
}
```

我们主要关注`CreateInitialMapMonocular()`函数，其通过前两帧图像的位姿以及3维地图点，创建初始地图，并进行相应的数据关联操作：

（1）创建关键帧

因为只有前两帧，所以将这两帧都设置为关键帧，并计算对应的BoW，并在地图中插入这两个关键帧。

（2）数据关联

根据3维地图点，创建对应的地图点，将地图点与关键帧，地图进行数据关联：

- 地图点与关键帧关联

  一个地图点可被多个关键帧观测到，将观测到这个地图点的关键帧与这个地图点进行关联，同时记录关键帧上哪一个特征点与这个地图点有关联。对于单目初始化来说，地图点需要关联第一步创建的两个关键帧。

  ```c++
  // Tracking.cc line:665 function: void Tracking::CreateInitialMapMonocular() 
    pMP->AddObservation(pKFini,i);
    pMP->AddObservation(pKFcur,mvIniMatches[i]);
  ```

  ​地图点与关键帧上的特征点关联后，计算最合适的描述子来描述该地图点，用于之后跟踪的匹配。

- 关键帧与地图点关联

  一个关键帧上的特征点由多个地图点投影而成，将关键帧与地图点关联。

  ```c++
  // Tracking.cc line:671 function: void Tracking::CreateInitialMapMonocular() 
  //Fill Current Frame structure
  mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
  mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;
  ```

- 关键帧与关键帧关联

  关键帧之间会共视一个地图点，如果共视的地图点个数越多，说明这两个关键帧之间的联系越紧密。对于某个关键帧，统计其与其他关键帧共视的特征点个数，如果大于某个阈值，那么将这两个关键帧进行关联。

  ```c++
  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();
  ```

- 将关键帧和地图点加入到地图中

（3）Global BA

​	对上述只有两个关键帧和地图点的地图进行全局BA。

（4） 地图尺寸初始化

​	由于单目SLAM的的地图点坐标尺寸不是真实的，比如x=1可能是1m也可能是1cm。选择地图点深度的中位数作为单位尺寸1当作基准来进行地图的尺寸初始化。












