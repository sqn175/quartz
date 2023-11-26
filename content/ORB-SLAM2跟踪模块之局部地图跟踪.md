---
title: ORB-SLAM2跟踪模块【4/4】：局部地图跟踪
date: 2017-09-13
tags:
  - SLAM
  - ORB-SLAM2
---
ORB-SLAM2估计得到相机的初始位姿后，通过与局部地图进行匹配，进一步优化当前图像帧的位姿态。

## 回顾：帧间匹配得到初始位姿

上一次主要说明了ORB-SLAM2跟踪过程中每帧初始位姿的估计。回顾一下，初始位姿的估计主要是在帧与帧之间进行，有三种方法来估计，TrackWithMotionModel，TrackReferenceKeyFrame，Relocalization。

TrackWithMotionModel假设相机做恒速运动，根据前一帧的位姿加上恒速位移推算出当前帧的位姿，之后遍历前一帧的特征点，根据推算出的当前帧位姿，将特征点对应的3D地图点投影到当前帧，在投影点附近搜索特征点进行匹配，从而缩小特征点匹配搜索范围，加快匹配速度，这一过程通过SearchByProjection()函数实现。最终得到一组匹配好的3D地图点-2D特征点。然后进行Bundle Adjustment，最小化重投影误差来进行当前帧位姿优化，得到初始位姿，优化函数为：`Optimizer::PoseOptimization(&mCurrentFrame);`

TrackReferenceKeyFrame是在运动模型还未建立或者刚完成重定位时，通过BoW进行当前帧与**参考关键帧**特征点匹配，这一过程通过SearchByBoW()函数实现。得到一组3D地图点-2D特征点匹配后，与TrackWithMotionModel一样，优化3D-2D的重投影误差来得到初始位姿，这里在优化前将上一帧的位姿作为当前帧的初始位姿，可以使得优化收敛加快。

Relocalization是在跟踪丢失时，将当前帧与候选关键帧通过BoW进行特征点匹配，之后通过EPnP算法估计位姿，和前两种方法一致，再通过最小化重投影误差对位姿进行优化。

以上三种方法都是基于帧间匹配，优化3D-2D重投影误差得到当前帧的初始位姿。这样得到的初始位姿不是可靠的，其只利用了两帧数据的信息，如果前一帧质量太差，得到的位姿可信度低。因此为了利用更多的信息，需要进一步将当前帧与局部地图进行匹配和优化，也就是`TrackLocalMap()` 。

## 局部地图跟踪

随着相机运动，我们向局部地图添加新的关键帧和3D地图点来维护局部地图，这样，即使跟踪过程中某帧出现问题，利用局部地图，我们仍然可以求出之后那些帧的正确位姿。以下是进行局部地图跟踪的流程：

### 更新局部地图

局部地图更新`UpdateLocalMap()`主要包括：对关键帧的更新`UpdateLocalKeyFrames()`和局部地图点的更新`UpdateLocalPoints()` 。

更新局部关键帧`mvpLocakKeyFrames`步骤如下：

1. 搜索具有共视关系的关键帧。由之前的帧间匹配，我们得到当前帧的地图点（MapPoint），通过遍历这些地图点，得到也能观测到这些地图点的关键帧（地图点上记录了共视信息）。同时记录关键帧的共视次数。关键帧和共视次数存储在`map<KeyFrame*,int> keyframeCounter;`  
2. 将步骤1得到的关键帧全部插入到`mvpLocakKeyFrames`；
3. 遍历步骤1`keyframeCounter`中每个关键帧，得到与之共视程度最高的10个关键帧，也全部插入到`mvpLocakKeyFrames`； 
4. 遍历步骤1`keyframeCounter`中每个关键帧，将每个关键帧的子关键帧（>=1）和父关键帧(=1，共视程度最高的之前的关键帧)都插入到`mvpLocakKeyFrames` ；
5. 更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧。

更新局部地图点`mvpLocalMapPoints`步骤如下：

1. 清空`mvpLocalMapPoints`；
2. 遍历之前得到的局部关键帧`mvpLocakKeyFrames`，将局部关键帧的地图点插入到`mvpLocalMapPoints`。

### 局部地图点匹配

局部地图点匹配`SearchLocalPoints()`将局部地图点与当前帧进行匹配。

步骤如下：

1. 遍历当前帧的`mvpMapPoints`，标记这些MapPoints不参与之后的搜索。这些MapPoints是在帧间匹配过程中用到的。

2. 将局部地图点`mvpLocalMapPoints`投影到当前帧，投影矩阵是通过帧间匹配得到。判断局部地图点是否在当前帧的视野内（视野内，mbTrackInView 为true），只有视野内的地图点才进行投影匹配。判断要求包括：投影点是否在图像内；地图点到相机中心距离是否在尺寸变换范围内；相机中心到地图点向量与地图点平均视角夹角是否小于60°。

3. 对视野内的地图点通过投影进行特征匹配：
   `matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);`，得到一组3D局部地图点-2D特征点匹配对。

### Motion-Only BA

Bundle Adjustment最小化重投影误差优化相机位姿：`Optimizer::PoseOptimization(&mCurrentFrame);`

可以看出，这几步和TrackWithMotionModel类似，只不过TrackWithMotionModel匹配点对是通过帧间匹配得到的，而TrackLocalMap是剔除TrackWithMotionModel中的匹配点，从局部地图中找新的更多的匹配点，从而实现对位姿的进一步优化。







