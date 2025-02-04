---
title: ORB-SLAM2局部映射模块
tags:
  - SLAM
  - ORB-SLAM2
date: 2017-05-24
---
ORB-SLAM2局部映射线程处理从跟踪线程传来的新的关键帧，进行local BA来优化相机位姿和局部地图点。同时搜索和当前关键帧共视的其他关键帧，寻找新的ORB特征匹配对，通过三角测量得到更多的地图点。根据跟踪线程提供的地图点信息，局部映射线程会严格剔除不合格的地图点，只保留高质量地图点。同时冗余的关键帧也会被剔除。局部映射线程流程如下图红线框内所示：
![[ORB-SLAM2局部映射.png]]

主要包括：

1. 插入关键帧；
2. 剔除新插入的地图点；
3. 新建地图点；
4. Local BA；
5. 剔除关键帧。

## 与跟踪线程的联系

局部映射线程和跟踪线程主要通过关键帧联系。跟踪线程对每一帧图像进行估计和优化，得到相机位姿，之后判断这帧图像是否可以当做关键帧:

```c++
// Tracking.cc
// Check if we need to insert a new keyframe
if(NeedNewKeyFrame())
  CreateNewKeyFrame();
```

判断条件在`NeedNewKeyFrame()`中体现为(New KeyFrame Decision)：

- 局部映射线程没有被回环检测线程停止；
- 距离上一次重定位超过1s，或者关键帧数量小于帧率；
- 局部映射线程是否空闲，即是否处于可以进行插入关键帧操作状态，或者距离插入关键帧已经过了1s；
- 当前帧跟踪到的点数大于15，且跟踪到的点与之前参考的关键帧重复度不能过高（不超过参考关键帧的90%）；

只有上述条件都符合，才能将当前帧作为关键帧插入到地图中。如果插入时局部映射线程正忙，此时向其发送停止BA的信号，使得局部映射线程尽快处理新到的关键帧。

`CreateNewKeyFrame()`将当前帧包装为关键帧，之后传入局部映射线程，插入到地图中。

```c++
mpLocalMapper->InsertKeyFrame(pKF);
mpLocalMapper->SetNotStop(false);
```

## 插入关键帧

插入关键帧在函数`ProcessNewKeyFrame()`中实现。跟踪线程将待插入的关键帧缓存到一个列表中。插入关键帧操作流程如下：

1. 每次从列表中pop出一个关键帧`mpCurrentKeyFrame`，然后计算该关键帧的BoW；

2. 将地图点与`mpCurrentKeyFrame`进行数据关联。在此前TrackLocalMap函数中，已经将这个关键帧与地图点进行了匹配，但是没有将地图点绑定到这个关键帧上。数据关联过程如下：

   ```c++
   // pMP是一个地图点指针
   // 添加地图点对当前关键帧的观测
   pMP->AddObservation(mpCurrentKeyFrame, i);
   // 更新地图点的平均观测方向和深度
   pMP->UpdateNormalAndDepth();
   // 由于观测到该点的关键帧+1，所以更新该点的最佳描述子
   pMP->ComputeDistinctiveDescriptors();
   ```

3. 更新关键帧之间的连接。

   这里需要强调一下ORB-SLAM2中关键帧之间的连接是通过共视图（Covisibility Graph）和生成树（Spanning Tree）表达的。

   **共视图：** 是一个有权重的无向图，图的结点为一个关键帧，如果两个关键帧能共同观测到一定数量的地图点，那么这两个关键帧之间建立一条边，边的权重为共同观测到的地图数量。

   **生成树：** 生成树是共视图的包含最少边的子图，每次向生成树添加一个关键帧时，将该关键帧与树中共视地图点数量最多的关键帧连接。从生成树中删除一个关键帧时，也要更新受到影响的所有关键帧的连接关系。

   如下图所示，分别是共视图和生成树（绿色部分）。

   ![[ORB-SLAM2共视图.png]]

   更新`mpCurrentKeyFrame`与其他关键帧的连接关系，也就是更新共视图和生成树：

   ```c++
   // Update links in the Covisibility Graph
   mpCurrentKeyFrame->UpdateConnections();
   ```

   `UpdateConnections()`函数流程如下：

   （1）获得`mpCurrentKeyFrame`观测到的所有地图点；

   （2）遍历地图点，得到所有和`mpCurrentKeyFrame`有共视地图点的关键帧，并统计这些关键帧的权重，即共视地图点数目；

   （3）如果共视地图点数目大于15，那么将连接`mpCurrentKeyFrame`和这个关键帧；如果所有关键帧与`mpCurrentKeyFrame`的共视地图点数目都不大于15，则选取共视地图点数最多的关键帧与`mpCurrentKeyFrame`建立连接；

   （4）将和`mpCurrentKeyFrame`共视的关键帧按照权重进行排序，完成共视图的更新；

   （5）更新生成树，将权重最高的关键帧作为`mpCurrentKeyFrame`的初始父关键帧：

   ```c++
   mpParent = mvpOrderedConnectedKeyFrames.front();
   mpParent->AddChild(this);
   mbFirstConnection = false;
   ```

4. 将`mpCurrentKeyFrame`插入到地图中：

   ```c++
   // Insert Keyframe in Map
   mpMap->AddKeyFrame(mpCurrentKeyFrame);
   ```

## 剔除新插入的地图点

剔除新插入的地图点在函数`MapPointCulling()`中实现。处理上一关键帧会插入新的地图点，在这里需要对这些新加入的地图点进行筛选，剔除质量不好的点。

筛选条件为：

- 跟踪到该地图点的帧数目`mnFound`小于预计能观测到该地图点的帧数目`mnVisible`的25%，那么剔除这个地图点（`MapPoint::SetBadFlag()` 函数实现）。这里解释一下，visible表示这个地图点在某一帧的视野内，通过`Frame::isInFrustum()`函数判断。虽然这个地图点能被这帧观测到，但是不一定能和这一帧上的特征点匹配上。如果匹配上，就是found了。
- 从该地图点创建开始，到这一个关键帧已经过了不少于2个关键帧，但是能观测到这个地图点的关键帧数目小于等于2，那么剔除这个地图点；

如果该地图点从创建开始，已经经过连续三个关键帧的检测而没有被剔除，那么说明这个地图点质量很高，之后不再对其进行检测。

## 新建地图点

新建地图点在函数`CreateNewMapPoints()`实现。

首先获得与当前关键帧`mpCurrentKeyFrame`共视权重最高的前20帧关键帧`vpNeighKFs`；

```c++
// Retrieve neighbor keyframes in covisibility graph
int nn = 10;
if(mbMonocular)
nn=20;
const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
```

然后遍历`vpNeighKFs`，对每个邻近关键帧`pKF2` ，进行如下步骤：

1. 计算`pKF2`到当前关键帧`mpCurrentKeyFrame`的位移与`pKF2`中地图点深度中值的比例，如果比例小于0.01，则跳过，不新建地图点；

2. 根据`mpCurrentKeyFrame`和`pKF2`的位姿，计算基础矩阵：
   $$
   \mathbf E= {\mathbf t_{12}}^\wedge\mathbf R_{12} \\
   \mathbf F=\mathbf K^{-T}\mathbf E\mathbf K^{-1}
   $$

3. 在`mpCurrentKeyFrame`和`pKF2`中没有经过匹配的特征点进行特征点匹配，用到的方法为：

   ```c++
   matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);
   ```

   这里先通过BoW缩小匹配范围，只对同一个node节点的特征点进行匹配。匹配是检测特征点对是否满足对极几何约束，然后选择描述子距离最小的作为匹配点对。这样可以缩小匹配范围，加快匹配速率。

4. 根据上一步得到的匹配点对，三角测量生成对应的3D地图点。对每个3D地图点，进行：

   - 视差角度检测，3D地图点到两帧相机中心向量夹角余弦值不小于0.9998；
   - 正深度检测，3D地图点深度大于0，需要在相机前方；
   - 重投影误差在两帧相机的重投影误差不大于阈值；
   - 尺寸连续性检测。

5. 如果新生成的3D地图点经过步骤4检测，那么三角测量生成3D地图点成功，将3D地图点与`pKF2`到当前关键帧`mpCurrentKeyFrame`进行数据关联。

这些新生成的3D地图点之后会在**剔除新插入的地图点**步骤进行进一步筛选。

对于当前关键帧`mpCurrentKeyFrame`，其对应关联的3D地图点是由两个关键帧建立的，但是其也有可能在其他关键帧中存在匹配关系。比如某3D地图点mp1是由`mpCurrentKeyFrame`和`pKF2`的某个匹配对（idxCurrent, idx2）三角测量得到，但是`pKF2`和`pKF1`存在匹配对(idx2, idx1)，也由此生成3D地图点mp2，那么此时（idxCurrent，idx1）也可能是匹配的，此时就要融合mp1和mp2。这个步骤在函数`SearchInNeighbors()`中实现，大致思路为：

1. 将当前关键帧的3D地图点投影到相邻（一级相邻和二级相邻）关键帧中，若相邻关键帧中有对应的匹配特征点，检查该特征点是否已经有了对应的3D地图点：

   - 如果有，将这两个3D地图点融合（选择观测数最多的）
   - 如果没有，为该特征点添加该3D地图点；

   匹配过程在`matcher.Fuse(pKFi,vpMapPointMatches);`实现。

2. 将所有相邻关键帧的3D地图点放入集合`vpFuseCandidates`，投影到当前关键帧中进行匹配融合：

   `matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates)` 。

3. 更新当前关键帧地图点的描述子，深度和观测方向等；

4. 更新当前关键帧与其他关键帧的连接关系。

## Local BA

局部BA对当前关键帧位姿，当前关键帧一级相连的关键帧位姿，以及这些关键帧观测到的地图点进行集束优化。参与优化的还有不属于上述关键帧，但是能观测到这些地图点的关键帧，这些关键帧的位姿在优化时不变。在优化过程中，会剔除观测外点。

在跟踪线程中只进行了motion-only BA，只对相机位姿进行了优化，这里Local BA对位姿和地图点进行了优化，保证了SLAM的实时性和准确性。

## 剔除关键帧

由于在跟踪线程插入的关键帧条件比较松，因此还需要进一步剔除关键帧，来保证之后进行优化时不至于复杂度过大，同时也减小了随着运行时间过程，所需存储空间过大。

剔除条件：如果某个关键帧90%的地图点能被至少3个其他关键帧观测到，那么这个关键帧时冗余的：

```c++
if(nRedundantObservations>0.9*nMPs)
	pKF->SetBadFlag();
```

最后，会将当前关键帧送入闭环检测队列中。



