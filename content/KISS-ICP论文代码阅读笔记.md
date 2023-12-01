---
date: 2023-11-27
draft: false
---
# 背景
我们站在现在的时间点看，可以喊出这句话：*机器人以及自动驾驶的发展都离不开激光雷达*。尽管在L2/L3智能驾驶领域还存在纯视觉vs激光雷达技术路线之争，但是在机器人及L4自动驾驶领域，激光雷达已经是一个不可或缺的核心传感器。相较于视觉，激光雷达作为传感器本身，对于环境变化的鲁棒性以及测量数据的精准度都更好，从而降低了算法开发的难度，增加了易用性。如下图所示，分别是AutoX RoboTaxi和未来机器人无人叉车，均使用了多线激光雷达。
![[AutoXDriverlessRoboTaxiCloseup2.jpg]]
![[未来机器人.png]]

多线激光雷达种类较多，按照旋转机构，可分为机械式（如速腾、禾赛、Ouster、Velodyne等品牌的多线雷达）、半固态（Livox多线雷达）、以及纯固态激光雷达（基本用于感知避障）。其中机械式以及半固态多线激光雷达因视场角大，天然适合用于定位导航。

激光里程计是激光雷达定位导航里的核心部分，其通过扫描帧配准，推算出两帧雷达点云之间的相对位姿，完成航迹推算的功能。目前实现该功能最基础的ICP (Iterative Closest Point)配准算法在1992年就被提出[BeslAndMcKay1992](http://www.cvl.iis.u-tokyo.ac.jp/class2004/wedenesday/report/besl.pdf)。ICP算法目前在主流的3D点云处理库都已被实现，如[Open3D ICP](http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)和[PCL ICP](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html#details)。目前这类方法都存在鲁棒性的问题，可能没法开箱即用，需要根据传感器参数、应用环境等进行调优。

因此，今天来研读一篇发在RAL的论文"KISS-ICP"，号称针对大部分场景，无需调参，开箱即用。
> *Vizzo I, Guadagnino T, Mersch B, et al. Kiss-icp: In defense of point-to-point icp–simple, accurate, and robust registration if done the right way[J]. IEEE Robotics and Automation Letters, 2023, 8(2): 1029-1036.*

其同步公开了源码[Github kiss-icp](https://github.com/PRBonn/kiss-icp)，star数已达到1.2k，看来确实够好用。

# 整体框架
**问题定义**：给定3D激光雷达扫描帧$\mathcal{P}=\left\{\boldsymbol{p}_i \mid \boldsymbol{p}_i \in \mathbb{R}^3\right\}$，即表达在激光雷达坐标系的点云，目标是计算此扫描帧时刻$t$对应的激光雷达全局位姿$\boldsymbol{T}_t \in S E(3)$。

KISS-ICP取名意思是keep It Small and Simple，也就是尽可能让算法流程简易化，其实就是动态参数化的ICP方法。整体流程分为四步：
1. **[去运动畸变](#去运动畸变)**。对运动过程中的扫描点云去畸变；
2. **[降采样](#降采样)**。降低计算量的重要步骤。
3. **[估计对应点](#估计对应点)**。基于动态阈值方法，关联输入点云与局部地图点云，找到点之间的对应关系；
4. **[点云配准](#点云配准)**。基于更加鲁棒的ICP优化方法计算相对位姿，并将输入点云融合到局部地图中。

# 去运动畸变
对于实际应用，激光雷达在扫描过程中，载体也在运动。激光雷达采集一帧数据过程中，采集第一个点的时刻与最后一个点的时刻，激光雷达传感器自身坐标系已经发生了变化。激光雷达采集一帧数据时间这个可以类比卷帘相机曝光时间。因此，需要对点云去运动畸变。如果直接将激光雷达输出的点云送到算法进行处理，会影响到定位精准度。

主流去畸变方法是基于恒速模型、基于轮式编码器和基于IMU三类方法。基于恒速模型是最简单的方案，其不需要额外传感器，也不需要复杂的多传感器时空同步标定等。KISS-ICP采用恒速模型，其认为激光雷达采集速率已经很高，且机器人应用速度比较低，加减速带来的影响很小。

令时刻$t$的位姿为$\boldsymbol{T}_{t}=\left(\boldsymbol{R}_{t}, \boldsymbol{t}_{t}\right)$ ，旋转矩阵$\boldsymbol{R}_t \in S O(3)$，平移向量$\boldsymbol{t}_t \in \mathbb{R}^3$。已知时刻$t-2$和时刻$t-1$的位姿，则$t-1$到$t-2$的相对位姿为：
$$
\mathrm{T}_{\text {pred }, t}=\left[\begin{array}{cc}
\boldsymbol{R}_{t-2}^{\top} \boldsymbol{R}_{t-1} & \boldsymbol{R}_{t-2}^{\top}\left(t_{t-1}-t_{t-2}\right) \\
0 & 1
\end{array}\right]
$$
则$t$时刻对应的速度$\boldsymbol{v}_t$、角速度$\boldsymbol{\omega}_t$可由恒速假设计算为：
$$
\begin{aligned}
\boldsymbol{v}_t & =\frac{\boldsymbol{R}_{t-2}^{\top}\left(\boldsymbol{t}_{t-1}-\boldsymbol{t}_{t-2}\right)}{\Delta t}, \\
\boldsymbol{\omega}_t & =\frac{\log \left(\boldsymbol{R}_{t-2}^{\top} \boldsymbol{R}_{t-1}\right)}{\Delta t},
\end{aligned}
$$
其中$\Delta t$是两时刻差，$\text { Log: } S O(3) \rightarrow \mathbb{R}^3$ 运算将$SO(3)$映射为Axis-angle表示$\mathbf{R}_{3 \times 3} \mapsto \boldsymbol \omega$，详细计算可参考[3D旋转变换](http://motion.pratt.duke.edu/RoboticSystems/3DRotations.html#Converting-from-a-rotation-matrix)。

雷达一帧的扫描时间为$\Delta t$，对于点云中每个点$\boldsymbol{p}_i \in \mathcal{P}$，其采样时刻相对于一帧第一个点的相对时间记为$s_i \in[0, \Delta t]$。基于相对时间和计算出的恒定速度，可以对点云去畸变。去畸变后的点云记为$\mathcal{P}^*$，去畸变后的点$\boldsymbol{p}_i^* \in \mathcal{P}^*$通过坐标变换，计算为：
$$
\boldsymbol{p}_i^*=\operatorname{Exp}\left(s_i \boldsymbol{\omega}_t\right) \boldsymbol{p}_i+s_i \boldsymbol{v}_t
$$
其中$\text { Exp: } \mathbb{R}^3 \rightarrow S O(3)$ 运算将Axis-angle表示映射为旋转矩阵，详细计算同样参考[3D旋转变换](http://motion.pratt.duke.edu/RoboticSystems/3DRotations.html#Converting-from-a-rotation-matrix)。

# 降采样
论文基于体素栅格来降采样，每个体素尺寸为$v\times v \times v$，每个体素中存储了原始点云。基于体素的降采样分为两步：
1. 降采样得到$\mathcal{P}_{\text {merge }}^*$。通过设定体素尺寸为$\alpha v$ 其中$\alpha \in(0.0,1.0]$ ，并且每个体素只保留一个点；这部分的降采样点后续会用来更新局部地图，所以不能一开始降采样太狠，否则后续局部地图匹配不鲁棒；
2. 继续降采样得到$\hat{\mathcal{P}}^*$。为了保证ICP的计算效率，需要进一步降采样。通过设定体素尺寸为$\beta v$，且$\beta \in[1.0,2.0]$，每个体素保留一个点。$\hat{\mathcal{P}}^*$采样自$\mathcal{P}_{\text {merge }}^*$，所以$\hat{\mathcal{P}}^* \subseteq \mathcal{P}^*$。

实现代码如下：
```c
KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    // 体素尺寸
    const auto voxel_size = config_.voxel_size;
    // 降采样1
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    // 降采样2
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}
```

其中`VoxelDownsample`实现代码如下：

```c
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame, double voxel_size) {
    tsl::robin_map<Voxel, Eigen::Vector3d, VoxelHash> grid;
    grid.reserve(frame.size());
    for (const auto &point : frame) {
        const auto voxel = Voxel((point / voxel_size).cast<int>());
        if (grid.contains(voxel)) continue;
        grid.insert({voxel, point});
    }
    std::vector<Eigen::Vector3d> frame_dowsampled;
    frame_dowsampled.reserve(grid.size());
    for (const auto &[voxel, point] : grid) {
        (void)voxel;
        frame_dowsampled.emplace_back(point);
    }
    return frame_dowsampled;
}
```

# 估计对应点

ICP算法第一步是找到Source点云与Target点云中的对应点，第二步是针对对应点 (Correspondence)，构建优化算法配准求解位姿态。论文中激光里程计遵循frame-to-map流程，也就是Source点云是激光点云帧，Target点云是局部地图，相比frame-to-frame更加鲁棒。

局部地图用体素栅格表示，具体对应代码中的`VoxelHashMap`类。有以下特点：

- 每个体素最多保留$N_{max}$个点。
- 更新时，将新的点云加入到体素中。如果体素已包含$N_{max}$个点，则这个体素不会被更新。
- 同时，为了保证计算实时性，局部地图只维护距离当前位姿$r_{max}$内的体素。

我们先看第一步，对于Source点云中的一个点，如何在Target点云中找到对应点？即如何估计对应点。方案是需要求解最近邻问题。论文中采用体素哈希表和快速最近邻搜索方案来求解这个问题。

## 体素哈希表

体素哈希表可以简单理解为把空间3D点分块后，对每个块根据其3D坐标值建立索引，这样就像图像处理一样，可以根据索引值快速access到对应的点云。

局部地图的核心数据结构定义为：

```c
tsl::robin_map<Voxel, VoxelBlock, VoxelHash> map_;
```
`Voxel, VoxelBlock, VoxelHash`对应的定义分别为：

```c
using Voxel = Eigen::Vector3i;

struct VoxelBlock {
    // buffer of points with a max limit of n_points
    std::vector<Eigen::Vector3d> points;
    int num_points_;
    inline void AddPoint(const Eigen::Vector3d &point) {
        if (points.size() < static_cast<size_t>(num_points_)) points.push_back(point);
    }
};
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};
```

其中`Voxel`是`map_`对应的索引值，等于坐标值除以体素尺寸：

```c
auto voxel = Voxel((point / voxel_size_).template cast<int>());
```

`VoxelBlock`存储了Voxel内的原始点云。

`VoxelHash`定义了Hash方法：
$$
\operatorname{hash}(x, y, z)=(x\cdot p_1 \textbf { xor } y\cdot p_2 \textbf { xor } z\cdot p_3) \bmod n
$$
其中$p_1,p_2,p_3$是大素数，避免hash冲突，可参考论文[Optimized Spatial Hashing](https://matthias-research.github.io/pages/publications/tetraederCollision.pdf)。

## 快速最近邻搜索

对于Source点云中$p$，可以根据建立的体素哈希表，查找该点在Target点云中的空间相邻体素，然后从体素中搜索与$p$距离最近的点，作为最近邻。处理代码如下所示：

```c
// Lambda Function to obtain the KNN of one point, maybe refactor
    auto GetClosestNeighboor = [&](const Eigen::Vector3d &point) {
        auto kx = static_cast<int>(point[0] / voxel_size_);
        auto ky = static_cast<int>(point[1] / voxel_size_);
        auto kz = static_cast<int>(point[2] / voxel_size_);
        std::vector<Voxel> voxels;
        // 从27个邻接Voxels中获取
        voxels.reserve(27);
        for (int i = kx - 1; i < kx + 1 + 1; ++i) {
            for (int j = ky - 1; j < ky + 1 + 1; ++j) {
                for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                    voxels.emplace_back(i, j, k);
                }
            }
        }

        using Vector3dVector = std::vector<Eigen::Vector3d>;
        Vector3dVector neighboors;
        neighboors.reserve(27 * max_points_per_voxel_);
        std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
            auto search = map_.find(voxel);
            if (search != map_.end()) {
                const auto &points = search->second.points;
                if (!points.empty()) {
                    for (const auto &point : points) {
                        neighboors.emplace_back(point);
                    }
                }
            }
        });
		// 在上一个循环里就可以把最近邻sort出来，不用再遍历一遍，是否更优？
        Eigen::Vector3d closest_neighbor;
        double closest_distance2 = std::numeric_limits<double>::max();
        std::for_each(neighboors.cbegin(), neighboors.cend(), [&](const auto &neighbor) {
            double distance = (neighbor - point).squaredNorm();
            if (distance < closest_distance2) {
                closest_neighbor = neighbor;
                closest_distance2 = distance;
            }
        });

        return closest_neighbor;
    };
```

## 动态最近邻阈值

在最近邻搜索过程中，可能存在异常点，虽然其距离是最近的，但不可用。因此，需要引入一个**阈值$\tau$**，若对应点对的距离超过这个阈值，则对应点对是无效的。这个阈值的设计很讲究，设小了，可能因为错误匹配点ICP失败，设大了，处理时间大大增加。初始位姿大误差、动态环境、传感器噪声等都会影响到阈值的选择。这是各类ICP算法中最重要的一个调优参数。

通常做法是根据应用场景，调优出一个固定的阈值，比如1m。但这限制了算法的通用性。因此KISS-ICP提出动态最近邻阈值方法：根据恒速模型预测的结果与当前ICP估计出来的结果的差异，动态调整最近邻阈值。这个思路比较直接，因为差异比较大的话，说明雷达在做加减速运动，机动性较强，需要把阈值宽松一点。

假设$t-1$时刻预测的位姿为$\boldsymbol T_{pred,t-1}$，ICP顺利解算得到的位姿为$\hat {\boldsymbol T}_{t-1}$，则位姿差$\Delta \boldsymbol T=\Big(\boldsymbol T_{pred,t-1}\Big)^{-1}\hat {\boldsymbol T}_{t-1}=[\Delta \boldsymbol R \space \vert \space \Delta \boldsymbol t]$ 。根据三角不等式，由位姿差带来的估计出的所有点云对的位置偏差上限为：
$$
\|\Delta \boldsymbol R \boldsymbol{p}+\Delta \boldsymbol{t}-\boldsymbol{p}\|_2 \leq \delta_{\mathrm{rot}}(\Delta \boldsymbol  R)+\delta_{\mathrm{trans}}(\Delta \boldsymbol{t})
$$
令$\delta(\Delta \mathrm{T})=\delta_{\mathrm{rot}}(\Delta \boldsymbol R)+\delta_{\text {trans }}(\Delta \boldsymbol t)$，其中$\delta_{\mathrm{rot}}(\Delta \boldsymbol  R)$是以最远距离点$r_{max}$及误差角度$\theta$计算出的弦长，$\delta_{\mathrm{trans}}(\Delta \boldsymbol{t})$为平移误差：
$$
\begin{aligned}
& \delta_{\text {rot }}(\Delta R)=2 r_{\max } \sin \Big(\frac{1}{2} \cdot \space \underbrace{\arccos \big(\frac{\operatorname{tr}(\Delta R)-1}{2}\big)}_\theta \Big) \\
& \delta_{\text {trans }}(\Delta \boldsymbol{t})=\|\Delta \boldsymbol{t}\|_2 .
\end{aligned}
$$
如下图所示：
![[KISS-ICP.png]]

下一步是如何根据$\delta(\Delta \mathrm{T})$估计出当前时刻$t$的阈值$\tau_t$ ？论文将$\delta$建模为零均值高斯噪声，$\delta \sim \mathcal(0,\sigma_t)$ ，那么$\tau_t$定义为3-sigma限：$\tau_t=3 \sigma_t$。

标准差$\sigma_t$计算如下：
$$
\sigma_t=\sqrt{\frac{1}{\left|\mathcal{M}_t\right|} \sum_{i \in \mathcal{M}_t} \delta\left(\Delta \boldsymbol T_i\right)^2},
$$
其中$\mathcal{M}_t$集合定义如下：
$$
\mathcal{M}_t=\left\{i \mid i<t \wedge \delta\left(\Delta \boldsymbol T_i\right)>\delta_{\min }\right\}
$$
包含了到当前时刻的所有点，且排除了静止、恒速等情况下的差异小的情况，防止标准差被快速拉小。（**是否需要包含所有点？对于突然剧烈运动工况，参数调整可能反应不过来。**）

# 点云配准
点云配准采用传统的点到点ICP方法。没有采用特征法，比如法向量、曲率、特征点等方案，是为了算法的通用性，避免对不同雷达和环境进行单独适配。点到点ICP配准步骤如下：
1. 点云预测。将当前时刻$t$雷达坐标系点云$\hat{\mathcal{P}}^*$通过预测的位姿，转换到全局坐标系，得到待配准点云$\mathcal{S}$：
$$
\mathcal{S}=\left\{s_i=\boldsymbol {T}_{t-1} \boldsymbol {T}_{\text {pred }, t} \boldsymbol{~p} \mid \boldsymbol{p} \in \hat{\mathcal{P}}^*\right\} .
$$
2. ICP迭代配准。
	 a. 估计局部地图$\mathcal{Q}=\left\{\boldsymbol{q}_i \mid \boldsymbol{q}_i \in \mathbb{R}^3\right\}$与待配准点云$\mathcal{S}$的点云对$\mathcal{C}\left(\tau_t\right)$，根据[估计对应点](#估计对应点)章节；
    b. 求解当前位姿更新量$\Delta \boldsymbol{T}_{\text {est }, j}$:	   
	$$
       \Delta \boldsymbol{T}_{\mathrm{est}, j}=\underset{\boldsymbol{T}}{\operatorname{argmin}} \sum_{(s, q) \in \mathcal{C}\left(\tau_t\right)} \rho\left(\|\boldsymbol{T} s-\boldsymbol{q}\|_2\right)
   $$
	c. 更新待配准点云$\mathcal{S}$：
	$$
	\left\{\boldsymbol{s}_i \leftarrow \Delta \boldsymbol{T}_{\text {est }, j} \boldsymbol{s}_i \mid \boldsymbol{s}_i \in \mathcal{S}\right\}
	$$
	d. 重复以上步骤，直到收敛。收敛条件是$\Vert \Delta \boldsymbol{T}_{\text {est }, j}\Vert < \gamma$，$\gamma$设置为$0.0001$。

3. 更新得到最终的位姿：
   $$
   \boldsymbol{T}_t=\Delta \boldsymbol{T}_{\boldsymbol{icp}, t} \boldsymbol{~T}_{t-1} \boldsymbol{~T}_{\text {pred }, t}
   $$
   其中$\Delta \boldsymbol{T}_{\mathrm{icp}, t}=\prod_j \Delta \\boldsymbol{T}_{\mathrm{est}, j}$.

4. 更新局部地图，通过更新后的位姿将$\mathcal{P}_{\text {merge }}^*$融合到局部地图。

对应的代码如下所示：

```c
Sophus::SE3d RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                           const VoxelHashMap &voxel_map,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance,
                           double kernel) {
    if (voxel_map.Empty()) return initial_guess;

    // 1. 点云预测
    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    // 2. ICP迭代匹配
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        // 2.a 估计对应点
        const auto &[src, tgt] = voxel_map.GetCorrespondences(source, max_correspondence_distance);
        // 2.b 求解位姿更新量
        const auto &[JTJ, JTr] = BuildLinearSystem(src, tgt, kernel);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        // 2.c 更新点云
        TransformPoints(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // 2.d 收敛条件
        if (dx.norm() < ESTIMATION_THRESHOLD_) break;
    }
    // 3. 最终位姿
    return T_icp * initial_guess;
```

# 结语

具体的实验结果可以参考论文，也有众多SLAM网友们在代码开源后纷纷测试了自己的bag包，整体效果是很不错的，确实简单易用。但目前KISS-ICP还存在以下几个主要方面：

- 主要针对稠密点云激光雷达设计，对于32线或者Livox雷达，可能运行失败，需要调整降采样模块，避免过分降采样；
- 没有考虑动态环境，比如行人较多的商场、学校；
- 没有对雨雪雾天气下的点云做预处理，当然这个要求可能有点过分，毕竟这是激光雷达传感器要去解决的问题；
- 还是一个单纯的里程计系统，无法作为建图工具；

不过作为改进版本的ICP，拿来深入理解学习ICP原理，跑一跑测试数据包，还是挺不错。