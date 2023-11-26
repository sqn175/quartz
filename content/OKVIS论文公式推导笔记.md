---
title: OKVIS论文公式推导笔记
date: 2017-10-17
tags:
  - SLAM
  - VIO
---

OKVIS是早期（2015年）开源的基于关键帧的视觉-惯导SLAM方案，是众多SLAM研究者入门必须了解的VIO方案之一，放在今天OKVIS的数据集测试结果仍然非常优秀。这篇博文主要针对OKVIS论文[1]整理其中的理论推导。

## 定义
定义需要估计的机器人状态$\mathbf x_R$：

$$
\mathbf x_R:=[{}_W\mathbf r_S^T,\mathbf q_{WS}^T,{}_S\mathbf v^T,\mathbf b_g^T,\mathbf b_a^T]^T
$$

其中${}_W \mathbf r_S^T \in \mathbb R^3$
是机器人在世界坐标系下的三维位置；单位四元数$\mathbf q_{WS}^T \in S^3$是机器人在世界坐标系下的朝向，对应的旋转矩阵为$\mathbf C_{WS} \in \mathbb R^{4 \times4}$。进一步将上述状态分为两个部分，位姿部分$\mathbf x_T:=[{}_W \mathbf r_S^T,\mathbf q_{WS}^T]$和速度/零偏部分$\mathbf x_{sb}:=[{}_S \mathbf v^T,\mathbf b_g^T,\mathbf b_a^T]$。

考虑相机外参的话，将其定义为$\mathbf x_{C_i}:=[{}_S\mathbf r_{C_i}^T,\mathbf q_{SC_i}^T]^T$，其可以是通过事先校正得到的固定值，也可以当做一个一阶高斯变量来估计，因为外参只受到温度影响。

## 非线性优化
考虑代价函数$\mathbf F(\mathbf x)$，$\mathbf x$是待估计的状态，$\mathbf x$的最优值可通过最小化代价函数求得：

$$
\mathbf x^*=\underset x {\mathrm{argmin}}\mathbf F(\mathbf x)
$$

由于代价函数一般是一个复杂的非线性函数，一般从一个初始值出发，不断地更新当前的状态估计值，使得状态估计值收敛。具体为：

1. 将$\mathbf F(\mathbf x)$在状态估计值$\mathbf {\check x}$近似为一个容易处理的解析形式；
2. 求解增量$\Delta \mathbf x$;
3. 更新当前状态估计值$\mathbf {\check x}\leftarrow \mathbf {\check x}+\Delta \mathbf x$;
4. 迭代，直至状态估计量收敛。

在高斯分布的假设下，代价函数通过最大似然估计可以得到，是一个最小二乘问题。OKVIS中代价函数将视觉+惯导的定位和建图问题建模到一个函数中：

$$
\mathbf F(\mathbf x):=\underbrace{\sum_{i=1}^I \sum_{k=1}^K \sum_{j \in\mathcal J(i,k)} \mathbf e_r^{i,j,k \;T}\mathbf W_r^{i,j,k}  \mathbf e_r^{i,j,k}}_{视觉}+ \underbrace{\sum_{k=1}^{K-1} \mathbf e_s^{k \;T} \mathbf W_s^k \mathbf e_s^k}_{惯导}
$$

其中$\mathbf e_r^{i,j,k}$为重投影误差，$\mathbf e_s^k$为IMU误差项。$i$为系统中的相机下标，$k$为相机采集图像的下标，$j$为路标点的下标，$\mathcal J(i,k)$为相机$i$观测在第$k$帧图像观测到的路标点,$\mathbf W$为信息矩阵。谷歌提供了***ceres库***可以用来求解非线性优化问题。

## 在流形上进行优化

在求解优化问题时，一些状态量往往不在欧式空间中。比如单位四元数不在欧式空间中，其不满足加法交换律，单位四元数加法定义为:

$$
\mathbf q_1 \oplus \mathbf q_2 =\mathbf q_1 \otimes \mathbf q_2
$$

是不可交换的。表达旋转只需要三个自由度，但是四元数有4个自由度，因此需要对四元数增加一个限制。当状态量不在欧式空间时，最小化代价函数求得的增量无法用加法去更新状态估计量[2]。

流形，是局部具有欧式空间性质的空间。在这种情况下，通常是在original space的限制下导出的流形上进行优化。也叫作"optimization in the error-space", "optimization in the tangent space", "optimization through local parametrization"。主旨是在minimal tangent space(Euclidean or error space)上进行更新状态量的步骤，从而保证在对应的original space状态量的更新是封闭的。

机器人对应的最小位姿误差状态量为$\delta \chi_T=[\delta  \mathbf p^T,\delta\alpha^T]^T$，路标点误差$\delta \mathbf l:=[\delta \beta^T,0]^T$，考虑相机外参的话，最小误差状态量为$\delta \chi_C=[\delta  \mathbf p^T,\delta\alpha^T]^T$.

## 重投影误差

重投影误差公式为：

$$
\mathbf e^{j,k}_{\mathbf r}=\mathbf z^{j,k}-\mathbf h(\mathbf T^k_{CS}\mathbf T^k_{SW} {}_W\mathbf l^j)
$$

其中，${}_W\mathbf l^j \in \mathbb R^4$ 是第$j$个路标点在世界坐标系下的齐次坐标表示，第四维值为1。下标$k$表示时刻$k$。$\mathbf z^{j,k} \in \mathbb R^3$表示路标点$j$在时刻$k$图像上的观测值，也是齐次坐标。$\mathbf T^k_{SW}$为变换矩阵，将世界坐标系中的点变换到第$k$次测量时的sensor坐标系中。$\mathbf T^k_{CS}$将第$k$次测量时sensor坐标系中的点变换到相机坐标系中。$\mathbf h()$为相机投影模型。以下在公式中省略时刻$k$。

为了进行非线性优化过程以及之后的边缘化步骤，需要求出重投影误差对最小机器人状态误差向量的雅克比矩阵。

### 变换矩阵的线性化

由[3]有旋转矩阵线性化表达为：

$$
\mathbf C \approx (\mathbf I - \delta\phi^\times) \mathbf {\bar C}
$$

其中，扰动$\delta \phi \in \mathbb R^{3\times1}$是$\delta \mathbf q \in \mathbb R^{4\times1}$的最小表示。

变换矩阵

$$
\mathbf T= 
\begin {bmatrix}
\mathbf C & \rho \\
\mathbf 0^T & 1
\end {bmatrix}
$$

其逆代表了逆向变换，

$$
\mathbf T^{-1}=
\begin {bmatrix}
\mathbf C^T & -\mathbf C^T\rho \\
\mathbf 0^T & 1
\end {bmatrix}
$$

定义变量$\mathbf x$：

$$
\mathbf x:=
\begin{bmatrix}
\rho \\
\phi
\end{bmatrix}_{6\times1}
$$

对$\mathbf x$在$\mathbf {\bar x}$进行扰动，

$$
\mathbf {\bar x}:=
\begin {bmatrix}
\mathbf {\rho} \\
\bar \phi
\end {bmatrix}_{6\times1}
$$

扰动量为：

$$
\delta \mathbf x:=
\begin {bmatrix}
\delta \rho\\
\delta \phi
\end{bmatrix}_{6\times1}
$$

那么有：

$$
\begin {aligned}
\mathbf T(\mathbf {\bar x} +\delta \mathbf x) 
&\approx
\begin {bmatrix}
\left (\mathbf I -\delta \phi^\times\right )\mathbf {\bar C} & \bar\rho + \delta \rho \\
\mathbf 0 ^T & 1
\end {bmatrix} \\
& =
\begin {bmatrix}
\mathbf I-\delta\phi^\times & \delta \rho +\delta \phi^\times\bar \rho \\
\mathbf 0^T & 1
\end {bmatrix}
\begin {bmatrix}
\mathbf {\bar C} & \bar \rho \\
\mathbf 0^T  &1
\end {bmatrix} \\
&= 
\begin {bmatrix}
\mathbf I-\delta\phi^\times & \delta \rho +\delta \phi^\times\bar \rho \\
\mathbf 0^T & 1
\end {bmatrix}
\mathbf {\bar T} \\
&=
\left ( \mathbf I-
\begin {bmatrix}
\delta \phi^\times &-\delta\varrho \\
\mathbf 0^T & 0
\end {bmatrix}
\right)\mathbf {\bar T}  \\
\end {aligned}
$$

其中，

$$
\delta \varrho:=\delta\rho+\delta\phi^\times\bar\rho
$$

令：

$$
\delta \mathbf t:=
\begin{bmatrix} 
\delta \varrho \\
\delta \phi
\end {bmatrix}
$$

对$3\times1$列向量$\mathbf r$和$\mathbf s$定义运算符$(\cdot)^\boxplus$：

$$
\begin {bmatrix}
\mathbf r \\
\mathbf s 
\end {bmatrix}^\boxplus:=
\begin {bmatrix}
\mathbf s^\times & -\mathbf r\\
\mathbf 0^T & 0
\end{bmatrix}
$$

我们有：

$$
\mathbf T(\mathbf {\bar x} +\delta \mathbf x)  \approx
(\mathbf I-\delta \mathbf t^\boxplus)\mathbf {\bar T}
$$

类似的，有

$$
\begin {aligned}
\mathbf T(\mathbf {\bar x}+\delta \mathbf x)^{-1}
& \approx
\begin {bmatrix}
\left[( \mathbf I -\delta \phi^\times)\mathbf {\bar  C} \right]^T & 
-\left[( \mathbf I -\delta \phi^\times)\mathbf {\bar C} \right]^T(\bar\rho + \delta \rho) \\
\mathbf 0^T &1
\end {bmatrix} \\
&=
\begin {bmatrix}
\mathbf {\bar C}^T(\mathbf I+\delta\phi^\times) & 
-\mathbf {\bar C}^T(\mathbf I+\delta\phi^\times)\bar\rho-\mathbf {\bar C}^T\delta\rho \\
\mathbf 0^T &1
\end {bmatrix} \\
&=
\begin {bmatrix}
\mathbf {\bar C}^T & -\mathbf {\bar C}^T \bar\rho \\
\mathbf 0^T & 1
\end {bmatrix}
\begin {bmatrix}
\mathbf I+\delta\phi^\times & -(\delta\rho+\delta\phi^\times\bar\rho) \\
\mathbf 0^T &1
\end{bmatrix} 
\end {aligned}
$$

即：

$$
\mathbf T(\mathbf {\bar x}+\delta \mathbf x)^{-1}=
\mathbf {\bar T}^{-1}(\mathbf I+\delta\mathbf t^\boxplus)
$$

设点$P$的齐次坐标为$\mathbf p$为：

$$
\mathbf p:=\begin {bmatrix} \mathbf u \\s \end{bmatrix}
$$

对变换矩阵$\mathbf T^{-1}$进行扰动后，有：

$$
\begin {aligned}
\mathbf T^{-1} \mathbf p
&\approx \mathbf {\bar T}^{-1}\mathbf p+\mathbf {\bar T}^{-1}\delta\mathbf t^\boxplus\mathbf p \\
&= \mathbf {\bar T}^{-1}\mathbf p + \mathbf {\bar T}^{-1}
\begin {bmatrix}
\delta\phi^\times & -\delta\varrho \\
\mathbf 0^T & 0
\end{bmatrix}
\begin{bmatrix}
\mathbf u \\
s
\end{bmatrix} \\
&= \mathbf {\bar T}^{-1}\mathbf p + \mathbf {\bar T}^{-1}
\begin {bmatrix}
- s\delta\varrho+\delta\phi^\times\mathbf u  \\
0
\end {bmatrix} \\
&= \mathbf {\bar T}^{-1}\mathbf p - \mathbf {\bar T}^{-1}
\begin {bmatrix}
s\mathbf I & \mathbf u^\times \\
\mathbf 0^T & \mathbf 0^T
\end{bmatrix}
\begin {bmatrix}
\delta\varrho \\
\delta \phi
\end{bmatrix}
\end {aligned}
$$

定义齐次坐标上的运算符$(\cdot)^\boxminus$为：

$$
\begin {bmatrix}
\mathbf u\\s
\end{bmatrix}^\boxminus=
\begin{bmatrix}
s\mathbf I & \mathbf u^\times \\
\mathbf 0^T & \mathbf 0^T
\end{bmatrix}
$$

那么有

$$
\mathbf {\bar T}^{-1}\delta\mathbf t^\boxplus\mathbf p =- \mathbf {\bar T}^{-1}\mathbf p^\boxminus\delta\mathbf t
$$

同样的，有：

$$
-\delta\mathbf t^\boxplus\mathbf {\bar T}\mathbf p=(\mathbf {\bar T}\mathbf p)^\boxminus\delta\mathbf t
$$

### 雅克比矩阵

在这里需要注意的是，机器人位姿状态量为$\mathbf x_T:=[{}_W\mathbf r_S^T,\mathbf q_{WS}^T]$ ，对应的变换矩阵为

$$
\mathbf T_{WS}=
\begin {bmatrix}
\mathbf C_{WS} & {}_W\mathbf r_S^T \\
\mathbf 0^T & 1
\end {bmatrix}
$$

其中$\mathbf C_{WS}$是四元数$\mathbf q_{WS}$对应的旋转矩阵。有$\mathbf T_{SW}=\mathbf T_{WS}^{-1}$，因此重投影误差公式为：

$$
\mathbf e^{j,k}_{\mathbf r}=\mathbf z^{j,k}-\mathbf h(\mathbf T^{k\;\;-1}_{SC}\mathbf T^{k\;\;-1}_{WS} {}_W\mathbf l^j)
$$

对其线性化,，设$\delta \mathbf t_T \in \mathbb R^{6\times1}$是$\mathbf T_{WS}$在值$\mathbf {\bar T}_{WS}$的扰动，$\delta \mathbf t_C \in \mathbb R^{6\times1}$是$\mathbf {T}_{SC}$在值$\mathbf {\bar T}_{SC}$的扰动，$\delta \mathbf l\in\mathbb R^{4\times1}$是${}_W\mathbf {l}^j$在值${}_W\mathbf {\bar l}^j$的扰动值。对重投影误差进行扰动得到：

$$
\mathbf e^{j,k}_{\mathbf r}
\approx 
\mathbf z^{j,k}-
\mathbf h\left(
\mathbf {\bar T}_{CS}^{-1} (\mathbf I+\delta \mathbf t_C^\boxplus)
\mathbf {\bar T}_{WS}^{-1}(\mathbf I+\delta\mathbf t_T^\boxplus)
({}_W\mathbf {\bar l}^j +\delta \mathbf l)
\right)
$$

展开，去除扰动项的积，得到：

$$
\mathbf e^{j,k}_{\mathbf r}
\approx
\mathbf z^{j,k}
-\mathbf h
\left(
\mathbf {\bar T}_{CS}\mathbf {\bar T}_{SW}{}_W\mathbf {\bar l}^j 
+ \mathbf {\bar T}_{CS}\mathbf {\bar T}_{SW}\delta\mathbf l 
+ \mathbf {\bar T}_{CS} \mathbf {\bar T}_{WS}^{-1}\delta\mathbf t_T^\boxplus{}_W\mathbf {\bar l}^j 
+ \mathbf {\bar T}_{SC}^{-1} \delta\mathbf t_C^\boxplus {}_S{\bar l}^j
\right)
$$

由之前推导有：

$$
\mathbf e^{j,k}_{\mathbf r}
\approx 
\mathbf z^{j,k}-
\mathbf h\left(
\mathbf {\bar T}_{CS}\mathbf {\bar T}_{SW}{}_W\mathbf {\bar l}^j 
+ \mathbf {\bar T}_{CS}\mathbf {\bar T}_{SW}\delta\mathbf l
- \mathbf {\bar T}_{CS}\mathbf {\bar T}_{WS}^{-1}{}_W\mathbf {\bar l}^{j\;\boxminus}\delta \mathbf t_T
- \mathbf T_{SC}^{-1}{}_S\mathbf l^{j\;\boxminus}\delta\mathbf t_C
\right)
$$

最后令

$$\mathbf J_r:=\frac{\delta\mathbf h(v)}{\delta v}| _{v=\mathbf {\bar T}_{CS}\mathbf {\bar T}_{SW}\mathbf {\bar l}^j}$$

得到：

$$
\mathbf e^{j,k}_{\mathbf r}
\approx 
\mathbf z^{j,k}
-\mathbf h(\mathbf {\bar T}_{CS}\mathbf {\bar T}_{SW}{}_W\mathbf {\bar l}^j )
-\mathbf J_r\mathbf {\bar T}_{CS}\mathbf {\bar T}_{SW}{}\delta\mathbf l
+\mathbf J_r  \mathbf {\bar T}_{CS}\mathbf {\bar T}_{WS}^{-1}{}_W\mathbf {\bar l}^{j\;\boxminus}\delta \mathbf t_T
+\mathbf J_r \mathbf T_{SC}^{-1}{}_S\mathbf l^{j\;\boxminus}\delta\mathbf t_C
$$

所以有：

$$
\begin {aligned}
\frac {\partial\mathbf e^{j,k}_{\mathbf r}}{\partial\delta \chi _T}
&=
\frac {\partial\mathbf e^{j,k}_{\mathbf r}}{\partial\delta \mathbf t _T}
\frac {\partial\delta \mathbf t _T}{\partial\delta \chi _T} \\
&=
\mathbf J_r  \mathbf {\bar T}_{CS}\mathbf {\bar T}_{WS}^{-1}{}_W\mathbf {\bar l}^{j\;\boxminus}  \frac {\partial\delta \mathbf t _T}{\partial\delta \chi _T} 
\end {aligned}
$$

将上式展开得到：

$$
\begin {aligned}
\frac {\partial\mathbf e^{j,k}_{\mathbf r}}{\partial\delta \chi _T}
&=
\mathbf J_r  \mathbf {\bar T}_{CS}
\begin {bmatrix}
\mathbf {\bar C}_{SW} & -\mathbf {\bar C}_{SW}{}_W\mathbf {\bar r}_S \\
\mathbf 0^T & 1
\end {bmatrix}
\begin{bmatrix}
{}_W\mathbf {\bar l_4}\mathbf I & {}_W\mathbf{\bar l}_{1:3}^\times \\
\mathbf 0^T & \mathbf 0^T
\end{bmatrix}
\begin {bmatrix}
\mathbf I & -{}_W\mathbf {\bar r}_S^{\;\times} \\
\mathbf 0^T &\mathbf 1^T
\end {bmatrix}\\
&=
\mathbf J_r  \mathbf {\bar T}_{CS}
\begin {bmatrix}
\mathbf {\bar C}_{SW} {}_W\mathbf{\bar l}_4 & \mathbf {\bar C}_{SW}[{}_W\mathbf {\bar l}_{1:3}-{}_W\mathbf {\bar r}_S\;{}_W\mathbf {\bar l}_4]^\times \\
\mathbf 0^T & \mathbf 0^T
\end {bmatrix}
\end {aligned}
$$

同理，有

$$
\frac {\partial\mathbf e^{j,k}_{\mathbf r}}{\partial\delta \chi _C}
=
\mathbf J_r
\begin {bmatrix}
\mathbf {\bar C}_{CS} {}_S\mathbf{\bar l}_4 & \mathbf {\bar C}_{CS}[{}_W\mathbf {\bar l}_{1:3}-{}_S\mathbf {\bar r}_C\;{}_S\mathbf {\bar l}_4]^\times \\
\mathbf 0^T & \mathbf 0^T
\end {bmatrix}
$$

显然，

$$
\frac {\partial\mathbf e^{j,k}_{\mathbf r}}{\partial\delta \chi _L}
=
-\mathbf J_r  \mathbf {\bar T}_{CS}
\begin {bmatrix}
\mathbf {\bar C}_{SW} \\
\mathbf 0^T
\end {bmatrix}
$$

## 参考
[1] Leutenegger, Stefan, et al. "Keyframe-based visual–inertial odometry using nonlinear optimization." The International Journal of Robotics Research 34.3 (2015): 314-334.

[2]Course on SLAM Institut de Robòtica i Informàtica industrial  [[PDF](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/toolbox/courseSLAM.pdf)]

[3] Furgale, Paul Timothy. *Extensions to the visual odometry pipeline for the exploration of planetary surfaces*. Diss. University of Toronto, 2011. 















