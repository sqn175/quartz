# VINS中的IMU 预积分

石钦 2018.1.18

##IMU动态方程

设真值为$\bar x$， 估计值(均值)为$x$ 。

设IMU陀螺仪和加速度计的观测值为 $\hat \omega$ 和 $\hat a$ ，有：

$$
\begin{aligned}
\hat a _t &= \bar{a_t} + \bar{b_{a_t}} + \bar{R_w^t} g^w+n_a \\
\hat \omega _t &= \bar {\omega _t} + \bar{ b_{w_t}} + n_w 
\end{aligned}
$$

IMU在载体系下的测量值，由载体运动和重力加速度决定，同时受到加计零偏 $a_t$ 和陀螺仪零偏 $b_{w_t}$ 以及对应的加性噪声影响。假设加性噪声为零均值高斯噪声，$n_a \sim N(0,\sigma _a^2)$ , $n_\omega \sim N(0, \sigma_\omega ^2)$ .零偏建模为随机游走模型，其导数为高斯白噪声， $n_{b_a} \sim N(0, \sigma_{b_a}^2)$， $n_{b_\omega} \sim N(0, \sigma_{b_\omega}^2)$：

$$
\dot b_{a_t} = n_{b_a}, \quad \dot b_{\omega_t} = n_{b_\omega}
$$

假设在时刻 $t_k$ 和 $t_{k+1}$ 载体接收到图像帧 $b_k$ 和 $b_{k+1}$ ，$t_{k+1}$时刻的载体状态包括位置，速度和朝向可由在时间段$[t_k,t_{k+1}]$ IMU观测值积分得到：

$$
p_{b_{k+1}}^w = p_{b_k}^w + v_{b_k}\Delta t_k + \iint_{t \in [t_k,t_{k+1}]} (R_t^w(\hat a_t-b_{a_t}-n_a) - g^w)dt^2 \\
v_{b_{k+1}}^w = v_{b_k}^w + \int_{t \in [t_k,t_{k+1}]} (R_t^w(\hat a_t-b_{a_t}-n_a) - g^w)dt \\
q_{b_{k+1}}^w = q_{b_k}^w \otimes \int_{t \in [t_k,t_{k+1}]} \frac1 2 \Omega (\hat \omega _t - b_{w_t} - n_w)q_t^{b_k}dt 
$$

其中，

$$
\Omega (\omega) = 
\begin{bmatrix}
-\lfloor \omega \rfloor _\times  & \omega \\
-\omega^T                                    & 0
\end{bmatrix} ,

\lfloor \omega \rfloor _\times = 
\begin{bmatrix}
0 & -\omega_z & \omega_y \\
\omega_z & 0 & -\omega_x \\
-\omega_y & \omega_x & 0
\end{bmatrix}
$$

注：通过角速度来递推四元数[参考][https://www.princeton.edu/~stengel/MAE331Lecture9.pdf]：

$$
q_{b_{k+1}}^w = q_{b_k} \otimes \int_{t \in [t_k,t_{k+1}]} dq(t)dt \\
\frac {dq(t)} {dt} = \frac 1 2 \Omega(\omega)q(t)
$$

## 连续时间预积分

为什么需要预积分。

$$
R_w^{b_k}p_{b_{k+1}}^w = R_w^{b_k}(p_{b_k}^w + v_{b_k}\Delta t_k - \frac 1 2 g^w \Delta t_k^2) + \alpha_{b_{k+1}}^{b_k} \\
R_w^{b_k}v_{b_{k+1}}^w = R_{b_k}^w( v_{b_k}^w-g^w\Delta t_k)+ \beta_{b{k+1}}^{b_k} \\
q_w^{b_k} \otimes q_{b_{k+1}}^w = \gamma_{b_{k+1}}^{b_k}
$$

其中，

$$
\alpha_{b_{k+1}}^{b_k} =\iint_{t \in [t_k,t_{k+1}]} R_t^{b_k}(\hat a_t-b_{a_t}-n_a)dt^2 \\
\beta_{b{k+1}}^{b_k} = \int_{t \in [t_k,t_{k+1}]} R_t^{b_k}(\hat a_t-b_{a_t}-n_a)dt \\
\gamma_{b_{k+1}}^{b_k} =\int_{t \in [t_k,t_{k+1}]} \frac1 2 \Omega (\hat \omega _t - b_{w_t} - n_w)\gamma _t^{b_k}dt
$$

设$$b_k$$为参考系，那么预积分项只和IMU观测值和零偏有关。在进行状态估计时，如果零偏作为状态量，需要进行更新时，我们可以通过一阶近似来更新预积分项，从而不需要重新进行积分操作。

## 连续时间状态误差动态方程

连续时间线性状态误差动态方程：

$$
\begin{aligned}
\delta \dot{\alpha}_t^{b_k} =& \delta \beta_t^{b_k} \label{a}\tag{a} \\ 
\delta \dot{\beta}_t^{b_k} = & -\mathbf R_t^{b_k} \lfloor \hat a_t-b_{a_t} \rfloor_\times \delta \theta_t^{b_k}-\mathbf R_t^{b_k}\delta b_{a_t} - \mathbf R_t^{b_k}n_a \label{b}\tag{b} \\
\delta \dot{\theta}_t^{b_k} = & -\lfloor \hat w^t-b_{w_t} \rfloor _\times \delta \theta _t^{b_k} - \delta b_{w_t} - n_w \label{c}\tag{c} \\
\delta \dot b_{a_t} =& n_{b_a} \label{d}\tag{d} \\
\delta \dot b_{w_t} =& n_{w_a} \label{e}\tag{e} 
\end{aligned}
$$


设真值为$\bar x$， 估计值为$x$， 误差为$\delta x$， 有$ \bar x = x \oplus \delta x$ ，$\delta x = \bar x \ominus x$ 。以下推导忽略上下标，记$\alpha :=\alpha _t^{b_k}$ 。

方程 $\eqref{a}$ 推导：

$$
\dot \alpha= \beta
$$

$$
\delta \dot{\alpha} =  \bar{\dot { \alpha}}-\dot \alpha =\bar{\beta}- \beta=\delta \beta
$$

方程$\eqref{b}$ 推导：

我们有：

$$
\bar{\mathbf R_t^{b_k}} = \mathbf R_t^{b_k}(\mathbf I + \lfloor\delta\theta\rfloor_\times)+O(\lVert \delta \theta\rVert^2)
$$

$$
\dot {\beta}=\mathbf R_t^{b_k}a_\mathcal B
$$

其中第一个方程为是旋转矩阵的扰动近似表示；

$a_\mathcal B$ 定义为：

$$
\begin{aligned}
a_\mathcal B &:= \hat a_t- b_{a_t} \\
\delta a_\mathcal B &:= -\delta b_{a_t}-n_a \\
\bar{a_{\mathcal B}}& = a_\mathcal B+ \delta a_\mathcal B =\hat a_t- b_{a_t}-\delta b_{a_t}-n_a
\end{aligned}
$$

与IMU测量模型对应，$a_{\mathcal B} = \hat a_t-\bar{b_{a_t}}-n_a= \bar{a_t} + \bar{R_w^t} g^w$ 。

$$
\begin{aligned}
\delta \dot \beta =& \bar {\dot \beta}- \dot \beta  \\
			   =& \bar{\mathbf R_t^{b_k}}\bar {a_\mathcal B} - \mathbf R_t^{b_k}a_{\mathcal B}\\
			   =&\mathbf R_t^{b_k}(\mathbf I + \lfloor\delta\theta\rfloor_\times)(a_\mathcal B+ \delta a_\mathcal B)  - \mathbf R_t^{b_k}a_\mathcal B \\
			   =&\mathbf R_t^{b_k} \delta a_\mathcal B + \mathbf R_t^{b_k} \lfloor\delta\theta\rfloor_\times a_\mathcal B + \mathbf R_t^{b_k} \lfloor\delta\theta\rfloor_\times \delta a_\mathcal B
\end{aligned}
$$

第三项是误差二次项，可不计。同时$\lfloor \boldsymbol a\rfloor_\times \boldsymbol b = -\lfloor \boldsymbol b\rfloor_\times \boldsymbol a$ ,可得到：

$$
\begin{align}
\delta \dot \beta =&  \mathbf R_t^{b_k} (\delta a_\mathcal B - \lfloor a_\mathcal B \rfloor _\times \delta \theta) \\
			   =& \mathbf R_t^{b_k}(-\delta b_{a_t}-n_a-\lfloor \hat a_t-b_{a_t}\rfloor_\times \delta \theta ) \\
			   =& -\mathbf R_t^{b_k}\lfloor \hat a_t- b_{a_t}\rfloor_\times \delta \theta -\mathbf R_t^{b_k}\delta b_{a_t} -\mathbf R_t^{b_k}n_a
\end{align}
$$

方程$\eqref{c}$ 推导：

我们有：

$$
\begin{aligned}
\dot {\bar q}=\frac 1 2 \bar q \otimes \bar w \\
\dot q=\frac 1 2 q \otimes  w \\
\bar q = q \otimes \delta q \\
\delta q = \begin{bmatrix} 1 \\ \frac 1 2 \delta \theta \end{bmatrix}
end{aligned}
$$

同加速度一样，我们定义：
$$
w_\mathcal := \hat w_t- b_{w_t} \\
\delta w_\mathcal  := -\delta b_{w_t}-n_w \\
\bar{w}=w+ \delta w =\hat w_t- b_{w_t}-\delta b_{w_t}-n_w
$$
与IMU测量模型对应，$\bar w = \hat w_t-\bar{b_{w_t}}-n_w= \bar{w_t}$ 。

我们有：
$$
\begin {align}
\dot {\bar q} =& \dot {(q \otimes \delta q)} \\
		     =& \dot q \otimes \delta q + q \otimes \dot{\delta q} \\
		     =& \frac 1 2 q \otimes w \otimes \delta q + q \otimes \dot {\delta q}
\end {align}
$$
同时，
$$
\begin {align}
\dot {\bar q} =& \frac 1 2 \bar q \otimes \bar w \\
		     =& \frac 1 2 q \otimes \delta q \otimes \bar w
\end {align}
$$
从上两个式子得到：
$$
q \otimes \dot {\delta q} = \frac 1 2 q \otimes \delta q \otimes \bar w -\frac 1 2 q \otimes w \otimes \delta q
$$
根据附录四元数运算，有：
$$
\begin {align}
2 \dot{\delta q} =& \begin{bmatrix} 0 \\ \dot{\delta \theta} \end{bmatrix} \\
			 =& \delta q \otimes \bar w - w \otimes \delta q \\
			 =& [\bar w]_R \delta q - [w]_L \delta q \\
			 =& \begin {bmatrix}
			 	0 & -(\bar w-w)^T \\
			 	\bar w - w & -\lfloor \bar w + w\rfloor_\times 
			 	\end{bmatrix}
			 	\begin {bmatrix}
			 	1 \\ \frac 1 2 \delta \theta
			 	\end{bmatrix} + O(\lVert \delta \theta\rVert^2) \\
			 =& \begin {bmatrix}
			 	0 & -\delta w^T \\
			 	\delta w & -\lfloor 2w + \delta w\rfloor_\times 
			 	\end{bmatrix}
			 	\begin {bmatrix}
			 	1 \\ \frac 1 2 \delta \theta
			 	\end{bmatrix} + O(\lVert \delta \theta\rVert^2) \\
\end {align}
$$
从而有：
$$
\dot {\delta \theta} = \delta w - \lfloor w \rfloor_\times \delta \theta- \frac 1 2 \lfloor \delta w \rfloor_\times \delta \theta +O(\lVert \delta \theta\rVert^2)
$$
忽略二次误差项，最终有：
$$
\dot {\delta \theta} = - \lfloor  \hat w_t- b_{w_t} \rfloor_\times \delta \theta-\delta b_{w_t}-n_w
$$

## 数值积分方法

在预积分过程中，我们需要对非线性微分方程进行积分。非线性方程如下：
$$
\dot x = f(t,x)
$$
通常，在实际系统中都是采样时间点，假设$t_n = n \Delta t$ 和 $ x_n := x(t_n)$ ，
$$
x_{n+1}=x_n + \int_{n \Delta t}^{(n+1) \Delta t}f(\tau,x(\tau))d\tau
$$
通常用到的数值积分方法为Runge-Kutta方法，在积分时间段内，通过多次迭代估计出导数，然后对这个导数进行积分。RK方法常用的有Euler方法，midpoint方法，RK4方法等。

VINS中使用的是midpoint方法。

midpoint方法假设在时间段内的导数为时间段中点处的导数值。为了得到中点处的导数值，需要首先进行一次迭代积分到时间段中点：
$$
k_1 = f(t_n,x_n) \\
x(t_n + \frac 1 2 \Delta t)=x_n+\frac 1 2 \Delta t \cdot k_1
$$
那么，中点处的导数值为：
$$
\begin {align}
k_2 =& f(t_n+\frac 1 2 \Delta t, x(t_n + \frac 1 2 \Delta t)) \\
       =& f(t_n+\frac 1 2 \Delta t, x_n + \frac 1 2 \Delta t \cdot k_1)) 
\end {align} 
$$
那么数值积分结果为：
$$
x_{n+1} = x_n + \Delta  t\cdot k_2
$$


在实际实现中，我们也需要对变换矩阵进行近似逼近，因为在积分时间段内，动态矩阵A不是恒定的：
$$
\dot {x(t)}=A(t)x(t)
$$
上式是连续时间系统，对应的离散时间表示引入变换矩阵$\Phi$ ：
$$
x(t_n+\tau)=\Phi(t_n+\tau | t_n)x(t_n)
$$
因此有：
$$
\begin {align}
\dot x(t_n+\tau) =& A(t_n+\tau)x(t_n+\tau) \\
			 =& A(t_n+\tau)\Phi(t_n+\tau|t_n)x(t_n) \\
			 =& \dot{\Phi(t_n+\tau|t_n)x(t_n)} \\
			 =& \dot \Phi(t_n+\tau|t_n)x(t_n)+\Phi(t_n+\tau|t_n)\dot x(t_n) \\
			 =& \dot \Phi(t_n+\tau|t_n)x(t_n)
\end {align}
$$
其中$\dot x(t_n)=\dot{x_n}=0$ ，因为其是一个离散采样点处的值。最终我们有：
$$
\dot \Phi(t_n+\tau |t_n)=A(t_n+\tau)\Phi(t_n+\tau|t_n)
$$
因为$x(t_n)=\Phi_{t_n|t_n}x(t_n)$ ,所以变换矩阵在积分初始值为：
$$
\Phi_{t_n|t_n}=\mathbf I
$$
令$f(t,\Phi(t))=A(t)\Phi(t)$ ，可用RK方法进行近似。



## 离散时间预积分（Euler方法表示）

在实际系统实现时，我们得到的是离散采样点，这里我们使用Euler方法表示离散时间系统的状态均值更新方程和方差传播方程。

$\alpha_{b_k}^{b_k}$，$\beta_{b_k}^{b_k}$为0，$\gamma_{b_k}^{b_k}$为单位四元数。状态__均值__计算如下(Euler 一阶近似方法)：
$$
\hat \alpha_{i+1}^{b_k}=\hat \alpha_i^{b_k} + \hat \beta_i^{b_k}\delta t + \frac 1 2R(\hat \gamma _i^{b_k})(\hat a_i - b_{a_i})\delta t^2 \\
\hat \beta_{i+1}^{b_k} = \hat \beta_i^{b_k} + R(\hat \gamma _i^{b_k})(\hat a_i - b_{a_i})\delta t \\
\hat \gamma _{i+1}^{b_k}=\hat \gamma_i^{b_k} \otimes \begin{bmatrix} 1 \\ \frac 1 2 (\hat \omega_i-b_{w_i})\delta t \end {bmatrix}
$$
其中$i$表示IMU测量时刻，$\delta t$是两个测量值之间的时间间隔。 

注: 由于$i$到$i+1$时刻转动的角度$(\hat \omega_i-b_{w_i})\delta t$很小，所以对应的四元数表示近似为$\begin{bmatrix} 1 \\ \frac 1 2 (\hat \omega_i-b_{w_i})\delta t \end {bmatrix}$



之前已经推导，连续时间动态误差方程为：
$$
\begin{align}
\begin{bmatrix}
\delta \dot{\alpha}_t^{b_k}  \\ 
\delta \dot{\beta}_t^{b_k} \\
\delta \dot{\theta}_t^{b_k}  \\
\delta \dot b_{a_t} \\
\delta \dot b_{w_t}  
\end{bmatrix}
=&
\begin {bmatrix}
\mathbf 0_{3\times3} & \mathbf I_3 & \mathbf 0_{3\times3} &\mathbf 0_{3\times3} &\mathbf 0_{3\times3} \\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf -\mathbf R_t^{b_k}\lfloor \hat a_t-b_{a_t} \rfloor_\times & -\mathbf R_t^{b_k} & \mathbf 0_{3\times3} \\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & -\lfloor\hat w - b_{w_t}\rfloor_\times & \mathbf 0_{3\times3} & -\mathbf I_3 \\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} \\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} 
\end{bmatrix}
\begin{bmatrix}
\delta {\alpha}_t^{b_k}  \\ 
\delta {\beta}_t^{b_k} \\
\delta {\theta}_t^{b_k}  \\
\delta  b_{a_t} \\
\delta  b_{w_t}  
\end{bmatrix}
 \\
+&
\begin{bmatrix}
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} \\
-\mathbf R_t^{b_k} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}& \mathbf 0_{3\times3} \\
\mathbf 0_{3\times3} & -\mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} \\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3}& \mathbf I_3 & \mathbf 0_{3\times3} \\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf I_3
\end{bmatrix}
\begin {bmatrix}
\mathbf n_a \\
\mathbf n_w \\
\mathbf n_{b_a} \\
\mathbf n_{b_w}
\end{bmatrix}
\end{align}
$$
即：
$$
\delta \dot{\mathbf z}_t^{b_k}=\mathbf F_t\mathbf z_t^{b_k} + \mathbf G_t \mathbf n_t
$$
那么方差$\mathbf P_{b_{k+1}}^{b_k}$更新可以通过一阶近似递推：
$$
\mathbf P_{t+\delta t}^{b_k}=(\mathbf I_{15}+\mathbf F_t\delta t)\mathbf P_t^{b_k}(\mathbf I_{15}+\mathbf F_t\delta t)^T +
						(\mathbf G_t \delta t)\mathbf Q(\mathbf G_t \delta t)^T
$$
其中$t \in [k,k+1]$ ，$\mathbf Q = diag(\sigma_a^2, \sigma_w^2,\sigma_{b_a}^2,\sigma_{b_w}^2)$ ，$\mathbf P_{b_k}^{b_k}=\mathbf 0$ 。

同时递推得到$\delta \mathbf z_{b_{k+1}}^{b_k}$相对于$\delta \mathbf z_{b_{k}}^{b_k}$的加可比矩阵 ：
$$
\mathbf J_{t+\delta t}=(\mathbf I+\mathbf F_t\delta t)\mathbf J_t, t\in[k,k+1]
$$
在零偏状态发生变化时，可以用加可比矩阵来一阶近似更正预积分项。

![update via bias](C:\Users\Admin\Dropbox\博客\IMU预积分\update via bias.png)

## 离散时间预积分（midpoint方法表示）

在VINS代码实现中，数值积分方法采用的是midpoint方法：

![2nd-order Euler state update](C:\Users\Admin\Dropbox\博客\IMU预积分\2nd-order Euler state update.png)

![2nd-order Euler state update](C:\Users\Admin\Dropbox\博客\IMU预积分\midpoint transition matrix.png)

对应代码如下：

对应式（11）：

```c++
Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
result_delta_v = delta_v + un_acc * _dt;
result_linearized_ba = linearized_ba;
result_linearized_bg = linearized_bg;   
```

对应式（12）：

```C++
Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
Vector3d a_0_x = _acc_0 - linearized_ba;
Vector3d a_1_x = _acc_1 - linearized_ba;
Matrix3d R_w_x, R_a_0_x, R_a_1_x;

R_w_x<<0, -w_x(2), w_x(1),
w_x(2), 0, -w_x(0),
-w_x(1), w_x(0), 0;
R_a_0_x<<0, -a_0_x(2), a_0_x(1),
a_0_x(2), 0, -a_0_x(0),
-a_0_x(1), a_0_x(0), 0;
R_a_1_x<<0, -a_1_x(2), a_1_x(1),
a_1_x(2), 0, -a_1_x(0),
-a_1_x(1), a_1_x(0), 0;

MatrixXd F = MatrixXd::Zero(15, 15);
F.block<3, 3>(0, 0) = Matrix3d::Identity();
F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt + 
  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
F.block<3, 3>(6, 6) = Matrix3d::Identity();
F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
F.block<3, 3>(9, 9) = Matrix3d::Identity();
F.block<3, 3>(12, 12) = Matrix3d::Identity();
//cout<<"A"<<endl<<A<<endl;

MatrixXd V = MatrixXd::Zero(15,18);
V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

```

更新加可比矩阵和方差：

```c++
jacobian = F * jacobian;
covariance = F * covariance * F.transpose() + V * noise * V.transpose();
```



## IMU残差

由$\eqref{5}$ 可得IMU测量模型为：

![equation IMU measurement](C:\Users\Admin\Dropbox\博客\IMU预积分\equation IMU measurement.png)

那么IMU预积分的残差项为：

![IMU residuals](C:\Users\Admin\Dropbox\博客\IMU预积分\IMU residuals.png)

对应代码为：

零偏状态变化时，通过加可比矩阵更新预积分项：

```c++
Eigen::Matrix<double, 15, 1> residuals;

Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

Eigen::Vector3d dba = Bai - linearized_ba;
Eigen::Vector3d dbg = Bgi - linearized_bg;

Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
```

更新完后，计算得到残差：

```c++
Eigen::Matrix<double, 15, 1> residuals;
residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
return residuals;
```



 









































## 附录

四元数运算：

![product of two quaternions](C:\Users\Admin\Dropbox\博客\IMU预积分\product of two quaternions.png)





## 参考

- **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen [arXiv:1708.03852](https://arxiv.org/abs/1708.03852v1)
- **Quaternion kinematics for the error-state KF** [PDF](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf)





























