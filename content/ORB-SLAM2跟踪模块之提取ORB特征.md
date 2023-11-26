---
title: ORB-SLAM2跟踪模块【1/4】：提取ORB特征
date: 2017-05-07
tags:
  - SLAM
  - ORB-SLAM2
---
ORB-SLAM2跟踪线程对相机输入的每一帧图像进行跟踪处理，如下图所示，主要包括4步，提取ORB特征、从上一帧或者重定位来估计初始位姿、局部地图跟踪和关键帧处理。
![[ORB-SLAM2流程.png]]
以下结合相关理论知识，阅读ORB-SLAM2源代码，从而理解ORB-SLAM2算法中ORB特征提取过程。

## ORB（Oriented FAST and Rotated BRIEF）

基于特征点的方法是SLAM的前端VO的主流方法，因为其运行稳定，对光照、运动物体不敏感。特征点由关键点（Key-point）和描述子（Descriptor）两部分组成。比如，当说到SIFT特征时，是指“提取SIFT关键点，并计算SIFT描述子”。关键点是指该特征点在图像里的位置，有些特征点还具有朝向、大小等信息。描述子通常是一个向量，按照某种人为设计的方式，描述了该关键点周围像素的信息。描述子是按照“外观相似的特征应该有相似的描述子”的原则设计的。因此，只要两个特征点的描述子在向量空间上的距离相近，就可以人为它们是同样的特征点。

常见的特征有SIFT特征，SURF特征等。那么为什么ORB-SLAM2选择ORB特征呢？

这是因为虽然SIFT考虑了图像变换过程中出现的光照、尺寸、旋转等变化，但需要较大计算量，在没有GPU加速的情况下，很难在SLAM这种系统中进行实时计算，另一方面，SIFT特征和SURF特征是受到专利保护的，需要付费使用。

ORB特征由Ethan Rublee, Vincent Rabaud, Kurt Konolige和Gary R. Bradski在他们2011年的论文《ORB: An efficient alternative to SIFT or SURF》提出，如论文题目所述，ORB特征在计算速度、匹配性能，以及在专利要求上都可以替代SIFT和SURF。

ORB取名已经反映出其是一个结合了改良后的FAST角点提取和BRIEF描述子的算法，提取ORB特征分为两步：

1. FAST关键点提取：找出图像中的FAST角点，相较于原版的FAST，ORB中计算了特征点的主方向，为后续的BRIEF描述子增加了旋转不变性；

2. BRIEF描述子：对上一步提取出关键点的周围图像区域进行描述。

### FAST关键点

FAST是一种角点，主要检测局部像素灰度变化明显的地方，以速度快著称。FAST只需要比较像素亮度大小，速度很快，它的检测过程如下：

1. 在图像中选取像素$p$，假设它的亮度为$I_p$；

2. 设置一个阈值$T$（比如$I_p$的20%）；

3. 以像素p为中心，选取半径为3的圆上的16个像素点；

4. 假如选取的圆上有连续的N个点的亮度大于$I_p+T$或者小于$I_p-T$，那么像素$p$可以被认为是特征点（N通常取12，即FAST-12）。

5. 循环以上四步，对每一个像素执行相同的操作。

FAST角点检测虽然速度很快，但是它存在一些问题。首先是FAST角点数量很大且不确定，因此ORB对其进行改进。ORB指定最终要提取的角点数量N，对原始FAST角点分别计算Harris响应值，然后选取前N个具有最大值的角点作为最终的角点集合。

其次，FAST不具有尺寸，因此ORB构建图像金字塔，对图像进行不同层次的降采样，获得不同分辨率的图像，并在金字塔的每一层上检测角点，从而获得多尺寸特征。

FAST没有计算旋转，因此ORB通过计算以FAST角点O为中心的图像块的质心C，那么向量$\vec {OC}$的方向就是特征点的方向，具体值通过图像块的矩得到。

通过各种改进，FAST特征具有了尺寸和旋转的描述，在ORB中，把这种改进后的FAST称为oFAST。

### BRIEF描述子

BRIEF描述子是一种二进制字符描述子，其描述向量定义如下：

$$f_n(\mathbf p)=\sum_{1\leq i\leq n}2^{i-1}\tau(\mathbf p;\mathbf x_i,\mathbf y_i)$$

其中，

$$\tau(\mathbf p;\mathbf x,\mathbf y)=\begin{cases}1,  & \mathbf p(x) < \mathbf p(y) \\[2ex]0, & \mathbf p(x) \geq \mathbf p(y)\end{cases}$$

$\mathbf p(x)$ 是图像块p中点$x$的强度。$\tau$的选择有很多种，常见的选择方式是围绕图像块中心的高斯分布。n选为256的话，$f_n(\mathbf p)$就是256维的向量。BRIEF由于使用了二进制表达，存储起来十分方便，适用于实时的图像匹配。原始的BRIEF描述子不具有旋转不变性，因此在图片发生旋转时，匹配性能会急速下降。ORB根据之前关键点的方向来旋转图像块，得到“steer BRIEF”。

BRIEF具有每个bit的方差很大，均值约为0.5的特性，但是“steer BRIEF”丧失了这种特性，其均值不再集中在0.5左右。可以理解为特定方向的角点关键点使得其产生发散。这样会导致使用“steer BRIEF”进行匹配时的错误率变高，因为“steer BRIEF”的方差发生了亏损，彼此之间区分度降低。同时我们希望每个$\tau$彼此不相干，这样得到的BRIEF更加有区分度。

为了解决上述问题，BRIEF采用了贪婪搜索，对所有可能的$\tau$进行搜索，找出既具有高方差，均值约为0.5，同时又不相干的$\tau$，最终结果称为rBRIEF。



由于考虑了旋转和缩放，使得ORB在平移、旋转和缩放的变换下仍具有良好的表现。同时，FAST和BRIEF的计算非常高效，使得ORB特征在实时SLAM系统中得以应用。

以下阅读ORB-SLAM2的源代码，理清其跟踪线程中对ORB特征的提取过程。

## 函数入口

ORB-SLAM2跟踪运行在主线程，是整个SLAM系统的基础。主程序在初始化SLAM系统后，

```c++
// Examples/Monocular/mono_kitti.cc  line:53
// Create SLAM system. It initializes all system threads and gets ready to process frames.
ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
```

就可以将每一帧图像送往跟踪函数，如下是单目SLAM主函数调用跟踪函数的代码：

```c++
// Examples/Monocular/mono_kitti.cc  line:84
// Pass the image to the SLAM system
SLAM.TrackMonocular(im,tframe);
```

`TrackMonocular()`函数调用`GrabImageMonocular()`函数实现跟踪功能：

```c++
// System.cc  line:260
cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
```

双目和RGB-D调用方式类似，分别是`SLAM.TrackStereo(imLeftRect,imRightRect,tframe); `和`SLAM.TrackRGBD(imRGB,imD,tframe);`

`mpTracker`是`System`类中的成员，是`Tracking`类的指针。`mpTracker`对输入的每一帧图像计算出对应的相机位姿，同时决定何时插入新的关键帧，创建新的地图点，并且在跟踪失效时进行重定位。`mpTracker`的初始化在`System` 类的对象`SLAM`初始化的构造函数中进行：

```c++
//System.cc  line:86~87
//Initialize the Tracking thread
//(it will live in the main thread of execution, the one that called this constructor)
mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                         mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);
```

那么`mpTracker`是如何实现上述功能的呢？我们来看`Tracking`类，其头文件为`Tracking.h`,其定义了接口如下：

```c++
// Tracking.h  line:61
// Preprocess the input and call Track(). Extract features and performs stereo matching.
cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);
```

## ORB特征提取

ORB-SLAM2是一个基于特征的方法，它对输入的图像提取出角点的特征，如下图所示：

![[ORB-SLAM2提取特征点.png]]

在提取出特征后，所有输入的图片都会删除，系统剩下的处理流程都是基于这些特征进行的，和相机类型无关。

单目的预处理流程实现过程在`cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp)`函数中体现为：首先将`im`转换为灰度图`mImGray`，然后预处理提取ORB特征：

```c++
// Tracking.cc  line:257~260
if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
  mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
else
  mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
```

得到预处理的结果`mCurrentFrame`，从而系统剩余部分的处理流程都是基于`mCurrentFrame`，和单目相机无关。`mCurrentFrame`是`Frame`类的对象，这里的预处理在`Frame`类的构造函数中进行。`Frame`类对单目相机输入的构造函数重载形式为：

```c++
// Frame.h
// Constructor for Monocular cameras.
Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
```

在`Frame.cc`文件中查看其该重载函数定义，

```c++
// Frame.cc line:181~191
// Scale Level Info
mnScaleLevels = mpORBextractorLeft->GetLevels();
mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
mfLogScaleFactor = log(mfScaleFactor);
mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

// ORB extraction
ExtractORB(0,imGray);
```

其先提取ORB特征参数，然后调用`Frame`类成员函数`ExtractORB()`来提取ORB特征,ORB特征参数存储在配置文件中，在`mpTracker`的初始化中加载读入，并传入`Frame`的构造函数中。

`ExtractORB()`函数定义为：

```c++
// Frame.cc  line:247~253
// Extract ORB on the image. 0 for left image and 1 for right image.
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}
```

其调用了`ORBextractor`类的重载运算符来提取ORB特征：

```c++
// ORBextractor.h  line:56~61
// Compute the ORB features and descriptors on an image.
// ORB are dispersed on the image using an octree.
// Mask is ignored in the current implementation.
void operator()( cv::InputArray image, cv::InputArray mask,
                std::vector<cv::KeyPoint>& keypoints,
                cv::OutputArray descriptors);
```

ORB-SLAM提取ORB特征时采用了8层金字塔，尺寸因子为1.2。对于像素为512\*384到752\*480的图片，提取1000个FAST角点，对于更高的分辨率，提取2000个FAST角点就可以了。

至此，得到当前帧ORB特征点`mvKeys`和描述子`mDescriptors`，均是`Frame`类对象` mCurrentFrame`的成员变量。提取出特征点后，需要对其去失真`UndistortKeyPoints();`。同时需要将图片分割为64 * 48大小的栅格，并将关键点按照位置分配到相应栅格中，从而降低匹配时的复杂度，实现函数为`AssignFeaturesToGrid();`	。

## 参考

[1] Mur-Artal, Raul, Jose Maria Martinez Montiel, and Juan D. Tardos. ["ORB-SLAM: a versatile and accurate monocular SLAM system" ](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)(PDF).*IEEE Transactions on Robotics* 31.5 (2015): 1147-1163.

[2] Rublee, Ethan; Rabaud, Vincent; Konolige, Kurt; Bradski, Gary (2011). ["ORB: an efﬁcient alternative to SIFT or SURF"](http://www.vision.cs.chubu.ac.jp/CV-R/pdf/Rublee_iccv2011.pdf) (PDF). *IEEE International Conference on Computer Vision (ICCV)*.

[3] 高翔，张涛.“视觉SLAM十四讲”

[4] OBR-SLAM2 [github 主页](https://github.com/raulmur/ORB_SLAM2)