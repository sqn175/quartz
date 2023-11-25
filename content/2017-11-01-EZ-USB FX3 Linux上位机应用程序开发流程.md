---
title: EZ-USB FX3 Linux上位机应用程序开发流程
key: 20171101
tags: 工程应用
---

Cypress EZ-USB FX3是新一代USB 3.0外设控制器，它可提供高度集成性和多项灵活功能，帮助开发人员轻松将USB3.0功能添加到任何系统中。我们现拟将EZ-USB FX3集成至raspberry pi zero（简称zero）中，使得zero能够通过USB3.0接收数据。
<!--more-->

相应的开发任务有：

- 固件开发：对FX3芯片进行编程以适应我们的传输任务。开发环境：任意系统 + 对应系统的fx3SDK。最终生成`.img`文件，烧录进FX3芯片。
- 上位机应用程序开发：在上位机管理USB传输事宜（比如连接USB设备、开始传输数据等）以及处理USB传输数据（比如存储数据至硬盘）。由于我们的上位机是zero，系统是基于Debian Linux的Raspbian Jessie Pixel系统，所以上位机应用程序的开发应该针对Linux进行。开发环境：Linux + CyUSB suite for Linux + libusb。

这里我们假设已经完成了固件开发，开始zero上位机的应用程序开发。

## 快速开始

1. 安装lib-usb-1.0.x

   这是Linux上的USB底层驱动。FX3的CyUSB使用了其中的API。我们使用libusb-1.0.9版本，因为CyUSB是基于这个版本开发的。

   - 解压`libusb-1.0.9.tar.bz2`，并进入文件根目录；

   - 根据`INSTALL`文件说明来进行安装：

     ```
     The simplest way to compile this package is:

     1. `cd' to the directory containing the package's source code and type
        `./configure' to configure the package for your system.
        Running `configure' might take a while.  While running, it prints
        some messages telling which features it is checking for.
     2. Type `make' to compile the package.
     3. Optionally, type `make check' to run any self-tests that come with
        the package.
     4. Type `make install' to install the programs and any data files and
        documentation.
     ```

2. 安装CyUSB Suite for Linux

   CyUSB Suite for Linux是应用层的USB3.0驱动，帮助我们管理EZ-USB FX3外围设备传输事宜。我们进行开发时用到其提供的头文件`cyusb.h`和动态链接库`libcyusb.so`。

   CyUSB Suite提供了一个实例应用程序`cyusb_linux`，按照以下步骤编译该实例：

   - 安装Qt4：`sudo apt-get install qt4-dev-tools`;


   - 解压`cyusb_linux_1.0.4.tar.gz`，并进入文件根目录；

   - 根据`README`文件说明来进行安装:

     ```
     1. cd to the main directory where files where extracted and execute 'make'.
        This will compile the libcyusb library and create a static library.
        For example, if the archive is extracted to /home/user/cyusb_linux_1.0.4; then
        e.g.: user@desktop:/home/user/cyusb_linux_1.0.4 $ make
     2. Make sure that the install.sh script is executable by changing the mode
        of install.sh file.
        e.g.: user@desktop:/home/user/cyusb_linux_1.0.4 $ chmod +x install.sh
     3. The install.sh script compiles the cyusb_linux GUI application, and installs
        the libcyusb library and the application in the system directories (/usr/local/).
        It also sets up a set of UDEV rules and updates the environment variables under
        the /etc directory.
        As these changes require root (super user) permissions, the install.sh script
        needs to be executed from a root login.
        e.g.: root@desktop:/home/user/cyusb_linux_1.0.4 $ ./install.sh
     4. The GUI application can now be launched using the 'cyusb_linux' command.
     ```

    如果不能启动`cyusb_linux`，执行命令：`ldconfig`后重试。

    下图是在zero上连接FX3 USB设备后，使用`cyubs_linux`烧录进`cyfxbulklpautoenum.img`，并进行数据传输的截图:

    ![cyusb_linux](/images/20171101/cyusb_linux.png){:class="img-responsive"}

    传输任务是先向USB3.0 发送32个Byte，32个Byte的值逐次加1.USB 3.0收到数后立即向上位机发送该数。由结果表明，上位机发送的数和接收的数一致。

## 开发任务

基于头文件`cyusb.h`、动态链接库`libcyusb.so`和实例Example代码、CyUSB接口说明(cyusb_linux_programmers_guide.pdf)开发适合我们项目需求的zero上位机应用程序。

