# ros2封装海康工业相机SDK
## 实现的功能
- 发布图像
- 断线重连
- 命令行调整曝光时间、增益、帧率、图像格式

## 使用
### 安装依赖(若已安装请跳过)
#### oepncv
首先安装opencv依赖
```
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
libatlas-base-dev gfortran python3-dev python3-numpy \
libtbb2 libtbb-dev libdc1394-22-dev
```
安装opencv
```
mkdir ~/opencv_build && cd ~/opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

cd ~/opencv_build/opencv
mkdir build && cd build
make -j$(nproc)
sudo make install
```
#### ros2
```
wget http://fishros.com/install -O fishros && . fishros
```
#### MVS SDK
下载安装包

访问 https://www.hikrobotics.com/cn/machinevision/service/download/?module=0 下载对应版本的MVS安装包
解压之后使用deb安装包下载sudo dpkg -i MVS-3.0.1_x86_64.deb

添加环境变量

```
export MVS_HOME=/opt/MVS
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MVS_HOME/lib
source ~/.bashrc
```
运行 MVS 客户端：
`bash /opt/MVS/bin/MVS.sh`

### 编译
创建ros2工作空间

`
mkdir -p ~/ros2_ws/src 
`

克隆代码

`git clone https://github.com/anleatr/RM_TRAIN.git`

`cd EM_TRAIN && mv task4-hik_camera ~/ros2_ws/src`

编译

`cd ~/ros2_ws/src && colcon build`

使用
```
souce install/setup.bash
ros2 launch hik_camera hik_rviz.launch.py
```
