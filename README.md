# ROS-ServingRobot


## ROS 설치 및 환경 설정
### 최신 라이브러리로 업데이트
```bash
$ sudo apt update
$ sudo apt upgrade
```
### 저장소 추가
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu ${lsb_release -sc} main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
### ROS 설치
```bash
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ sudo apt-get install python-pip
$ sudo pip install -U rosdep
$ sudo rosdep init
$ rosdep update
```
### 환경변수 등록
```bash
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool
```
### 작업환경 구축
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```
### 작업공간 환경변수 등록
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
## WT61C 자이로센서 사용
### 6축자이로센서 포트 설정
uart_open 포트값을 ttyUSB0로 설정하여 코딩
```C
fd = uart_open(fd, "/dev/ttyUSB0");
```

## RPLiDAR 라이더센서 설정
### 작업공간에 RPLiDAR 센서 라이브러리 복사 및 설치
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/Slamtec/rplidar_ros.git
$ cd ~/catkin_ws && catkin_make
```
### RPLiDAR 센서 포트수정
```bash
$ roscd rplidar_ros && cd launch
$ nano rplidar.launch # 편집기로 3번째줄 /dev/ttyUSB0를 /dev/ttyUSB1로 수정
```
```html
<param name="serial_port" type="string" value="/dev/ttyUSB1"/>
```
## 아두이노 설정
### ROS 아두이노 패키지 설치
```bash
$ sudo apt install ros-melodic-rosserial-arduino
$ sudo apt install ros-melodic-rosserial
```
### 아두이노IDE에 ros_llb 설치
```bash
$ cd <arduino IDE path>/libraries
$ rm -rf ros_lib
$ rosrun rosserial_arduino make_libraries.py .
```
### ROS serial_node 실행 및 포트설정
```bash
$ rosrun rosserial_python serial_node.py _=port:=/dev/ttyUSB2
```
