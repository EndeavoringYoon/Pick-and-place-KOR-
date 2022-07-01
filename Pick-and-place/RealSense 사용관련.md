# RealSense

# 기본 설치

[https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

에서 

```json
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

까지 해주시면 기본적인 sdk와 패키지 설치는 끝난 겁니다.

```json
realsense-viewer
```

를 통해 실행 가능한지를 확인할 수 있습니다.

ROS에서 realsense를 구동하기 위해서는 추가적인 패키지 realsense-ros에 대한 설치가 필요한데, 

```json
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```

로 설치 가능하고,

```json
roslaunch realsense2_camera rs_camera.launch
```

를 통해 카메라를 실행하실 수 있습니다.

```json
rviz
```

로 rviz를 켠 뒤에, 

![rviz.png](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/main/Pick-and-place/RealSense%20%EC%82%AC%EC%9A%A9%EA%B4%80%EB%A0%A8/rviz.png)

add에서 image를 선택해서 생성한 후, image topic을 바꿔가면서 잡히는지를 확인할 수 있습니다.

![color image를 받아왔을 때](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/main/Pick-and-place/RealSense%20%EC%82%AC%EC%9A%A9%EA%B4%80%EB%A0%A8/rviz_1.png)

color image를 받아왔을 때

![depth image를 받아왔을 때](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/main/Pick-and-place/RealSense%20%EC%82%AC%EC%9A%A9%EA%B4%80%EB%A0%A8/rviz_2.png)

depth image를 받아왔을 때

아까 realsenseROS를 다운받았던 github에서도 언급되었지만 realsense를 이용하여 pointcloud를 실행할 수 있습니다.

```json
roslaunch realsense2_camera rs_camera.launch
```

아까의 이 코드를 꺼주고, 그 자리에

```json
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```

를 켜주시면 pointcloud용 노드가 실행됩니다.

다시 rviz에서 Add-pointcloud2(1도 있지만 2가 더 작동이 잘 된다고 합니다)를 한 뒤 pointcloud2의 topic과 global options의 fixed frame을 설정해주시면

![rviz_3.png](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/main/Pick-and-place/RealSense%20%EC%82%AC%EC%9A%A9%EA%B4%80%EB%A0%A8/rviz_3.png)

pointcloud도 잘 실행됨을 알 수 있습니다.
