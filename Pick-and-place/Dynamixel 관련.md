# Dynamixel

현재 20.04 ROS noetic 기준으로 설치 완료해봤습니다.

참고 사이트

[https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)

ROS 설치는 위에서 언급하였기 때문에 건너뛰셔도 됩니다.

catkin_ws부터 만드셔야 하는데 터미널 여신 뒤에 (Ctrl+Alt+T)

```json
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```

진행하시면 됩니다.

```jsx
sudo apt-get updage
sudo apt-get upgrade
```

만 진행해주시고

```json
cd catkin_ws/src

# Main packages

git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git

# Dependent packages

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

cd로 src 폴더로 이동 후 git clone 3줄 진행하시면 workbench 설치에 필요한 모든 패키지를 받아올 수 있습니다.

OpenCR을 사용할 것이기 때문에 아두이노도 당연히 설정을 해줘야 하는데

아두이노 IDE 설치해주시고[https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)

위의 사이트 따라서 보드 추가하는 것까지 진행해주셔야 Arduino에서 OpenCR을 읽을 수 있습니다. 이후에 보드에서 모터를 읽기 위해서 4.1.5.3의 Port setting 이후 File-Example-OpenCR-10.ETC-usb_to_dxl 선택 후 verify-upload 진행해주시면 됩니다.

Dynamixel Wizard 2.0을 이용해서 모터가 정상적으로 연결되어 있는지 확인할 수 있는데 Dynamixel Wizard의 경우는 

[https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

여기서 받으시면 되고, 

![Screenshot from 2022-06-23 19-52-09.png](Dynamixel%20d0d68fcf1443460c8de148df3f84747c/Screenshot_from_2022-06-23_19-52-09.png)

이런 식으로 Protocol 2.0 선택 후 이전의 4.1.3에서 ls /dev/tty*으로 찾았던 포트번호를 이용해서(저의 경우는 ttyACM0) 그 포트번호를 선택 후 baudrate와 id는 전체로 놓고 한번 검색하시면 찾을 수 있습니다. 이후에 option에 다시 들어가셔서 찾으신 baudrate값으로 변경하시면 이후의 검색에서는 더 빠르게 찾을 수 있습니다.

![Screenshot from 2022-06-23 20-04-06.png](Dynamixel%20d0d68fcf1443460c8de148df3f84747c/Screenshot_from_2022-06-23_20-04-06.png)

Wizard에서는 Torque를 켜고 옆의 붉은색 막대기를 움직여 모터의 각도를 움직여볼 수 있고, 토크를 끈 뒤에는 여러 설정을 바꿀 수 있지만 Workbench를 설치할 것이기 때문에 이후의 모든 조정은 Workbench로 진행할 것 같습니다. ID 등 다양한 설정을 바꿀 수 있습니다.

위의 방법으로 진행하면 현재 SDK와 Workbench는 설치되어 있는 상태이고, Wizard 2.0을 통해 OpenCR과 모터 자체에 문제가 없고, 또한 연결에도 문제가 없음을 확인하였습니다. 이제 ROS의 특성상 catkin_make를 통해서 저희가 설치한 SDK와 Workbench 패키지를 빌드해줘야 Workbench의 작동을 확인할 수 있습니다.

다시 터미널을 켜주시고

```json
cd ~/catkin_ws
catkin_make

source devel/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

까지 해주시면

5.1의 ROS Tutorials를 실행하실 수 있습니다.

여기서 ttyUSB0는 본인의 포트번호 넣어주시면 아까 Dynamixel Wizard에서 확인했던 모터가 검색되는 것을 알 수 있습니다.

컴퓨터에 OpenCR 및 모터 연결하는 방법에 대해 알아보았고, 이후에는 Control하는 방법에 대해 알아보도록 하겠습니다.

## Dynamixel_yaml

dynamixel_workbench/dynamixel_workbench_controllers/config에 가면 다양한 yaml 파일들을 볼 수 있다.

dynamixel_workbench/dynamixel_workbench_controllers/launch에 있는 dynamixel_controllers.launch 파일을 보면 이 controller의 작동구조를 알 수 있는데, 여기서 상단의 usb_port는 기존에 ls /dev/tty*으로 검색했던 자신의 default 포트번호를 넣으면 된다.

다시 config로 돌아와서 다양한 yaml 파일을 볼 수 있는데, 크게