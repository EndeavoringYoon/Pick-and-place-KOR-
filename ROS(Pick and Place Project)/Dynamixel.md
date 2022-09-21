# Dynamixel

현재 20.04 ROS noetic 기준으로 설치 완료했습니다.

### 참고 사이트

[https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)

ROS 설치는 위에서 언급하였기 때문에 건너뛰셔도 됩니다.

# Creating catkin_ws

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

# 패키지 설치

```json
cd catkin_ws/src

# Main packages

git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git

# Dependent packages

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

src 폴더로 이동 후 git clone 3줄 진행하시면 workbench 설치에 필요한 모든 패키지를 받아올 수 있습니다.

OpenCR을 사용할 것이기 때문에 아두이노도 당연히 설정을 해줘야 하는데

아두이노 IDE 설치해주시고[https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)

위의 사이트 따라서 보드 추가하는 것까지 진행해주셔야 Arduino에서 OpenCR 보드를 읽을 수 있습니다. 이후에 보드에서 모터를 읽기 위해서 4.1.5.3의 Port setting 이후 File-Example-OpenCR-10.ETC-usb_to_dxl 선택 후 verify-upload 진행해주시면 됩니다.

기본적으로 robotis사에서는 openmanipulator 이용 시 U2D2를 사용하거나, OpenCR을 위의 방법을 이용해 U2D2와 유사하게 만드는 방법을 권장하고 있습니다. 자세한 내용은 로보티즈 사의 FAQ 참고 바랍니다.

# Dynamixel Wizard 2.0 설치

Dynamixel Wizard 2.0을 이용해서 모터가 정상적으로 연결되어 있는지 확인할 수 있는데 Dynamixel Wizard의 경우는 

[https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

여기서 받으시면 되고, 

![Screenshot from 2022-06-23 19-52-09.png](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/Dynamixel/Screenshot_from_2022-06-23_19-52-09.png)

이런 식으로 Protocol 2.0 선택 후 

<aside>
💡 protocol의 경우에는 모터 종류에 따라 2.0을 지원하지 않는 경우가 있으므로 반드시 로보티즈사의 e-manual을 참고하기 바랍니다.

</aside>

터미널에서

```jsx
ls /dev/tty*
```

 위 코드를 이용하여 찾은 포트 번호를(OpenCR의 경우에는 ttyACM0를, U2D2의 경우에는 ttyUSB0의 형태로 나오게 됩니다.) 선택 후 baudrate와 id는 전체로 놓고 한번 검색하시면 찾을 수 있습니다. 이후에 option에 다시 들어가셔서 찾으신 baudrate값으로 변경하시면 이후의 검색에서는 더 빠르게 찾을 수 있습니다.

![Screenshot from 2022-06-23 20-04-06.png](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/Dynamixel/Screenshot_from_2022-06-23_20-04-06.png)

Wizard에서는 토크를 켜고 옆의 붉은색 막대기를 움직여 모터의 각도를 움직여볼 수 있고, 토크를 끈 뒤에는 ID나 control 방법 등 여러 설정을 바꿀 수 있습니다.

위의 방법으로 진행하면 현재 SDK와 Workbench는 설치되어 있는 상태이고, Wizard 2.0을 통해 OpenCR과 모터 자체에 문제가 없고, 또한 연결에도 문제가 없음을 확인하였습니다. 이제 ROS의 특성상 catkin_make를 통해서 저희가 설치한 SDK와 Workbench 패키지를 빌드를 해줘야 Workbench의 작동을 확인할 수 있습니다.

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

여기서 ttyUSB0는 본인의 포트 번호에 맞게, baudrate와 id도 wizard에서 찾았던 내용을 바탕으로 맞게 넣어주시면 Dynamixel Wizard에서 확인했던 모터가 검색되는 것을 알 수 있습니다.

컴퓨터에 OpenCR 및 모터 연결하는 방법에 대해 알아보았고, 이후에는 Control하는 방법에 대해 알아보도록 하겠습니다.

## Dynamixel_yaml, launch files

dynamixel_workbench/dynamixel_workbench_controllers/config에 가면 다양한 yaml 파일들을 볼 수 있다.

dynamixel_workbench/dynamixel_workbench_controllers/launch에 있는 dynamixel_controllers.launch 파일을 보면 이 controller의 작동 구조를 알 수 있는데, 여기서 상단의 usb_port는 기존에 ls /dev/tty*의 코드를 이용해 검색했던 자신의 default 포트 번호를 넣으면 됩니다. baudrate도 마찬가지입니다.

config/basic.yaml 파일을 가시면

```json
모터 이름:
	모터 ID
```

와 같은 형태의 파일 내용을 보실 수 있는데 wizard에서 찾으신 ID를 입력하시고 본인의 목적에 맞게 모터 이름을 설정해 주시면 되겠습니다.
