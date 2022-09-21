# 개발과정1-설치부터 gazebo 내 조작까지

## 참고자료

오픈매니퓰레이터 패키지 설치 관련[https://github.com/robotpilot/ros-seminar/blob/master/13_매니퓰레이터.pdf](https://github.com/robotpilot/ros-seminar/blob/master/13_%EB%A7%A4%EB%8B%88%ED%93%B0%EB%A0%88%EC%9D%B4%ED%84%B0.pdf)

(kinetic version이기 때문에 ros-kinetic-industrial-ros-controllers와 같은 패키지는 ros-noetic-ros-controllers처럼 noetic으로 변경해서 입력하시기 바랍니다. 일부 패키지는 설치되지 않을 수도 있습니다.)

# 1. 설치과정 중 발생 가능한 문제점

## 1-1. 패키지 설치 중

앞서 언급했지만 workbench의 경우 noetic에 대해서 업데이트를 진행하긴 했는데 stable한지는 장담할 수 없습니다. 업데이트가 중단되었다는 이야기가 있습니다.

![prepare neotic release에서 끝난 것을 볼 수 있습니다.](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%951-%EC%84%A4%EC%B9%98%EB%B6%80%ED%84%B0%20gazebo%20%EB%82%B4%20%EC%A1%B0%EC%9E%91%EA%B9%8C%EC%A7%80/Untitled.png)

prepare neotic release에서 끝난 것을 볼 수 있습니다.

대표적인 오류는 no such file or directory가 뜨는

[dynamixel.cpp fails compilation when using distributed workspace](https://forums.developer.nvidia.com/t/dynamixel-cpp-fails-compilation-when-using-distributed-workspace/111679)

과 같은 오류가 있는데

이 문제의 경우 파일 내의 #include 내의 경로가 일치하지 않아서 생기는 문제이므로 위와 같은 방법으로 에러 난 부분을 바꿔 주시거나 직접 해당 파일이 존재하는 컴퓨터 내 경로로 바꿔주시면 해결됩니다.

workbench 설치가 완전히 끝난 후에는 전에 말씀드린 것처럼 Wizard를 이용해서 모터 연결 문제나 기타 오류가 없는지 확인 먼저 하시고, 이후에 workbench를 이용해서 본격적인 움직임을 해야 하는데 이때 주의 사항이

## 1-2. 모터 id

- 모터 id가 동일할 경우 여러 모터를 연결해서 동작하면 인식조차 되지 않습니다. 반드시 모터마다 다른 id를 가질 수 있도록 하나씩 연결해서 아두이노나 Wizard에서 수정하시기 바랍니다.

[ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)

## 1-3. Have you typed ‘make’ in …

ROBOTIS e-Manual 5.1.2 Controllers에 보시면

```json
roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch
```

과 rqt를 이용해서 dynamixel을 조종해보는 방법이 제시되고 있는데 

![error.png](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%951-%EC%84%A4%EC%B9%98%EB%B6%80%ED%84%B0%20gazebo%20%EB%82%B4%20%EC%A1%B0%EC%9E%91%EA%B9%8C%EC%A7%80/error.png)

다음과 같은 에러가 뜰 수 있습니다.

catkin_ws에서 build랑 devel 폴더를 삭제하시고 catkin_make를 다시 하시거나, dynamixel_workbench_msgs에 터미널로 접속하셔서

```json
cmake .. && make
cd ~/catkin_ws
source devel/setup.bash

#이후에 각기 다른 두 terminal 창에

roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch
#이거랑
rqt
```

실행해주시면 다시 될 겁니다. 

rqt의 경우 source devel/setup.bash를 끝낸 터미널 창에서 여시기 바랍니다.

# 하드웨어 고려 사항

![Openmanipulator-X description ](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%951-%EC%84%A4%EC%B9%98%EB%B6%80%ED%84%B0%20gazebo%20%EB%82%B4%20%EC%A1%B0%EC%9E%91%EA%B9%8C%EC%A7%80/manipulatorX.png)

Openmanipulator-X description 

출처:[https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/#hardware-specification](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/#hardware-specification)

Openmanipulator-X 초기 조립 시에 토크가 꺼져 있는 상태이거나, 모터의 회전판이 고정되지 않은 상태로 조립하시다 보면 다시 맞춰주셔야 하는 일이 생깁니다. 

모터의 경우 position 회전 기준으로는 0~4095이지만, radian 값을 이용한 position control에서는 -3.14rad~3.14rad 값을 받으니 주의하시기 바랍니다. dynamixel wizard를 이용한 위치 조절(0~4095)과 나중에 언급될 open_manipulator_controller를 이용한 초기 세팅 방법(-3.14~3.14)이 있습니다.

이후에 moveit쪽에서 사용하는 값이 radian을 기준으로 하기 때문에 open_manipulator_controller와 동일하고, 이에 따라 후자의 방법을 권장드립니다.

# Simulation 실행 시 주의사항

## 3-1. rViz 실행 시 주의사항

현재 rviz를 다운로드 받은 파일 그대로 실행할 경우 joint_state_publisher 창이 뜨지 않는 문제가 발생하고 있습니다. 현재 노드 이름이 바뀌었기 때문에 발생하는데,

```json
roslaunch open_manipulator_description open_manipulator_rviz.launch
```

를 실행하기 전에, 또는 그 외에도

testbot_description/launch 폴더의 testbot.launch 파일이나 open_manimpulator/open_manimpulator_description/launch 폴더의 open_manipulator_rviz.launch 파일 실행시 먼저 디렉토리에서 열어보신 후에 join_state_publisher는 모두 joint_state_publisher_gui로, state_publisher는 robot_state_publisher로 바꿔주시면 작동합니다.

## 3-2. Gazebo 실행 시 주의사항

현재 gazebo에서 joint 정보를 받아오지 못한다고 에러가 출력되지만,  yaml 등의 위치에 pid값이 없는 상태로 gazebo에 넘겨주게 되면 저런 오류가 뜨나, 문제가 있는 것은 아닙니다.

```json
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```

각기 다른 terminal창에서 실행하셔야 하고, 실행 전에 cd catkin_ws && catkin_make && source devel/setup.bash는 꼭 하셔야 합니다.

이후에 gazebo 실행중에 gripper가 진동하는 현상을 발견했고, 이를 해결하고자 오로카 홈페이지에 질문을 올렸습니다.

[https://cafe.naver.com/openrt?iframe_url_utf8=%2FArticleRead.nhn%253Fclubid%3D25572101%2526articleid%3D27327%2526commentFocus%3Dtrue](https://cafe.naver.com/openrt?iframe_url_utf8=%2FArticleRead.nhn%253Fclubid%3D25572101%2526articleid%3D27327%2526commentFocus%3Dtrue)

그 결과 effort 값을 1로 설정하라는 답변을 받을 수 있었습니다.

(여담이지만 gazebo 자체의 물리엔진 문제라는 이야기도 있습니다.)

![Screenshot from 2022-07-19 15-24-39.png](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%951-%EC%84%A4%EC%B9%98%EB%B6%80%ED%84%B0%20gazebo%20%EB%82%B4%20%EC%A1%B0%EC%9E%91%EA%B9%8C%EC%A7%80/Screenshot_from_2022-07-19_15-24-39.png)

open_manipulator/open_manipulator_description에 있는 urdf.xacro 파일 안에 보시면 effort값이 1로 설정되어 있는 것을 확인할 수 있습니다. 

처음에 controller를 사용할 때 controller에서 지속적으로 0.00으로 값을 보내고 있어 진동이 발생하는 것 같습니다. controller 패키지를 사용하지 않고 저희가 독자적으로 command를 보내거나, moveit을 사용하게 되면 떨림이 없어지니 걱정하지 않으셔도 됩니다.

![Screenshot from 2022-07-19 15-38-51.png](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%951-%EC%84%A4%EC%B9%98%EB%B6%80%ED%84%B0%20gazebo%20%EB%82%B4%20%EC%A1%B0%EC%9E%91%EA%B9%8C%EC%A7%80/Screenshot_from_2022-07-19_15-38-51.png)

# 2. 기존 패키지를 활용한 컨트롤러 파일 만들기

## 2-1. 실제 모터 조절

### single_motor_control

motor control의 경우에는 dynamixel_workbench_msgs.srv에서 DynamixelCommand를 가져와서 /dynamixel_workbench/dynamixel_command 노드로 대신 서비스를 해줘야 합니다. publisher-subscriber 형태가 아닌 service-action 형태임에 유의하시기 바랍니다.

```python
#!/usr/bin/env python

import time
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand

def single_motor_control_talker ():
    #initializing node
    rospy.init_node('just_trying', anonymous=True)
    #set the loop rate
    rate = rospy.Rate(1) # 1Hz
    #keep publishing until a Ctrl-C is pressed
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    func = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
    resp = func("",1,"Goal_Position",0)#(빈칸,모터 id, 명령 종류, 각도(0~4095가 0~360도에 대응))
    resp = func("",2,"Goal_Position",0)
    time.sleep(2)
    resp = func("",1,"Goal_Position",2048)
    resp = func("",2,"Goal_Position",2048)
    print(resp.comm_result)

if __name__ == '__main__' :
    try:
        single_motor_control_talker()
    except rospy.ROSInterruptException:
        pass
```

실제 모터 각도 조절용 예시 코드입니다. OpenCR에 연결한 모터 번호와, 원하는 각도에 맞춰서 바꿔가며 모터 각도를 조절해주시면 되겠습니다.

## 2-2. Gazebo 내 모터 조절

### gripper_control

gazebo에서 gripper를 control하는 topic은 gripper_position/command인데, 우리는 이 또한 위의 motor_control과 마찬가지로(물론 이건 publisher-subscriber 형태입니다) 역할을 대신할 코드입니다.

```python
#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

def set_gripper_position_talker ():
    pub = rospy.Publisher('gripper_position/command', Float64, queue_size=10)
    rospy.init_node('set_gripper_position_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        gripper_position = 0.01
        rospy.loginfo(gripper_position)
        pub.publish(gripper_position)
        rate.sleep()

if __name__ == '__main__' :
    try:
        set_gripper_position_talker()
    except rospy.ROSInterruptException:
        pass
```

여기서 gripper_position의 값을 변경하는 것으로 gazebo 안에서 gripper가 움직이는 것을 알 수 있습니다.

## 2-3. Gazebo 내부에서의 joint control에 관하여

joint의 경우 radian을 이용해서 control 하게 되는데 그 범위는 gui 기준으로 -2.827~+2.827입니다. 원칙적으로는 -3.14~3.14가 맞겠지만 확인해 본 결과 끝까지 돌아가지는 않는 것 같습니다. collision 때문인 듯 합니다. 참고 바랍니다.
