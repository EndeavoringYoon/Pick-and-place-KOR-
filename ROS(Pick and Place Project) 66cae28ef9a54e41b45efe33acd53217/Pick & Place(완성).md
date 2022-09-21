# Pick & Place(완성)

# 1. 프로그램 설명

## 1-1. 개요

내용은 크게 

- moveit 패키지와
- path_planning.py
- sending_trajectory.py
- sending_jointstates.py
- cam.py

로 구성되어 있습니다. cam.py의 경우 realsense를 이용하여 각각의 class별로 중심의 위치와 width값을 로봇팔을 기준으로 calibration하여 반환하는 프로그램을 제작해주신 박준성 선배님의 github에 가시면 자세히 나와 있습니다.

[https://github.com/engineerJPark/Segmentation-to-World-Coordinate-by-FCN-and-Camera-Calibration](https://github.com/engineerJPark/Segmentation-to-World-Coordinate-by-FCN-and-Camera-Calibration)

## 1-2. 흐름도

### moveit

moveit 패키지는 rviz를 이용한 가상 환경을 제공합니다.

### path_planning.py

path_planning.py에서 cam.py의 클래스를 import해 물체의 position과 width를 가져옵니다.

moveitCommander나 move_group의 함수를 이용하여 특정 상황에서 planning이나 move를 요청할 경우 가상 환경에서 로봇의 모델링을 이용하여 계산하고 그 경로를 publish합니다. 이번 pick and place에서 이 python 파일은 cam.py를 이용해 물체의 위치와 width를 수신하고 그 곳까지 가는 경로를 compute_cartesian_path를 이용해 이동, 원하는 goal_position에 물체를 내려놓는 과정까지, 즉 전반적인 planning을 담당합니다.

### sending_trajectory.py

path_planning.py에서 publish된 값은 sending_trajectory.py에서 send_trajectory라는 노드를 통해 ROBOTIS사에서 제공하는 dynamixel_workbench_controller에 publish됩니다. 형식은 arm을 움직일 경우 JointTrajectory, gripper를 움직일 경우 DynamixelCommand입니다. gripper의 경우 실제 로봇과 모델링의 차이로 인해 rad에 따라 gripper의 width로 변환하는 함수를 제작한 뒤 직접 Command로 publish해주는 방식을 택했습니다.

### sending_jointstates.py

rviz에서 로봇의 이동이나 현 상태를 파악하기 위해서, 또는 planning을 할 때 초기 위치를 가져오기 위해서는 JointStates가 반드시 필요합니다. 그래서 dynamixel_workbench_controller에서 JointStates를 수신하여 moveit 패키지에 publish하는 이 파일을 제작하였습니다.

아래의 3가지 python 파일들은 하나의 파일로 합칠 수도 있지만, 동아리 차원에서의 프로젝트였기 때문에 보편적인 실력을 고려하여 wrapping하지 않았고, 흐름을 어느 정도 파악할 수 있도록 제어부와 구동부 등을 나눠서 프로그램을 제작하다 보니 프로그램의 개수가 많아진 점 양해 부탁드리겠습니다.

## 1-3. 실행 방법

1. moveit 패키지 실행
2. dynamixel_workbench_controller 실행
3. dynamixel_workbench_operator joint_operator 실행
4. rqt에서 /dynamixel_workbench/execution을 눌러 controller 연결 여부 확인
5. sending_trajectory.py 실행
6. sending_jointstates.py 실행
7. realsense 실행
8. path_planning.py 실행

후 path_planning에서 진행해주시면 됩니다.

자세한 코드는 [Codes](Codes%20e7972d692b0f4c6d9d1c1cf6a76b97bb.md)를 참고해주시기 바랍니다.