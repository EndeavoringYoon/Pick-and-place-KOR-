# 개발과정2-moveit

## 참고자료

moveit: [http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/index.html](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/index.html)

ROS로봇프로그래밍: [https://github.com/robotpilot/ros-seminar/blob/master/13_매니퓰레이터.pdf](https://github.com/robotpilot/ros-seminar/blob/master/13_%EB%A7%A4%EB%8B%88%ED%93%B0%EB%A0%88%EC%9D%B4%ED%84%B0.pdf)

# 1. 설치

moveit은 기본으로 설치해 주셔야 합니다

```json
sudo apt install ros-noetic-moveit
```

이후에 나오지만 Openmanipulator-X의 경우에는 kdl_kinematics_plugin/KDLKinematicsPlugin으로 IK(Inverse Kinematics)를 풀 수 없습니다. 따라서 다른 플러그인을 쓸 건데

[https://github.com/dudasdavid/open_manipulator_ikfast_plugin](https://github.com/dudasdavid/open_manipulator_ikfast_plugin)

이곳에 가셔서 ikfast_plugin을 src/open_manipulator 폴더에 git clone을 해주면 이후의 moveit_setup_assistant 설정 시에 사용할 수 있습니다.

# 2. Moveit setup assistant

```json
roslaunch moveit_setup_assistant setup_assitant.launch
```

를 통해 Moveit setup assistant를 실행하면

## 2-1. Creating Moveit

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled.png)

이런 형태의 ui가 여러분을 맞이하게 됩니다.

Create New Moveit Configuration Package를 클릭하고

Browse에서 open_manipulator/open_manipulator_description/open_manipulator_robot.urdf.xacro 파일을 선택하고 Load Files를 누르면

Self Collisions로 넘어가시면 됩니다.

여기선 Sampling Density는 최대로 한 후 Generate Collision Matrix를 클릭, 이후 Virtual Joints는 무시하고 Planning Groups로 넘어가시면 됩니다

## 2-2. Define Planning Groups

Planning Groups에서는 

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%201.png)

ros-seminar pdf에서는 이렇게 안내해주고 있는데, kdl_kinematics_plugin의 경우 6DOF를 사용하는데 저희 Openmanipulator는 4DOF기 때문에 불필요한(기구학적 영향을 미치지 못하는) joint를 2개 추가하거나, 다른 kinematic solver를 사용해야 합니다. 여기서 저희는 joint를 추가하는 대신 아까 설치한 openmanipulator/IKFastplugin을 선택하여 사용하도록 하겠습니다.

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%202.png)

kdl의 경우 demo.launch에서 정상 작동하긴 하나, 나중에 보실 파란색 구가 움직이지 않는 문제와 함께 이후에 move_group.compute_cartesian_path가 계산되지 않는 문제가 있습니다.

### Editing Joint Collection

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%203.png)

Add joints에서도 joint1,2,3,4만 arm으로, gripper의 경우 gripper, gripper_sub만 선택해줍니다. end_effector_joint의 경우 urdf로 가서 값을 좀 바꿔줘야 합니다.

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%204.png)

(end_effector_link가 link5에 child link로 선언이 되어 있는데, 그 link의 collision 범위가 end effector link를 포함해서 설정되는 문제가 있어 end effector link(사진의 붉은색 상자)가 너무 앞에 있으면 gripper를 열고 물체를 잡으러 이동하는 명령을 내릴 때 물체 용도로 스폰하는 박스가 들어가지 않는 문제가 있습니다.) 저는 0.06으로 설정했는데 편하신 값으로 맞춰주시면 되겠습니다. Default 값은 0.126입니다. catkin_ws/src/open_manipulator/open_manipulator_description/urdf/open_manipulator.urdf.xacro로 들어가셔서 end effector joint의 설정값을 바꿔주시면 됩니다.

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%205.png)

planning group으로 gripper도 추가해 주시되, 다른 설정은 none으로 해주시면 됩니다(Default planner같은 경우 이후에 RRTConnect로 자동으로 설정되어 동작합니다). joint는 gripper, gripper_sub 선택해주시면 되겠습니다.

## 2-3. Define Robot Poses

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%206.png)

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%207.png)

Define Robot Poses는 자유이나 arm의 init(0,0,0,0), home(0,-1.05,0.35,0.7), gripper의 open(0.015, 0.015), close(0.0, 0.0) 설정해주시면 나중에 좋습니다.

## 2-4. End Effectors

End Effectors는 pdf와 다르게 설정했습니다.

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%208.png)

end effector group은 gripper, connected link는 link5, parent group은 arm으로 해주시면 됩니다.

## 2-5. Passive Joints

Passive Joints는 pdf와 다르게 설정하지 않고 넘어가겠습니다.

- passive joint에 gripper_sub를 넣는 경우와 gripper group에 gripper_sub를 넣는 것의 차이점은 아직 잘 모르겠지만, 직접 테스트 해보시고 맞는 방향으로 사용하시는 것을 권장합니다. Openmanipulator-X의 경우에는 1개의 모터로 2개의 gripper를 모두 컨트롤하긴 하지만, 모델링에서는 마지막 모터와 gripper 사이의 link가 구현된 것이 아닌 gripper joint와 gripper_sub joint로 연결된 상태라서 딱히 상관이 없는 것으로 보입니다.

## 2-6. Controllers

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%209.png)

Auto Add FollowJointsTrajectory Controllers를 하셔도 되지만, Add controller로 직접 설정해주시는 걸 권해드립니다. Controller type은 gripper의 경우 effort controllers로 고정이나 arm_controller의 경우 어느 정도 유연성이 있는 것 같습니다. FollowJointTrajectory나 position_controllers 정도로 설정해 주시면 됩니다.

이후에 나오는 내용이지만 이 Controllers는 moveit에서 fake controller를 생성할 때 반영되는 내용인데, 이후의 과정에서 fake를 사용하되, 시뮬레이션에서만 활용하고 실제 controller는 dynamixel_workbench_controllers를 사용할 것이기 때문에 저는 workbench에서 사용할 jointTrajectory를 채택하도록 하겠습니다.

Author Information 적어주시고 Configuration Files에서 path 선택해주신 다음(open_manipulator 폴더 안에 새로 폴더를 생성 후) Generate Package 해주시면 되겠습니다.

# 3. 완성 후 moveit

```json
catkin_make && source devel/setup.bash
roslaunch (folder name) demo.launch # rviz만 실행
roslaunch (folder name) demo_gazebo.launch # rviz와 gazebo 모두 실행
```

이후 실행해보시면 잘 동작합니다.

![Untitled](https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/%EA%B0%9C%EB%B0%9C%EA%B3%BC%EC%A0%952-moveit/Untitled%2010.png)

# 4. 완성파일
https://github.com/EndeavoringYoon/Pick-and-place-KOR-/blob/master/ROS(Pick%20and%20Place%20Project)/open_manipulator_moveit.zip
