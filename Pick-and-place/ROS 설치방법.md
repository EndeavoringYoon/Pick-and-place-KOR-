# ROS

ROS 설치방법

[https://wiki.ros.org/noetic/Installation/Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)

위의 링크 방법 그대로 실행하시면 됩니다.

1.4에서는 full install, environment setup은 처음 1줄 후

```jsx
source /opt/ros/noetic/setup.bash
```

이후의 catkin_make 사용마다 위의 줄을 치기는 귀찮기 때문에

```jsx
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

가 추가로 들어가게 되는데 이건 dynamixel 설치 이후에 진행하셔도 괜찮습니다.