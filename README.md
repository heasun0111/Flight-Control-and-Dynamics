## Flight-Control-and-Dynamics
2019년에 수강한 항공제어SW 수업 및 프로젝트 정리

</br>

### 프로젝트 내용
-드론을 목표 지점까지 이동한다 </br>
-목표 지점에 도달한 후, 원을 그린다 </br>
-드론이 실제 그린 원과 목표치의 오차를 구한다 </br>


</br>

### 프로젝트 구현 환경 및 언어
Linux, ROS, JMAVSim, C++


</br>

### 환경 세팅

mavros 설치 방법
```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
cd Downloads
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

</br>

시뮬레이션 통신 확인
```
# jmavsim 실행
cd src/Firmware
make px4_sitl_default jmavsim

# jmavsim과 통신하는 mavros node 실행
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# source code build
cd catkin_ws
catkin_make
source devel/setup.bash

# run rosnode
rosrun 1d_controller controller 

# topic 값으로 통신 되는지 확인가능 => connected: True 뜨는지 확인
rostopic echo /mavros/state
```

