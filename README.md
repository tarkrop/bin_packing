# bin_packing
지능로봇 상자 적층 노드

<br/>
<br/>

## 레포지토리 클론받기
```
cd ${HOME}/ros2_ws/src
git clone https://github.com/tarkrop/bin_packing.git
```
<br/>
<br/>

## 패키지 빌드

```
cd ${HOME}/ros2_ws
colcon build --packages-select bin_packing
source /opt/ros/humble/setup.bash
```

<br/>
<br/>

## 실행하기

```
ros2 run bin_packing packing_node
```
<br/>
<br/>

## 데이터 처리

상자 데이터 받기: geometry_msgs/Point<br/>
토픽 이름 - set_packing_item(토픽 보낼 때 상자 당 한 번만 보내기)<br/>
x = width<br/>
y = height<br/>
z = depth<br/>
<br/>

배치 위치 출력: geometry_msgs/Point<br/>
토픽 이름 - packing_position<br/>
(x, y, z) -> 상자의 중심 위치 출력(한 번만 출력)<br/>