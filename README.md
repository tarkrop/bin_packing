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
ros2 launch bin_packing bin_packing_launch.py
```
<br/>
<br/>

## 데이터 처리

- 상자 데이터 받기: geometry_msgs/PointStamped<br/>
토픽 이름 - set_packing_item(토픽 보낼 때 상자 당 한 번만 보내기)<br/>
frame_id = '0'  -> 0~3 사이의 값으로 보내기. 먼저 나갈수록 우선순위가 높아 숫자를 높임.

point -> 기존에 보내던 Point msg를 사용하기 위해서 point.x, point.y, point.z로 사용
x = width<br/>
y = height<br/>
z = depth<br/>
<br/>

- 배치 위치 출력: geometry_msgs/Point<br/>
토픽 이름 - packing_position<br/>
(x, y, z) -> 상자의 중심 위치 출력(한 번만 출력)<br/>
<br/>

## 적재 공간 분할

- 패키지 내부의 config 폴더의 param.yaml에서 destination 수를 변경 가능함. 최대 4까지

```
packing_node:
  ros__parameters:
    destination: 4
```
<br/>
값 변경 후 colcon build 필요
<br/>
<br/>



## 시각화

- 컨테이너 및 박스 확인

  launch 파일에 포함됨