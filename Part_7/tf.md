# TF
+ TF(Transform) library는 다양한 로봇, 물체, 지도 frame을 동시에 추적하기 위해 사용

  + 기본적으로 Tree 구조에 시간에 따른 Buffer를 쌓음. 즉 TF는 (ros time)에 매우 민감함
  
  + 원격 관제를 위해서 로봇과 원격 조종 컴퓨터 간의 시간을 맞추는 등의 노력이 필요

  + 각 Frame 간의 Transformation matrix 관계, vector 관계 등을 추정, 혹은 좌표 변환 시킴
    + 5초 전에 head frame은 world frame을 기준으로 어디에 위치하고 있었나?
    + 현재 내 그리퍼의 위치는 내 로봇 중심에서 얼마나 떨어져 있나?
    + 현재 내 모바일 로봇은 맵 기준으로 어디에 위치하고 있나?
+ 주요 특징
  + Listening for transform
  
    + 모든 프레임, 혹은 특정 프레엠을 받아보거나, 특정 시점의 프레임을 받아 볼 수 있음
  
  + Broadcasting transform
    + 특정 프레임을 기준으로 상대 위치 (혹은 회전)를 전송할 수 있음

+ Geometry_msgs::Pose vs Geometry_msgs::PoseStamped
+ ROS Msgs (poseStamped, transform Stamped)와 tf Stamped는 다른 구조체임
```cpp
geometry_msgs::TransformStamped transformStamped;

tf2::Stamped<tf2::Transform> tfs;

tf2::convert(transformStamped, tfs);
```