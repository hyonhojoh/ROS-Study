# Publish velocity toggle
```cpp
//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

bool forward = true;
int cnt = 0;

bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
		cnt++;
        forward = !forward;
        ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
	return true;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"pubvel_toggle");
	ros::NodeHandle nh;
        
	ros::ServiceServer server = 
		nh.advertiseService("toggle_forward",&toggleForward);
                
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
    ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
		msg.linear.x = forward?1.0:0.0;
		msg.angular.z= forward?0.0:1.0;
		if (cnt>0)
			pub.publish(msg);
		
		ros::spinOnce();
		rate.sleep();
	}
}
```

## 구체적 분석

include ROS Service Type Empty

std_srvs::Empty를 사용하기 위함
```cpp
#include <std_srvs/Empty.h>
```

forward: 직진할 것인지, 회전할 것인 지를 결정하는 전역변수 선언

cnt: CallBack 함수가 몇번 실행되었는 지 알려줌
```cpp
bool forward = true;
int cnt = 0;
```

Service CallBack 함수: Client에서 Service에 대한 Call이 들어오면 실행하는 함수

인자로 Empty type의 request, response를 받음

service call을 받으면 forward를 반대로 하고 cnt++ 함

true를 return
```cpp
bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
		cnt++;
        forward = !forward;
        ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
	return true;
}
```

+ Server Object

"toggle_forward"라는 service를 advertise

toggle_forward라는 service에 대한 call이 오면 toggleForward 함수 실행
```cpp
ros::ServiceServer server = 
		nh.advertiseService("toggle_forward",&toggleForward);
```

직진할 것인지, 회전할 것인지를 service를 통해 전달받고,

움직임을 위한 publisher object를 선언
```cpp
ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
```

true: msg.linear.x = 1.0, msg.angular.z = 0.0

false: msg.linear.x = 0.0, msg.angular.z = 1.0

2hz로 위의 값이 넣어진 msg가 publish
```cpp
while(ros::ok()){
		geometry_msgs::Twist msg;
		msg.linear.x = forward?1.0:0.0;
		msg.angular.z= forward?0.0:1.0;
		if (cnt>0)
			pub.publish(msg);
		
		ros::spinOnce();
		rate.sleep();
	}
```