# Publish velocity toggle Plus
```cpp
//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <turtlebot_srv_ex/Changerate.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;
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

bool changeRate(
	turtlebot_srv_ex::Changerate::Request &req,
	turtlebot_srv_ex::Changerate::Response &resp){

	ROS_INFO_STREAM("Changing rate to "<<req.newrate);

	newfrequency = req.newrate;
	ratechanged = true;

	return true;
}


int main(int argc, char **argv){
    ros::init(argc,argv,"pubvel_toggle_rate");
	ros::NodeHandle nh;
        
	ros::ServiceServer server = 
		nh.advertiseService("toggle_forward",&toggleForward);
                
    ros::ServiceServer server0 =
        nh.advertiseService("change_rate",&changeRate);
                
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
    ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
		msg.linear.x = forward?1.0:0.0;
		msg.angular.z=forward?0.0:1.0;
		if (cnt >0)
			pub.publish(msg);
		ros::spinOnce();
		if(ratechanged) {
			rate = ros::Rate(newfrequency);
			ratechanged = false;
		}
		rate.sleep();
	}
}
```

## 구체적 분석
turtlebot_srv_ex package의 Changerate.srv를 쓰기 위한 include
```cpp
#include <turtlebot_srv_ex/Changerate.h>
```

loop를 새로운 frequency로 바꾸는 service를 위한 전역변수
```cpp
double newfrequency;
bool ratechanged = false;
```

"change_rate"라는 service에 대한 server object를 server0라고 선언

call을 받으면 changeRate 콜백함수 실행
```cpp
ros::ServiceServer server0 =
    nh.advertiseService("change_rate",&changeRate);
```

Custom Service를 사용하는 방법
```cpp
bool CallBack(
	package_name::srv_name::Request &req
	package_name::srv_name::Response &resp) {}
```

turtlebot_srv_ex package의 Changerate srv를 인자로 받는 Callback함수 changeRate

req.newrate를 받아 newfrequency에 저장하고, ratechanged를 true로 변경

```cpp
bool changeRate(
	turtlebot_srv_ex::Changerate::Request &req,
	turtlebot_srv_ex::Changerate::Response &resp){

	ROS_INFO_STREAM("Changing rate to "<<req.newrate);

	newfrequency = req.newrate;
	ratechanged = true;

	return true;
}
```

Changerate.srv
```cpp
float64 newrate
---
bool ret
```

ratechanged가 true이면 newfrequency로 frequency를 변경하여 rate.sleep()

다시 freqency를 바꿀 수 있게 ratechanged = false
```cpp
if(ratechanged) {
	rate = ros::Rate(newfrequency);
	ratechanged = false;
}
rate.sleep();
```

### call 방법
```
$ rosservice call /change_rate
```
인 상태에서 tab을 누르면 아래와 같이 자동완성

원하는 hz를 넣어주면 됨
```
$ rosservice call /change_rate "newrate: 0.0"
```