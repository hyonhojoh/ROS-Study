# Spawn Turtle
```cpp
//This program spawns a new turtlesim turtle by calling
// the appropriate service.

#include <ros/ros.h>
//The srv class for the service.
#include <turtlesim/Spawn.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "spawn_turtle");
    ros::NodeHandle nh;

    //Create a client object for the spawn service. This
    //needs to know the data type of the service and its name.
    ros::ServiceClient spawnClient
		= nh.serviceClient<turtlesim::Spawn>("spawn");

    // Create the request and response objects.
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    req.x = 2;
    req.y = 3;
    req.theta = M_PI/2;
    req.name = "NewTurtle";

    ros::service::waitForService("spawn", ros::Duration(5));
    bool success = spawnClient.call(req,resp);

    if(success){
	ROS_INFO_STREAM("Spawned a turtle named "
			<< resp.name);
    }else{
	ROS_ERROR_STREAM("Failed to spawn.");
    }
}
```

service type을 사용하기 위한 include
```cpp
#include <turtlesim/Spawn.h>
```


client object 생성

service의 type과 service name을 입력해야 함
```cpp
ros::ServiceClient spawnClient
	= nh.serviceClient<turtlesim::Spawn>("spawn");
```

Create the request and response objects.
```cpp
turtlesim::Spawn::Request req;
turtlesim::Spawn::Response resp;
```

request object에 value 대입
```cpp
req.x = 2;
req.y = 3;
req.theta = M_PI/2;
req.name = "NewTurtle";
```

"spawn"이라는 service가 존재 확인을 위한 5초 기다림
```cpp
ros::service::waitForService("spawn", ros::Duration(5));
```

spawn object를 통해 server에게 call

(server는 이미 turtlesim_node에서 켜져 있음)

request와 response object를 인자로 함

만약 response가 온다면, true를 return

```cpp
bool success = spawnClient.call(req,resp);
```

