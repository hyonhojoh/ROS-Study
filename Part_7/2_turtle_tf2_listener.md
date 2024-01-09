# Turtle TF2 Listener
```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");
  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient spawner =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
```

## 구체적 분석
+ tf2_ros::TransformListener object를 생성하기 위한 include
```cpp
#include <tf2_ros/transform_listener.h>
```

+ "/spawn"이라는 service가 call될 때까지 wait

+ 이 node는 "/spawn" service가 call 되야 실행하는 node임

+ "/spawn"에 대한 service client 객체를 만들고 값을 넣어 turtle2가 생성
```cpp
ros::service::waitForService("spawn");
ros::ServiceClient spawner =
node.serviceClient<turtlesim::Spawn>("spawn");
turtlesim::Spawn turtle;
turtle.request.x = 4;
turtle.request.y = 2;
turtle.request.theta = 0;
turtle.request.name = "turtle2";
spawner.call(turtle);
```

+ "turtle2/cmd_vel" publisher 생성
```cpp
ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
```

+ buffer를 통해 TransformListener object 생성
```cpp
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
```

+ turtle2와 turtle1에 대한 transform을 저장한 후, 이를 이용하여 turtle2가 turtle1을 계속 따라가게하는 값을 publish
```cpp
while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
 }
```