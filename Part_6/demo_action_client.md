# Demo Action Client
```cpp
#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "demo_action_ex/Demo_actionAction.h"

int main (int argc, char **argv)
{
 ros::init(argc, argv, "demo_action_client");

 if(argc != 3){
   ROS_INFO("%d",argc);
   ROS_WARN("Usage: demo_action_client <goal> <time_to_preempt_in_sec>");
   return 1;
 }

 // create the action client
 // true causes the client to spin its own thread
 actionlib::SimpleActionClient<demo_action_ex::Demo_actionAction> ac("demo_action", true);

 ROS_INFO("Waiting for action server to start.");

 // wait for the action server to start
 ac.waitForServer(); //will wait for infinite time

 ROS_INFO("Action server started, sending goal.");

 // send a goal to the action
 demo_action_ex::Demo_actionGoal goal;
 goal.count = atoi(argv[1]);

 ROS_INFO("Sending Goal [%d] and Preempt time of [%d]",goal.count, atoi(argv[2]));
 ac.sendGoal(goal);

 //wait for the action to return
 bool finished_before_timeout = ac.waitForResult(ros::Duration(atoi(argv[2])));
 //Preempting task
 ac.cancelGoal();

 if (finished_before_timeout){
   actionlib::SimpleClientGoalState state = ac.getState();
   ROS_INFO("Action finished: %s",state.toString().c_str());
   //Preempting the process
   ac.cancelGoal();
 }
 else{
   ROS_INFO("Action did not finish before the time out.");
 }

 //exit
 return 0;
}
```

## 구체적 분석

+ actionlib::SimpleActionClient<demo_action_ex::Demo_actionAction>
  + client object를 생성하기 위한 include
```cpp
#include <actionlib/client/simple_action_client.h>
```

+ actionlib::SimpleClientGoalState state = ac.getState() 를 사용하기 위한 include
```cpp
#include <actionlib/client/terminal_state.h>
```

+ argument count가 3이 아닐 때
  + demo_action_client 3 3 => argc=3
  + 명령도 포함함

+ 명령 형식을 맞춰서 입력하지 않을경우, usage를 출력하고, 종료시킴
```cpp
if(argc != 3){
   ROS_INFO("%d",argc);
   ROS_WARN("Usage: demo_action_client <goal> <time_to_preempt_in_sec>");
   return 1;
}
```

+ __Client Object__
  + 아래와 같이 client object 생성
```cpp
actionlib::SimpleActionClient<demo_action_ex::Demo_actionAction> ac("demo_action", true);
```

+ Server가 켜질 때까지 wait
```cpp
ac.waitForServer();
```

+ goal object 생성
  + goal.count에 client가 입력한 정보 대입
```cpp
demo_action_ex::Demo_actionGoal goal;
 goal.count = atoi(argv[1]);
```

+ goal을 server로 보냄
  + service의 call과 비슷함
```cpp
ac.sendGoal(goal);
```

+ ac.waitForResult를 통해 atoi(argv[2])의 시간동안 result를 기다림
  + success이면 finished_before_timeout에 true
  + 제 시간 안에 goal에 도달하지 못하면 finished_before_timeout에 false

+ executeCB 정지

```cpp
//wait for the action to return
bool finished_before_timeout = ac.waitForResult(ros::Duration(atoi(argv[2])));
//Preempting task
ac.cancelGoal();
```

+ 성공하면, state를 얻어와서 출력
+ 실패하면, 실패 문구 출력
```cpp
if (finished_before_timeout){
  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s",state.toString().c_str());
  //Preempting the process
  ac.cancelGoal();
}
else{
  ROS_INFO("Action did not finish before the time out.");
}
```
