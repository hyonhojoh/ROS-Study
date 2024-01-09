# Service

+ 예제: turtlesim을 실행한 뒤 (rosrun turtlesim turtlesim_node)
  + rosservice list 로 실행 가능한 서비스 확인
  ```
  $ rosservice list
  ```

  + rosservice call /spawn 을 사용하여 turtlesim 노드에 로봇 소환
  ```
  $ rosservice call /spawn
  ```
  을 입력한 상태에서 tab을 누르면 아래와 같이 어떤 data를 call 할 것인 지 입력하게 나옴

  ```
  ~$ rosservice call /spawn "x: 0.0
  y: 0.0
  theta: 0.0
  name: ''"
  ```

이렇게 일일히 terminal로 call 해 줄 수 없기 때문에 code상으로 구현하는 방법에 대해 알아보자.

## CMakeLists.txt
service를 사용하기 위해 CMakeLists.txt에 추가해줘야 할 것들이 있다.
```cmake
find_package(catkin REQUIRED COMPONENTS
    ...
    std_msgs # std_msgs가 포함된 이유는 srv file에 포함된 msg의
             # type이 std_msgs일 수 있기 때문
             # (실제로는 float64, bool 사용) <- ros에서 제공하는 msg
    message_generation
    )

# Generate services in the 'srv' folder
add_service_files(
    FILES
    changerate.srv
)

# Generate added messages and services with any dependencies listed here
generate_message(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    ...
    
    CATKIN_DEPENDS  ...
                    std_msgs
                    message_runtime
    ...
)

add_dependencies(pubvel_toggle_plus
    ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
```

## package.xml
service를 사용하기 위해 package.xml에 추가해줘야 할 것들이 있다.

```xml
<build_export_depend>message_runtime</build_export_depend>

<exec_depend>message_runtime</exec_depend>

<depend>message_generation</depend>
```

## Client Code
### Call Service
+ request / response Type: 각 서비스들은 정해진 서비스 규격을 따른다.
```cpp
#include<turtleSime/Spawn.h>
```

+ Client Object: 각 서비스들은 지정된 service object를 통해 "Call" 된다.
```cpp
ros::ServiceClient client = node_handle.serviceClient<service_type>(service_name);
```

+ Service Object: msg를 geometry_msg/Twist로 생성하고, msg안에 다양한 값들을 넣어준다.

1. 
```cpp
package_name::service_type::Request req;
```
2. 
```cpp
package_name::service_type::Responsce resp;
```
3. 
```cpp
bool success = service_client.call(srv);
                    or
bool success = service_client.call(request, response);
```

## Server Code
### Service Server
+ 서비스 콜백: 토픽의 subscribe와 마찬가지로 콜백함수가 존재하여, 리턴은 bool값으로 함.
```cpp
bool function_name(
    package_name::service_type::Request &req,
    package_name::service_type::Request &resp
){ ... }
```

+ 서버 오브젝트: 각 서비스들은 지정된 서비스 오브젝트를 통해 콜백함수와 결합된다.
```cpp
ros::ServiceServer server = node_handle.advertiseService(
    servic_name, pointer_to_callback_function);
```

+ 표준 서비스: http://wiki.ros.org/std_srvs


## Custom Service
+ package에 srv폴더를 만들고, 그 안에 .srv 파일을 만들면 됨
+ ---으로 구분되고, 위가 request, 아래가 response

```cpp
req_msg_type req_msg_name
    ...
---
resp_msg_type resp_msg_name
    ...
```

Custum.srv
```cpp
float64 newrate
---
bool ret
```

### CMakeLists.txt
+ add_service_files
```cmake
## Generate services in the 'srv' folder
add_service_files(
   FILES
   Custom.srv
 )

catkin_package(
    ...
    CATKIN_DEPENDS ... mssage_runtime
    ...
)

## add_dependencies ${${PROJECT_NAME}_EXPORTED_TARGETS}
add_dependencies(pubvel_toggle_plus
    ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
```