# demo_motion

## 说明：
- 使用了标准的ros程序主框架结构，订阅motion话题
- 订阅主题名称为motion_servo_cmd，消息类型为protocol::msg::MotionServoCmd获取运行数据参数接口
- 可以通过定时器控制运动频率，通过std_msgs::msg::Float64消息类型计算频率
  
## 远程登陆狗子：
将下载线连接狗子后，打开终端运行 ```ssh mi@192.168.55.1```
输入密码123 远程登陆狗子上
## 在其桌面建立工作空间，使用scp命令将该功能包放到对应src中
## 编译
```
colcon build
```
## 运行
```
ros2 run demo_motion demo_motion  --ros-args -r __ns:=/`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`
```
## 结果
狗子站立，然后以0.2m/s速度向前走

## 其他资料
1. ROS2运动接口介绍：https://miroboticslab.github.io/blogs/#/cn/developer_guide?id=%e5%9f%ba%e7%a1%80%e8%83%bd%e5%8a%9b
(包括伺服指令和结果指令接口介绍，消息定义，参数类型等等)

2. 相关code错误码定义：https://miroboticslab.github.io/blogs/#/cn/cyberdog_system_cn
3. 运动管理模块介绍：https://miroboticslab.github.io/blogs/#/cn/motion_manager_cn
4. 一次结果指令在运动管理模块的详细响应流程图：![流程图](https://github.com/mylinfh/demo_motion/assets/95616729/dc7b5ca9-f90f-4f58-bc38-1a17ebf58096)
