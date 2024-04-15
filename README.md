# demo_motion

# 说明：
- 使用了标准的ros程序主框架结构，订阅motion话题
- 订阅主题名称为motion_servo_cmd，消息类型为protocol::msg::MotionServoCmd获取运行数据参数接口
- 可以通过定时器控制运动频率，通过std_msgs::msg::Float64消息类型计算频率
  
# 远程登陆狗子：```ssh mi@192.168.55.1```输入密码123 远程登陆狗子上
# 并在其上面建立工作空间，将该功能包放到对应src中
# 编译
```
colcon build
```
# 运行
```
ros2 run demo_motion demo_motion  --ros-args -r __ns:=/`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`
```
# 结果
狗子站立，然后以0.2m/s速度向前走