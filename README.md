## 使用

### Terminal 1 (controller)
```bash
$ colcon build --packages-select turpong
$ . install/setup.bash
$ ros2 run turpong keybr
```

### Terminal 2 (server)
```bash
$ . install/setup.bash
$ ros2 launch turpong launch.py
```

### 控制
- `w` `s` 控制左边的板
- `↑` `↓` 控制右边的板


## 待实现
- 暂停/退出游戏
- 重置游戏


## 疑难杂症
- 编译时出现以下警告，但不影响运行????
```bash
$ colcon build
[0.611s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/lerane/turPong/install/turpong' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.611s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/lerane/turPong/install/turPong' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.611s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/lerane/turPong/install/interfaces' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.611s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/lerane/turPong/install/turpong' in the environment variable CMAKE_PREFIX_PATH doesn't exist
[0.611s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/lerane/turPong/install/turPong' in the environment variable CMAKE_PREFIX_PATH doesn't exist
[0.611s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/lerane/turPong/install/interfaces' in the environment variable CMAKE_PREFIX_PATH doesn't exist
```
- 如果两只龟同时长按控制, 有一边会卡住# turPongGame-using-ROS-2-Cpp
