# URC2026 Auronomous Controller 
## Launch
UARTの起動
```
ros2 launch uart_control serial_subscriber.launch.py 

ros2 launch uart_control serial_publisher.launch.py 

```

RealSense起動
```
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true publish_tf:=true
```

センサの起動
```
ros2 launch ares_sensor sensor_data_publisher.launch.py 
```

コントローラの起動
```
ros2 launch rover_controller rover_controller.launch.py 
```

LiDARの起動
```
ros2 launch livox_ros_driver2 msg_MID360_launch.py 
ros2 run livox_to_pointcloud2 livox_to_pointcloud2_node --ros-args -r /livox_pointcloud:=/livox/lidar
```


```
ros2 launch ares_nav2 controller_bringup.launch.py
```

Nav2の起動
```
ros2 launch ares_nav2 navigation_sim.launch.py 
```

Aruco goalへ近づくプログラムの起動
```
ros2 launch ares_nav2 aruco_nav2_goal.launch.py 
```

waypointsへ移動
```
ros2 launch ares_nav2 main.launch.py 
```


