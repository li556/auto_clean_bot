# 劲旅无人清扫车项目
## 构建方法
！！！基于ubuntu 20.04，ROS2 foxy
### 1.设备驱动构建
- `inno_sw_ros2`
    ```bash
    cd ~/auto_clean_bot/src/drivers/inno_sw_ros2
    chmod +x src/inno_lidar_ros/src/inno_sdk/build/build_unix.sh
    bash build.bash
    ```

