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
- `miivii_gsml_ros`
    ```bash
    sudo apt install ros-foxy-camera-info-manager
    ```
- `gmsl_ros2`
    当处于其他硬件平台是使用该驱动
    ``` bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
    国内使用 rosdepc, 可以使用[一键脚本](https://github.com/fishros/install)安装
    ```
    rosdep install --from-paths src --ignore-src -r -y
    ```
