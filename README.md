# ORB_SLAM3 ROS2 Wrapper

### Camera calibration
https://navigation.ros.org/tutorials/docs/camera_calibration.html

```
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.031 --pattern=chessboard --no-service-check --ros-args -r image:=/camera/image_raw -p camera:=/camera/camera_info
```

## Installation
add this to  /etc/docker/daemon.json
```
{
    "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "exec-opts": ["native.cgroupdriver=cgroupfs"]
}
```

add this line to your .bashrc file:
```
sed -i -e '$a\'$'\n''xhost +local:docker' ~/.bashrc
source ~/.bashrc
```
## Run

`ros2 launch launch/orb_slam3_launch.py`

## Acknowledgements

https://github.com/UZ-SLAMLab/ORB_SLAM3
https://github.com/abhineet123/ORB_SLAM2
https://github.com/zang09/ORB_SLAM3_ROS2

