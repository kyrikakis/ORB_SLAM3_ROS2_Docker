Streaming video between pi4 <> docker
pi4:
`libcamera-vid -t 0 --width 1920 --height 1080 --inline --listen -o tcp://0.0.0.0:8888`
docker:
`ffplay tcp://192.168.1.210:8888 -vf "setpts=N/30" -ffls nobuffer -flags low_delay -framedrop`
### Camera calibration
https://navigation.ros.org/tutorials/docs/camera_calibration.html

## Installation
add this line to your .bashrc file:
```
sed -i -e '$a\'$'\n''xhost +local:docker' ~/.bashrc
source ~/.bashrc
```
## Run

Run monocular:
`source install/setup.bash && ros2 run orbslam3 mono-pcloud /workspaces/ORB_SLAM3_ROS2_Docker/vocabulary/ORBvoc.txt /workspaces/ORB_SLAM3_ROS2_Docker/config/monocular/TUM1_wide.yaml`