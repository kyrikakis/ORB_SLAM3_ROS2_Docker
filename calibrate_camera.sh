docker exec -it orbslam3_ros2 bash -i -c "ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.027 --ros-args -r image:=/camera/image_raw -p camera:=/camera"
