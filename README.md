Streaming video between pi4 <> docker
pi4:
`libcamera-vid -t 0 --width 1920 --height 1080 --inline --listen -o tcp://0.0.0.0:8888`
docker:
`ffplay tcp://192.168.1.210:8888 -vf "setpts=N/30" -ffls nobuffer -flags low_delay -framedrop`