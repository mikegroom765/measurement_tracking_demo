# measurement_tracking_demo

A docker container for using RTABMap with a realsense depth camera, to track and record aruco markers. 

Requires NVIDIA Container Toolkit, install instructions here: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker

# those following 3 lines would need to be done only one time
$ XAUTH=/tmp/.docker.xauth

$ touch $XAUTH

$ xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

May have to do:  
$xhost +local:docker~
