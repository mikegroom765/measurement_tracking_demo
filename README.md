# measurement_tracking_demo

Requires NVIDIA Container Toolkit, install instructions here: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker

# those following 3 lines would need to be done only one time
$ XAUTH=/tmp/.docker.xauth

$ touch $XAUTH

$ xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
