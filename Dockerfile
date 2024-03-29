FROM osrf/ros:noetic-desktop-full

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && apt-get install -y git && apt-get install -y \
    ros-noetic-rtabmap \
    ros-noetic-rtabmap-ros \
    ros-noetic-realsense2-camera \
    ros-noetic-fiducials \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-jsk-rviz-plugins

# Create a Catkin workspace and clone demo code
RUN source /opt/ros/noetic/setup.bash \
    && mkdir -p /measurement_demo_ws/src \
    && cd /measurement_demo_ws/src \
    && catkin_init_workspace \

# ADD https://api.github.com/repos/mikegroom765/measurement_tracking_demo/git/refs/heads/master version.json
# RUN git clone -b master https://github.com/mikegroom765/$measurement_tracking_demo.git $GIT_HOME/
    && git clone https://github.com/mikegroom765/measurement_tracking_demo.git

# Build the Catkin workspace and ensure it's sourced

RUN source /opt/ros/noetic/setup.bash \
    && cd measurement_demo_ws \
    && catkin_make \
    && catkin_make --only-pkg-with-deps measurement_tracking_demo \
    && source /measurement_demo_ws/devel/setup.bash \
    && cd src/measurement_tracking_demo/src/ \
    && chmod +x poseUserDataUsage.cpp \
    && cd / \
    && ./ros_entrypoint.sh \
    && cd measurement_demo_ws 
    
RUN echo "source /measurement_demo_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /measurement_demo_ws