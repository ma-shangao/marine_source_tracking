
FROM ros:jazzy

RUN apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y python3-pip\
 && apt-get install -y git-all 

# Dependencies for libigl
RUN apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y build-essential \
 && apt-get install -y cmake \
 && apt-get install -y libx11-dev \
 && apt-get install -y mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev \
 && apt-get install -y libxrandr-dev \
 && apt-get install -y libxi-dev \
 && apt-get install -y libxmu-dev \
 && apt-get install -y libblas-dev \
 && apt-get install -y libxinerama-dev \
 && apt-get install -y libxcursor-dev

WORKDIR /work/ros_ws/src
COPY ./ marine_source_tracking/

WORKDIR /work/ardu_ws/src
WORKDIR /work/ardu_ws
RUN vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
# # Install dependencies
RUN /bin/bash -c "apt-get update && rosdep install -y -r --from-paths . --ignore-src"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-up-to ardupilot_msgs"

WORKDIR /work/ros_ws
RUN /bin/bash -c "apt-get update && rosdep install -y -r --from-paths . --ignore-src"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /work/ardu_ws/install/setup.bash \
    && colcon build"

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /work/ardu_ws/install/setup.bash \
    && colcon test"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /work/ardu_ws/install/setup.bash \
    && colcon test-result --verbose"

COPY ./experiments /root/
