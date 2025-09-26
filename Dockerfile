
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
 && apt-get install -y libxcursor-dev \
 && apt-get install -y openjdk-17-jre

WORKDIR /work/ros_ws/src
COPY ./ marine_source_tracking/

WORKDIR /work/ardu_ws/src
WORKDIR /work/ardu_ws
# jazzy version of https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos
RUN vcs import --recursive --input /work/ros_ws/src/marine_source_tracking/ardu_ros2_jazzy.repos src
RUN git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
WORKDIR /work/ardu_ws/Micro-XRCE-DDS-Gen
RUN /bin/bash -c "./gradlew assemble"
ENV PATH="$PATH:/work/ardu_ws/Micro-XRCE-DDS-Gen/scripts"
# Install dependencies for upstream workspace
WORKDIR /work/ardu_ws
RUN /bin/bash -c "apt-get update && rosdep install -y -r --from-paths . --ignore-src"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-up-to micro_ros_agent ardupilot_msgs"

WORKDIR /work/ros_ws
RUN /bin/bash -c "apt-get update && rosdep install -y -r --from-paths . --ignore-src"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /work/ardu_ws/install/setup.bash \
    && colcon build"

# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
#     && source /work/ardu_ws/install/setup.bash \
#     && colcon test"
# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
#     && source /work/ardu_ws/install/setup.bash \
#     && colcon test-result --verbose"
RUN /bin/bash -c "rm -r src"

COPY ./experiments /root/
