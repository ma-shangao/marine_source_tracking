# Uncertainty-Aware Active Source Tracking of Marine Pollution using Unmanned Surface Vehicles

## Depedencies
- Gazebo Harmonic (not ros-gz)
- [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim)
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo), including the additional models

## Usage
Build the provided dockerfile, run the container based on the built image and allow it to access the network of the host machine.

On the host machine:
Start the simulation environment, sdf files can be found in experimnents folder:
```bash
gz sim -v4 test_world.sdf
```

On the container:

Start the SITL:
```bash
cd /work/ardu_ws/src/ardupilot
Tools/autotest/sim_vehicle.py -v Rover -f rover-skid --model JSON  --console --map --custom-location=$YOUR_PREFERRED_LOCATION --enable-dds
# Arm the vehicle
# Set Guided Mode
```

Start the micro-ROS agent:
```bash
source /work/ardu_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

Launch the pollution simulation:
```bash
source /work/ardu_ws/install/setup.bash
source /work/ros_ws/install/setup.bash
ros2 launch pollution_sim sim_odom_tf.launch.py scenario:=$SCENARIO_NAME
```

Launch the tracking server and client:
```bash
source /work/ardu_ws/install/setup.bash
source /work/ros_ws/install/setup.bash
ros2 launch pollution_ipp source_loc.launch.py start_client:=true scenario:=$SCENARIO_NAME st:=$START_POSITION_NAME
```
