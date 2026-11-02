# Uncertainty-Aware Active Source Tracking of Marine Pollution using Unmanned Surface Vehicles

## Publication
The code in this repository was developed as part of our research on active sensing of marine pollution using unmanned surface vehicles. The work was published in IROS 2026, and the paper can be found [here](https://arxiv.org/abs/2509.20593). Please cite the paper if you use this code in your research.

Ma, S., Wang, Y., Huang, Y., Bucknall, R. and Liu, Y. (2026) 'Active tracking of marine pollution sources: an uncertainty-aware categorical Bayesian framework for unmanned surface vehicles', 2026 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). Pittsburgh, USA, 27 September - 1 October. IEEE, in press.

```bibtex
@inproceedings{ma_active_2026,
	address = {Pittsburgh, USA},
	title = {Active tracking of marine pollution sources: an uncertainty-aware categorical {Bayesian} framework for unmanned surface vehicles},
	booktitle = {2026 {IEEE}/{RSJ} {International} {Conference} on {Intelligent} {Robots} and {Systems} ({IROS}) {Proceedings}},
	publisher = {IEEE},
	author = {Ma, Song and Wang, Yanchao and Huang, Yewei and Bucknall, Richard and Liu, Yuanchang},
	month = sep,
	year = {2026},
	note = {In press},
}
```

## Depedencies
- Gazebo Harmonic (not ros-gz)
- [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim)
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo), including the additional models

## Usage
Build the provided dockerfile, run the container based on the built image and allow it to access the network of the host machine.

On the host machine:
Start the simulation environment, sdf files can be found in experiments folder:
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

## Credits
This repository used code from:
- [odom_to_tf_ros2](https://github.com/gstavrinos/odom_to_tf_ros2)
- [rapidcsv](https://github.com/d99kris/rapidcsv)

The local estimation kernal and propagation was based on the implementation of [GasSourceLocalization](https://github.com/MAPIRlab/GasSourceLocalization).
