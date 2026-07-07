# Gazebo Simulation

This directory contains the code for simulating the Viper quadcopter in Gazebo Fortress. This version was selected as development was on an Ubuntu 22.04 machine.

If you wish to launch the model directly from command line without ROS 2 bridge, then `cd` into `gazebo` directory and run:

```bash
ign gazebo worlds/world.sdf
```
This will run with the `sdf` files you see in the `gazebo` dev directory. If simulation launched with `viper-sim.py`, it's actually running from the `install/viper/share` directory. Make sure to `colcon build` to copy it over.

Then in another terminal, run the following to make drone fly:

```bash
ign topic -t /X3/gazebo/command/motor_speed --msgtype ignition.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
```
Flight controller has a safety check for enable button, so above command will not work directly when flight controller running. Without the enable button held down, the flight controller sends a [0, 0, 0, 0] motor speed command immediately. If want to try out, launch this world by itself without flight controller code running. Will lose joystick support though. 

Reset pose:
```bash
ign service -s /world/quadcopter_teleop/set_pose \
    --reqtype ignition.msgs.Pose \
    --reptype ignition.msgs.Boolean \
    --req "name: 'viper', position: {x: 0, y: 0, z: 3}, orientation: {x: 0, y: 0, z: 0, w: 1}" \
    --timeout 2000
```

## PID Tuning
Within the `model.sdf` file, at the very bottom, we can change the joint type to lock the quadopter along certain axis or in certain positions in space, allowing for easier PID tuning. Use the ball joint for rotation about all three axes, and the roll joint for tuning along a certain axis.

## Directory Structure
```bash
gazebo
├── config
│   └── bridge.yaml  # Config options for ros_gz_bridge
├── env-hooks
│   └── viper.dsv.in  # Defines the IGN_GAZEBO_RESOURCE_PATH
├── models
│   └── viper
│       ├── model.config  # Needed for loading model file correctly using the models/viper directory
│       └── model.sdf
├── README.md
└── worlds
    └── world.sdf
```

`IGN_GAZEBO_RESOURCE_PATH` currently points to `gazebo` directory. This allows launching of Gazebo simulation independently by just calling `ign gazebo worlds/world.sdf` from `gazebo` directory.

See original quadcopter example [here](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/quadcopter.sdf).


