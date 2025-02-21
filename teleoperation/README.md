[日本語](teleoperation/README_jp.md)

# OpenArm Bilateral Control
This repository performs bilateral control using OpenArm, which is being developed by Reazon HI Lab.

## Quick Start Guide

### Enter root directory of the teleoperation project

```console
$ cd teleoperation/
```
### Install the required depdencies with: `rosdep`

```console
# When using rosdep for the first time, initialize it (required only for the first time)
sudo rosdep init

# Install dependencies
sudo apt-get update
rosdep update
rosdep install -i --from-path src/reazon_bilateral_aloha/ -y
```

### Build

```console
$ colcon build --packages-select openarm_bilateral && source install/setup.bash
```

### Enable SocketCAN (Enable Canable)
Set the leader's IP name to can0 and the follower's IP name to can1.

```console
$ sudo pkill -f slcand # you can do this to kill past slcand processes

# Set up usb2can interfaces
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up type can bitrate 1000000

sudo slcand -o -c -s8 /dev/ttyACM1 can1
sudo ip link set can1 up type can bitrate 1000000
```

Activate bilateral control. At this time, control starts after moving to the initial position.


```
$  ros2 launch openarm_bilateral bilateral_openarm_main.launch.py
```

## Switching between Bilateral and Unilateral Control

Unilateral control can be achieved by uncommenting `DoControl()` in `timer_callback()` in the FollowerNode and LeaderNode classes in `src/bilateral_main.cpp` and applying `DoControl_u()`.

### `src/bilateral_main.cpp`

```cpp
void timer_callback()
{
    control_f_->DoControl();
    // control_f_->DoControl_u();
}
```

```cpp
void timer_callback()
{
    control_l_->DoControl();
    // control_l_->DoControl_u();
}
```


## Caution
At initial setup place the leader and follower robots so that their tips are facing straight down. The program calibrates and performs zeroing initialization during the start of every program execution. If it deviates from this point, the robot may behave unexpectedly due to errors in gravity compensation, etc.
