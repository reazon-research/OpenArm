# Real Hardware Motor Control Examples

This repository provides examples and tools for controlling **DAMIAO motors** through **SocketCAN**.

## Setup

### Initial Motor Configuration
- Install DAMIAO debugger tool from [DAMIAO documentation](https://github.com/dmBots/DAMIAO-Motor/blob/main/README_EN.md)
   - Use the USB to CAN DAMIAO debugging tool that should be provided with the motor.
   - Connect the motor debugging serial port to the adapter via the GH1.25 cable-3pin.
   - Power the motor via XT30(2+2)-F plug cable.
- Open the debugger tool and ensure that the interface is connected with UART@921600bps.
   - Starting with the shoulder motor as ID 1. Set the CAN IDs from 0x01 to 0x07 and the Master IDs from 0x11 to 0x17 for all 7 motors.
   - The motor for the end effector can be set to ID 8.
- **Disclaimer 1:** Ensure that the motor configuration is done before assembling the robot, otherwise some motor debugging serial ports may not be in an accessible location.
- **Disclaimer 2:** The DAMIAO debugger tool is currently only available in Chinese so translation may need to be done to fully understand the tool UI.

### Prerequisites

1. **Ensure Python Version >= 3.10**:

   ```bash
   python3 --version
   ```
2. **Install Required Libraries and Tools:**

    ```bash
    sudo apt-get update && sudo apt-get install -y can-utils
    git clone git@github.com:reazon-research/OpenArm.git
    cd OpenArm/control
    pip install -r requirements.txt # being in a virtual environment may help
    ```
### Connect Motor to PC
1. **Set up CAN adapter:**
   - There are many options for CAN interfaces. The [Canable2.0](https://canable.io/) and [2-CH CANFD HAT](https://www.waveshare.com/wiki/2-CH_CAN_FD_HAT) (CANFD support tested, designed for Raspberry Pi) are the specific adapters that the HI Lab team has been testing with.
3. **Bring Up the can0 Interface:**

    ```bash
    # Using CAN
    sudo ip link set can0 up type can bitrate 1000000
    # Using CANFD
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on
    ```
### Verify the Setup

1. **Use the following command to verify the can0 interface:**

    ```bash
    ip link show can0
2. **Expected Output:**
    ```bash
    3: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UNKNOWN mode DEFAULT group default qlen
## Programs

### Calibration
This program provides a method to manually calibrate positional limits for all the motors and saves it into `joint_limits.json`. To calibrate, you will have to manually move each motor to its lower and upper limits when prompted by the code. 
```bash
python3 calibration.py

# If you want to set current robot position as zero position
python3 calibration.py --set_zero_position

# If you want to print joint limits from a file
python3 calibration.py --load_limits
```

### Go To Pose
Go To Pose is a script that will move to a defined pose by the user and hold it until a KeyboardInterrupt. It also validates the given trajectory against the joint limits that were determined in the calibration script. The default filename for the limits is `joint_limits_openarm.json`
```bash
python3 go_to_pose.py --goal_positions '{"1": 0.5, "2": 0.5, "3": 1.0, "4": 1.0, "5": -0.5, "6": 0.0, "7": 0.0}'

# The default filename for the limits is `joint_limits_openarm.json`, if you want to use an alternate file, do this command:
python3 go_to_pose.py --goal_positions '{"1": 0.5, "2": 0.5, "3": 1.0, "4": 1.0, "5": -0.5, "6": 0.0, "7": 0.0}' --filname "custom_limits.json"
```

### Record and Replay
Record and replay are test scripts that allow you to manually record a motion using the record program and replay that script by utilizing the replay script.
```bash
python3 record.py # this will create a file named record00.npz

python3 replay.py # this will play the trajectory saved in record00.npz
```
