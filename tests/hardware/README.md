# Real Hardware Motor Control Examples

This repository provides examples and tools for controlling **DAMIAO motors** through **SocketCAN**.

## Setup

### Prerequisites

1. **Ensure Python Version >= 3.10**:

   ```bash
   python3 --version
2. **Install Required Libraries and Tools:**

    ```bash
    sudo apt-get update && sudo apt-get install -y can-utils
    git clone <repository_url>
    cd <repository_name>
    pip install -r requirements.txt
### Connect Motor to PC

1. **Kill Existing slcand Processes (if any):**

    ```bash
    sudo pkill -f slcand
2. **Check for Your USB Device:**
    - Identify the USB-CAN adapter (should appear as /dev/ttyACM0 or similar):
    ```bash
    ls /dev/tty* | grep ttyACM
3. **Set Up the slcand Interface:**
    - Replace /dev/ttyACM0 with your detected device. Match the bitrate (-s8 for 1 Mbps):
    ```bash
    sudo slcand -o -c -s8 /dev/ttyACM0 can0
4. **Bring Up the can0 Interface:**

    ```bash
    sudo ip link set can0 up type can bitrate 1000000
### Verify the Setup

1. **Use the following command to verify the can0 interface:**

    ```bash
    ip link show can0
2. **Expected Output:**

    ```bash
    3: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UNKNOWN mode DEFAULT group default qlen
### Running Examples

#### Single Motor Test - DEVELOPMENT IN PROGRESS
Ensure that DM motor type, CAN ID, and master id are configured correctly within the python script, otherwise edit `single_motor_test.py`
```bash
python3 single_motor_test.py