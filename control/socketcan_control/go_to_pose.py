#!/usr/bin/env python3
#
# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from DM_SocketCAN import *
from datetime import datetime
import click
import json
import time

openarm_DEVICENAME0 = "can0"

TICK = 0.01
POSE0 = [0, 0, 0, 0, 0, 0, 0]
KP1 = [35, 35, 25, 33, 6, 5, 4]
KD = [1.2, 3.8, 1.0, 0.5, 0.2, 0.2, 0.2]

def get_current_positions(openarm):
    """
    Retrieve the current joint positions of all motors.
    """
    status = openarm.get_present_status()  # Get motor statuses
    current_positions = np.array([motor_status[2] for motor_status in status])  # Extract positions
    return current_positions

def load_joint_limits(filename="joint_limits.json"):
    """
    Load joint limits from a file.
    """
    try:
        with open(filename, "r") as f:
            limits = json.load(f)
        print(f"Joint limits loaded from '{filename}'.")
        return {int(k): v for k, v in limits.items()}  # Convert keys back to integers
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None

def validate_goal_positions(goal_positions, joint_limits):
    """
    Validate that all goal positions are within the joint limits.
    Ensures the goal position lies between the lower and upper limits,
    regardless of whether the limits are positive or negative.
    """
    for motor_id, goal_position in enumerate(goal_positions, start=1):  # Iterate over array
        if motor_id not in joint_limits:
            print(f"Warning: No joint limits found for Motor {motor_id}. Skipping validation.")
            continue

        lower = joint_limits[motor_id]["lower"]
        upper = joint_limits[motor_id]["upper"]

        # Determine the actual range (min, max) to validate against
        limit_min, limit_max = min(lower, upper), max(lower, upper)

        if not (limit_min <= goal_position <= limit_max):
            print(f"Error: Goal position for Motor {motor_id} ({goal_position:.4f}) "
                  f"is out of bounds (Valid Range: {limit_min:.4f} to {limit_max:.4f}).")
            return False

    return True

def hold(openarm, goal_positions):
    print("Holding position. Press Ctrl+C to return to zero position.")
    while True:
        openarm.move_towards_sync(goal_positions, KP1, KD)
        openarm.get_present_status()
        time.sleep(TICK)  # Maintain the position at regular intervals

def move(openarm, start, end):
    openarm.set_goal_positions_sync(start, KP1, KD)
    time.sleep(3)

    p0 = np.array(start)
    p1 = np.array(end)

    t0 = datetime.now().timestamp()
    t_schedule = t0
    for t in np.linspace(0, 1, 1000):
        pose = (1-t) * p0 + t * p1
        openarm.move_towards_sync(pose, KP1, KD)
        openarm.get_present_status()
        t_schedule += TICK
        td = t_schedule - datetime.now().timestamp()
        if td > 0:
            time.sleep(td)
        else:
            print("timeout", td)
    
def move_to_zero(openarm, joint_limits):

    p0 = np.array(get_current_positions(openarm))
    p1 = np.array(POSE0)

    if not validate_goal_positions(p0, joint_limits):
        print("Current robot positions out of joint limits")
        return
    else:
        t0 = datetime.now().timestamp()
        t_schedule = t0
        for t in np.linspace(0, 1, 1000):
            pose = (1-t) * p0 + t * p1
            openarm.move_towards_sync(pose, KP1, KD)
            openarm.get_present_status()
            t_schedule += TICK
            td = t_schedule - datetime.now().timestamp()
            if td > 0:
                time.sleep(td)
            else:
                print("timeout", td)

if __name__ == "__main__":

    @click.command()
    @click.option('--goal_positions', default=None, help="Goal positions for the motors as a JSON string (e.g., '{\"1\": 0.5, \"2\": 1.2}').")
    @click.option('--filename', default="joint_limits_openarm.json", help="Filename to load joint limits from (default: 'joint_limits.json').")

    def main (goal_positions, filename):
        # Initialize motor controller
        openarm = DamiaoPort(openarm_DEVICENAME0,
                                [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                                DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                                DM_Motor_Type.DM4310, DM_Motor_Type.DM4310, DM_Motor_Type.DM4310],
                                [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
                                [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
                                [True, True, True, True, True, True, True])

        # Load joint limits
        joint_limits = load_joint_limits(filename)
        if not joint_limits:
            print("Failed to load joint limits. Exiting.")
            return

        # Parse goal positions
        if goal_positions:
            goal_positions = json.loads(goal_positions)
            goal_positions = {int(k): float(v) for k, v in goal_positions.items()}
        else:
            print("Error: No goal positions provided.")
            return

        goal_positions = np.array([goal_positions[motor_id] for motor_id in range(1, len(POSE0) + 1)])  # Convert goal_positions to a numpy array

        # Validate goal positions
        if not validate_goal_positions(goal_positions, joint_limits):
            print("One or more goal positions are invalid. Exiting.")
            openarm.disable()
            return

        try:
            move(openarm, POSE0, goal_positions)
        except KeyboardInterrupt:
            print("Key Pressed")
            move_to_zero(openarm, joint_limits)
        
        try:
            hold(openarm, goal_positions)
        except KeyboardInterrupt:
            print("Returning to Zero Position")
            move_to_zero(openarm, joint_limits)
            
        print("Synchronous movement complete.")

        # Disable motors after movement
        print("Disabling motors and performing CANbus shutdown")
        openarm.disable()

    main()
