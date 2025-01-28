#!/usr/bin/env python3
from DM_SocketCAN import *
from datetime import datetime
import click
import json
import time

openarm_DEVICENAME0 = "can0"

TICK = 0.01
POSE0 = [0, 0, 0, 0, 0, 0, 0]
# Per-motor PD values
KP1 = [35, 35, 25, 33, 6, 5, 4]
KD = [1.2, 3.8, 1.0, 0.5, 0.2, 0.2, 0.2]

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
    for motor_id, goal_position in goal_positions.items():
        lower = joint_limits[motor_id]["lower"]
        upper = joint_limits[motor_id]["upper"]
        
        # Determine the actual range (min, max) to validate against
        limit_min, limit_max = min(lower, upper), max(lower, upper)
        
        if not (limit_min <= goal_position <= limit_max):
            print(f"Error: Goal position for Motor {motor_id} ({goal_position:.4f}) "
                  f"is out of bounds (Valid Range: {limit_min:.4f} to {limit_max:.4f}).")
            return False
    return True

def move(openarm, goal_positions):
    openarm.set_goal_positions_sync(POSE0, KP1, KD)
    time.sleep(3)

    p0 = np.array(POSE0)
    p1 = np.array([goal_positions[motor_id] for motor_id in range(1, len(POSE0) + 1)])  # Convert goal_positions to a numpy array

    t0 = datetime.now().timestamp()
    t_schedule = t0
    for t in np.linspace(0, 1, 1000):
        pose = t * p1 + (1-t) * p0
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

        # Validate goal positions
        if not validate_goal_positions(goal_positions, joint_limits):
            print("One or more goal positions are invalid. Exiting.")
            return
        
        try:
            move(openarm, goal_positions)
        except KeyboardInterrupt:
            print("Key Pressed")

        print("Synchronous movement complete.")

        # Disable motors after movement
        print("Disabling motors and performing CANbus shutdown")
        openarm.disable()

    main()
