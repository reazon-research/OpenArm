#!/usr/bin/env python3
from datetime import datetime
from DM_SocketCAN import *
import numpy as np
import time
import json

FOLLOWER_DEVICENAME0 = "can0"
TICK = 0.01

# Default values for joint limits (will be overwritten during calibration)
JOINT_LIMITS = {
    motor_id: {"lower": -np.pi, "upper": np.pi}  # Default to -π to π
    for motor_id in [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
}

def calibrate_joint_limits(follower):
    """
    Calibrate the joint limits of the robot manually.
    """
    print("Calibration started. Move each motor manually to its limits.")
    limits = {}
    try:
        for motor in follower.motors:
            motor_id = motor.can_id
            input(f"Calibrating Motor {motor_id}. Press Enter when ready.")

            # Disable torque (set to zero) to allow manual movement
            follower.control.controlMIT(motor, 0, 0, 0, 0, 0)
            print(f"Torque disabled for Motor {motor_id}. You can now move it freely.")
            
            # Initialize limits for this motor
            limits[motor_id] = {"lower": None, "upper": None}

            print(f"Move Motor {motor_id} to its LOWER limit and press Enter.")
            input("Press Enter when the motor is in its lower limit position.")
            lower_limit = motor.getPosition()
            limits[motor_id]["lower"] = lower_limit
            print(f"Lower limit recorded: {lower_limit:.4f} radians")

            print(f"Move Motor {motor_id} to its UPPER limit and press Enter.")
            input("Press Enter when the motor is in its upper limit position.")
            upper_limit = motor.getPosition()
            limits[motor_id]["upper"] = upper_limit
            print(f"Upper limit recorded: {upper_limit:.4f} radians")

            # Re-enable torque after calibration
            follower.control.controlMIT(motor, 0, 0, 0, 0, motor.goal_tau)
            print(f"Torque re-enabled for Motor {motor_id}.")

        # Save calibrated limits to a file
        with open("joint_limits.json", "w") as f:
            json.dump(limits, f, indent=4)
        print("Calibration complete. Limits saved to 'joint_limits.json'.")

    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
        # Ensure torque is re-enabled in case of interruption
        for motor in follower.motors:
            follower.control.controlMIT(motor, 0, 0, 0, 0, motor.goal_tau)
    return limits

def load_joint_limits(filename="joint_limits.json"):
    """
    Load joint limits from a file.
    """
    try:
        with open(filename, "r") as f:
            limits = json.load(f)
        print(f"Joint limits loaded from '{filename}'.")
        return limits
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None

if __name__ == "__main__":
    import click

    @click.command()
    @click.option('--calibrate', is_flag=True, default=False, help="Run joint limit calibration.")
    @click.option('--load_limits', is_flag=True, default=False, help="Load joint limits from file.")
    def main(calibrate, load_limits):
        # Initialize motor controller
        follower = DamiaoPort(FOLLOWER_DEVICENAME0,
                              [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4310, DM_Motor_Type.DM4310, DM_Motor_Type.DM4310],
                              [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
                              [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
                              [True, True, True, True, True, True, True])
        
        # Calibration Option
        if calibrate:
            calibrate_joint_limits(follower)

        # Load Limits Option
        if load_limits:
            limits = load_joint_limits()
            if limits:
                for motor in follower.motors:
                    motor_id = motor.can_id
                    if str(motor_id) in limits:
                        print(f"Motor {motor_id}: Lower Limit: {limits[str(motor_id)]['lower']}, Upper Limit: {limits[str(motor_id)]['upper']}")

        try:
            # Placeholder for other operations
            print("Ready for other operations.")
        except KeyboardInterrupt:
            print("Exiting...")

        follower.disable()

    main()
