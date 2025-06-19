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
import click
import numpy as np
import json

openarm_DEVICENAME0 = "can0"
POSE0 = [0, 0, 0, 0, 0, 0, 0]
K0 = [0, 0, 0, 0, 0, 0, 0]

# Default values for joint limits (will be overwritten during calibration)
JOINT_LIMITS = {
    motor_id: {"lower": -np.pi, "upper": np.pi}  # Default to -π to π
    for motor_id in [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
}

def calibrate_joint_limits(openarm):
    """
    Calibrate the joint limits of the robot manually.
    """

    openarm.disable()
    print("Torque has been disabled for all motors")

    print("Calibration started. Move each motor manually to its limits.")
    limits = {}
    try:
        for motor in openarm.motors:
            motor_id = motor.SlaveID
            
            # Initialize limits for this motor
            limits[motor_id] = {"lower": None, "upper": None}

            print(f"Move Motor {motor_id} to its LOWER limit and press Enter.")
            input("Press Enter when the motor is in its lower limit position.")
            openarm.control.controlMIT(motor, 0, 0, 0, 0, 0)
            lower_limit = float(motor.getPosition())  # Convert to Python float
            limits[motor_id]["lower"] = lower_limit
            print(f"Lower limit recorded: {lower_limit:.4f} radians")

            print(f"Move Motor {motor_id} to its UPPER limit and press Enter.")
            input("Press Enter when the motor is in its upper limit position.")
            openarm.control.controlMIT(motor, 0, 0, 0, 0, 0)
            upper_limit = float(motor.getPosition())  # Convert to Python float
            limits[motor_id]["upper"] = upper_limit
            print(f"Upper limit recorded: {upper_limit:.4f} radians")

            # Re-enable torque after calibration
            openarm.control.controlMIT(motor, 0, 0, 0, 0, motor.goal_tau)
            print(f"Torque re-enabled for Motor {motor_id}.\n")

        # Save calibrated limits to a file
        with open("joint_limits.json", "w") as f:
            # Convert all motor IDs to strings to ensure compatibility with JSON
            json.dump({str(k): v for k, v in limits.items()}, f, indent=4)
        print("Calibration complete. Limits saved to 'joint_limits.json'.")

    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
        # Ensure torque is re-enabled in case of interruption
        for motor in openarm.motors:
            openarm.control.controlMIT(motor, 0, 0, 0, 0, motor.goal_tau)
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
    
    @click.command()
    @click.option('--set_zero_position', is_flag=True, default=False, help="Load joint limits from file.")
    @click.option('--load_limits', is_flag=True, default=False, help="Load joint limits from file.")
    def main(set_zero_position, load_limits):
        # Initialize motor controller
        openarm = DamiaoPort(openarm_DEVICENAME0,
                              [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4310, DM_Motor_Type.DM4310, DM_Motor_Type.DM4310],
                              [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
                              [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
                              [True, True, True, True, True, True, True])
        
        if set_zero_position:
            openarm.set_zero_position()
            return openarm.shutdown()
        else:
            calibrate_joint_limits(openarm)

        # Load Limits Option
        if load_limits:
            limits = load_joint_limits("joint_limits_openarm.json")
            if limits:
                for motor in openarm.motors:
                    motor_id = motor.SlaveID
                    if str(motor_id) in limits:
                        print(f"Motor {motor_id}: Lower Limit: {limits[str(motor_id)]['lower']}, Upper Limit: {limits[str(motor_id)]['upper']}")

        print("Exiting...")

        openarm.shutdown()

    main()
