#!/usr/bin/ev python 3
from DM_SocketCAN import *
from datetime import datetime
import numpy as np
import json
import time

openarm_DEVICENAME0 = "can0"
HARDSTOP_THRESHOLD = 0.01
STABILITY_COUNT = 10
KP1 = [15, 5, 20, 5, 3, 3, 0.5]
KD = [1.2, 3.8, 1.0, 0.5, 0.2, 0.2, 0.2]

#READ FIRST
#CURRENTLY AUTOCALIBRAITON ONLY SETUP FOR RIGHT ARM

#hardcoding offsets for each motor
HARDSTOP_OFFSETS = {
    0x01: 0.7853,
    0x02: 0.1745,
    0x03: 2.0944,
    0x04: 0.3490,
    0x05: 2.0944,
    0x06: 1.5708,
    0x07: 0.9599,
}

#hardcoding motor directions to move in by default (RIGHT HAND SIDE)
HARDSTOP_DIRECTIONS = {
    0x01: -1,
    0x02: -1,
    0x03: -1,
    0x04: -1,
    0x05: -1,
    0x06: -1,
    0x07: -1,
}

def detect_hardstop(openarm, index):
    motor = openarm.motors[index]
    #moves a single motor towards it's hardstop until movement stops.
    direction = HARDSTOP_DIRECTIONS[motor.SlaveID]

    prev_position = None
    step_size = 0.005
    stable_counter = 0

    while stable_counter < 150:
        #move motor towards desired direction
        openarm.control.controlMIT(motor, KP1[index]*2, KD[index], step_size*direction, 0, 0)
        step_size += 0.01
        sleep(0.05)
        #get current position
        current_position = motor.getPosition()
        #check if position delta is lower than threshold 0.1 rad
        if prev_position is not None:
            delta = abs(current_position - prev_position)
            if delta < HARDSTOP_THRESHOLD:
                stable_counter += 1
        else:
            stable_counter = 0
        prev_position = current_position

    print(f"Hardstop detected at {current_position:.4f} radians for Motor {index + 1}")
    return current_position
    

def calibrate_joint_limits(openarm):
    #moves all motors to their hardstops and applies offsets
    print("Moving motors to hardstops...")

    HARDSTOP_POSITIONS = {}
    ZERO_POSITIONS = {
    }
    for i,motor in enumerate(openarm.motors):
            HARDSTOP_POSITIONS[motor.SlaveID] = detect_hardstop(openarm, i)
            ZERO_POSITIONS[motor.SlaveID] = HARDSTOP_POSITIONS[motor.SlaveID] + HARDSTOP_OFFSETS[motor.SlaveID]
            p0 = motor.getPosition()
            p1 = ZERO_POSITIONS[motor.SlaveID]
                
            t0 = datetime.now().timestamp()
            t_schedule = t0
            for t in np.linspace(0, 1, 100):
                pose = (1-t) * p0 + t * p1
                openarm.control.controlMIT(motor, KP1[i], KD[i],  pose, 0, 0)
                t_schedule += 0.01
                td = t_schedule - datetime.now().timestamp()
                if td > 0:
                    time.sleep(td)
                else:
                    print("timeout", td)
            sleep(3)

    calibrated_positions = {
         motor_id: HARDSTOP_POSITIONS[motor_id] + HARDSTOP_OFFSETS[motor_id]
         for motor_id in HARDSTOP_POSITIONS
    }

    with open("calibrated_positions.json", "w") as f:
         json.dump(calibrated_positions, f, indent=4)

    print("Calibration complete. Positions saved.")
    return calibrated_positions

if __name__ == "__main__":
     openarm = DamiaoPort(openarm_DEVICENAME0,
                          [DM_Motor_Type.DM4340] *4 + [DM_Motor_Type.DM4310] *3,
                          [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
                          [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
                          [True] *7, control_mode=Control_Type.VEL)
     calibrated_positions = calibrate_joint_limits(openarm)
     print("Calibrated zero positions:", calibrated_positions)

     openarm.shutdown()