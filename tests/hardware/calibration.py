#!/usr/bin/env python3
from datetime import datetime
from DM_SocketCAN import *
import numpy as np
from numpy import pi
import time
import keyboard

FOLLOWER_DEVICENAME0 = "can0"
TICK = 0.01
POSE0 = [0, 0, 0, 0, 0, 0, 0]
K0 = [0, 0, 0, 0, 0, 0, 0]
KP1 = [35, 35, 25, 33, 6, 5, 4]
KD = [1.2, 3.8, 1.0, 0.5, 0.2, 0.2, 0.2]

def set_joint_limits(follower):
    pose = POSE0
    follower.set_goal_positions_sync(pose, KP1, KD)
    time.sleep(3)
    print("ready")
    t0 = datetime.now().timestamp()
    t_schedule = t0
    i = 0

    for i in range(7):
        current_motor = follower.motors[i]
        while True:
            print("move to motor %d to upper most joint limit", current_motor.can_id)
            follower.move_towards_sync(pose, K0, K0)
            if keyboard.is_pressed():
                print("set upper joint limit to %d", current_motor.getPosition())
                break
        while True:
            print("move to motor %d to lower most joint limit", current_motor.can_id)
            follower.move_towards_sync(pose, K0, K0)
            if keyboard.is_pressed():
                print("set lower joint limit to %d", current_motor.getPosition())
                break

if __name__ == "__main__":
    import click
    @click.command()
    @click.option('--set_zero_position', is_flag=True, default=False)
    def main(set_zero_position):
        save = "record00.npz"
        follower = DamiaoPort(FOLLOWER_DEVICENAME0,
                              [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4310, DM_Motor_Type.DM4310, DM_Motor_Type.DM4310],
                              [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
                              [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
                              [True, True, True, True, True, True, True])
        if set_zero_position:
            follower.set_zero_position()
            return follower.disable()
        try:
            record(follower)
        except KeyboardInterrupt:
            print("key pressed")
        follower.set_goal_positions_sync(POSE0, KP1, KD)
        if save:
            follower.save_status(save)
        time.sleep(3)
        follower.disable()
        print("done")
    main()
