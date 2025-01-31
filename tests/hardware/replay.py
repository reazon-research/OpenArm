#!/usr/bin/env python3
from datetime import datetime
from DM_SocketCAN import *
import numpy as np
from numpy import pi
import time

FOLLOWER_DEVICENAME0 = "can0"
TICK = 0.01
POSE0 = [0, 0, 0, 0, 0, 0, 0]
KP = [35, 35, 25, 33, 6, 5, 4]
KD = [1.2, 3.8, 1.0, 0.5, 0.2, 0.2, 0.2]

def current_pose(follower):
    if len(follower.stat_data) == 0:
        return False
    current_stat = np.array(follower.stat_data[-1])
    return current_stat[:,2]

def replay(follower, logfile, delay=8):
    time_, data = np.load(logfile).values()
    print(data.shape)
    pose = POSE0
    poses = data[:,:,2]
    follower.set_goal_positions_sync(pose, KP, KD)
    time.sleep(3)
    t0 = datetime.now().timestamp()
    t_schedule = t0
    for n in range(2):
        for r in range(5):
            #steps = 2 ** r
            steps = r + 1
            for i in range(len(poses)-1):
                p0 = poses[i]
                p1 = poses[i+1]
                for t in np.linspace(0, 1, steps):
                    pose = t * p0 + (1-t) * p1
                    follower.move_towards_sync(pose, KP, KD)
                    follower.get_present_status()
                    t_schedule += TICK
                    td = t_schedule - datetime.now().timestamp()
                    if td > 0:
                        time.sleep(td)
                    else:
                        print("timeout", td)
            print("replay finished {} {}".format(n, r))

if __name__ == "__main__":
    import click
    @click.command()
    @click.option('--set_zero_position', is_flag=True, default=False)
    @click.option('--logfile', default="record00.npz")
    def main(set_zero_position, logfile):
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
            replay(follower, logfile)
        except KeyboardInterrupt:
            print("key pressed")
        follower.set_goal_positions_sync(POSE0, KP, KD)
        save = datetime.now().strftime("%Y%m%d%H%M%S")
        follower.save_status(save)
        time.sleep(3)
        follower.disable()
        print("done")
    main()
