#!/usr/bin/env python3
# Used to test single motor functionality

from DM_SocketCAN import *
import time

DEFAULT_DEVICENAME0 = "can0"
POSE0 = [0]
KP = [35]
KD = [1.2]

if __name__ == "__main__":
    import click
    @click.command()
    @click.option('--set_zero_position', is_flag=True, default=False)
    def main(set_zero_position):
        motor = DamiaoPort(DEFAULT_DEVICENAME0,
                              [DM_Motor_Type.DM4340],
                              [0x02],
                              [0x12],
                              [True])
        if set_zero_position:
            motor.set_zero_position()
            return motor.disable()
        try:
            while True:
                motor.move_towards([2], [0], [0])
                time.sleep(1)
                motor.move_towards([5], [0], [0])
        except KeyboardInterrupt:
            print("key pressed")

        motor.set_goal_positions_sync(POSE0, KP, KD)
        time.sleep(3)
        motor.disable()
        print("done")
    main()
