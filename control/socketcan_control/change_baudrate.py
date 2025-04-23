#!/usr/bin/env python3
from DM_SocketCAN import *
import click

CAN_DEVICE_NAME = "can0"
SAVE_MOTOR_PARAMETER_PERMANENTLY = False

if __name__ == "__main__":
    print(
        "WARNING: setting the baudrate above 1Mbps requires specialized hardware supporting CAN-FD."
    )
    print(f"flashing {CAN_DEVICE_NAME}")

    @click.command()
    @click.argument("baudrate", type=int)
    def main(baudrate):
        baudrates = [
            125000,
            200000,
            250000,
            500000,
            1000000,
            2000000,
            2500000,
            3200000,
            4000000,
            5000000,
        ]
        if baudrate not in baudrates:
            print(f"available baudrates are {baudrates}")
            return -1
        print(
            "CAUTION: Motor parameters can be modified up to 10,000 times. Be sure not to use this command too often!"
        )
        follower = DamiaoPort(
            CAN_DEVICE_NAME,
            [
                DM_Motor_Type.DM4340,
                DM_Motor_Type.DM4340,
                DM_Motor_Type.DM4340,
                DM_Motor_Type.DM4340,
                DM_Motor_Type.DM4310,
                DM_Motor_Type.DM4310,
                DM_Motor_Type.DM4310,
            ],
            [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
            [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
            [True, True, True, True, True, True, True],
        )
        addr = 35
        bval = baudrates.index(baudrate)
        for i, motor in enumerate(follower.motors, start=1):
            follower.control.read_motor_param(motor, addr)
            if SAVE_MOTOR_PARAMETER_PERMANENTLY:
                print(f"saving motor parameter for motor {i}")
                follower.control.save_motor_param(motor)
            follower.control.change_motor_param(motor, addr, bval)
            follower.control.read_motor_param(motor, addr)
        follower.disable()
        print("done")

    main()
