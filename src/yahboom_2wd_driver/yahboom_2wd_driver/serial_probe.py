#!/usr/bin/env python3
"""Small commissioning tool for checking the Yahboom serial connection."""

from __future__ import annotations

import argparse
import glob
import time

try:
    from Rosmaster_Lib import Rosmaster  # type: ignore
except ImportError:
    from .vendor.Rosmaster_Lib import Rosmaster  # type: ignore


def main() -> None:
    parser = argparse.ArgumentParser(description='Probe Yahboom Rosmaster serial connection.')
    parser.add_argument('--serial-port', default='/dev/myserial')
    parser.add_argument('--car-type', type=int, default=4)
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    print('Candidate serial devices:')
    for pattern in ('/dev/myserial', '/dev/ttyUSB*', '/dev/ttyACM*', '/dev/serial/by-id/*'):
        for path in sorted(glob.glob(pattern)):
            print(f'  {path}')

    print(f'Opening {args.serial_port} ...')
    bot = Rosmaster(car_type=args.car_type, com=args.serial_port, debug=args.debug)
    bot.create_receive_threading()
    bot.set_auto_report_state(True, False)
    time.sleep(0.2)

    version = bot.get_version()
    car_type = bot.get_car_type_from_machine()
    battery = bot.get_battery_voltage()
    motion = bot.get_motion_data()
    attitude = bot.get_imu_attitude_data(ToAngle=False)
    encoders = bot.get_motor_encoder()

    print(f'MCU version: {version}')
    print(f'MCU car type: {car_type}')
    print(f'Battery voltage: {battery:.2f} V')
    print(f'Motion vx, vy, wz: {motion}')
    print(f'IMU roll, pitch, yaw [rad]: {attitude}')
    print(f'Encoder M1..M4 ticks: {encoders}')
    bot.set_beep(50)
    bot.set_car_motion(0.0, 0.0, 0.0)
    print('Probe complete.')


if __name__ == '__main__':
    main()
