#!/usr/bin/env python3
"""Direct M2/M4 motor commissioning tool.

This tool uses Rosmaster.set_motor(), which is PWM duty control in the uploaded
Yahboom library. Use low values first and keep the robot lifted from the table.
"""

from __future__ import annotations

import argparse
import time

try:
    from Rosmaster_Lib import Rosmaster  # type: ignore
except ImportError:
    from .vendor.Rosmaster_Lib import Rosmaster  # type: ignore


def clamp_int(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def main() -> None:
    parser = argparse.ArgumentParser(description='Test Yahboom M2/M4 motors using PWM.')
    parser.add_argument('--serial-port', default='/dev/myserial')
    parser.add_argument('--car-type', type=int, default=4)
    parser.add_argument('--left-port', type=int, default=2)
    parser.add_argument('--right-port', type=int, default=4)
    parser.add_argument('--left-sign', type=int, default=1)
    parser.add_argument('--right-sign', type=int, default=1)
    parser.add_argument('--speed', type=int, default=15, help='PWM percent, use 10..20 first')
    parser.add_argument('--duration', type=float, default=1.0)
    parser.add_argument('--turn', action='store_true', help='Run left and right in opposite directions')
    args = parser.parse_args()

    if args.left_port == args.right_port or args.left_port not in (1, 2, 3, 4) or args.right_port not in (1, 2, 3, 4):
        raise SystemExit('Motor ports must be different and in [1, 2, 3, 4].')

    bot = Rosmaster(car_type=args.car_type, com=args.serial_port, debug=False)
    bot.create_receive_threading()
    bot.set_auto_report_state(True, False)
    bot.set_beep(50)
    time.sleep(0.1)

    speed = clamp_int(args.speed, -30, 30)
    left = args.left_sign * speed
    right = args.right_sign * (-speed if args.turn else speed)
    motor_values = [127, 127, 127, 127]
    motor_values[args.left_port - 1] = left
    motor_values[args.right_port - 1] = right

    print('Keep the robot lifted. Sending motor PWM values:', motor_values)
    print('Before encoders:', bot.get_motor_encoder())
    bot.set_motor(*motor_values)
    time.sleep(args.duration)
    bot.set_motor(*(0 if i + 1 in (args.left_port, args.right_port) else 127 for i in range(4)))
    time.sleep(0.2)
    print('After encoders:', bot.get_motor_encoder())
    print('Done.')


if __name__ == '__main__':
    main()
