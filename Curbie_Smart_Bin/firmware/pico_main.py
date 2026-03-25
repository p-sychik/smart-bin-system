"""Optional Pi Pico firmware for the dual-servo hook setup.

Save this as `main.py` on the Pico if you want a clean copy of the current
two-servo Kitronik protocol in the new Curbie_Smart_Bin workspace.
"""

import select
import sys
import time

import PicoRobotics


SERVO_LEFT = 2
SERVO_RIGHT = 7
NEUTRAL = 90


def clamp_degrees(value):
    return max(0, min(180, int(value)))


def set_servos(left_degrees, right_degrees):
    board.servoWrite(SERVO_LEFT, clamp_degrees(left_degrees))
    board.servoWrite(SERVO_RIGHT, clamp_degrees(right_degrees))


def parse_command(line):
    """Parse `S<left>,<right>` commands from the Raspberry Pi."""
    line = (line or '').strip()
    if not line or not line.startswith('S'):
        return None
    try:
        left_text, right_text = line[1:].split(',', 1)
        return int(left_text), int(right_text)
    except ValueError:
        return None


print('Starting Kitronik dual-servo firmware...')
board = PicoRobotics.KitronikPicoRobotics()

print('Startup sweep...')
set_servos(45, 45)
time.sleep(0.5)
set_servos(135, 135)
time.sleep(0.5)
set_servos(NEUTRAL, NEUTRAL)
print('READY')
print(f'Listening on stdin for S<left>,<right> commands (ports {SERVO_LEFT} and {SERVO_RIGHT})')

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

while True:
    if not poll.poll(10):
        continue
    line = sys.stdin.readline()
    if not line:
        continue
    result = parse_command(line)
    if result is None:
        print('IGNORED')
        continue
    set_servos(*result)
