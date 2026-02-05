from machine import Pin, PWM
import select
import sys

# Servo setup: Adjust the pins based on wiring.
servo1 = PWM(Pin(0))
servo2 = PWM(Pin(1))

servo1.freq(50)
servo2.freq(50)

# Angle positions: Changeable, might not be exact, set_angle formula might
# need adjustment, but try it out first.
UNLOCKED_ANGLE = 0
LOCKED_ANGLE = 90


def set_angle(servo, angle):
    """Set servo to angle (0-180 degrees)."""
    pulse_us = 500 + (angle * 2000 // 180)
    duty = int(pulse_us * 65535 // 20000)
    servo.duty_u16(duty)


def lock():
    set_angle(servo1, LOCKED_ANGLE)
    set_angle(servo2, LOCKED_ANGLE)
    print("LOCKED")


def unlock():
    set_angle(servo1, UNLOCKED_ANGLE)
    set_angle(servo2, UNLOCKED_ANGLE)
    print("UNLOCKED")


unlock()
print("READY")

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

while True:
    if poll.poll(100):
        cmd = sys.stdin.readline().strip().upper()

        if cmd == "LOCK":
            lock()
        elif cmd == "UNLOCK":
            unlock()
        elif cmd == "STATUS":
            print("OK")
