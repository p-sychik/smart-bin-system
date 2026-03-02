import select
import sys

try:
    # Kitronik board mode: "port 4" means servo channel 4.
    from PicoRobotics import KitronikPicoRobotics
except ImportError:
    KitronikPicoRobotics = None

if KitronikPicoRobotics is None:
    from machine import Pin, PWM


MODE = "KITRONIK" if KitronikPicoRobotics is not None else "PWM"

# Single-servo setup.
SERVO_PORT = 4      # Kitronik servo channel (1-8)
SERVO_PIN = 4       # Raw PWM pin fallback
UNLOCKED_ANGLE = 0
LOCKED_ANGLE = 90


if MODE == "KITRONIK":
    board = KitronikPicoRobotics()
    servo_pwm = None
else:
    board = None
    servo_pwm = PWM(Pin(SERVO_PIN))
    servo_pwm.freq(50)


def set_angle(angle):
    """Set hook servo angle (0-180)."""
    angle = max(0, min(180, int(angle)))
    if MODE == "KITRONIK":
        board.servoWrite(SERVO_PORT, angle)
        return

    pulse_us = 500 + (angle * 2000 // 180)
    duty = int(pulse_us * 65535 // 20000)
    servo_pwm.duty_u16(duty)


def lock():
    set_angle(LOCKED_ANGLE)
    print("LOCKED")


def unlock():
    set_angle(UNLOCKED_ANGLE)
    print("UNLOCKED")


unlock()
print("READY")
print("MODE=", MODE, "SERVO_PORT=", SERVO_PORT, "SERVO_PIN=", SERVO_PIN)

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
