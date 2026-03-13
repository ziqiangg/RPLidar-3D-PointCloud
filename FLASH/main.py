from machine import Pin, PWM
import sys
import time
import select

# ===== Servo calibration =====
SERVO_PIN = 15

MIN_US = 450
MAX_US = 2450

# ===== Servo setup =====
pwm = PWM(Pin(SERVO_PIN))
pwm.freq(50)

# ===== Convert angle to pulse =====
def angle_to_us(angle):
    if angle < 0:
        angle = 0
    if angle > 180:
        angle = 180

    return int(MIN_US + (angle / 180) * (MAX_US - MIN_US))


# ===== Move servo =====
def move_servo(angle):

    us = angle_to_us(angle)

    duty = int((us / 20000) * 65535)

    pwm.duty_u16(duty)

    print("DONE:{}".format(angle))


# ===== Sweep servo =====
def sweep(start, end, step, settle):

    if step <= 0:
        print("ERR:STEP")
        return

    angle = start

    while angle <= end:

        us = angle_to_us(angle)
        duty = int((us / 20000) * 65535)

        pwm.duty_u16(duty)

        print("POS:{}".format(angle))

        time.sleep(settle)

        angle += step

    print("SWEEP_DONE")


# ===== Serial command listener =====
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

print("PICO_SERVO_READY")

while True:

    events = poller.poll(100)

    if events:

        cmd = sys.stdin.readline().strip()

        # ----- Move to single angle -----
        if cmd.startswith("ANGLE:"):

            try:
                angle = float(cmd.split(":")[1])

                move_servo(angle)

            except:
                print("ERR:ANGLE")


        # ----- Sweep command -----
        elif cmd.startswith("SWEEP:"):

            try:

                parts = cmd.split(":")[1].split(",")

                start = int(parts[0])
                end = int(parts[1])
                step = int(parts[2])
                settle = float(parts[3])

                sweep(start, end, step, settle)

            except:
                print("ERR:SWEEP")


        # ----- Home -----
        elif cmd == "HOME":

            move_servo(0)
            print("HOMED")


        else:

            print("ERR:CMD")
