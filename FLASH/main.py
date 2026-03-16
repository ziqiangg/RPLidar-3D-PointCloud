from machine import Pin, PWM
import sys
import time
import select

# ===== Servo calibration =====
SERVO_PIN = 15

MIN_US = 450
MAX_US = 2450

POLICIES = {
    "ROBUST_3D": {
        "min_angle": 0,
        "max_angle": 180,
    },
    "PANORAMA": {
        "min_angle": 0,
        "max_angle": 160,
    },
}
active_policy = "ROBUST_3D"

# ===== Servo setup =====
pwm = PWM(Pin(SERVO_PIN))
pwm.freq(50)

# ===== Convert angle to pulse =====
def _clamp_to_policy(angle):
    policy = POLICIES.get(active_policy, POLICIES["ROBUST_3D"])
    min_a = policy["min_angle"]
    max_a = policy["max_angle"]

    if angle < min_a:
        return min_a
    if angle > max_a:
        return max_a
    return angle


def angle_to_us(angle):
    angle = _clamp_to_policy(angle)

    return int(MIN_US + (angle / 180) * (MAX_US - MIN_US))


def set_policy(policy_name):
    global active_policy
    key = str(policy_name).strip().upper()
    if key in POLICIES:
        active_policy = key
        print("POLICY_SET:{}".format(active_policy))
    else:
        print("ERR:POLICY")


# ===== Move servo =====
def move_servo(angle):

    angle = _clamp_to_policy(angle)

    us = angle_to_us(angle)

    duty = int((us / 20000) * 65535)

    pwm.duty_u16(duty)

    print("DONE:{}".format(angle))


# ===== Sweep servo =====
def sweep(start, end, step, settle):

    if step <= 0:
        print("ERR:STEP")
        return

    start = _clamp_to_policy(start)
    end = _clamp_to_policy(end)

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


        # ----- Motion policy -----
        elif cmd.startswith("POLICY:"):

            try:
                policy_name = cmd.split(":", 1)[1]
                set_policy(policy_name)

            except:
                print("ERR:POLICY")


        # ----- Home -----
        elif cmd == "HOME":

            move_servo(0)
            print("HOMED")


        else:

            print("ERR:CMD")
