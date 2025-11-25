# Crazyflie Obstacle Avoidance – Slow + Medium Detection Range
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# Connection setup
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
logging.basicConfig(level=logging.ERROR)

# Waypoints
z0 = 0.4
x0 = 1.0
x1 = 0.0
x2 = -1.0
y0 = 0.0
y1 = -0.4

newsequence = [
    (x1, y0, z0, 3.0),
    (x0, y0, z0, 3.0),
    (x0, y1, z0, 3.0),
    (x1, y1, z0, 3.0),
    (x2, y1, z0, 3.0),
    (x2, y0, z0, 3.0),
    (x1, y0, z0, 3.0),
]

# ------------------------------
# Helper functions
# ------------------------------

def is_close(distance, threshold=0.45):   # ← REDUCED detection range
    return distance is not None and distance < threshold


def get_pos(commander):
    try:
        x = float(getattr(commander, "_x", 0.0))
        y = float(getattr(commander, "_y", 0.0))
        z = float(getattr(commander, "_z", z0))
        return x, y, z
    except:
        return 0.0, 0.0, z0


def move_towards(commander, tx, ty, tz):
    cx, cy, cz = get_pos(commander)

    step = 0.05   # slow speed

    if abs(tx - cx) > step:
        cx += step if tx > cx else -step
    else:
        cx = tx

    if abs(ty - cy) > step:
        cy += step if ty > cy else -step
    else:
        cy = ty

    commander.go_to(cx, cy, tz)
    time.sleep(0.05)


def move_with_avoidance(commander, multiranger, tx, ty, tz, duration):
    start = time.time()
    print(f">>> Moving to ({tx}, {ty}, {tz}) for {duration}s")

    while time.time() - start < duration:

        front = multiranger.front
        right = multiranger.right

        # ---- FRONT OBSTACLE: new maneuver ----
        if is_close(front):
            print("Obstacle detected in FRONT → executing NEW bypass")
            cx, cy, cz = get_pos(commander)

            sidestep_right = 0.5
            forward = 0.8
            sidestep_left = 0.8

            # 1) Move RIGHT 0.5
            commander.go_to(cx, cy + sidestep_right, cz)
            time.sleep(1.0)

            # 2) Move FORWARD 0.8
            commander.go_to(cx + forward, cy + sidestep_right, cz)
            time.sleep(1.0)

            # 3) Move LEFT 0.8
            commander.go_to(cx + forward, cy + sidestep_right - sidestep_left, cz)
            time.sleep(1.0)

            print("Bypass complete")
            return

        # ---- RIGHT OBSTACLE SMALL SIDESTEP ----
        if is_close(right):
            print("Obstacle on RIGHT → shift LEFT 0.5m")
            cx, cy, cz = get_pos(commander)
            commander.go_to(cx, cy - 0.5, cz)
            time.sleep(1.0)
            return

        # ---- No obstacle: move slowly toward waypoint ----
        move_towards(commander, tx, ty, tz)

    return


# ------------------------------
# MAIN PROGRAM
# ------------------------------

if __name__ == "__main__":
    try:
        cflib.crtp.init_drivers()
        cf = Crazyflie(rw_cache="./cache")

        with SyncCrazyflie(URI, cf=cf) as scf:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            with PositionHlCommander(
                scf,
                default_height=z0,
                controller=PositionHlCommander.CONTROLLER_PID
            ) as commander:

                with Multiranger(scf) as multiranger:
                    print("Takeoff...")
                    time.sleep(3)

                    for i, (tx, ty, tz, t) in enumerate(newsequence):
                        print(f"Waypoint {i+1}/{len(newsequence)}: ({tx}, {ty}, {tz})")
                        move_with_avoidance(commander, multiranger, tx, ty, tz, t)
                        time.sleep(0.3)

                    print("Sequence complete — hovering...")
                    time.sleep(5)

                    print("Landing...")
                    commander.land(0.0, 2.0)
                    time.sleep(3)

    except Exception as e:
        print("Unexpected error:", e)
        try:
            commander.land(0.0, 2.0)
        except:
            pass
        time.sleep(2)
        print("Drone disarmed safely.")
