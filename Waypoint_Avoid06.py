# Fixed full script: hard-coded bypass (A+1)
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# Connection setup
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E707')
logging.basicConfig(level=logging.ERROR)

# Waypoints (your newsequence)
z0 = 0.4
x0 = 1.0
x1 = 0.0
x2 = -1.0
y0 = 0.0
y1 = -0.4

newsequence = [
    (x1, y0, z0, 3.0),  # Start (initial hover)
    (x0, y0, z0, 3.0),  # Forward
    (x0, y1, z0, 3.0),  # Right
    (x1, y1, z0, 3.0),  # Back
    (x2, y1, z0, 3.0),  # Back again (further back)
    (x2, y0, z0, 3.0),  # Left
    (x1, y0, z0, 3.0),  # Return to start
]

# Helper: detect nearby obstacle
def is_close(distance, threshold=0.25):
    """Detect nearby obstacle within threshold (m)."""
    return distance is not None and distance < threshold

def get_commander_position(commander):
    """
    Safely read current commander position. PositionHlCommander stores recent
    position in _x, _y, _z (private) — use with fallback to (0,0,default_z).
    """
    try:
        cx = float(getattr(commander, "_x", 0.0))
        cy = float(getattr(commander, "_y", 0.0))
        cz = float(getattr(commander, "_z", 0.4))
    except Exception:
        cx, cy, cz = 0.0, 0.0, 0.4
    return cx, cy, cz

def move_with_avoidance(commander, multiranger, target_x, target_y, target_z, duration):
    """
    Move to target waypoint with a hard-coded bypass when obstacle is detected in front.
    Steps (when obstacle seen):
      1) sidestep left 0.5 m (relative to current position)
      2) move forward 0.5 m
      3) move right 0.5 m (rejoin original Y line)
    After bypass the function returns immediately so main sequence continues.
    """
    start_time = time.time()
    print(f">>> Moving to ({target_x}, {target_y}, {target_z}) for {duration}s")

    while time.time() - start_time < duration:
        # If obstacle directly in front: execute hard-coded bypass (relative to current pos)
        if is_close(multiranger.front):
            print("Obstacle ahead — executing hard-coded bypass")

            # read current position
            cx, cy, cz = get_commander_position(commander)

            # 1) Sidestep LEFT by 0.5 m
            sidestep_y = cy - 0.5
            print(f" Sidestep to ({cx:.2f}, {sidestep_y:.2f}, {cz:.2f})")
            commander.go_to(cx, sidestep_y, cz)
            time.sleep(1.0)

            # 2) Move FORWARD by 0.5 m (in X)
            forward_x = cx + 0.5
            print(f" Move forward to ({forward_x:.2f}, {sidestep_y:.2f}, {cz:.2f})")
            commander.go_to(forward_x, sidestep_y, cz)
            time.sleep(1.0)

            # 3) Move RIGHT by 0.5 m to rejoin original Y-line
            rejoin_y = sidestep_y + 0.5  # should equal original cy
            print(f" Rejoin path at ({forward_x:.2f}, {rejoin_y:.2f}, {cz:.2f})")
            commander.go_to(forward_x, rejoin_y, cz)
            time.sleep(1.0)

            print("Bypass complete — continuing to next waypoint\n")
            return  # important: stop this waypoint's loop and continue main sequence

        # If obstacle on right, do a simple left sidestep to create clearance then continue
        if is_close(multiranger.right):
            cx, cy, cz = get_commander_position(commander)
            sidestep_y = cy - 0.5
            print(f" Obstacle right — sidestep left to ({cx:.2f}, {sidestep_y:.2f}, {cz:.2f})")
            commander.go_to(cx, sidestep_y, cz)
            time.sleep(1.0)
            return

        # No obstacle: go to target waypoint
        commander.go_to(target_x, target_y, target_z)
        time.sleep(0.1)

    # duration elapsed (arrived or timed out in this waypoint)
    return

# Main program with failsafe
if __name__ == '__main__':
    try:
        cflib.crtp.init_drivers()
        cf = Crazyflie(rw_cache='./cache')

        with SyncCrazyflie(URI, cf=cf) as scf:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            with PositionHlCommander(scf, default_height=z0) as commander:
                with Multiranger(scf) as multiranger:

                    print("Takeoff...")
                    time.sleep(3)

                    # FAILSAFE VARIABLES
                    start_time = time.time()
                    max_flight_duration = 120  # total mission timeout (seconds)
                    min_safe_altitude = 0.15

                    # Execute sequence
                    for i, (tx, ty, tz, t) in enumerate(newsequence):
                        print(f" Waypoint {i+1}/{len(newsequence)}: ({tx}, {ty}, {tz})")

                        # Failsafe: overall mission timeout
                        if time.time() - start_time > max_flight_duration:
                            print(" Failsafe: Maximum flight time exceeded. Landing.")
                            commander.land(0.0, 2.0)
                            raise SystemExit

                        # Failsafe: radio link lost
                        if not scf.cf.link:
                            print(" Failsafe: Lost radio link! Attempting to land.")
                            commander.land(0.0, 2.0)
                            time.sleep(3)
                            raise SystemExit

                        # Failsafe: altitude too low/high via multiranger.up reading (if available)
                        if getattr(multiranger, "up", None) is not None:
                            if multiranger.up and multiranger.up < min_safe_altitude:
                                print("Failsafe: Altitude too low — landing.")
                                commander.land(0.0, 2.0)
                                raise SystemExit

                        # Call move_with_avoidance (pass duration)
                        move_with_avoidance(commander, multiranger, tx, ty, tz, t)
                        # small pause before next waypoint
                        time.sleep(0.5)

                    print("Sequence complete — hovering 5s...")
                    time.sleep(5)

                    print("Landing...")
                    commander.land(0.0, 2.0)
                    time.sleep(3)
                    print("Mission completed successfully ")

    except KeyboardInterrupt:
        print("\n Manual abort (Ctrl+C) — landing immediately.")
        try:
            commander.land(0.0, 2.0)
        except Exception:
            pass
        time.sleep(3)

    except Exception as e:
        print(f" Unexpected error: {e}")
        try:
            commander.land(0.0, 2.0)
        except Exception:
            pass
        time.sleep(3)
        print("Drone disarmed and safe.")
