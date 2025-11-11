# Import required libraries
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# Connection setup
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E705')
logging.basicConfig(level=logging.ERROR)

# Flight path coordinates
z0 = 0.4
x0 = 1.0
x1 = 0.0
x2 = -1.0
y0 = 0.0
y1 = -0.4
y2 = 0.4

# Define the waypoint sequence (x, y, z, time)
newsequence = [
    (x1, y0, z0, 3.0),  # Start (initial hover)
    (x0, y0, z0, 3.0),  # Forward
    (x0, y1, z0, 3.0),  # Right
    (x1, y1, z0, 3.0),  # Back
    (x2, y1, z0, 3.0),  # Back again (further back)
    (x2, y0, z0, 3.0),  # Left
    (x1, y0, z0, 3.0),  # Return to start
]

# Helper functions
def is_close(distance, threshold=0.25):
    """Detect nearby obstacle within threshold (m)."""
    return distance is not None and distance < threshold


def move_with_avoidance(commander, multiranger, target_x, target_y, target_z, duration):
    """Move toward a target position while avoiding obstacles."""
    start_time = time.time()
    print(f" Moving to ({target_x}, {target_y}, {target_z}) for {duration}s")

    while time.time() - start_time < duration:
        # Obstacle avoidance logic
        if is_close(multiranger.front):
            print(" Obstacle ahead — detouring left 0.5 m")
            commander.go_to(target_x, target_y - 0.5, target_z)
            time.sleep(1.0)
        elif is_close(multiranger.right):
            print(" Obstacle on right — sidestepping left 0.5 m")
            commander.go_to(target_x, target_y - 0.5, target_z)
            time.sleep(1.0)
        else:
            commander.go_to(target_x, target_y, target_z)
            time.sleep(0.1)

# Main program
if __name__ == '__main__':
    try:
        cflib.crtp.init_drivers()
        cf = Crazyflie(rw_cache='./cache')

        with SyncCrazyflie(URI, cf=cf) as scf:
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            with PositionHlCommander(scf, default_height=0.4) as commander:
                with Multiranger(scf) as multiranger:
                    print(" Takeoff...")
                    time.sleep(3)

                    # Fail-safe variables
                    start_time = time.time()
                    max_flight_duration = 60

                    # Execute sequence
                    for i, (tx, ty, tz, t) in enumerate(newsequence):
                        print(f"➡️ Step {i+1}/{len(newsequence)}: ({tx}, {ty}, {tz})")
                        if time.time() - start_time > max_flight_duration:
                            print(" Failsafe: Max flight time exceeded — landing.")
                            commander.land(0.0, 2.0)
                            raise SystemExit
                        move_with_avoidance(commander, multiranger, tx, ty, tz, t)
                        time.sleep(0.5)

                    print("Sequence complete — hovering 5s...")
                    time.sleep(5)

                    print(" Landing...")
                    commander.land(0.0, 2.0)
                    time.sleep(3)
                    print("Mission completed successfully ")

    except KeyboardInterrupt:
        print("\n Manual abort — landing immediately.")
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
        print("Drone disarmed safely.")
