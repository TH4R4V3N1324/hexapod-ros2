#!/usr/bin/env python3
"""
Walk the simulated hexapod through outdoor scenarios and score each run.

Each scenario starts its own simulation (headless unless --gui, and never RViz), spawns the robot at
the start of a lane in one of the outdoor worlds, walks or turns it, then shuts the simulation
down, so every run starts from a clean state. Scoring uses Gazebo's true pose (/ground_truth/odom)
against what was commanded and against the EKF's estimate (/odometry/filtered).

    ros2 run hexapod_bringup scenario_runner.py                 # every scenario
    ros2 run hexapod_bringup scenario_runner.py --only hill     # names containing "hill"
    ros2 run hexapod_bringup scenario_runner.py --only flat,rocks --disable terrain_map
    ros2 run hexapod_bringup scenario_runner.py --list
    ros2 run hexapod_bringup scenario_runner.py --only rocks --gui   # watch in the Gazebo window

To watch one scenario instead, launch its world with the GUI and the spawn point --list shows:
    ros2 launch hexapod_bringup hexapod_simulation.launch.py world:=outdoor_slopes spawn_y:=1.5

The worlds come from hexapod_description/scripts/generate_outdoor_worlds.py; keep the lane
positions below in step with it. Don't run it alongside another simulation: the topics would
clash. Exits non-zero if any scenario fails.
"""
import argparse
import csv
import math
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from hexapod_interfaces.srv import GetCapabilities
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

FALL_TILT = math.radians(45)   # body tilt that counts as a fall
STARTUP_TIMEOUT = 120.0        # s of wall time for the simulation to come up
SETTLE_TIME = 4.0              # s of sim time standing still before each run

# Pass marks. Speed is relative to the flat dry baseline, since the gait doesn't quite reach
# the commanded speed even on ideal ground.
FAIL_SPEED, WARN_SPEED = 0.5, 0.8          # fraction of the baseline speed
WARN_ODOM_ERROR = 0.15                     # odometry position error, fraction of distance walked
WARN_DRIFT = 0.1                           # sideways drift, fraction of distance walked
WARN_YAW_ERROR = math.radians(15)          # odometry heading error


@dataclass
class Scenario:
    name: str
    world: str
    description: str
    spawn: tuple            # x, y, z (m)
    distance: float = 0.0   # m to walk forward
    turn: float = 0.0       # rad to turn on the spot (anticlockwise)


SCENARIOS = [
    Scenario('flat_dry', 'outdoor_low_friction', 'Flat dry ground (mu 1.0), the baseline', (-0.3, 0.0, 0.5), distance=2.2),
    Scenario('wet_grass', 'outdoor_low_friction', 'Flat wet grass (mu 0.6)', (-0.3, 1.5, 0.5), distance=2.2),
    Scenario('mud', 'outdoor_low_friction', 'Flat mud (mu 0.4)', (-0.3, 3.0, 0.5), distance=2.2),
    Scenario('wet_tile', 'outdoor_low_friction', 'Flat wet tile (mu 0.25)', (-0.3, 4.5, 0.5), distance=2.2),
    Scenario('wet_tile_turn', 'outdoor_low_friction', 'Turn 180 deg on wet tile (mu 0.25)', (1.0, 4.5, 0.5), turn=math.pi),
    Scenario('mud_hill_10', 'outdoor_low_friction', 'Over a 10 deg hill of mud (mu 0.4)', (0.0, -1.5, 0.5), distance=3.0),
    Scenario('hill_5', 'outdoor_slopes', 'Over a 5 deg hill', (0.0, 0.0, 0.5), distance=3.0),
    Scenario('hill_10', 'outdoor_slopes', 'Over a 10 deg hill', (0.0, 1.5, 0.5), distance=3.0),
    Scenario('hill_15', 'outdoor_slopes', 'Over a 15 deg hill', (0.0, 3.0, 0.5), distance=3.0),
    Scenario('hill_20', 'outdoor_slopes', 'Over a 20 deg hill', (0.0, 4.5, 0.5), distance=3.0),
    Scenario('side_slope_10', 'outdoor_slopes', 'Along a 10 deg side slope', (0.0, -2.0, 0.7), distance=2.0),
    Scenario('side_slope_15', 'outdoor_slopes', 'Along a 15 deg side slope', (0.0, -4.0, 0.7), distance=2.0),
    Scenario('slabs', 'rough_terrain', 'Over low slabs and a step (2-3 cm)', (0.0, 0.0, 0.5), distance=2.0),
    Scenario('rocks_centre', 'outdoor_rocks', 'Through the rock field, centre lane', (0.0, 0.0, 0.5), distance=3.0),
    Scenario('rocks_left', 'outdoor_rocks', 'Through the rock field, left lane', (0.0, 0.8, 0.5), distance=3.0),
    Scenario('rocks_right', 'outdoor_rocks', 'Through the rock field, right lane', (0.0, -0.8, 0.5), distance=3.0),
    Scenario('rocks_turn', 'outdoor_rocks', 'Turn 180 deg among rocks', (2.0, 0.0, 0.5), turn=math.pi),
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def roll_pitch_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def tilt_of(q):
    """Angle between the body's z axis and vertical."""
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y))))


def planar(odom):
    """x, y and yaw of an odometry message."""
    p = odom.pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


def delta_in_start_frame(start, end):
    """Forward, left and turn from start to end, in the start pose's frame."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return dx * c + dy * s, -dx * s + dy * c, math.remainder(end[2] - start[2], 2 * math.pi)


class ScenarioNode(Node):
    """Drives one scenario and records the true and estimated poses."""

    def __init__(self):
        super().__init__('scenario_runner', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.caps_client = self.create_client(GetCapabilities, 'get_capabilities')
        self.truth = None
        self.estimate = None
        self.max_tilt = 0.0
        self.tilt_sum = 0.0
        self.tilt_samples = 0
        self.trace = []
        self.recording = False
        self.create_subscription(Odometry, 'ground_truth/odom', self.truth_callback, 10)
        self.create_subscription(Odometry, 'odometry/filtered', self.estimate_callback, 10)

    def truth_callback(self, msg):
        self.truth = msg
        if self.recording:
            tilt = tilt_of(msg.pose.pose.orientation)
            self.max_tilt = max(self.max_tilt, tilt)
            self.tilt_sum += tilt
            self.tilt_samples += 1
            p = msg.pose.pose.position
            roll, pitch = roll_pitch_of(msg.pose.pose.orientation)
            self.trace.append((self.now(), p.x, p.y, p.z, math.degrees(roll), math.degrees(pitch)))

    def estimate_callback(self, msg):
        self.estimate = msg

    def now(self):
        """Simulation time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def wait_until_ready(self, deadline):
        """Wait for the gait controller, both poses and the clock. False if the deadline passes."""
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.truth and self.estimate and self.now() > 0.0 and self.caps_client.service_is_ready():
                return True
        return False

    def capabilities(self):
        future = self.caps_client.call_async(GetCapabilities.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def drive(self, twist, duration, deadline):
        """Publish a velocity command for a duration of sim time. False if the deadline passes."""
        start = self.now()
        while self.now() - start < duration:
            if time.monotonic() > deadline:
                return False
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        return True


# gait_controller's debug lines for each foot event (see adaptToTerrain)
EVENT_PATTERN = re.compile(r'\[DEBUG\] \[(\d+\.\d+)\] \[hexapod_gait_controller\]: Leg \d (touched down|found ground|stumbled)')
EVENTS = {'touched down': 'touchdowns', 'found ground': 'ground_searches', 'stumbled': 'stumbles'}


def count_foot_events(log_path, start, end):
    """Count early touchdowns, searches for ground below and stumbles logged between two wall times."""
    counts = dict.fromkeys(EVENTS.values(), 0)
    with open(log_path, errors='replace') as f:
        for line in f:
            match = EVENT_PATTERN.search(line)
            if match and start <= float(match.group(1)) <= end:
                counts[EVENTS[match.group(2)]] += 1
    return counts


def launch_simulation(scenario, log_path, gui):
    """Start the scenario's world, in its own process group so it can all be stopped."""
    x, y, z = scenario.spawn
    cmd = ['ros2', 'launch', 'hexapod_bringup', 'hexapod_simulation.launch.py',
           f'world:={scenario.world}', f'gui:={str(gui).lower()}', 'rviz:=false', 'gait_log_level:=debug',
           f'spawn_x:={x}', f'spawn_y:={y}', f'spawn_z:={z}']
    log = open(log_path, 'w')
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True), log


def stop_simulation(process, log):
    """Stop the launch and everything it started."""
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=wait)
            break
        except subprocess.TimeoutExpired:
            continue
    log.close()
    time.sleep(2.0)


def run_scenario(scenario, speed_fraction, log_dir, gui, disable):
    """Run one scenario in a fresh simulation. Returns a dict of results."""
    result = {'name': scenario.name, 'description': scenario.description}
    process, log = launch_simulation(scenario, log_dir / f'{scenario.name}.log', gui)
    node = ScenarioNode()
    try:
        deadline = time.monotonic() + STARTUP_TIMEOUT
        if not node.wait_until_ready(deadline):
            result['error'] = 'simulation did not come up'
            return result
        caps = node.capabilities()
        if caps is None:
            result['error'] = 'no reply from get_capabilities'
            return result
        for feature in disable:
            client = node.create_client(SetBool, f'set_{feature}')
            future = client.call_async(SetBool.Request(data=False)) if client.wait_for_service(timeout_sec=10.0) else None
            if future:
                rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            if not future or not future.result() or not future.result().success:
                result['error'] = f'could not turn off {feature}'
                return result

        twist = Twist()
        if scenario.turn:
            rate = speed_fraction * caps.max_angular_vel
            twist.angular.z = math.copysign(rate, scenario.turn)
            duration = abs(scenario.turn) / rate
        else:
            speed = speed_fraction * caps.max_linear_vel
            twist.linear.x = speed
            duration = scenario.distance / speed

        # Generous wall-clock limit, in case the sim runs much slower than real time
        deadline = time.monotonic() + 10 * (duration + SETTLE_TIME) + 60
        if not node.drive(Twist(), SETTLE_TIME, deadline):
            result['error'] = 'timed out settling'
            return result
        truth_start, estimate_start = planar(node.truth), planar(node.estimate)
        z_start = node.truth.pose.pose.position.z
        node.max_tilt = tilt_of(node.truth.pose.pose.orientation)
        node.recording = True
        result['wall_start'] = time.time()
        t0 = node.now()
        if not node.drive(twist, duration, deadline):
            result['error'] = 'timed out walking'
            return result
        elapsed = node.now() - t0
        result['wall_end'] = time.time()
        node.drive(Twist(), 1.0, deadline)
        node.recording = False
        # Body pose through the run, to see tilt section by section (e.g. up a ramp, across the top)
        with open(log_dir / f'{scenario.name}_trace.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'x', 'y', 'z', 'roll_deg', 'pitch_deg'])
            writer.writerows([f'{v:.4f}' for v in row] for row in node.trace)

        forward, left, turned = delta_in_start_frame(truth_start, planar(node.truth))
        est_forward, est_left, est_turned = delta_in_start_frame(estimate_start, planar(node.estimate))
        distance = math.hypot(forward, left)
        result.update({
            'commanded': scenario.distance if not scenario.turn else math.degrees(scenario.turn),
            'forward_m': forward,
            'drift_m': left,
            'turned_deg': math.degrees(turned),
            'climb_m': node.truth.pose.pose.position.z - z_start,
            'speed': forward / elapsed if not scenario.turn else 0.0,
            'turn_rate': turned / elapsed if scenario.turn else 0.0,
            'commanded_rate': twist.angular.z,
            'odom_error_m': math.hypot(est_forward - forward, est_left - left),
            'odom_error_frac': math.hypot(est_forward - forward, est_left - left) / max(distance, 0.1),
            'yaw_error_deg': math.degrees(math.remainder(est_turned - turned, 2 * math.pi)),
            'max_tilt_deg': math.degrees(node.max_tilt),
            'mean_tilt_deg': math.degrees(node.tilt_sum / max(node.tilt_samples, 1)),
            'fell': node.max_tilt > FALL_TILT,
        })
        return result
    finally:
        node.destroy_node()
        stop_simulation(process, log)


def grade(result, baseline_speed):
    """PASS, WARN or FAIL, and the reasons for anything less than PASS."""
    if 'error' in result:
        return 'FAIL', [result['error']]
    if result['fell']:
        return 'FAIL', [f'fell (tilted {result["max_tilt_deg"]:.0f} deg)']
    status, reasons = 'PASS', []

    if result['turn_rate']:
        achieved = result['turn_rate'] / result['commanded_rate']
        label = 'turn rate'
    else:
        achieved = result['speed'] / baseline_speed if baseline_speed else 1.0
        label = 'speed'
    if achieved < FAIL_SPEED:
        status = 'FAIL'
        reasons.append(f'{label} {achieved:.0%} of {"commanded" if result["turn_rate"] else "baseline"}')
    elif achieved < WARN_SPEED:
        status = 'WARN'
        reasons.append(f'{label} {achieved:.0%} of {"commanded" if result["turn_rate"] else "baseline"}')

    if not result['turn_rate'] and abs(result['drift_m']) > WARN_DRIFT * max(abs(result['forward_m']), 0.1):
        status = 'WARN' if status == 'PASS' else status
        reasons.append(f'drifted {result["drift_m"]:+.2f} m sideways')
    if not result['turn_rate'] and result['odom_error_frac'] > WARN_ODOM_ERROR:
        status = 'WARN' if status == 'PASS' else status
        reasons.append(f'odometry off by {result["odom_error_frac"]:.0%} of distance')
    if abs(math.radians(result['yaw_error_deg'])) > WARN_YAW_ERROR:
        status = 'WARN' if status == 'PASS' else status
        reasons.append(f'odometry heading off by {result["yaw_error_deg"]:+.0f} deg')
    return status, reasons


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--only', default='', help='run scenarios whose name contains any of these comma-separated texts')
    parser.add_argument('--disable', action='append', default=[], metavar='FEATURE',
                        help='turn off a gait_controller feature for every run (e.g. terrain_map); repeatable')
    parser.add_argument('--list', action='store_true', help='list the scenarios and exit')
    parser.add_argument('--speed', type=float, default=0.6, help='fraction of top speed to walk and turn at (default 0.6)')
    parser.add_argument('--gui', action='store_true', help='show the Gazebo window for each scenario')
    parser.add_argument('--log-dir', default='/tmp/hexapod_scenarios', help='where to write logs and results.csv')
    args = parser.parse_args()

    if args.list:
        for s in SCENARIOS:
            x, y, z = s.spawn
            print(f'{s.name:<15} {s.world:<22} spawn_x:={x} spawn_y:={y} spawn_z:={z}  {s.description}')
        return

    patterns = args.only.split(',')
    scenarios = [s for s in SCENARIOS if any(p in s.name for p in patterns)]
    # The flat baseline sets the reference speed, so run it first whenever any walking scenario runs
    baseline = next(s for s in SCENARIOS if s.name == 'flat_dry')
    if any(not s.turn for s in scenarios) and baseline not in scenarios:
        scenarios.insert(0, baseline)
    if not scenarios:
        print(f'No scenario name contains "{args.only}"')
        sys.exit(1)

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    if args.disable:
        print(f'gait_controller features off: {", ".join(args.disable)}')
    rclpy.init()
    results = []
    baseline_speed = None
    for i, scenario in enumerate(scenarios, 1):
        print(f'[{i}/{len(scenarios)}] {scenario.name}: {scenario.description} ...', flush=True)
        started = time.monotonic()
        result = run_scenario(scenario, args.speed, log_dir, args.gui, args.disable)
        if 'wall_end' in result:
            result.update(count_foot_events(log_dir / f'{scenario.name}.log', result['wall_start'], result['wall_end']))
        if scenario.name == 'flat_dry' and 'error' not in result:
            baseline_speed = result['speed']
        status, reasons = grade(result, baseline_speed)
        result['status'] = status
        result['notes'] = '; '.join(reasons)
        results.append(result)
        took = time.monotonic() - started
        if 'error' in result:
            print(f'      {status}  {result["error"]} ({took:.0f} s, log: {log_dir / scenario.name}.log)', flush=True)
            continue
        if scenario.turn:
            print(f'      {status}  turned {result["turned_deg"]:+.0f} deg, drifted {math.hypot(result["forward_m"], result["drift_m"]):.2f} m, '
                  f'odom heading error {result["yaw_error_deg"]:+.1f} deg, tilt {result["mean_tilt_deg"]:.1f} deg mean, '
                  f'{result["max_tilt_deg"]:.1f} max ({took:.0f} s){"  " + result["notes"] if result["notes"] else ""}', flush=True)
        else:
            print(f'      {status}  forward {result["forward_m"]:.2f} of {scenario.distance:.1f} m, drift {result["drift_m"]:+.2f} m, '
                  f'climbed {result["climb_m"]:+.2f} m, odom error {result["odom_error_m"]:.2f} m, '
                  f'tilt {result["mean_tilt_deg"]:.1f} deg mean, {result["max_tilt_deg"]:.1f} max ({took:.0f} s){"  " + result["notes"] if result["notes"] else ""}',
                  flush=True)
        print(f'            feet: {result["touchdowns"]} early touchdowns, {result["ground_searches"]} ground searches, '
              f'{result["stumbles"]} stumbles', flush=True)
    rclpy.shutdown()

    fields = ['name', 'status', 'notes', 'description', 'commanded', 'forward_m', 'drift_m', 'turned_deg', 'climb_m',
              'speed', 'turn_rate', 'odom_error_m', 'odom_error_frac', 'yaw_error_deg', 'mean_tilt_deg', 'max_tilt_deg', 'fell',
              'touchdowns', 'ground_searches', 'stumbles', 'error']
    with open(log_dir / 'results.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)

    counts = {s: sum(r['status'] == s for r in results) for s in ('PASS', 'WARN', 'FAIL')}
    print(f'\n{counts["PASS"]} passed, {counts["WARN"]} warnings, {counts["FAIL"]} failed. '
          f'Results in {log_dir / "results.csv"}')
    sys.exit(1 if counts['FAIL'] else 0)


if __name__ == '__main__':
    main()
