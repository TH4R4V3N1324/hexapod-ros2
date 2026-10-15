# Hardware tests

Small programs to check each piece of the robot's hardware on its own, without ROS
running. Each one prints what it finds, then a list of `PASS` / `WARN` / `FAIL`
checks with hints on what to look at, and exits non-zero if anything failed.

| Test | Hardware | Runs in |
|---|---|---|
| `test_servos` | ST3215 bus servos | container |
| `test_power` | INA226 power monitors (left 0x40, right 0x41) | container |
| `test_imu` | ISM330DHCX IMU (0x6A) | container |
| `test_lidar` | RPLIDAR A2M12 | container (or the Pi, with `python3-serial`) |
| `test_camera.sh` | Raspberry Pi Camera Module 3 | the Pi |
| `test_tof.py` | Arducam ToF camera (B0410) | the Pi |

Every test takes `--help`.

## Running them

The servo, power, IMU and lidar tests are built into the robot's image. Open a
shell in the running container on the Pi:

```bash
ssh raven@hexapod
docker exec -it hexapod_ros2 bash
ros2 run hexapod_hardware_interface test_servos scan
```

Only one program can use the servo bus or the lidar at a time. The container
runs `hexapod_fake.launch.py` (mock hardware) by default, so they're free. If
it runs `hexapod_robot.launch.py`, stop it and use a one-off container instead:

```bash
cd ~/hexapod-ros2/hexapod_ws
docker compose stop
docker compose run --rm hexapod bash
```

The I2C tests can run alongside the robot, though `test_imu` resets the IMU,
so the IMU node logs a read failure and reinitialises.

The camera tests run on the Pi itself, outside the container, because the
Pi 5's cameras need Raspberry Pi OS's own libcamera:

```bash
~/hexapod-ros2/hexapod_ws/src/hexapod_hardware_interface/hardware_tests/test_camera.sh
```

## Suggested order for a first bring-up

1. **Power:** `test_power`. Both monitors found, battery voltage sensible,
   current positive (if it reads negative, the shunt is wired backwards).
2. **Servo IDs:** new servos all come as ID 1. Connect them **one at a time**,
   then `test_servos scan --all` to find it and `test_servos set-id 1 <leg*10+joint>`
   to give it its ID (e.g. 23 for leg 2's tibia).
3. **Servos:** `test_servos scan` should list all 18 with sensible voltage and
   temperature.
4. **Directions and offsets:** `test_servos wiggle <id>` on each servo. It moves
   +10°, back, -10°, back, so you can see which joint it drives and which way
   positive is. Joints that turn the wrong way for the URDF need `direction` -1
   in `hexapod.ros2_control.xacro`. `test_servos monitor` with torque off
   (`test_servos limp`) shows each joint's angle as you move it by hand, which
   helps find the `offset` for each joint.
5. **IMU:** `test_imu` with the robot still and level checks gravity reads on
   +Z, as `imu_link` expects. While it streams, raise the front (X goes
   positive), raise the left (Y goes positive) and turn it anticlockwise
   (gyro Z goes positive).
6. **Lidar:** `test_lidar` reads the lidar's info and health, scans for 5 s and
   prints the nearest object per 45° sector. Hold a hand close to it to see
   which sector it lands in.
7. **Cameras:** `test_camera.sh` and `test_tof.py`, below.

## Servos

```
test_servos scan [--all]           ping the 18 leg servos (or every ID) and show their state
test_servos monitor [id ...]       live position, load, voltage, temperature
test_servos move <id> <degrees>    move to an angle from centre (0 = 2048 steps) and hold
test_servos wiggle <id> [degrees]  +deg, back, -deg, back, then hold
test_servos limp [id ...]          torque off (default: every servo). The legs will drop!
test_servos set-id <old> <new>     change a servo's ID
```

Options: `--port /dev/ttyAMA0`, `--baud 1000000`, `--echo` (if the bus adapter
loops TX back to RX), `--speed <°/s>` for `move` and `wiggle` (default 30).

`move` and `wiggle` leave torque on, so the joint holds its position. Ctrl+C
stops the sequence but the servo still holds; run `limp` to release it. Support
the body before running `limp` on a standing robot.

The status column decodes the servo's error bits: voltage, sensor, overheat,
overcurrent, angle, overload.

## Cameras

**Camera Module 3:** `test_camera.sh` finds the camera, takes an autofocused
photo (`~/camera_test.jpg`, copy it off with `scp raven@hexapod:camera_test.jpg .`)
and times 5 s of 720p video. It needs `rpicam-apps` (`sudo apt install rpicam-apps-lite`).

**Arducam ToF:** install Arducam's SDK and camera overlay first, following
[Arducam_tof_camera](https://github.com/ArduCAM/Arducam_tof_camera)
(`Install_dependencies.sh`, then `pip install ArducamDepthCamera`), and reboot.
Then:

```bash
test_tof.py --seconds 10 --save depth.pgm
```

It streams the median distance at the centre of the image (point it at a wall
a measured distance away), the share of pixels with a confident depth and the
frame rate. `--index` picks the camera if both are on the overlay, `--range
2000` selects near mode.

`test_tof.py` was written against the SDK's current Python API
(`ArducamCamera`, `Connection.CSI`, `FrameType.DEPTH`, `Control.RANGE`) and
hasn't been run on the real camera. Older SDK releases named these
differently.
