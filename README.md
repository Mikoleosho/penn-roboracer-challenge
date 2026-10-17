# Penn RoboRacer Technical Challenge — LiDAR Wall Follower

A ROS 2 node that drives the AutoDRIVE RoboRacer practice track autonomously, using only the LiDAR scan.

- **Video (Driver's Eye, unlisted):** https://youtu.be/KRAcqvSctgA
- **Fastest clean lap:** 9.496 s (default parameters, no wall contact)

## What's here

`autodrive_devkit/` is the AutoDRIVE ROS 2 package (`autodrive_roboracer`) with one new node:

- [`autodrive_roboracer/wall_follow_node.py`](autodrive_devkit/autodrive_roboracer/wall_follow_node.py): the wall follower
- `setup.py`: registers `wall_follow_node = autodrive_roboracer.wall_follow_node:main`

The bridge (`autodrive_bridge.py`) is unchanged. `autodrive_devkit.Dockerfile` and `autodrive_devkit.sh` are copied from the AutoDRIVE repo, so the challenge's `docker build` / `docker run` commands work from this repo's root.

## How it works

The node subscribes to `/autodrive/roboracer_1/lidar` (1,080 beams over 270°) and publishes `throttle_command` and `steering_command` in [-1, 1]. It runs at the 40 Hz scan rate.

1. **Wall centring (PD).** For each side, two beams (90° and 45° off the nose) give the car's angle to the wall, α, and its distance, `b·cos α`. That distance is projected 1 m ahead (`+ L·sin α`), so the car corrects before it drifts. The error is half the difference between the left and right projected distances, and a PD controller turns it into a steering angle. Side distances are clipped at 2.5 m, so an opening on one side doesn't pull the car into it.
2. **Corner entry (follow the gap).** When the free distance straight ahead drops below 4 m, steering blends toward the heading of the deepest gap in the scan. Before the gap is chosen, every sharp jump in range is padded by the car's half width (a disparity extender), so the chosen heading leaves room for the body. This turns the car in early instead of reacting late to the outside wall.
3. **Speed.** The target speed scales from `v_min` to `v_max` with the free distance ahead and drops with steering lock. A P loop on odometry speed tracks it, and negative throttle brakes into corners.

The node also logs lap times, best lap, and any wall contact (from the bridge's `collision_count` topic), so you can watch the console for clean laps.

## Run

Follow the challenge setup (Docker, `colcon build`, bridge). Then, in a second terminal:

```bash
docker exec -it penn-challenge bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 run autodrive_roboracer wall_follow_node
```

Connect the simulator to `127.0.0.1:4567` and switch Driving Mode to **Autonomous**.

### Tuning

Every gain is a ROS parameter. Pass floats, for example `8.0`, not `8`:

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args -p v_max:=9.0 -p corner_dist:=5.0
# or live, while it is driving:
ros2 param set /wall_follow_node v_max 9.0
```

| Parameter | Default | Effect |
|---|---|---|
| `v_max` / `v_min` | 7.0 / 2.0 m/s | Straight-line and tight-corner target speeds |
| `full_speed_dist` | 8.0 m | Free distance ahead needed for `v_max` |
| `kp_wall` / `kd_wall` | 1.0 / 0.08 | Centring PD gains |
| `lookahead` | 1.0 m | Wall-distance projection |
| `corner_dist` | 4.0 m | When to start blending toward the gap |
| `car_half_width` | 0.25 m | Padding at obstacle edges |
| `steer_slowdown` | 0.5 | Fraction of speed removed at full lock |

To lower lap time, raise `v_max` and `full_speed_dist` step by step until a lap shows wall contact, then back off.
