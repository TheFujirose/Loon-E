# TODO

## Replace the phone GPS source

GPS used to come from a phone, through the `phone` node (`src/loone/loone/phone.py`).
That node was removed in commit `c91a52a` along with the phone itself, so right now
**nothing on the boat publishes latitude or longitude**. The sensor that replaces it
isn't decided; `src/loone/loone/hardware/gps.py` is a shell for it.

Speed and heading now come from the ZED's odometry instead. That heading is relative
to where the camera started, not to true north, until something anchors it.

| Consumer | Needs | Today |
|----------|-------|-------|
| `navsat_transform` | `sensor_msgs/NavSatFix` on `/navsatfix`, `header.frame_id = 'gps_link'` (remapped from `gps/fix` in `launch/launch.py`) | No fixes, so GPS waypoints can't be converted to map coordinates (`/fromLL`, `/toLL`) |
| Basestation | Latitude / longitude, read from index 0 and 1 of `/phone` (`[lat, long, speed, heading]`) | Stay at 0 on the website |

What's needed:

- [ ] Pick the GPS hardware and implement `loone/hardware/gps.py` (if it can't share the
      bus through `HardwareController`, write a node or use an existing ROS 2 driver instead).
- [ ] Publish `NavSatFix` on `/navsatfix`, stamped with the `gps_link` frame. If the
      driver publishes on another topic, update the `gps/fix` remapping in `launch/launch.py`.
- [ ] Measure the antenna position and set the `gps_link` joint in the URDF
      (`src/loone_urdf`), so `navsat_transform` removes the right offset.
- [ ] Fill in a real `position_covariance` if the receiver reports one (the phone sent
      `COVARIANCE_TYPE_UNKNOWN`).
- [ ] Point the basestation's latitude / longitude at the new source.
- [ ] Decide whether a true heading should replace the ZED's relative one in
      `hardware_node.odom_callback` and on the website's compass.
- [ ] If it's a new node, add it to `launch/launch.py` (with
      `condition=UnlessCondition(sim)` like `hardware_node`) and its parameters to
      `config/config.yaml`.
- [ ] Add a test for it in `src/loone/test/` (the old `test_phone.py` was removed with
      the phone node).

## Write the task logic

There are no tasks right now. `src/loone/loone/task.py` is a shell for them, and until
it exists `terminal_node` relays `motor_command` and `ball_launcher_command` typed by hand.

## Connect nav2 to the propellers

nav2's `controller_server` publishes velocity commands (`geometry_msgs/Twist`) on
`/cmd_vel`. They used to go through `thrust_mixer` and the `ros2_control` chain to
the motors, but those were removed (commits `492ab75`, `83f3d7a`). `hardware_node`
now drives the propellers directly from `motor_command`, so **nothing reads
`/cmd_vel`**, and no launch file loads `config/nav2_params.yaml`.

- [ ] Decide whether nav2 is still the plan for navigation.
- [ ] If it is: have `hardware_node` (or a small new node) turn `/cmd_vel` into
      propeller throttles, and start nav2 with `config/nav2_params.yaml` from a launch file.
- [ ] If it isn't: remove `config/nav2_params.yaml` and the nav2 dependencies in
      `package.xml`.
