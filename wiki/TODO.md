# TODO

## Replace the phone GPS source

GPS used to come from a phone, through the `phone` node (`src/loone/loone/phone.py`).
That node was removed in commit `c91a52a`, so right now **nothing on the boat
publishes position or heading**. The new GPS source has to fill the two topics the
phone node used to publish:

| Topic | Type | Contents | Used by |
|-------|------|----------|---------|
| `/navsatfix` | `sensor_msgs/NavSatFix` | Latitude / longitude fix, `header.frame_id = 'gps_link'` | `navsat_transform` (remapped from `gps/fix` in `launch/launch.py`) |
| `/phone` | `std_msgs/Float32MultiArray` | `[lat, long, speed, heading]` | `hardware_node`, which reads the heading (index 3) for drive command 3 |

Until this is done:

- `navsat_transform` gets no fixes, so GPS waypoints can't be converted to map
  coordinates (`/fromLL`, `/toLL`).
- `hardware_node` has no heading, so drive command 3 always stops the propellers
  instead of driving.

What's needed:

- [ ] Pick the GPS hardware and write a node for it (or use an existing ROS 2 driver).
- [ ] Publish `NavSatFix` on `/navsatfix`, stamped with the `gps_link` frame. If the
      driver publishes on another topic, update the `gps/fix` remapping in `launch/launch.py`.
- [ ] Measure the antenna position and set the `gps_link` joint in the URDF
      (`src/loone_urdf`), so `navsat_transform` removes the right offset.
- [ ] Fill in a real `position_covariance` if the receiver reports one (the phone sent
      `COVARIANCE_TYPE_UNKNOWN`).
- [ ] Give `hardware_node` a heading again: either publish the same
      `[lat, long, speed, heading]` array on `/phone`, or change `phone_callback` in
      `loone/hardware_node.py` to read from the new source (and rename the topic,
      since it's no longer a phone).
- [ ] Add the node to `launch/launch.py` (with `condition=UnlessCondition(sim)` like
      `hardware_node`) and its parameters to `config/config.yaml`.
- [ ] Add a test for the node in `src/loone/test/` (the old `test_phone.py` was removed
      with the phone node).

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
