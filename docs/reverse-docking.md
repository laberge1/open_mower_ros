# Reverse docking

Reverse docking supports mowers with rear charging contacts using GPS straight docking. The default is forward docking.
With reverse docking enabled, the mower navigates forward to the approach start, turns to face away from the station,
then backs along the straight approach and into the charger. It drives forward out of the station to undock or retry
a failed docking attempt. Leave room for the turn at the approach start.

## Configuration

For OpenMower OS v2, set the following in your persistent mower parameter YAML (or `custom_params.yaml` for a custom
robot), alongside your existing `mower_logic` parameters:

```yaml
mower_logic:
  docking_reverse: true
  perimeter_signal: 0
```

For legacy environment-based configuration:

```bash
export OM_DOCKING_REVERSE=True
export OM_PERIMETER_SIGNAL=0
```

Restart the ROS stack after changing the persistent configuration. Direction is latched when `mower_logic` starts;
changes through dynamic reconfigure are rejected and the startup value is restored. Restart the simulator too when
testing a new direction, so its physical dock heading matches the mower logic.

Reverse docking with a nonzero `perimeter_signal` is unsupported. The mower logs an error and refuses docking and
undocking motion, including perimeter undocking selected from idle. Set the signal to zero to use reverse docking.

## Dock recording and distances

Record the first docking position outside the station and the second closer to the station. Their connecting line
defines travel **toward** the station, regardless of the mower's facing direction when recording. Reverse mode rotates
the mower's body heading by 180 degrees; it does not change the recorded line or map format. An existing correctly
recorded line can be reused if the station and contact positions still match the mower.

- `docking_approach_distance` places the approach start that far outside the second recorded point.
- `docking_distance` limits travel toward the station beyond the second recorded point. Contact voltage stops insertion
  using the existing `docking_extra_time` setting.
- `undock_distance` and `undock_angled_distance` remain positive distances. Reverse mode changes the drive direction.
- `undock_angle`, its randomization, and `undock_use_curve` preserve the same exit side and world-space path in both modes.

Nonzero straight paths include their configured endpoint and have at least three poses for the FTC controller. This
also corrects the previous insertion path's endpoint, which could end approximately 0.1 m short. Zero-length segments
are skipped. If both undocking distances are zero, undocking fails and the mower stays idle; it does not report success.

## Automated tests

In a ROS Noetic development environment, initialize submodules and build normally, then run:

```bash
catkin_make
catkin_make run_tests_mower_logic
catkin_test_results
```

`test_docking_geometry` covers both docking directions, angle wraparound, approach and insertion endpoints, short and
zero-length paths, retries starting from the current pose, straight and curved exits, perimeter configuration, and the
startup direction lock. `test_docking_ros` checks pose conversion and preservation of path frames, timestamps, and height.

The geometry test uses only standard C++ and GoogleTest. It can also run without ROS using the `mower_logic/include`
and `mower_map/include` directories plus GoogleTest headers and libraries.

## Simulation and mower validation

Run the existing simulation with a recorded dock and enough navigation space around the approach start. Run each case
with `docking_reverse: false` and again with `true`, restarting the stack between configurations.

| Case | Expected result |
| --- | --- |
| Normal docking | Navigate forward to the approach start; in reverse mode, finish the turn there and use negative `linear.x` along the approach and insertion. Enter docked idle after charge voltage is detected. |
| Normal undocking | Reverse mode uses positive `linear.x`; forward mode uses negative `linear.x`. Straight and curved exits reach the same world-space side in both modes. |
| Failed insertion | Remove charger contact or place the simulated dock beyond the insertion endpoint. Recovery exits in the correct direction and retries stop at the configured limit. |
| Lost contact | Enable `docking_redock`, then remove contact while in docked idle. Recovery exits and approaches again using the configured direction. |
| Abort | Stop during approach, insertion, an approach/undocking wait, or extra contact time. Motion stops and no successful docking event is emitted after an abort. |
| Restart on charger | Simulator body heading and localization both match the recorded heading plus 180 degrees in reverse mode. Subsequent undocking exits the station. |
| Invalid perimeter combination | A nonzero signal with reverse mode logs an error; no docking or undocking path is sent, including start requests while on the charger. |
| Runtime direction change | A dynamic reconfigure request for the opposite direction restores the startup value and logs the restart requirement. |

Inspect path orientations, velocity commands, and localization explicitly: the simulator's charging model detects
proximity, so simulated contact alone does not establish that the mower faced the correct direction. Confirm the same
cycles on the real mower, including rear contact detection, repeated docking, recovery, and stopping, before upstream
submission. For the personal branch, also confirm that the NTRIP client connects and delivers corrections during use.
