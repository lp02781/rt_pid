# rt_pid — a simple PID controller, real-time loop + ROS 2 node, sharing memory

```
  rt_pid_loop              /dev/shm/rt_pid            pid_node
  real-time, 1 kHz     +---------------------+        normal ROS 2 node
  no ROS code          |  setpoint, kp,ki,kd | <----  writes
                       |  control_input      | ---->  reads
                       |  measurement        |
                       +---------------------+
```

The control loop has no ROS code in it. ROS uses DDS, which starts threads and
allocates memory whenever it wants, and that would make the loop miss deadlines.

## Interface

| | name | type |
|---|---|---|
| in  | `~/setpoint` | `std_msgs/Float64` |
| in  | parameters `setpoint`, `kp`, `ki`, `kd` | `double` |
| out | `~/control_input` | `std_msgs/Float64` |
| out | `~/measurement` | `std_msgs/Float64` |

## Build and run

```bash
cd ~/Downloads/rt_pid_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch rt_pid rt_pid.launch.py cpu:=3 setpoint:=20.0
```

Change things while it runs:

```bash
ros2 topic pub -1 /pid/setpoint std_msgs/msg/Float64 "{data: 35.0}"
ros2 param set /pid kp 2.0
ros2 topic echo /pid/control_input
```

## Real-time permission

By default you are not allowed to use real-time priority (`ulimit -r` is 0).
The loop still runs, just as a normal program. To fix it:

```bash
sudo ./src/rt_pid/scripts/setup_rt.sh   # then LOG OUT and LOG BACK IN
ulimit -r                               # should print 99
```

Or for a single run, with no setup: `sudo chrt -f 80 <the binary>`.

## Reserving a CPU core

`--cpu 3` only means "run me on core 3". It does not keep other programs off
core 3. To really reserve it, add this to the kernel boot line in
`/etc/default/grub`, then `sudo update-grub` and reboot:

```
isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2
```

Check what is reserved with `cat /sys/devices/system/cpu/isolated`.

## Real hardware

In `rt_pid_loop.cpp`, replace this one line:

```cpp
measurement += (control_input - FRICTION * measurement) / WEIGHT * dt;
```

with a read from your sensor into `measurement`, and a write of `control_input`
to your motor. Nothing else changes.
