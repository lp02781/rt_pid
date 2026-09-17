// The block of memory both programs share.
//
//   pid_node    writes setpoint, kp, ki, kd
//   rt_pid_loop writes control_input, measurement
//
// Every field is a std::atomic, which on a normal PC is just a plain load or
// store. No locks, so the fast loop can never get stuck waiting for the ROS node.

#ifndef RT_PID__SHM_DATA_HPP_
#define RT_PID__SHM_DATA_HPP_

#include <atomic>

namespace rt_pid
{

inline constexpr const char * SHM_NAME = "/rt_pid";

struct SharedData
{
  // --- set by the ROS node ---
  std::atomic<double> setpoint;  // where we want to be
  std::atomic<double> kp;
  std::atomic<double> ki;
  std::atomic<double> kd;

  // --- set by the PID loop ---
  std::atomic<double> control_input;  // what the PID sends to the plant
  std::atomic<double> measurement;    // what the plant is actually doing

  std::atomic<int> ready;  // 1 once the loop has filled everything in
};

// If atomics were secretly built on a lock, that lock would live inside one
// program only and the two would not really be in sync.
static_assert(std::atomic<double>::is_always_lock_free, "need lock-free atomics");

}  // namespace rt_pid

#endif  // RT_PID__SHM_DATA_HPP_
