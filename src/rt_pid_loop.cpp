// The real-time PID loop. Runs 1000 times a second on a PREEMPT_RT kernel.
//
// No ROS code here on purpose: ROS uses DDS, which starts threads and
// allocates memory whenever it wants, and that would make us miss deadlines.
// ROS lives in pid_node instead, and the two share memory.

#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <new>

#include "rt_pid/shm_data.hpp"

namespace
{
volatile sig_atomic_t running = 1;
void stop(int) {running = 0;}

constexpr long NS_PER_SEC = 1000000000L;

// Move a time forward, carrying nanoseconds into seconds.
void add_ns(timespec & t, long ns)
{
  t.tv_nsec += ns;
  while (t.tv_nsec >= NS_PER_SEC) {
    t.tv_nsec -= NS_PER_SEC;
    t.tv_sec += 1;
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  long period_ns = 1000000;  // 1 ms, so 1000 loops per second
  int priority = 80;
  int cpu = -1;              // -1 means any core

  for (int i = 1; i < argc - 1; ++i) {
    if (strcmp(argv[i], "--period-us") == 0) {period_ns = atol(argv[++i]) * 1000;}
    if (strcmp(argv[i], "--prio") == 0) {priority = atoi(argv[++i]);}
    if (strcmp(argv[i], "--cpu") == 0) {cpu = atoi(argv[++i]);}
  }

  signal(SIGINT, stop);
  signal(SIGTERM, stop);

  // --- make the shared memory ---
  shm_unlink(rt_pid::SHM_NAME);  // clear out anything a crashed run left behind
  int fd = shm_open(rt_pid::SHM_NAME, O_CREAT | O_RDWR, 0660);
  if (fd < 0) {perror("shm_open"); return 1;}
  if (ftruncate(fd, sizeof(rt_pid::SharedData)) != 0) {perror("ftruncate"); return 1;}

  void * addr = mmap(
    nullptr, sizeof(rt_pid::SharedData), PROT_READ | PROT_WRITE,
    MAP_SHARED | MAP_POPULATE, fd, 0);   // MAP_POPULATE: load it into RAM now
  close(fd);
  if (addr == MAP_FAILED) {perror("mmap"); return 1;}

  auto * shm = new(addr) rt_pid::SharedData();
  shm->setpoint = 10.0;
  shm->kp = 0.5;
  shm->ki = 4.0;
  shm->kd = 0.01;
  shm->ready = 1;  // set last, so the ROS node knows the rest is filled in

  // --- become real-time ---

  // Keep all our memory in RAM. Otherwise the kernel can make us wait while it
  // fetches a page, which costs hundreds of microseconds.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {perror("mlockall");}

  // Pin to one core. For best results that core should also be reserved at
  // boot with isolcpus=N nohz_full=N rcu_nocbs=N. See the README.
  if (cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {perror("sched_setaffinity");}
  }

  // SCHED_FIFO: when we are ready to run, we run, and ordinary programs wait.
  sched_param sp{};
  sp.sched_priority = priority;
  const bool is_rt = sched_setscheduler(0, SCHED_FIFO, &sp) == 0;
  if (!is_rt) {
    printf("could not get real-time priority, running as a normal program\n"
           "  fix: sudo ./src/rt_pid/scripts/setup_rt.sh   (then log out and in)\n"
           "  or:  sudo chrt -f %d <this program>\n", priority);
  }

  const double dt = period_ns / 1e9;  // one loop, in seconds
  printf("%.0f Hz | %s | cpu %d\n", 1.0 / dt, is_rt ? "real-time" : "normal", cpu);
  fflush(stdout);  // print now; we must not call printf inside the loop

  // --- the plant we control: a motor that speeds up when pushed ---
  constexpr double WEIGHT = 0.05;
  constexpr double FRICTION = 0.10;
  double measurement = 0.0;

  double error_sum = 0.0;   // the I part remembers this
  double last_error = 0.0;  // the D part needs this
  long worst_late_us = 0;

  // Wake at exact times, not "sleep for 1 ms". A relative sleep starts counting
  // from whenever we call it, so any lateness piles up loop after loop.
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);  // never CLOCK_REALTIME: it can jump back

  while (running) {
    add_ns(next, period_ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    // How late the kernel woke us. This is the number that says whether the
    // real-time setup is working.
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    const long late_us =
      ((now.tv_sec - next.tv_sec) * NS_PER_SEC + (now.tv_nsec - next.tv_nsec)) / 1000;
    if (late_us > worst_late_us) {worst_late_us = late_us;}

    // --- PID ---
    const double error = shm->setpoint - measurement;

    error_sum += error * dt;
    const double derivative = (error - last_error) / dt;
    last_error = error;

    const double control_input =
      shm->kp * error +          // P: push harder the further off we are
      shm->ki * error_sum +      // I: add up the error, to close the last gap
      shm->kd * derivative;      // D: brake when the error is closing fast

    // --- apply it to the plant (replace with a real motor write) ---
    measurement += (control_input - FRICTION * measurement) / WEIGHT * dt;

    // --- hand the results to the ROS node ---
    shm->control_input = control_input;
    shm->measurement = measurement;
  }

  printf("\nworst wakeup: %ld us late\n", worst_late_us);
  shm->ready = 0;
  munmap(addr, sizeof(rt_pid::SharedData));
  shm_unlink(rt_pid::SHM_NAME);
  return 0;
}
