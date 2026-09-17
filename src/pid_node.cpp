// The ROS 2 side. Not real-time, and that is the point: all the slow,
// unpredictable work happens here instead of in the control loop.
//
//   in   parameters setpoint, kp, ki, kd
//   in   topic  ~/setpoint
//   out  topic  ~/control_input
//   out  topic  ~/measurement

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rt_pid/shm_data.hpp"

using namespace std::chrono_literals;

class PidNode : public rclcpp::Node
{
public:
  PidNode()
  : Node("pid")
  {
    declare_parameter("setpoint", 10.0);
    declare_parameter("kp", 0.5);
    declare_parameter("ki", 4.0);
    declare_parameter("kd", 0.01);

    // Open the shared memory the loop made. No O_CREAT: if it is not there,
    // the loop is not running and we should say so, not make an empty one.
    const int fd = shm_open(rt_pid::SHM_NAME, O_RDWR, 0660);
    if (fd < 0) {
      throw std::runtime_error("rt_pid_loop is not running");
    }
    void * addr = mmap(
      nullptr, sizeof(rt_pid::SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (addr == MAP_FAILED) {
      throw std::runtime_error("mmap failed");
    }
    shm_ = static_cast<rt_pid::SharedData *>(addr);

    send_gains();  // push our parameters down before anything else

    control_input_pub_ = create_publisher<std_msgs::msg::Float64>("~/control_input", 10);
    measurement_pub_ = create_publisher<std_msgs::msg::Float64>("~/measurement", 10);

    // Set the parameter rather than writing memory directly, so that
    // "ros2 param get setpoint" always matches what the loop is using.
    setpoint_sub_ = create_subscription<std_msgs::msg::Float64>(
      "~/setpoint", 10,
      [this](const std_msgs::msg::Float64 & msg) {
        set_parameter(rclcpp::Parameter("setpoint", msg.data));
      });

    // Runs whenever anyone changes a parameter.
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "setpoint") {shm_->setpoint = p.as_double();}
          if (p.get_name() == "kp") {shm_->kp = p.as_double();}
          if (p.get_name() == "ki") {shm_->ki = p.as_double();}
          if (p.get_name() == "kd") {shm_->kd = p.as_double();}
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    // 100 Hz is plenty for watching. The loop still runs at 1000 Hz.
    timer_ = create_wall_timer(10ms, [this]() {publish();});
  }

  ~PidNode() override
  {
    // Only unmap. The loop owns the memory and may still be using it.
    if (shm_ != nullptr) {munmap(shm_, sizeof(rt_pid::SharedData));}
  }

private:
  void send_gains()
  {
    shm_->setpoint = get_parameter("setpoint").as_double();
    shm_->kp = get_parameter("kp").as_double();
    shm_->ki = get_parameter("ki").as_double();
    shm_->kd = get_parameter("kd").as_double();
  }

  void publish()
  {
    if (shm_->ready == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "rt_pid_loop has stopped");
      return;
    }
    std_msgs::msg::Float64 msg;
    msg.data = shm_->control_input;
    control_input_pub_->publish(msg);
    msg.data = shm_->measurement;
    measurement_pub_->publish(msg);
  }

  rt_pid::SharedData * shm_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_input_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measurement_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PidNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("pid"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
