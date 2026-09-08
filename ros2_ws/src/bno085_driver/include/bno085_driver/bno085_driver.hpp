#ifndef BNO085_DRIVER__BNO085_DRIVER_HPP_
#define BNO085_DRIVER__BNO085_DRIVER_HPP_

#include <memory>
#include <string>

#include "bno085_driver/serial_port.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace bno085_driver
{

class Bno085Driver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Bno085Driver(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

  void readSerial();
  void processReceivedData();
  void publishImu(double heading_deg);

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  void updateDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & status);


  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  rclcpp::Time last_data_time_;
  double receive_frequency_hz_{0.0};

  bool data_received_{false};
  bool driver_active_{false};

  std::unique_ptr<SerialPort> serial_port_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr serial_timer_;

  std::string rx_buffer_;

};

}  // namespace bno085_driver

#endif  // BNO085_DRIVER__BNO085_DRIVER_HPP_
