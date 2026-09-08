// Copyright 2026 Furkan Kara

#include "bno085_driver/bno085_driver.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "bno085_driver/nmea_parser.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace bno085_driver
{

Bno085Driver::Bno085Driver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("bno085_driver", options)
{
  RCLCPP_INFO(
    get_logger(),
    "BNO085 lifecycle driver created");
}

Bno085Driver::CallbackReturn Bno085Driver::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "Configuring BNO085 driver");

  serial_port_ = std::make_unique<SerialPort>();

  if (!serial_port_->open("/dev/ttyACM0")) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to open serial port /dev/ttyACM0");

    serial_port_.reset();

    return CallbackReturn::FAILURE;
  }
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  rx_buffer_.clear();
  rx_buffer_.reserve(256U);

  diagnostic_updater_ =
    std::make_unique<diagnostic_updater::Updater>(this);

  diagnostic_updater_->setHardwareID("BNO085_STM32");

  diagnostic_updater_->add(
    "BNO085 Driver",
    this,
    &Bno085Driver::updateDiagnostics);

  RCLCPP_INFO(
    get_logger(),
    "Serial port /dev/ttyACM0 opened at 115200 baud");

  return CallbackReturn::SUCCESS;
}

Bno085Driver::CallbackReturn Bno085Driver::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "Activating BNO085 driver");

  rx_buffer_.clear();

  if (serial_port_) {
    serial_port_->flushInput();
  }

  driver_active_ = true;
  data_received_ = false;
  receive_frequency_hz_ = 0.0;

  imu_publisher_->on_activate();

  serial_timer_ =
    create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&Bno085Driver::readSerial, this));

  return CallbackReturn::SUCCESS;
}

Bno085Driver::CallbackReturn Bno085Driver::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "Deactivating BNO085 driver");

  serial_timer_.reset();
  if (imu_publisher_) {
    imu_publisher_->on_deactivate();
  }
  driver_active_ = false;
  return CallbackReturn::SUCCESS;
}

Bno085Driver::CallbackReturn Bno085Driver::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "Cleaning up BNO085 driver");

  serial_timer_.reset();
  rx_buffer_.clear();
  tf_broadcaster_.reset();
  imu_publisher_.reset();
  serial_port_.reset();
  diagnostic_updater_.reset();
  driver_active_ = false;
  data_received_ = false;
  receive_frequency_hz_ = 0.0;
  return CallbackReturn::SUCCESS;
}

Bno085Driver::CallbackReturn Bno085Driver::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "Shutting down BNO085 driver");

  return CallbackReturn::SUCCESS;
}

void Bno085Driver::publishImu(double heading_deg)
{
  sensor_msgs::msg::Imu message;

  message.header.stamp = now();
  message.header.frame_id = "imu_link";

  constexpr double kPi = 3.14159265358979323846;

  const double yaw_rad =
    heading_deg * kPi / 180.0;

  tf2::Quaternion quaternion;

  quaternion.setRPY(
    0.0,
    0.0,
    yaw_rad);

  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();

  /*
   * Only yaw is supplied by the heading sensor.
   * Roll and pitch uncertainty are intentionally large.
   */
  message.orientation_covariance[0] = 1e6;
  message.orientation_covariance[4] = 1e6;
  message.orientation_covariance[8] = 0.01;

  /*
   * Angular velocity and linear acceleration
   * are not provided.
   */
  message.angular_velocity_covariance[0] = -1.0;
  message.linear_acceleration_covariance[0] = -1.0;

  imu_publisher_->publish(message);
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = message.header.stamp;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "imu_link";

  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();

  tf_broadcaster_->sendTransform(transform);
}

void Bno085Driver::updateDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const bool serial_open =
    serial_port_ && serial_port_->isOpen();

  bool data_recent = false;

  if (driver_active_ && data_received_) {
    data_recent =
      (now() - last_data_time_).seconds() < 0.5;
  }

  if (!serial_open) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Serial port closed");
  } else if (!driver_active_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Driver inactive");
  } else if (!data_recent) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "No recent sensor data");
  } else {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "BNO085 connected");
  }

  if (!serial_open) {
    status.add("Connection", "Disconnected");
  } else if (!driver_active_) {
    status.add("Connection", "Inactive");
  } else if (data_recent) {
    status.add("Connection", "Connected");
  } else {
    status.add("Connection", "Not receiving");
  }

  const double diagnostic_frequency_hz =
    data_recent ? receive_frequency_hz_ : 0.0;

  status.add(
    "Frequency (Hz)",
    diagnostic_frequency_hz);
}

void Bno085Driver::readSerial()
{
  if ((!serial_port_) ||
    (!serial_port_->isOpen()))
  {
    return;
  }

  char buffer[128];

  const int bytes_read =
    serial_port_->read(
    buffer,
    sizeof(buffer));

  if (bytes_read < 0) {
    RCLCPP_ERROR(
      get_logger(),
      "Serial read error");

    return;
  }

  if (bytes_read == 0) {
    return;
  }

  rx_buffer_.append(
    buffer,
    static_cast<std::size_t>(bytes_read));

  processReceivedData();
}
void Bno085Driver::processReceivedData()
{
  std::size_t newline_position;

  while ((newline_position = rx_buffer_.find('\n')) !=
    std::string::npos)
  {
    std::string sentence =
      rx_buffer_.substr(
      0U,
      newline_position);

    rx_buffer_.erase(
      0U,
      newline_position + 1U);

    if ((!sentence.empty()) &&
      (sentence.back() == '\r'))
    {
      sentence.pop_back();
    }

    if (!sentence.empty()) {
      double heading_deg;
      if (parseHdm(sentence, heading_deg)) {
        const rclcpp::Time current_time = now();
        if (data_received_) {
          const double dt = (current_time - last_data_time_).seconds();

          if (dt > 0.0) {
            receive_frequency_hz_ = 1.0 / dt;
          }
        }
        last_data_time_ = current_time;
        data_received_ = true;
        publishImu(heading_deg);
      } else {
        RCLCPP_WARN(get_logger(), "Invalid NMEA sentence: %s", sentence.c_str());
      }
    }
  }
}
}  // namespace bno085_driver
