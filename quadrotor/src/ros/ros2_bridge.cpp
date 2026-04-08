#include "ros/ros2_bridge.hpp"
#include "ros/publisher/data/clock_data_publisher.hpp"
#include "ros/publisher/data/imu_data_publisher.hpp"
#include "ros/publisher/data/odom_data_publisher.hpp"
#include "ros/publisher/data/transform_data_publisher.hpp"
#include "ros/subscriber/data/cmd_vel_command_subscriber.hpp"

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#if defined(QUADROTOR_HAS_ROS2)
#include <rclcpp/rclcpp.hpp>
#endif

namespace quadrotor {

#if defined(QUADROTOR_HAS_ROS2)
namespace {

namespace fs = std::filesystem;

struct RosBridgeConfig {
  VehicleIdentity identity;
  Ros2Config ros2;
  RosInterfaceConfig interfaces;
  RosFrameConfig frames;
  std::vector<SensorConfig> sensors;
};

std::string NormalizeNamespace(std::string value) {
  if (value.empty() || value == "/") {
    return "";
  }
  if (value.front() != '/') {
    value.insert(value.begin(), '/');
  }
  while (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string TrimSlashes(std::string value) {
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string ResolveFrameId(const RosBridgeConfig& config, const std::string& frame_name) {
  const std::string trimmed_frame = TrimSlashes(frame_name);
  const std::string prefix = TrimSlashes(config.identity.frame_prefix);
  if (prefix.empty()) {
    return trimmed_frame;
  }
  if (trimmed_frame.empty()) {
    return prefix;
  }
  return prefix + "/" + trimmed_frame;
}

std::string ResolveTopicName(const RosBridgeConfig& config, const std::string& topic_name) {
  if (topic_name.empty()) {
    return NormalizeNamespace(config.identity.ros_namespace);
  }
  if (!topic_name.empty() && topic_name.front() == '/') {
    return topic_name;
  }

  const std::string ros_namespace = NormalizeNamespace(config.identity.ros_namespace);
  if (ros_namespace.empty()) {
    return "/" + TrimSlashes(topic_name);
  }
  return ros_namespace + "/" + TrimSlashes(topic_name);
}

RosBridgeConfig BuildRosBridgeConfig(const QuadrotorConfig& config) {
  RosBridgeConfig bridge_config;
  bridge_config.identity = config.identity;
  bridge_config.ros2 = config.ros2;
  bridge_config.interfaces = config.interfaces;
  bridge_config.frames = config.frames;
  bridge_config.sensors = config.sensors;
  return bridge_config;
}

void EnsureWritableRosHome() {
#if defined(__linux__)
  if (std::getenv("ROS_HOME") == nullptr) {
    const fs::path ros_home = fs::path("/tmp") / "quadrotor_ros_home";
    std::error_code error_code;
    fs::create_directories(ros_home, error_code);
    setenv("ROS_HOME", ros_home.string().c_str(), 0);
  }
#endif
}

bool CycloneDdsAvailable() {
  static const fs::path kCandidates[] = {
      fs::path("/opt/ros/humble/share/rmw_cyclonedds_cpp"),
      fs::path("/opt/ros/humble_py311/share/rmw_cyclonedds_cpp"),
  };
  for (const fs::path& candidate : kCandidates) {
    if (fs::exists(candidate)) {
      return true;
    }
  }
  return false;
}

void EnsurePreferredRmwImplementation() {
#if defined(__linux__)
  if (std::getenv("RMW_IMPLEMENTATION") == nullptr && CycloneDdsAvailable()) {
    setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp", 0);
  }
#endif
}

std::string ActiveRmwImplementation() {
  const char* value = std::getenv("RMW_IMPLEMENTATION");
  return value == nullptr ? std::string("<unset>") : std::string(value);
}

class Ros2BridgeImpl final : public Ros2Bridge {
 public:
  Ros2BridgeImpl(
      RosBridgeConfig config,
      std::shared_ptr<CommandMailbox> command_mailbox,
      std::shared_ptr<TelemetryCache> telemetry_cache)
      : config_(std::move(config)),
        command_mailbox_(std::move(command_mailbox)),
        telemetry_cache_(std::move(telemetry_cache)) {}

  ~Ros2BridgeImpl() override { Stop(); }

  void Start() override {
    if (started_) {
      return;
    }
    if (config_.ros2.publish_rate_hz <= 0.0) {
      throw std::runtime_error("ros2.publish_rate_hz must be positive.");
    }

    EnsureWritableRosHome();
    EnsurePreferredRmwImplementation();

    try {
      if (!rclcpp::ok()) {
        int argc = 1;
        char program_name[] = "quadrotor_ros2_bridge";
        char* argv[] = {program_name, nullptr};
        rclcpp::init(argc, argv);
        owns_rclcpp_runtime_ = true;
      }

      node_ = std::make_shared<rclcpp::Node>(config_.ros2.node_name);

      const std::string odom_topic = ResolveTopicName(config_, config_.interfaces.odom_topic);
      const std::string imu_topic = ResolveTopicName(config_, config_.interfaces.imu_topic);
      const std::string clock_topic = ResolveTopicName(config_, config_.interfaces.clock_topic);
      const std::string cmd_vel_topic =
          ResolveTopicName(config_, config_.interfaces.cmd_vel_topic);
      const std::string odom_frame = ResolveFrameId(config_, config_.frames.odom);
      const std::string base_frame = ResolveFrameId(config_, config_.frames.base);
      const std::string imu_frame = ResolveFrameId(config_, config_.frames.imu);

      odom_publisher_ = std::make_unique<OdomDataPublisher>(
          node_,
          odom_topic,
          odom_frame,
          base_frame);
      imu_publisher_ = std::make_unique<ImuDataPublisher>(
          node_,
          imu_topic,
          imu_frame);

      if (config_.ros2.publish_clock) {
        clock_publisher_ = std::make_unique<ClockDataPublisher>(node_, clock_topic);
      }
      if (config_.ros2.publish_tf) {
        transform_publisher_ =
            std::make_unique<TransformDataPublisher>(node_, odom_frame, base_frame);
      }

      cmd_vel_subscription_ =
          std::make_unique<CmdVelCommandSubscriber>(node_, cmd_vel_topic, command_mailbox_);

      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.ros2.publish_rate_hz));
      telemetry_timer_ = node_->create_wall_timer(
          period,
          [this]() {
            PublishTelemetry();
          });

      for (const SensorConfig& sensor : config_.sensors) {
        if (sensor.enabled) {
          RCLCPP_WARN(
              node_->get_logger(),
              "Configured ROS sensor bridge '%s' of type '%s' is not implemented yet.",
              sensor.name.c_str(),
              sensor.type.c_str());
        }
      }

      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);
      spin_thread_ = std::thread([this]() { executor_->spin(); });
      started_ = true;
    } catch (const std::exception& error) {
      if (executor_ && node_) {
        executor_->remove_node(node_);
      }
      telemetry_timer_.reset();
      cmd_vel_subscription_.reset();
      odom_publisher_.reset();
      imu_publisher_.reset();
      clock_publisher_.reset();
      transform_publisher_.reset();
      executor_.reset();
      node_.reset();
      if (owns_rclcpp_runtime_ && rclcpp::ok()) {
        rclcpp::shutdown();
      }
      owns_rclcpp_runtime_ = false;

      throw std::runtime_error(
          "Failed to start ROS2 bridge (RMW_IMPLEMENTATION=" + ActiveRmwImplementation() +
          "): " + error.what());
    }
  }

  void Stop() override {
    if (!started_) {
      return;
    }

    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (executor_ && node_) {
      executor_->remove_node(node_);
    }

    telemetry_timer_.reset();
    cmd_vel_subscription_.reset();
    odom_publisher_.reset();
    imu_publisher_.reset();
    clock_publisher_.reset();
    transform_publisher_.reset();
    executor_.reset();
    node_.reset();

    if (owns_rclcpp_runtime_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }

    owns_rclcpp_runtime_ = false;
    started_ = false;
  }

 private:
  void PublishTelemetry() {
    const std::optional<TelemetrySnapshot> snapshot = telemetry_cache_->ReadLatest();
    if (!snapshot.has_value()) {
      return;
    }

    odom_publisher_->Publish(*snapshot);
    imu_publisher_->Publish(*snapshot);

    if (transform_publisher_) {
      transform_publisher_->Publish(*snapshot);
    }

    if (clock_publisher_) {
      clock_publisher_->Publish(snapshot->sim_time);
    }
  }

  RosBridgeConfig config_;
  std::shared_ptr<CommandMailbox> command_mailbox_;
  std::shared_ptr<TelemetryCache> telemetry_cache_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<CmdVelCommandSubscriber> cmd_vel_subscription_;
  std::unique_ptr<OdomDataPublisher> odom_publisher_;
  std::unique_ptr<ImuDataPublisher> imu_publisher_;
  std::unique_ptr<ClockDataPublisher> clock_publisher_;
  std::unique_ptr<TransformDataPublisher> transform_publisher_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  std::thread spin_thread_;
  bool owns_rclcpp_runtime_ = false;
  bool started_ = false;
};

}  // namespace

bool Ros2BridgeAvailable() {
  return true;
}

std::unique_ptr<Ros2Bridge> CreateRos2Bridge(
    const QuadrotorConfig& config,
    std::shared_ptr<CommandMailbox> command_mailbox,
    std::shared_ptr<TelemetryCache> telemetry_cache) {
  return std::make_unique<Ros2BridgeImpl>(
      BuildRosBridgeConfig(config),
      std::move(command_mailbox),
      std::move(telemetry_cache));
}

#else

bool Ros2BridgeAvailable() {
  return false;
}

std::unique_ptr<Ros2Bridge> CreateRos2Bridge(
    const QuadrotorConfig&,
    std::shared_ptr<CommandMailbox>,
    std::shared_ptr<TelemetryCache>) {
  return nullptr;
}

#endif

}  // namespace quadrotor
