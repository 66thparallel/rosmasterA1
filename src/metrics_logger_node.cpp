#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rosmaster_a1
{

class MetricsLoggerNode : public rclcpp::Node
{
public:
  MetricsLoggerNode()
  : Node("metrics_logger_node")
  {
    const auto configured_log_dir = declare_parameter<std::string>("log_dir", "metrics_logs");
    summary_period_s_ = declare_parameter<double>("summary_period_s", 2.0);
    if (summary_period_s_ <= 0.0) {
      throw std::invalid_argument("The 'summary_period_s' parameter must be positive.");
    }

    log_dir_ = resolve_log_directory(configured_log_dir);
    open_csv(controller_file_, log_dir_ / "controller_metrics.csv", controller_headers());
    open_csv(planner_file_, log_dir_ / "planner_metrics.csv", planner_headers());

    controller_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/metrics/controller", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        log_controller_metrics(*message);
      });
    planner_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/metrics/planner", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        log_planner_metrics(*message);
      });
    summary_timer_ = create_wall_timer(
      std::chrono::duration<double>(summary_period_s_), [this]() { log_summary(); });

    RCLCPP_INFO(get_logger(), "Metrics logger writing CSV files to %s", log_dir_.c_str());
  }

private:
  struct ControllerSummary
  {
    std::size_t count{0U};
    double control_latency_ms_sum{0.0};
    double cross_track_error_max{0.0};
    double cross_track_error_sum{0.0};
    double heading_error_sum{0.0};
    double steering_oscillation_max{0.0};
  };

  struct PlannerSummary
  {
    std::size_t count{0U};
    double loop_rate_hz_sum{0.0};
  };

  static std::filesystem::path resolve_log_directory(const std::string & configured_log_dir)
  {
    std::filesystem::path log_dir(configured_log_dir);
    if (log_dir.empty()) {
      throw std::invalid_argument("The 'log_dir' parameter must not be empty.");
    }
    if (log_dir.is_relative()) {
      log_dir = std::filesystem::current_path() / log_dir;
    }

    std::error_code error;
    std::filesystem::create_directories(log_dir, error);
    if (error) {
      throw std::runtime_error("Unable to create metrics log directory '" + log_dir.string() + "': " + error.message());
    }
    return log_dir;
  }

  static void open_csv(
    std::ofstream & stream, const std::filesystem::path & file_path,
    const std::vector<std::string> & headers)
  {
    stream.open(file_path, std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
      throw std::runtime_error("Unable to open metrics log file '" + file_path.string() + "'.");
    }
    write_csv_row(stream, headers);
  }

  void log_controller_metrics(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    const auto values = values_to_map(message);
    const double cross_track_error = parse_double(values, "cross_track_error");
    const double heading_error = parse_double(values, "heading_error");
    const double steering_oscillation = parse_double(values, "steering_oscillation");
    const double control_latency_ms = parse_double(values, "control_latency_ms");

    write_csv_row(controller_file_, {
      format_double(stamp_to_seconds(message)),
      format_double(cross_track_error),
      format_double(heading_error),
      format_double(parse_double(values, "steering_command")),
      format_double(steering_oscillation),
      format_double(parse_double(values, "commanded_speed")),
      format_double(parse_double(values, "curvature")),
      format_double(control_latency_ms),
      format_double(parse_double(values, "lookahead_distance")),
      format_double(parse_double(values, "target_x")),
      format_double(parse_double(values, "target_y")),
      std::to_string(parse_integer(values, "path_pose_count")),
    });

    ++controller_summary_.count;
    controller_summary_.control_latency_ms_sum += control_latency_ms;
    controller_summary_.cross_track_error_sum += cross_track_error;
    controller_summary_.heading_error_sum += std::abs(heading_error);
    controller_summary_.cross_track_error_max = std::max(
      controller_summary_.cross_track_error_max, cross_track_error);
    controller_summary_.steering_oscillation_max = std::max(
      controller_summary_.steering_oscillation_max, steering_oscillation);
  }

  void log_planner_metrics(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    const auto values = values_to_map(message);
    const double loop_rate_hz = parse_double(values, "loop_rate_hz");
    write_csv_row(planner_file_, {
      format_double(stamp_to_seconds(message)),
      get_value(values, "frame_id"),
      std::to_string(parse_integer(values, "waypoint_count")),
      format_double(parse_double(values, "publish_interval_ms")),
      format_double(loop_rate_hz),
    });

    ++planner_summary_.count;
    planner_summary_.loop_rate_hz_sum += loop_rate_hz;
  }

  void log_summary() const
  {
    if (controller_summary_.count > 0U) {
      const double count = static_cast<double>(controller_summary_.count);
      RCLCPP_INFO(
        get_logger(),
        "controller metrics: mean_cte=%.3f, max_cte=%.3f, mean_heading=%.3f, mean_latency_ms=%.3f, peak_steering_osc=%.3f",
        controller_summary_.cross_track_error_sum / count,
        controller_summary_.cross_track_error_max,
        controller_summary_.heading_error_sum / count,
        controller_summary_.control_latency_ms_sum / count,
        controller_summary_.steering_oscillation_max);
    }

    if (planner_summary_.count > 0U) {
      RCLCPP_INFO(
        get_logger(), "planner metrics: mean_loop_rate_hz=%.3f",
        planner_summary_.loop_rate_hz_sum / static_cast<double>(planner_summary_.count));
    }
  }

  static std::unordered_map<std::string, std::string> values_to_map(
    const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    std::unordered_map<std::string, std::string> values;
    if (message.status.empty()) {
      return values;
    }
    for (const auto & key_value : message.status.front().values) {
      values[key_value.key] = key_value.value;
    }
    return values;
  }

  static double parse_double(
    const std::unordered_map<std::string, std::string> & values, const std::string & key)
  {
    try {
      const double value = std::stod(get_value(values, key));
      return std::isfinite(value) ? value : 0.0;
    } catch (const std::exception &) {
      return 0.0;
    }
  }

  static std::int64_t parse_integer(
    const std::unordered_map<std::string, std::string> & values, const std::string & key)
  {
    try {
      return std::stoll(get_value(values, key));
    } catch (const std::exception &) {
      return 0;
    }
  }

  static std::string get_value(
    const std::unordered_map<std::string, std::string> & values, const std::string & key)
  {
    const auto value = values.find(key);
    return value == values.end() ? "" : value->second;
  }

  static double stamp_to_seconds(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    return static_cast<double>(message.header.stamp.sec) +
           static_cast<double>(message.header.stamp.nanosec) * 1e-9;
  }

  static std::string format_double(const double value)
  {
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    return stream.str();
  }

  static void write_csv_row(std::ofstream & stream, const std::vector<std::string> & values)
  {
    for (std::size_t index = 0U; index < values.size(); ++index) {
      if (index > 0U) {
        stream << ',';
      }
      stream << escape_csv(values[index]);
    }
    stream << '\n';
    stream.flush();
    if (!stream.good()) {
      throw std::runtime_error("Unable to write metrics CSV row.");
    }
  }

  static std::string escape_csv(const std::string & value)
  {
    if (value.find_first_of(",\"\n\r") == std::string::npos) {
      return value;
    }

    std::string escaped_value{"\""};
    for (const char character : value) {
      if (character == '\"') {
        escaped_value += "\"\"";
      } else {
        escaped_value += character;
      }
    }
    escaped_value += '\"';
    return escaped_value;
  }

  static std::vector<std::string> controller_headers()
  {
    return {
      "stamp_sec", "cross_track_error", "heading_error", "steering_command",
      "steering_oscillation", "commanded_speed", "curvature", "control_latency_ms",
      "lookahead_distance", "target_x", "target_y", "path_pose_count",
    };
  }

  static std::vector<std::string> planner_headers()
  {
    return {"stamp_sec", "frame_id", "waypoint_count", "publish_interval_ms", "loop_rate_hz"};
  }

  double summary_period_s_;
  std::filesystem::path log_dir_;
  std::ofstream controller_file_;
  std::ofstream planner_file_;
  ControllerSummary controller_summary_;
  PlannerSummary planner_summary_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr controller_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr planner_subscription_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

}  // namespace rosmaster_a1

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosmaster_a1::MetricsLoggerNode>());
  rclcpp::shutdown();
  return 0;
}