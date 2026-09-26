// Read-only acceptance instrumentation. Jazzy rclpy does not expose publisher GID.
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <tuple>

class AuthorityProbe : public rclcpp::Node {
 public:
  explicit AuthorityProbe(const std::string &path)
      : Node("m6_tf_authority_probe"), path_(path) {
    dynamic_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(100),
        [this](const tf2_msgs::msg::TFMessage &msg, const rclcpp::MessageInfo &info) {
          observe(msg, info, "dynamic");
        });
    static_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(100).transient_local(),
        [this](const tf2_msgs::msg::TFMessage &msg, const rclcpp::MessageInfo &info) {
          observe(msg, info, "static");
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { write(); });
  }
  ~AuthorityProbe() override { write(); }

 private:
  void observe(const tf2_msgs::msg::TFMessage &msg, const rclcpp::MessageInfo &info,
               const std::string &kind) {
    std::ostringstream gid;
    gid << std::hex << std::setfill('0');
    for (auto b : info.get_rmw_message_info().publisher_gid.data)
      gid << std::setw(2) << static_cast<unsigned>(b);
    for (const auto &tf : msg.transforms)
      ++counts_[{tf.header.frame_id, tf.child_frame_id, kind, gid.str()}];
  }
  void write() {
    std::ofstream out(path_ + ".tmp");
    for (const auto &[key, count] : counts_) {
      const auto &[parent, child, kind, gid] = key;
      out << parent << '\t' << child << '\t' << kind << '\t' << gid << '\t' << count << '\n';
    }
    out.close();
    std::rename((path_ + ".tmp").c_str(), path_.c_str());
  }
  std::string path_;
  std::map<std::tuple<std::string, std::string, std::string, std::string>, size_t> counts_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_, static_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::string path = argv[1];
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AuthorityProbe>(path));
  rclcpp::shutdown();
}
