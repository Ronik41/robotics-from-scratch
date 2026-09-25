// Simulation-only motor electronics: bounded torque, deadband, brake and a
// watchdog independent of the ROS firmware process. No pose/odometry access.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>

namespace rover {
class MotorDriver : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate {
  using Wall = std::chrono::steady_clock;
  std::mutex mutex;
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::array<gz::sim::Entity, 2> joints{};
  std::array<double, 2> effort{}, speed{}, applied{};
  double sim = 0, stamp = -1, lastReport = -1;
  Wall::time_point received{};
  bool valid = false, enable = false, estop = false, ready = false;
  size_t rejected = 0;

  void Command(const gz::msgs::Double_V &msg) {
    std::lock_guard<std::mutex> lock(mutex);
    // [simulation acquisition time, left Nm, right Nm, mode: 0 brake/1 run/2 e-stop]
    bool ok = msg.data_size() == 4;
    for (auto x : msg.data()) ok = ok && std::isfinite(x);
    ok = ok && msg.data(0) > stamp && msg.data(0) <= sim+.03
            && sim-msg.data(0) <= .15
            && std::abs(msg.data(1)) <= 2 && std::abs(msg.data(2)) <= 2
            && (msg.data(3) == 0 || msg.data(3) == 1 || msg.data(3) == 2);
    if (!ok) { valid = false; ++rejected; return; }
    stamp = msg.data(0);
    effort = {msg.data(1), msg.data(2)};
    enable = msg.data(3) == 1;
    estop = estop || msg.data(3) == 2;
    received = Wall::now();
    valid = true;
  }

  bool Reset(const gz::msgs::Boolean &req, gz::msgs::Boolean &reply) {
    std::lock_guard<std::mutex> lock(mutex);
    // Explicit reset only while the last valid frame requests a brake and the
    // physical wheels are stationary. Never enables torque as a side effect.
    bool ok = req.data() && ready && valid && !enable
              && sim-stamp <= .15 && Wall::now()-received < std::chrono::seconds(1)
              && std::abs(speed[0]) < .15 && std::abs(speed[1]) < .15;
    if (ok) { estop = false; valid = false; }
    reply.set_data(ok);
    return true;
  }

 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    joints = {model.JointByName(ecm, "left_wheel_joint"),
              model.JointByName(ecm, "right_wheel_joint")};
    for (auto joint : joints) {
      if (joint == gz::sim::kNullEntity) throw std::runtime_error("Missing motor joint");
      ecm.CreateComponent(joint, gz::sim::components::JointVelocity());
    }
    node.Subscribe("/motor/effort", &MotorDriver::Command, this);
    node.Advertise("/motor/reset", &MotorDriver::Reset, this);
    publisher = node.Advertise<gz::msgs::StringMsg>("/motor/driver_state");
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    std::lock_guard<std::mutex> lock(mutex);
    double now = std::chrono::duration<double>(info.simTime).count();
    if (now < sim) { valid = false; estop = true; }
    sim = now;
    if (info.paused) return;
    ready = true;
    for (size_t i=0; i<2; ++i) {
      auto velocity = ecm.Component<gz::sim::components::JointVelocity>(joints[i]);
      if (!velocity || velocity->Data().empty()) { ready = false; speed[i] = 0; }
      else speed[i] = velocity->Data()[0];
    }
    bool expired = sim-stamp > .15 || Wall::now()-received > std::chrono::milliseconds(750);
    bool drive = ready && valid && enable && !estop && !expired;
    for (size_t i=0; i<2; ++i) {
      // Finite damping brake is a model of normally-on motor braking. It is
      // torque-limited, not an instantaneous pose/velocity teleport.
      applied[i] = drive ? std::copysign(std::max(0., std::abs(effort[i])-.04), effort[i])
                         : std::clamp(-speed[i], -2., 2.);
      auto force = ecm.Component<gz::sim::components::JointForceCmd>(joints[i]);
      if (force) force->Data() = {applied[i]};
      else ecm.CreateComponent(joints[i], gz::sim::components::JointForceCmd({applied[i]}));
    }
    if (sim-lastReport >= .05) {
      std::ostringstream out;
      out << "{\"sim_time\":" << sim << ",\"state\":\""
          << (estop ? "ESTOP" : drive ? "DRIVE" : expired ? "WATCHDOG" : "BRAKE")
          << "\",\"estop_latched\":" << (estop ? "true" : "false")
          << ",\"applied_nm\":[" << applied[0] << ',' << applied[1]
          << "],\"wheel_rad_s\":[" << speed[0] << ',' << speed[1]
          << "],\"rejected\":" << rejected << '}';
      gz::msgs::StringMsg msg; msg.set_data(out.str()); publisher.Publish(msg);
      lastReport = sim;
    }
  }
};
}
GZ_ADD_PLUGIN(rover::MotorDriver, gz::sim::System,
              rover::MotorDriver::ISystemConfigure, rover::MotorDriver::ISystemPreUpdate)
