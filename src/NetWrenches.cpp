#include "stonefish_net/NetWrenches.h"

#include <utility>

#include <Stonefish/entities/SolidEntity.h>

#include "convert.h"
#include "pose_now.h"

namespace stonefish_net
{

NetWrenches::NetWrenches(rclcpp::Node& node, std::vector<Group> groups) : groups_(std::move(groups))
{
  for (const auto& g : groups_) {
    publishers_.push_back(node.create_publisher<geometry_msgs::msg::WrenchStamped>(g.topic, 10));
    RCLCPP_INFO(node.get_logger(), "Red: fuerza sobre %zu cuerpos en %s (marco %s)", g.bodies.size(), g.topic.c_str(),
                g.frame_id.c_str());
  }
}

void NetWrenches::publish(const SolidContacts& solids, const rclcpp::Time& stamp)
{
  const std::vector<core::Wrench> pending = solids.pending();
  for (size_t k = 0; k < groups_.size(); ++k) {
    const Group& g = groups_[k];
    std::vector<core::Wrench> about_cg;
    std::vector<Eigen::Vector3d> cg;
    for (size_t b : g.bodies) {
      if (b >= pending.size()) continue;
      about_cg.push_back(pending[b]);
      cg.push_back(eig(cg_now(solids.bodies()[b].solid).getOrigin()));
    }
    const core::Wrench w = core::wrench_in_frame(about_cg, cg, isometry(o_now(g.frame)));
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = g.frame_id;
    msg.wrench.force.x = w.force.x();
    msg.wrench.force.y = w.force.y();
    msg.wrench.force.z = w.force.z();
    msg.wrench.torque.x = w.torque.x();
    msg.wrench.torque.y = w.torque.y();
    msg.wrench.torque.z = w.torque.z();
    publishers_[k]->publish(msg);
  }
}

}  // namespace stonefish_net
