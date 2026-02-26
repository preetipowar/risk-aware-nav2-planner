#define RISK_AWARE_PLANNER_HPP_
#define RISK_AWARE_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace risk_aware_planner
{

class RiskAwarePlanner : public nav2_core::GlobalPlanner
{
public:
 RiskAwarePlanner() = default;
 ~RiskAwarePlanner() override = default;

 void configure(
   const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
   std::string name,
   std::shared_ptr<tf2_ros::Buffer> tf,
   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

 void cleanup() override;
 void activate() override;
 void deactivate() override;

 nav_msgs::msg::Path createPlan(
   const geometry_msgs::msg::PoseStamped & start,
   const geometry_msgs::msg::PoseStamped & goal) override;

private:

 rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
 std::string global_frame_;

 nav2_costmap_2d::Costmap2D * costmap_;
};

}

#endif
