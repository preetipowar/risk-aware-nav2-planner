#include "risk_aware_planner/risk_aware_planner.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <limits>

namespace risk_aware_planner
{

void RiskAwarePlanner::configure(
 const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
 std::string name,
 std::shared_ptr<tf2_ros::Buffer> tf,
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
 node_ = parent.lock();
 global_frame_ = costmap_ros->getGlobalFrameID();
 costmap_ = costmap_ros->getCostmap();

 RCLCPP_INFO(node_->get_logger(), "RiskAwarePlanner configured");
}

void RiskAwarePlanner::cleanup()
{
 RCLCPP_INFO(node_->get_logger(), "RiskAwarePlanner cleanup");
}

void RiskAwarePlanner::activate()
{
 RCLCPP_INFO(node_->get_logger(), "RiskAwarePlanner activated");
}

void RiskAwarePlanner::deactivate()
{
 RCLCPP_INFO(node_->get_logger(), "RiskAwarePlanner deactivated");
}

nav_msgs::msg::Path RiskAwarePlanner::createPlan(
 const geometry_msgs::msg::PoseStamped & start,
 const geometry_msgs::msg::PoseStamped & goal)
{
 RCLCPP_INFO(node_->get_logger(), "RiskAwarePlanner running");

 nav_msgs::msg::Path path;
 path.header.frame_id = global_frame_;
 path.header.stamp = node_->now();

 double dx = goal.pose.position.x - start.pose.position.x;
 double dy = goal.pose.position.y - start.pose.position.y;

 double distance = std::sqrt(dx * dx + dy * dy);

 double sample_resolution = 0.05;
 int steps = std::max(10, (int)(distance / sample_resolution));

 std::vector<double> offsets = {0.0, 0.3, -0.3, 0.6, -0.6};

 double best_score = std::numeric_limits<double>::max();
 double best_risk = 0.0;

 nav_msgs::msg::Path best_path;

 for (double offset : offsets)
 {
   nav_msgs::msg::Path candidate;
   candidate.header = path.header;

   double risk = 0.0;
   unsigned char max_cost = 0;

   for (int i = 0; i <= steps; i++)
   {
     double t = (double)i / steps;

     double x = start.pose.position.x + t * dx;
     double y = start.pose.position.y + t * dy;

     geometry_msgs::msg::PoseStamped pose;
     pose.header = path.header;

     pose.pose.position.x = x;
     pose.pose.position.y = y;
     pose.pose.orientation = start.pose.orientation;

     unsigned int mx, my;

     if (costmap_->worldToMap(x, y, mx, my))
     {
       unsigned char cost = costmap_->getCost(mx, my);

       risk += cost * cost;

       if (cost > max_cost)
         max_cost = cost;
     }

     candidate.poses.push_back(pose);
   }

   double score = risk + 1000 * fabs(offset);

   RCLCPP_INFO(node_->get_logger(),
     "Candidate offset %.2f risk %.2f max_cost %d",
     offset, risk, max_cost);

   if (score < best_score)
   {
     best_score = score;
     best_risk = risk;
     best_path = candidate;

     RCLCPP_INFO(node_->get_logger(),
       "New best path chosen with offset %.2f (risk %.2f)",
       offset, risk);
   }
 }

 RCLCPP_INFO(node_->get_logger(),
   "Selected path risk: %.2f", best_risk);

 return best_path;
}

}

PLUGINLIB_EXPORT_CLASS(
 risk_aware_planner::RiskAwarePlanner,
 nav2_core::GlobalPlanner)
