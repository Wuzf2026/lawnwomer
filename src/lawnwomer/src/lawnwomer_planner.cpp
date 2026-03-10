#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace lawnwomer {
class LawnwomerPlanner : public nav_core::BaseGlobalPlanner {
public:
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override {}
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override {
    plan.push_back(start);
    plan.push_back(goal);
    return true;
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lawnwomer::LawnwomerPlanner, nav_core::BaseGlobalPlanner)