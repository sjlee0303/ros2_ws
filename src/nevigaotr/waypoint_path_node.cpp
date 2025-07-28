#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>

class WaypointPathNode : public rclcpp::Node
{
public:
  WaypointPathNode()
  : Node("waypoint_path_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cone_map", 1,
      std::bind(&WaypointPathNode::mapCallback, this, std::placeholders::_1));

    pub_path_   = create_publisher<nav_msgs::msg::Path>("/nav_path", 1);
    pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("/wp_marker", 1);

    lane_half_      = declare_parameter("lane_half",      0.9);
    pair_long_tol_  = declare_parameter("pair_long_tol",  0.7);
    hist_keep_ = declare_parameter("hist_keep", 50);
  }

private:
  /* ---------- parameters ---------- */
  double   lane_half_;
  double   pair_long_tol_;
  unsigned hist_keep_;

  /* ---------- data ---------- */
  std::vector<Eigen::Vector3d> waypoints_;

  /* ---------- ROS ---------- */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

  /* ---------- callbacks ---------- */
  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    /* 1) yellow / blue 분리  (intensity: 0=yellow, 1=blue) */
    std::vector<Eigen::Vector3d> left, right;

    sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x"),
                                                 y_it(*msg, "y"),
                                                 z_it(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> inten_it(*msg, "intensity");
    for (; x_it != x_it.end();
         ++x_it, ++y_it, ++z_it, ++inten_it)
    {
      Eigen::Vector3d p(*x_it, *y_it, *z_it);
      (*inten_it == 0 ? left : right).push_back(p);
    }

    std::sort(left.begin(), left.end(),
              [](auto &a, auto &b){ return a.x() < b.x(); });
    std::sort(right.begin(), right.end(),
              [](auto &a, auto &b){ return a.x() < b.x(); });

    /* 2) 좌‧우 짝짓기 → 중점 */
    std::size_t i = 0, j = 0;
    while (i < left.size() || j < right.size())
    {
      if (i < left.size() && j < right.size() &&
          std::abs(left[i].x() - right[j].x()) < pair_long_tol_)
      {
        waypoints_.push_back(0.5 * (left[i] + right[j]));
        ++i; ++j;
      }
      else if (i < left.size() &&
               (j == right.size() || left[i].x() < right[j].x()))
      {
        Eigen::Vector3d pseudo = left[i];
        pseudo.y() -= lane_half_;
        waypoints_.push_back(0.5 * (left[i] + pseudo));
        ++i;
      }
      else
      {
        Eigen::Vector3d pseudo = right[j];
        pseudo.y() += lane_half_;
        waypoints_.push_back(0.5 * (right[j] + pseudo));
        ++j;
      }
    }

    if (waypoints_.size() > hist_keep_)
      waypoints_.erase(waypoints_.begin(),
                       waypoints_.end() - hist_keep_);

    publishPath(msg->header.stamp);
  }

  void publishPath(const builtin_interfaces::msg::Time &stamp)
  {
    if (waypoints_.size() < 2) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = "map";      // 고정 프레임

    for (const auto &p : waypoints_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = 0.0;
      path_msg.poses.push_back(ps);
    }
    pub_path_->publish(path_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointPathNode>());
  rclcpp::shutdown();
  return 0;
}
