#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <chrono>
#include <algorithm>
using namespace std::chrono_literals;

#include "interfaces_pkg/msg/cone_info_array.hpp"

/* ───────────────────── 데이터 구조 ───────────────────── */
struct MapCone {
  Eigen::Vector3d p;
  rclcpp::Time    last_seen;
  std::string     color;
};

/* ───────────────────── 클래스 ────────────────────────── */
class ConeMapBuilder : public rclcpp::Node
{
public:
  ConeMapBuilder()
  : Node("cone_map_builder"),
    tf_buf_(this->get_clock()),
    tf_listener_(tf_buf_)
  {
    merge_radius_ = declare_parameter("merge_radius", 0.4);
    ema_alpha_    = declare_parameter("ema_alpha",    0.3);
    forget_after_ = declare_parameter("forget_after", 3.0);

    source_frame_ = declare_parameter("source_frame", "velodyne");
    target_frame_ = declare_parameter("target_frame", "velodyne");

    sub_ = create_subscription<interfaces_pkg::msg::ConeInfoArray>(
      "/cones/cone_info", 10,
      std::bind(&ConeMapBuilder::coneCallback, this, std::placeholders::_1));

    pub_cloud_  = create_publisher<sensor_msgs::msg::PointCloud2>("/cone_map", 1);
    pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cone_marker", 1);

    timer_cleanup_ = create_wall_timer(
      500ms, std::bind(&ConeMapBuilder::cleanup, this));
  }

private:
  /* parameters */
  double merge_radius_, ema_alpha_, forget_after_;
  std::string source_frame_, target_frame_;

  /* data */
  std::vector<MapCone> cones_;

  /* ROS handles */
  rclcpp::Subscription<interfaces_pkg::msg::ConeInfoArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         pub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  pub_marker_;
  tf2_ros::Buffer              tf_buf_;
  tf2_ros::TransformListener   tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_cleanup_;

  /* ────────────── 콜백들 ────────────── */
  void coneCallback(const interfaces_pkg::msg::ConeInfoArray::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped T;
    try {
      T = tf_buf_.lookupTransform(target_frame_, source_frame_, now);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF lookup failed: %s", e.what());
      return;
    }

    for (const auto &c : msg->cones) {
      geometry_msgs::msg::PointStamped p_in, p_out;
      p_in.header.stamp = now;
      p_in.header.frame_id = source_frame_;
      p_in.point.x = c.x; p_in.point.y = c.y; p_in.point.z = c.z;
      tf2::doTransform(p_in, p_out, T);

      Eigen::Vector3d p_map(p_out.point.x, p_out.point.y, p_out.point.z);

      int near_idx = -1; double best = merge_radius_;
      for (size_t i=0;i<cones_.size();++i) {
        if (cones_[i].color!=c.cone_color) continue;
        double d=(cones_[i].p-p_map).norm();
        if (d<best){best=d; near_idx=i;}
      }

      if (near_idx>=0) {
        cones_[near_idx].p =
          ema_alpha_*p_map + (1.0-ema_alpha_)*cones_[near_idx].p;
        cones_[near_idx].last_seen = now;
      } else {
        cones_.push_back({p_map, now, c.cone_color});
      }
    }
    publishCloudAndMarker(now);
  }

  void cleanup()
  {
    auto now = get_clock()->now();
    cones_.erase(std::remove_if(cones_.begin(), cones_.end(),
      [&](const MapCone &c){ return (now - c.last_seen).seconds() > forget_after_; }),
      cones_.end());
  }

  void publishCloudAndMarker(const rclcpp::Time &stamp)
  {
    /* ---------- PointCloud2 ---------- */
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = target_frame_;
    cloud.height = 1;
    cloud.width  = cones_.size();

    const size_t sz = 4;   // float32
    cloud.fields.resize(4);

    auto make_field = [&](const std::string &name, int idx)
    {
    sensor_msgs::msg::PointField f;                 // ★ msg 네임스페이스
    f.name = name;
    f.offset = idx * sz;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;  // ★
    f.count = 1;
    return f;
    };
    cloud.fields[0]=make_field("x",0);
    cloud.fields[1]=make_field("y",1);
    cloud.fields[2]=make_field("z",2);
    cloud.fields[3]=make_field("intensity",3);

    cloud.point_step = 4*sz;
    cloud.row_step   = cloud.point_step * cloud.width;
    cloud.is_bigendian=false; cloud.is_dense=true;
    cloud.data.resize(cloud.row_step);

    sensor_msgs::PointCloud2Iterator<float>
      x_it(cloud,"x"), y_it(cloud,"y"), z_it(cloud,"z"), inten_it(cloud,"intensity");

    for(const auto &c:cones_){
      *x_it=c.p.x(); *y_it=c.p.y(); *z_it=c.p.z();
      *inten_it = (c.color=="yellow")?0.0f:1.0f;
      ++x_it;++y_it;++z_it;++inten_it;
    }
    pub_cloud_->publish(cloud);

    /* ---------- MarkerArray ---------- */
    visualization_msgs::msg::Marker m;
    m.header = cloud.header;
    m.type = m.SPHERE_LIST;
    m.scale.x = m.scale.y = m.scale.z = 0.25;

    for(const auto &c:cones_){
      geometry_msgs::msg::Point p; p.x=c.p.x(); p.y=c.p.y(); p.z=c.p.z();
      m.points.push_back(p);
      std_msgs::msg::ColorRGBA col; col.a=1.0;
      if(c.color=="yellow"){col.r=1.0; col.g=1.0;} else {col.b=1.0;}
      m.colors.push_back(col);
    }
    visualization_msgs::msg::MarkerArray ma; ma.markers.push_back(m);
    pub_marker_->publish(ma);
  }
};
/* ───────────────────── main ─────────────────── */
int main(int argc,char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeMapBuilder>());
  rclcpp::shutdown();
  return 0;
}
