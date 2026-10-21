// Publish MoveIt's complete obstacle map as Foxglove-compatible voxel markers.
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>

class OctomapMarkers : public rclcpp::Node
{
  using GetScene = moveit_msgs::srv::GetPlanningScene;
  using Marker = visualization_msgs::msg::Marker;
  using Markers = visualization_msgs::msg::MarkerArray;

public:
  OctomapMarkers() : Node("moveit_octomap_markers")
  {
    const double scale = declare_parameter<double>("cube_scale", 1.0);
    if (!std::isfinite(scale) || scale <= 0.0) {
      throw std::invalid_argument("cube_scale must be finite and greater than zero");
    }
    parameter_callback_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "cube_scale" &&
              (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
               !std::isfinite(parameter.as_double()) || parameter.as_double() <= 0.0)) {
            result.successful = false;
            result.reason = "cube_scale must be a finite positive double";
          }
        }
        return result;
      });
    publisher_ = create_publisher<Markers>(
      "/occupied_cells_vis_array", rclcpp::QoS(1).reliable().transient_local());
    client_ = create_client<GetScene>("/get_planning_scene");
    timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { requestMap(); });
    RCLCPP_INFO(get_logger(), "Publishing MoveIt's full octomap on /occupied_cells_vis_array");
  }

private:
  void requestMap()
  {
    const auto now = std::chrono::steady_clock::now();
    if (pending_) {
      if (now - requested_at_ < std::chrono::seconds(10)) {
        return;
      }
      client_->remove_pending_request(request_id_);
      pending_ = false;
      RCLCPP_WARN(get_logger(), "Planning scene request timed out; retrying");
    }
    if (!client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<GetScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;
    pending_ = true;
    requested_at_ = now;
    request_id_ = client_->async_send_request(request, [this](GetSceneFuture future) {
      pending_ = false;
      try {
        publishMap(future.get()->scene.world.octomap);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Cannot display MoveIt octomap: %s", error.what());
      }
    }).request_id;
  }

  void publishMap(const octomap_msgs::msg::OctomapWithPose & map)
  {
    Markers markers;
    Marker clear;
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);
    if (map.octomap.data.empty()) {
      publisher_->publish(markers);
      return;
    }
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(map.octomap));
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (!tree) {
      throw std::runtime_error("Expected MoveIt OcTree, received " + map.octomap.id);
    }
    const double cube_scale = get_parameter("cube_scale").as_double();
    // Each depth has a different voxel size, so use one CUBE_LIST per depth.
    std::vector<Marker> levels(tree->getTreeDepth() + 1);
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (!tree->isNodeOccupied(*it)) {
        continue;
      }
      auto & marker = levels[it.getDepth()];
      if (marker.points.empty()) {
        marker.header = map.header;
        if (marker.header.frame_id.empty()) {
          marker.header = map.octomap.header;
        }
        if (marker.header.frame_id.empty()) {
          throw std::runtime_error("Octomap has no coordinate frame");
        }
        // Keep historical maps attached to the current fixed-frame transform.
        marker.header.stamp = builtin_interfaces::msg::Time();
        marker.ns = "moveit_octomap";
        marker.id = static_cast<int>(it.getDepth());
        marker.type = Marker::CUBE_LIST;
        marker.action = Marker::ADD;
        marker.pose = map.origin;
        marker.scale.x = marker.scale.y = marker.scale.z = it.getSize() * cube_scale;
        marker.color.r = 0.2f;
        marker.color.g = 0.65f;
        marker.color.b = 0.95f;
        marker.color.a = 1.0f;
      }
      geometry_msgs::msg::Point point;
      point.x = it.getX();
      point.y = it.getY();
      point.z = it.getZ();
      marker.points.push_back(point);
    }
    for (auto & marker : levels) {
      if (!marker.points.empty()) {
        markers.markers.push_back(std::move(marker));
      }
    }
    publisher_->publish(markers);
  }

  using GetSceneFuture = rclcpp::Client<GetScene>::SharedFuture;
  rclcpp::Publisher<Markers>::SharedPtr publisher_;
  rclcpp::Client<GetScene>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  bool pending_ = false;
  int64_t request_id_ = 0;
  std::chrono::steady_clock::time_point requested_at_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapMarkers>());
  rclcpp::shutdown();
  return 0;
}
