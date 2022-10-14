/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <moveit_reach_plugins/display/moveit_reach_display.h>
#include <moveit_reach_plugins/utils.h>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <reach_core/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>

const static std::string PLANNING_SCENE_TOPIC = "planning_scene_display";
const static std::string MARKER_TOPIC = "reach_neighbors";

namespace moveit_reach_plugins
{
namespace display
{
MoveItReachDisplay::MoveItReachDisplay(moveit::core::RobotModelConstPtr model, const std::string& planning_group,
                                       std::string collision_mesh_filename, std::string collision_mesh_frame,
                                       double marker_scale)
  : model_(model)
  , jmg_(model_->getJointModelGroup(planning_group))
  , collision_mesh_filename_(std::move(collision_mesh_filename))
  , collision_mesh_frame_(std::move(collision_mesh_frame))
  , marker_scale_(marker_scale)
  , scene_(new planning_scene::PlanningScene(model_))
{
  // Check that the input collision mesh frame exists
  if (!scene_->knowsFrameTransform(collision_mesh_frame_))
    throw std::runtime_error("Specified collision mesh frame '" + collision_mesh_frame_ + "' does not exist");

  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::CollisionObject obj =
      utils::createCollisionObject(collision_mesh_filename_, collision_mesh_frame_, object_name);
  if (!scene_->processCollisionObjectMsg(obj))
    throw std::runtime_error("Failed to add collision mesh to planning scene");

  scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1, true);
  neighbors_pub_ = nh_.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 1, true);
}

void MoveItReachDisplay::showEnvironment() const
{
  moveit_msgs::PlanningScene scene_msg;
  scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_.publish(scene_msg);
}

void MoveItReachDisplay::updateRobotPose(const std::map<std::string, double>& pose) const
{
  std::vector<std::string> joint_names = jmg_->getActiveJointModelNames();
  std::vector<double> joints = utils::transcribeInputMap(pose, joint_names);

  moveit_msgs::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_msg.robot_state.is_diff = true;
  scene_msg.robot_state.joint_state.name = joint_names;
  scene_msg.robot_state.joint_state.position = joints;
  scene_pub_.publish(scene_msg);
}

void MoveItReachDisplay::showResults(const reach::ReachDatabase& /*db*/) const
{
}

void MoveItReachDisplay::showReachNeighborhood(const std::vector<reach::ReachRecord>& neighborhood) const
{
  if (!neighborhood.empty())
  {
    std::vector<geometry_msgs::Point> pt_array;

    for (const reach::ReachRecord& rec : neighborhood)
    {
      // Visualize points reached around input point
      const Eigen::Vector3d& pt = rec.goal.translation();
      pt_array.push_back(tf2::toMsg(pt));
    }

    // Create points marker, publish it, and move robot to result state for  given point
    visualization_msgs::Marker pt_marker = utils::makeMarker(pt_array, collision_mesh_frame_, marker_scale_);
    neighbors_pub_.publish(pt_marker);
  }
}

reach::Display::ConstPtr MoveItReachDisplayFactory::create(const YAML::Node& config) const
{
  auto planning_group = reach::get<std::string>(config, "planning_group");
  auto collision_mesh_filename = reach::get<std::string>(config, "collision_mesh_filename");
  auto collision_mesh_frame = reach::get<std::string>(config, "collision_mesh_frame");
  auto marker_scale = reach::get<double>(config, "marker_scale");

  moveit::core::RobotModelConstPtr model = moveit::planning_interface::getSharedRobotModel("robot_description");
  if (!model)
    throw std::runtime_error("Failed to initialize robot model pointer");

  return boost::make_shared<MoveItReachDisplay>(model, planning_group, collision_mesh_filename, collision_mesh_frame,
                                                marker_scale);
}

}  // namespace display
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::display::MoveItReachDisplayFactory, reach::DisplayFactory)
