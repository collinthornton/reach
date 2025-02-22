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
#ifndef MOVEIT_REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H
#define MOVEIT_REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H

#include <reach_core/plugins/reach_display_base.h>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
}  // namespace core
}  // namespace moveit

namespace planning_scene
{
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace moveit_reach_plugins
{
namespace display
{
class MoveItReachDisplay : public reach::plugins::DisplayBase
{
public:
  MoveItReachDisplay();

  virtual bool initialize(XmlRpc::XmlRpcValue& config) override;

  virtual void showEnvironment() override;

  virtual void updateRobotPose(const std::map<std::string, double>& pose) override;

private:
  moveit::core::RobotModelConstPtr model_;

  planning_scene::PlanningScenePtr scene_;

  const moveit::core::JointModelGroup* jmg_;

  std::string collision_mesh_filename_;

  std::string collision_mesh_frame_;

  ros::NodeHandle nh_;

  ros::Publisher scene_pub_;
};

}  // namespace display
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_MOVEIT_REACH_DISPLAY_H
