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
#ifndef REACH_CORE_REACH_STUDY_H
#define REACH_CORE_REACH_STUDY_H

#include <reach_core/interfaces/display.h>
#include <reach_core/interfaces/evaluator.h>
#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/interfaces/target_pose_generator.h>
#include <reach_core/interfaces/logger.h>
#include <reach_core/utils.h>

#include <boost/filesystem/path.hpp>
#include <thread>

namespace reach
{
/**
 * @brief The ReachStudy class
 */
class ReachStudy
{
public:
  struct Parameters
  {
    int max_steps;
    float step_improvement_threshold;
    float radius;
  };

  ReachStudy(IKSolver::ConstPtr ik_solver, Evaluator::ConstPtr evaluator, TargetPoseGenerator::ConstPtr pose_generator,
             Display::ConstPtr display, Logger::Ptr logger, const Parameters params, const std::string& study_name,
             const size_t max_threads = std::thread::hardware_concurrency());

  void load(const std::string& filename);
  void run();
  void optimize();
  void save(const std::string& filename) const;

  StudyResults getResults() const;
  ReachDatabase::ConstPtr getDatabase() const;

  std::tuple<double, double> getAverageNeighborsCount() const;

protected:
  Parameters params_;
  ReachDatabase::Ptr db_;

  // Plugins
  IKSolver::ConstPtr ik_solver_;
  Evaluator::ConstPtr evaluator_;
  Display::ConstPtr display_;
  Logger::Ptr logger_;

  const size_t max_threads_;

  const VectorIsometry3d target_poses_;

  SearchTreePtr search_tree_ = nullptr;
};

/**
 * @brief Helper function for configuring a ReachStudy object and performing a reach study
 */
void runReachStudy(const YAML::Node& config, const std::string& config_name = "reach_study",
                   const boost::filesystem::path& results_dir = "/tmp", const bool wait_after_completion = true);

}  // namespace reach

#endif  // REACH_CORE_REACH_STUDY_H
