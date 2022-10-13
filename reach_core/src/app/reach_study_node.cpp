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
#include <reach_core/reach_study.h>

#include <boost/program_options.hpp>
#include <pluginlib/class_loader.h>
#include <yaml-cpp/yaml.h>

static const std::string PACKAGE = "reach_core";
static const std::string IK_BASE_CLASS = "reach::IKSolverFactory";
static const std::string DISPLAY_BASE_CLASS = "reach::DisplayFactory";
static const std::string TARGET_POSE_GENERATOR_BASE_CLASS = "reach::TargetPoseGeneratorFactory";
static const std::string EVALUATOR_BASE_CLASS = "reach::EvaluatorFactory";

int main(int argc, char** argv)
{
  try
  {
    namespace bpo = boost::program_options;
    bpo::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
      ("help", "produce help message")
      ("config-file", bpo::value<std::string>()->required(), "configuration file")
      ("config-name", bpo::value<std::string>()->required(), "reach study configuration name")
      ("results-dir", bpo::value<std::string>()->required(), "reach study results directory")
    ;
    // clang-format on

    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
      std::cout << desc << std::endl;
      return 1;
    }

    bpo::notify(vm);

    // Load the configuration file
    const YAML::Node& config = YAML::LoadFile(vm["config-file"].as<std::string>());
    const YAML::Node& opt_config = config["optimization"];
    const YAML::Node& ik_config = config["ik_solver"];
    const YAML::Node& pose_gen_config = config["target_pose_generator"];
    const YAML::Node& eval_config = config["evaluator"];
    const YAML::Node& display_config = config["display"];

    // Extract the study parameters
    reach::ReachStudy::Parameters params;
    params.radius = opt_config["radius"].as<double>();
    params.max_steps = opt_config["max_steps"].as<int>();
    params.step_improvement_threshold = opt_config["step_improvement_threshold"].as<double>();

    // Load the IK Solver plugin
    pluginlib::ClassLoader<reach::IKSolverFactory> solver_loader(PACKAGE, IK_BASE_CLASS);
    reach::IKSolver::ConstPtr ik_solver;
    {
      reach::IKSolverFactory::Ptr factory = solver_loader.createInstance(ik_config["name"].as<std::string>());
      ik_solver = factory->create(ik_config);
    }

    // Load the target pose generator plugin
    pluginlib::ClassLoader<reach::TargetPoseGeneratorFactory> target_pose_generator_loader_(
        PACKAGE, TARGET_POSE_GENERATOR_BASE_CLASS);
    reach::TargetPoseGenerator::ConstPtr target_pose_generator;
    {
      reach::TargetPoseGeneratorFactory::Ptr factory =
          target_pose_generator_loader_.createInstance(pose_gen_config["name"].as<std::string>());
      target_pose_generator = factory->create(pose_gen_config);
    }

    // Load the evaluator plugin
    pluginlib::ClassLoader<reach::EvaluatorFactory> eval_loader(PACKAGE, EVALUATOR_BASE_CLASS);
    reach::Evaluator::ConstPtr evaluator;
    {
      reach::EvaluatorFactory::Ptr factory = eval_loader.createInstance(eval_config["name"].as<std::string>());
      evaluator = factory->create(eval_config);
    }

    // Load the display plugin
    pluginlib::ClassLoader<reach::DisplayFactory> display_loader(PACKAGE, DISPLAY_BASE_CLASS);
    reach::Display::ConstPtr display;
    {
      reach::DisplayFactory::Ptr factory = display_loader.createInstance(display_config["name"].as<std::string>());
      display = factory->create(display_config);
    }

    const std::string config_name = vm["config-name"].as<std::string>();
    boost::filesystem::path results_dir(vm["results-dir"].as<std::string>());

    // Initialize the reach study
    reach::ReachStudy rs(ik_solver, evaluator, target_pose_generator, display, params, config_name);

    const boost::filesystem::path db_file = results_dir / config_name / "study.db";
    const boost::filesystem::path opt_db_file = results_dir / config_name / "study_optimized.db";

    if (boost::filesystem::exists(opt_db_file))
    {
      // Attempt to load the optimized database first, if it exists
      rs.load(opt_db_file.string());
    }
    else if (boost::filesystem::exists(db_file))
    {
      // Then try to load the un-optimized database, if it exists
      rs.load(db_file.string());
    }
    else
    {
      // Otherwise, run the reach study
      rs.run();
      rs.save((results_dir / config_name / "study.db").string());
      rs.optimize();
      rs.save((results_dir / config_name / "study_optimized.db").string());
    }
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}
