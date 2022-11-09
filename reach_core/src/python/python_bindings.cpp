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

#include <boost/python.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
struct IKSolverWrap : IKSolver, bp::wrapper<IKSolver>
{
  std::vector<std::string> getJointNames() const
  {
    return this->get_override("getJointNames")();
  }

  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d&, const std::map<std::string, double>&) const
  {
    return this->get_override("solveIK")();
  }
};

struct IKSolverFactoryWrap : IKSolverFactory, bp::wrapper<IKSolverFactory>
{
  IKSolver::ConstPtr create(const YAML::Node&) const
  {
    return this->get_override("create")();
  }
};

struct EvaluatorWrap : Evaluator, bp::wrapper<Evaluator>
{
  double calculateScore(const std::map<std::string, double>&) const
  {
    return this->get_override("calculateScore")();
  }
};

struct EvaluatorFactoryWrap : EvaluatorFactory, bp::wrapper<EvaluatorFactory>
{
  Evaluator::ConstPtr create(const YAML::Node&) const
  {
    return this->get_override("create")();
  }
};

struct TargetPoseGeneratorWrap : TargetPoseGenerator, bp::wrapper<TargetPoseGenerator>
{
  VectorIsometry3d generate() const
  {
    return this->get_override("generate")();
  }
};

struct TargetPoseGeneratorFactoryWrap : TargetPoseGeneratorFactory, bp::wrapper<TargetPoseGeneratorFactory>
{
  TargetPoseGenerator::ConstPtr create(const YAML::Node&) const
  {
    return this->get_override("create")();
  }
};

struct DisplayWrap : Display, bp::wrapper<Display>
{
  void showEnvironment() const
  {
    this->get_override("showEnvironment")();
  }

  void updateRobotPose(const std::map<std::string, double>&) const
  {
    this->get_override("updateRobotPose")();
  }

  void showReachNeighborhood(const std::vector<ReachRecord>&) const
  {
    this->get_override("showReachNeighborhood")();
  }

  void showResults(const ReachDatabase&) const
  {
    this->get_override("showResults")();
  }
};

struct DisplayFactoryWrap : DisplayFactory, bp::wrapper<DisplayFactory>
{
  Display::ConstPtr create(const YAML::Node&) const
  {
    return this->get_override("create")();
  }
};

struct LoggerWrap : Logger, bp::wrapper<Logger>
{
  void setMaxProgress(unsigned long)
  {
    this->get_override("setMaxProgress")();
  }

  void printProgress(unsigned long) const
  {
    this->get_override("printProgress")();
  }

  void printResults(const StudyResults&) const
  {
    this->get_override("printResults")();
  }

  void print(const std::string&) const
  {
    this->get_override("print")();
  }
};

struct LoggerFactoryWrap : LoggerFactory, bp::wrapper<LoggerFactory>
{
  Logger::ConstPtr create(const YAML::Node&) const
  {
    return this->get_override("create")();
  }
};

BOOST_PYTHON_MODULE(reach_core_python)
{
  bp::class_<YAML::Node>("YAMLNode").def("LoadFile", &YAML::LoadFile);
  bp::class_<boost::filesystem::path>("Path", bp::init<std::string>());

  bp::def("runReachStudy", runReachStudy);

  // Wrap the IKSolvers
  {
    bp::class_<IKSolverWrap, boost::noncopyable>("IKSolver")
        .def("getJointNames", bp::pure_virtual(&IKSolver::getJointNames))
        .def("solveIK", bp::pure_virtual(&IKSolver::solveIK));

    bp::class_<IKSolverFactoryWrap, boost::noncopyable>("IKSolverFactory")
        .def("create", bp::pure_virtual(&IKSolverFactory::create))
        .def("getSection", &IKSolverFactory::getSection);
  }

  // Wrap the Evaluators
  {
    bp::class_<EvaluatorWrap, boost::noncopyable>("Evaluator")
        .def("calculateScore", bp::pure_virtual(&Evaluator::calculateScore));

    bp::class_<EvaluatorFactoryWrap, boost::noncopyable>("EvaluatorFactory")
        .def("create", bp::pure_virtual(&EvaluatorFactory::create))
        .def("getSection", &EvaluatorFactory::getSection);
  }
  // Wrap the TargetPoseGenerators
  {
    bp::class_<TargetPoseGeneratorWrap, boost::noncopyable>("TargetPoseGenerator")
        .def("generate", bp::pure_virtual(&TargetPoseGenerator::generate));
  }

  // Wrap the Displays
  {
    bp::class_<DisplayWrap, boost::noncopyable>("Display")
        .def("showEnvironment", bp::pure_virtual(&Display::showEnvironment))
        .def("updateRobotPose", bp::pure_virtual(&Display::updateRobotPose))
        .def("showReachNeighborhood", bp::pure_virtual(&Display::showReachNeighborhood))
        .def("showResults", bp::pure_virtual(&Display::showResults));

    bp::class_<DisplayFactoryWrap, boost::noncopyable>("DisplayFactory")
        .def("create", bp::pure_virtual(&DisplayFactory::create))
        .def("getSection", &DisplayFactory::getSection);
  }

  // Wrap the Loggers
  {
    bp::class_<LoggerWrap, boost::noncopyable>("Logger")
        .def("setMaxProgress", bp::pure_virtual(&Logger::setMaxProgress))
        .def("printProgress", bp::pure_virtual(&Logger::printProgress))
        .def("printResults", bp::pure_virtual(&Logger::printResults))
        .def("print", bp::pure_virtual(&Logger::print));

    bp::class_<LoggerFactoryWrap, boost::noncopyable>("LoggerFactory")
        .def("create", bp::pure_virtual(&LoggerFactory::create))
        .def("getSection", &LoggerFactory::getSection);
  }

  // Wrap the Parameters
  {
    bp::class_<ReachStudy::Parameters>("Parameters")
        .def_readwrite("max_steps", &ReachStudy::Parameters::max_steps)
        .def_readwrite("step_improvement_threshold", &ReachStudy::Parameters::step_improvement_threshold)
        .def_readwrite("radius", &ReachStudy::Parameters::radius);
  }

  bp::class_<ReachStudy>(
      "ReachStudy", bp::init<IKSolver::ConstPtr, Evaluator::ConstPtr, TargetPoseGenerator::ConstPtr, Display::ConstPtr,
                             Logger::ConstPtr, const ReachStudy::Parameters, const std::string&>())
      .def("load", &ReachStudy::load)
      .def("save", &ReachStudy::save)
      .def("getDatabase", &ReachStudy::getDatabase)
      .def("run", &ReachStudy::run)
      .def("optimize", &ReachStudy::optimize)
      .def("getAverageNeighborsCounts", &ReachStudy::getAverageNeighborsCount);
}
}  // namespace reach
