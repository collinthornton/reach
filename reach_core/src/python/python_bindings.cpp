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
#include <boost/python/converter/builtin_converters.hpp>
#include <boost/python/numpy.hpp>
#include <cstdarg>
#include <mutex>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
void print_py_error()
{
  try
  {
    PyErr_Print();
    bp::object sys(bp::handle<>(PyImport_ImportModule("sys")));
    bp::object err = sys.attr("stderr");
    std::string err_text = bp::extract<std::string>(err.attr("getvalue")());
    std::cout << err_text << std::endl;
  }
  catch (...)
  {
    std::cout << "Failed to parse python error" << std::endl;
  }
  PyErr_Clear();
}

template <typename Function, typename ClassT, typename... Args>
auto call_and_handle(Function func, ClassT* obj, const char* function_name, Args&&... args)
{
  try
  {
    return (obj->*func)(std::forward<Args>(args)...);
  }
  catch (const bp::error_already_set& e)
  {
    std::cout << "Exception in " << function_name << ": ";
    print_py_error();
    throw e;
  }
  catch (const std::exception& e)
  {
    std::cout << "Exception in " << function_name << ": " << e.what() << std::endl;
    throw e;
  }
  catch (...)
  {
    std::cout << "Exception in " << function_name << ": UNKOWN" << std::endl;
    throw;
  }
}

class ReachStudyPython : public ReachStudy
{
public:
  ReachStudyPython(const IKSolver* ik_solver, const Evaluator* evaluator, const TargetPoseGenerator* pose_generator,
                   const Display* display, Logger* logger, const ReachStudy::Parameters params,
                   const std::string& study_name)
    : ReachStudy(IKSolver::ConstPtr(std::move(ik_solver), [](const IKSolver*) {}),
                 Evaluator::ConstPtr(std::move(evaluator), [](const Evaluator*) {}),
                 TargetPoseGenerator::ConstPtr(std::move(pose_generator), [](const TargetPoseGenerator*) {}),
                 Display::ConstPtr(std::move(display), [](const Display*) {}),
                 Logger::Ptr(std::move(logger), [](Logger*) {}), params, study_name, 1)
  {
  }
};

struct IKSolverPython : IKSolver, bp::wrapper<IKSolver>
{
  std::vector<std::string> getJointNamesFunc() const
  {
    std::vector<std::string> names;

    bp::list name_list = this->get_override("getJointNames")();

    for (int i = 0; i < bp::len(name_list); ++i)
    {
      std::string name = bp::extract<std::string>(name_list[i]);
      names.push_back(name);
    }

    return names;
  }
  std::vector<std::string> getJointNames() const
  {
    return call_and_handle(&IKSolverPython::getJointNamesFunc, this, "getJointNames()");
  }

  std::vector<std::vector<double>> solveIKFunc(const Eigen::Isometry3d& target,
                                               const std::map<std::string, double>& seed) const
  {
    std::vector<std::vector<double>> output;

    bp::tuple shape = bp::make_tuple(4, 4);
    bp::numpy::dtype dtype = bp::numpy::dtype::get_builtin<double>();
    bp::numpy::ndarray array = bp::numpy::zeros(shape, dtype);

    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        array[i, j] = target.matrix()(i, j);
      }
    }

    bp::dict dictionary;
    for (std::pair<std::string, double> pair : seed)
    {
      dictionary[pair.first] = pair.second;
    }

    bp::list list = this->get_override("solveIK")(array, dictionary);

    for (int i = 0; i < bp::len(list); ++i)
    {
      std::vector<double> sub_vec;
      bp::list sub_list = bp::extract<bp::list>(list[i]);
      for (int j = 0; j < bp::len(sub_list); ++j)
      {
        sub_vec.push_back(bp::extract<double>(sub_list[j]));
      }
      output.push_back(sub_vec);
    }

    return output;
  }
  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                           const std::map<std::string, double>& seed) const
  {
    return call_and_handle(&IKSolverPython::solveIKFunc, this, "solveIK()", target, seed);
  }
};

struct EvaluatorPython : Evaluator, bp::wrapper<Evaluator>
{
  double calculateScoreFunc(const std::map<std::string, double>& map) const
  {
    bp::dict dictionary;
    for (auto pair : map)
    {
      dictionary[pair.first] = pair.second;
    }

    double score = this->get_override("calculateScore")(dictionary);

    return score;
  }
  double calculateScore(const std::map<std::string, double>& map) const
  {
    return call_and_handle(&EvaluatorPython::calculateScoreFunc, this, "calculateScore()", map);
  }
};

struct TargetPoseGeneratorPython : TargetPoseGenerator, bp::wrapper<TargetPoseGenerator>
{
  VectorIsometry3d generateFunc() const
  {
    VectorIsometry3d eigen_list;

    bp::list np_list = this->get_override("generate")();

    // Convert the list of 4x4 numpy arrays to VectorIsometry3d
    for (int i = 0; i < bp::len(np_list); ++i)
    {
      Eigen::Isometry3d eigen_mat;
      bp::numpy::ndarray np_array = bp::numpy::from_object(np_list[i]);

      for (int j = 0; j < 4; ++j)
      {
        for (int k = 0; k < 4; ++k)
        {
          eigen_mat.matrix()(j, k) = bp::extract<double>(np_array[j][k]);
        }
      }
      eigen_list.push_back(eigen_mat);
    }

    return eigen_list;
  }
  VectorIsometry3d generate() const
  {
    return call_and_handle(&TargetPoseGeneratorPython::generateFunc, this, "generate()");
  }
};

struct DisplayPython : Display, bp::wrapper<Display>
{
  void showEnvironmentFunc() const
  {
    this->get_override("showEnvironment")();
  }
  void showEnvironment() const
  {
    return call_and_handle(&DisplayPython::showEnvironmentFunc, this, "showEnvironment()");
  }

  void updateRobotPoseFunc(const std::map<std::string, double>& map) const
  {
    bp::dict dictionary;
    for (auto pair : map)
    {
      dictionary[pair.first] = pair.second;
    }

    this->get_override("updateRobotPose")(dictionary);
  }
  void updateRobotPose(const std::map<std::string, double>& map) const
  {
    return call_and_handle(&DisplayPython::updateRobotPoseFunc, this, "updateRobotPose()", map);
  }

  void showReachNeighborhoodFunc(const std::vector<ReachRecord>& neighborhood) const
  {
    this->get_override("showReachNeighborhood")(neighborhood);
  }
  void showReachNeighborhood(const std::vector<ReachRecord>& neighborhood) const
  {
    return call_and_handle(&DisplayPython::showReachNeighborhoodFunc, this, "showReachNeighborhood()", neighborhood);
  }

  void showResultsFunc(const ReachDatabase& results) const
  {
    this->get_override("showResults")(results);
  }
  void showResults(const ReachDatabase& results) const
  {
    return call_and_handle(&DisplayPython::showResultsFunc, this, "showResults()", results);
  }
};

struct LoggerPython : Logger, bp::wrapper<Logger>
{
  void setMaxProgressFunc(unsigned long max_progress)
  {
    this->get_override("setMaxProgress")(max_progress);
  }
  void setMaxProgress(unsigned long max_progress)
  {
    return call_and_handle(&LoggerPython::setMaxProgressFunc, this, "setMaxProgress()", max_progress);
  }

  void printProgressFunc(unsigned long progress) const
  {
    this->get_override("printProgress")(progress);
  }
  void printProgress(unsigned long progress) const
  {
    return call_and_handle(&LoggerPython::printProgressFunc, this, "printProgress()", progress);
  }

  void printResultsFunc(const StudyResults& results) const
  {
    this->get_override("printResults")(results);
  }
  void printResults(const StudyResults& results) const
  {
    return call_and_handle(&LoggerPython::printResultsFunc, this, "printResults()", results);
  }

  void printFunc(const std::string& msg) const
  {
    this->get_override("print")(msg);
  }
  void print(const std::string& msg) const
  {
    return call_and_handle(&LoggerPython::printFunc, this, "print()", msg);
  }
};

BOOST_PYTHON_MODULE(reach_core_python)
{
  Py_Initialize();
  PyEval_InitThreads();
  bp::numpy::initialize();

  bp::class_<YAML::Node>("YAMLNode").def("LoadFile", &YAML::LoadFile);
  bp::class_<boost::filesystem::path>("Path", bp::init<std::string>());

  bp::def("runReachStudy", runReachStudy);

  // Wrap the IKSolvers
  {
    bp::class_<IKSolverPython, boost::noncopyable>("IKSolver")
        .def("getJointNames", bp::pure_virtual(&IKSolver::getJointNames))
        .def("solveIK", bp::pure_virtual(&IKSolver::solveIK));
  }

  // Wrap the Evaluators
  {
    bp::class_<EvaluatorPython, boost::noncopyable>("Evaluator")
        .def("calculateScore", bp::pure_virtual(&Evaluator::calculateScore));
  }
  // Wrap the TargetPoseGenerators
  {
    bp::class_<TargetPoseGeneratorPython, boost::noncopyable>("TargetPoseGenerator")
        .def("generate", bp::pure_virtual(&TargetPoseGenerator::generate));
  }

  // Wrap the Displays
  {
    bp::class_<ReachDatabase>("ReachDatabase").def("calculateResults", &ReachDatabase::calculateResults);

    bp::class_<DisplayPython, boost::noncopyable>("Display")
        .def("showEnvironment", bp::pure_virtual(&Display::showEnvironment))
        .def("updateRobotPose", bp::pure_virtual(&Display::updateRobotPose))
        .def("showReachNeighborhood", bp::pure_virtual(&Display::showReachNeighborhood))
        .def("showResults", bp::pure_virtual(&Display::showResults));
  }

  // Wrap the Loggers
  {
    bp::class_<StudyResults>("StudyResults").def("print", &StudyResults::print);

    bp::class_<LoggerPython, boost::noncopyable>("Logger")
        .def("setMaxProgress", bp::pure_virtual(&Logger::setMaxProgress))
        .def("printProgress", bp::pure_virtual(&Logger::printProgress))
        .def("printResults", bp::pure_virtual(&Logger::printResults))
        .def("print", bp::pure_virtual(&Logger::print));
  }

  // Wrap the Parameters
  {
    bp::class_<ReachStudy::Parameters>("Parameters")
        .def_readwrite("max_steps", &ReachStudy::Parameters::max_steps)
        .def_readwrite("step_improvement_threshold", &ReachStudy::Parameters::step_improvement_threshold)
        .def_readwrite("radius", &ReachStudy::Parameters::radius);
  }

  bp::class_<ReachStudyPython>("ReachStudy",
                               bp::init<const IKSolver*, const Evaluator*, const TargetPoseGenerator*, const Display*,
                                        Logger*, const ReachStudy::Parameters, const std::string&>())
      .def("load", &ReachStudyPython::load)
      .def("save", &ReachStudyPython::save)
      .def("getDatabase", &ReachStudyPython::getDatabase)
      .def("run", &ReachStudyPython::run)
      .def("optimize", &ReachStudyPython::optimize)
      .def("getAverageNeighborsCounts", &ReachStudyPython::getAverageNeighborsCount);

  bp::register_ptr_to_python<ReachDatabase::ConstPtr>();
  bp::register_ptr_to_python<IKSolver::ConstPtr>();
  bp::register_ptr_to_python<Evaluator::ConstPtr>();
  bp::register_ptr_to_python<Display::ConstPtr>();
  bp::register_ptr_to_python<Logger::Ptr>();
  bp::register_ptr_to_python<TargetPoseGenerator::ConstPtr>();
}
}  // namespace reach
