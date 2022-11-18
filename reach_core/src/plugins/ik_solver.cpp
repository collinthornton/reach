#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/utils.h>

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <yaml-cpp/yaml.h>

namespace reach
{
boost::python::list IKSolver::solveIK(const boost::python::numpy::ndarray& target,
                                      const boost::python::dict& seed) const
{
  Eigen::Isometry3d cpp_target;
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      cpp_target.matrix()(i, j) = boost::python::extract<double>(target[i, j]);
    }
  }

  std::map<std::string, double> cpp_seed = pythonDictToMap<std::string, double>(seed);

  std::vector<std::vector<double>> cpp_solution = solveIK(cpp_target, cpp_seed);

  boost::python::list solution;
  for (std::vector<double> inner : cpp_solution)
  {
    solution.append(inner);
  }

  return solution;
}

IKSolver::ConstPtr IKSolverFactory::create(const boost::python::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}
}  // namespace reach
