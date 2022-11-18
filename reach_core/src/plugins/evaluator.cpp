#include <reach_core/interfaces/evaluator.h>
#include <reach_core/utils.h>

#include <boost/python.hpp>
#include <yaml-cpp/yaml.h>

namespace reach
{
double Evaluator::calculateScore(const boost::python::dict& pose) const
{
  return calculateScore(pythonDictToMap<std::string, double>(pose));
}

Evaluator::ConstPtr EvaluatorFactory::create(const boost::python::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}
}  // namespace reach
