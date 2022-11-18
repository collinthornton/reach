#include <reach_core/interfaces/display.h>
#include <reach_core/utils.h>

#include <boost/python.hpp>
#include <yaml-cpp/yaml.h>

namespace reach
{
void Display::updateRobotPose(const boost::python::dict& dict_joint_positions) const
{
  return updateRobotPose(pythonDictToMap<std::string, double>(dict_joint_positions));
}

Display::ConstPtr DisplayFactory::create(const boost::python::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}
}  // namespace reach
