#include <reach_core/interfaces/target_pose_generator.h>
#include <reach_core/utils.h>

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;
namespace np = boost::python::numpy;

namespace reach
{
TargetPoseGenerator::ConstPtr TargetPoseGeneratorFactory::create(const bp::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}

bp::list TargetPoseGenerator::generatePython() const
{
  bp::list output;
  VectorIsometry3d cpp_output = generate();

  for (Eigen::Isometry3d& cpp_matrix : cpp_output)
  {
    np::ndarray matrix = np::zeros(bp::make_tuple(4, 4), np::dtype::get_builtin<double>());

    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        matrix[i, j] = cpp_matrix.matrix()(i, j);
      }
    }
    output.append(matrix);
  }

  return output;
}

}  // namespace reach
