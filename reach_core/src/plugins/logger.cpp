#include <reach_core/interfaces/logger.h>
#include <reach_core/reach_database.h>
#include <reach_core/utils.h>

#include <boost/progress.hpp>
#include <iostream>
#include <mutex>

namespace reach
{
Logger::Ptr LoggerFactory::create(const boost::python::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}

}  // namespace reach
