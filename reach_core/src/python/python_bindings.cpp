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

namespace reach
{
BOOST_PYTHON_MODULE(reach_core_python)
{
  boost::python::class_<YAML::Node>("YAMLNode").def("LoadFile", &YAML::LoadFile);

  boost::python::class_<boost::filesystem::path>("Path", boost::python::init<std::string>());
  boost::python::def("runReachStudy", runReachStudy);
}
}  // namespace reach
