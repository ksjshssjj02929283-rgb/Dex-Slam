// Copyright (c) 2021 dcm360 Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FILE_MANAGER_HPP_
#define FILE_MANAGER_HPP_

#include <boost/filesystem.hpp>

#include <string>
#include <iostream>
#include <fstream>

#include "glog/logging.h"

namespace dcm
{

namespace utility
{

class FileManager
{
public:
  static bool CreateFile(std::ofstream & ofs, std::string file_path);
  static bool InitDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path);
  static bool RemoveDirectory(std::string directory_path);
  static bool RemoveFile(std::string file_name);

};

}  // namespace utility

}  // namespace dcm

#endif  // FILE_MANAGER_HPP_
