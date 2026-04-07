// Copyright (c) 2026 Dexforce Inc
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

#include <string>

#include "cartographer/utility/include/file_manager.hpp"

namespace cartographer
{

namespace utility
{
/*

*/
bool FileManager::CreateFile(std::ofstream & ofs, std::string file_path)
{
  ofs.close();
  boost::filesystem::remove(file_path.c_str());

  ofs.open(file_path.c_str(), std::ios::out);
  if (!ofs) {
    LOG(WARNING) << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
    return false;
  }

  return true;
}

bool FileManager::InitDirectory(std::string directory_path, std::string use_for)
{
  if (boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::remove_all(directory_path);
  }

  return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path, std::string use_for)
{
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(WARNING) << "无法创建文件夹: " << std::endl << directory_path << std::endl << std::endl;
    return false;
  }

  std::cout << use_for << "存放地址：" << std::endl << directory_path << std::endl << std::endl;
  return true;
}

bool FileManager::CreateDirectory(std::string directory_path)
{
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(WARNING) << "无法创建文件夹: " << std::endl << directory_path << std::endl << std::endl;
    return false;
  }

  return true;
}

bool FileManager::RemoveDirectory(std::string directory_path)
{
  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(WARNING) << directory_path << " is not exists." << std::endl;
    return true;
  }
  boost::filesystem::remove_all(directory_path);
  if (!boost::filesystem::is_directory(directory_path)) {
    LOG(INFO) << "remove directory " << directory_path << " success." << std::endl;
    return true;
  } else {
    LOG(ERROR) << "cannot remove diretory : " << directory_path << std::endl;
    return false;
  }
}

bool FileManager::RemoveFile(std::string file_name)
{
  if (!boost::filesystem::exists(file_name)) {
    LOG(WARNING) << file_name << " is not exists." << std::endl;
    return true;
  }
  boost::filesystem::remove(file_name);
  if (!boost::filesystem::exists(file_name)) {
    LOG(INFO) << "remove file " << file_name << " success." << std::endl;
    return true;
  } else {
    LOG(ERROR) << "cannot remove diretory : " << file_name << std::endl;
    return false;
  }
}

}  // namespace utility

}  // namespace cartographer
