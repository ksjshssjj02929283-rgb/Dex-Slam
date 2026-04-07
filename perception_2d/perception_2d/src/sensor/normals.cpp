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
#include "perception_2d/sensor/normals.hpp"

namespace perception_2d
{
namespace sensor
{

Normals TransformNormals(const Normals & normals, const transform::Rigid3f & transform)
{
  Normals transformed_normals;
  for (int i = 0; i < static_cast<int>(normals.size()); i++) {
    transformed_normals.push_back(transform.rotation() * normals[i]);
  }
  return transformed_normals;
}

}  // namespace sensor
}  // namespace perception_2d
