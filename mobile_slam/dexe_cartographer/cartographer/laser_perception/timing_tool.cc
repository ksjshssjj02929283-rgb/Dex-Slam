// Copyright (c) 2021 Geekplus Inc
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

#include "cartographer/laser_perception/timing_tool.h"

TimingTool::TimingTool()
{
}

void TimingTool::timingStart()
{
  startTime = std::chrono::system_clock::now();
}

void TimingTool::timingStop()
{
  stopTime = std::chrono::system_clock::now();
}

double TimingTool::timeCost()
{
  auto echoTime =
    std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime).count() / 1e3;
  return echoTime;
}
