// Copyright (c) 2019, qleonardolp
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

#include "findClosestUpdateRate.h"

int findClosestUpdateRate(const XsIntArray & supportedUpdateRates, const int desiredUpdateRate)
{
  if (supportedUpdateRates.empty()) {
    return 0;
  }

  if (supportedUpdateRates.size() == 1) {
    return supportedUpdateRates[0];
  }

  int uRateDist = -1;
  int closestUpdateRate = -1;
  for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end();
    ++itUpRate)
  {
    const int currDist = std::abs(*itUpRate - desiredUpdateRate);

    if ((uRateDist == -1) || (currDist < uRateDist)) {
      uRateDist = currDist;
      closestUpdateRate = *itUpRate;
    }
  }
  return closestUpdateRate;
}
