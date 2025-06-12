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

#include "mastercallback.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>


XsDeviceSet WirelessMasterCallback::getWirelessMTWs() const
{
  XsMutexLocker lock(m_mutex);
  return m_connectedMTWs;
}

void WirelessMasterCallback::onConnectivityChanged(XsDevice * dev, XsConnectivityState newState)
{
  XsMutexLocker lock(m_mutex);
  switch (newState) {
    case XCS_Disconnected:
      m_connectedMTWs.erase(dev);
      break;
    case XCS_Rejected:
      m_connectedMTWs.erase(dev);
      break;
    case XCS_PluggedIn:
      m_connectedMTWs.erase(dev);
      break;
    case XCS_Wireless:
      m_connectedMTWs.insert(dev);
      break;
    case XCS_File:
      m_connectedMTWs.erase(dev);
      break;
    case XCS_Unknown:
      m_connectedMTWs.erase(dev);
      break;
    default:
      m_connectedMTWs.erase(dev);
      break;
  }
}
