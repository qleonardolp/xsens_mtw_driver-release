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

#ifndef MASTERCALLBACK_H
#define MASTERCALLBACK_H

#include <xsensdeviceapi.h> // The Xsens device API header
#include <xstypes.h>
#include <set>


typedef std::set < XsDevice * > XsDeviceSet;

class WirelessMasterCallback: public XsCallback
{
public:
  XsDeviceSet getWirelessMTWs() const;

protected:
  virtual void onConnectivityChanged(XsDevice * dev, XsConnectivityState newState);

private:
  mutable XsMutex m_mutex;
  XsDeviceSet m_connectedMTWs;
};

#endif  // MASTERCALLBACK_H
