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

#ifndef MTWCALLBACK_H
#define MTWCALLBACK_H

#include <rclcpp/rclcpp.hpp>
#include <xsens/xscallback.h>
#include <xsens/xsmutex.h>
#include <mutex>
#include <condition_variable>
#include <list>

struct XsDataPacket;
struct XsDevice;

typedef std::pair < rclcpp::Time, XsDataPacket > RosXsDataPacket; //

class MtwCallback: public XsCallback
{
public:
  MtwCallback(int mtwIndex, XsDevice * device, size_t maxBufferSize = 5);

  bool dataAvailable() const;

  XsDataPacket const * getOldestPacket() const;

  RosXsDataPacket next(const std::chrono::milliseconds & timeout);

  void deleteOldestPacket();

  int getMtwIndex() const;

  XsDevice const & device() const;

protected:
  virtual void onLiveDataAvailable(XsDevice *, const XsDataPacket * packet);

private:
  mutable XsMutex m_mutex;
  std::list < XsDataPacket > m_packetBuffer;
  int m_mtwIndex;
  XsDevice * m_device;

  std::mutex m_stdmutex;
  std::condition_variable m_condition;
  std::list < RosXsDataPacket > m_rosBuffer;
  size_t m_maxBufferSize;
};

#endif  // MTWCALLBACK_H
