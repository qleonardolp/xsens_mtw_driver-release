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

#include "mtwcallback.h"

#include <xsens/xsdevice.h>
#include <xsens/xsdatapacket.h>

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------

MtwCallback::MtwCallback(int mtwIndex, XsDevice * device, size_t maxBufferSize)
:m_mtwIndex(mtwIndex)
  , m_device(device)
  , m_maxBufferSize(maxBufferSize)
{}

bool MtwCallback::dataAvailable() const
{
  XsMutexLocker lock(m_mutex);
  return !m_packetBuffer.empty();
}

XsDataPacket const * MtwCallback::getOldestPacket() const
{
  XsMutexLocker lock(m_mutex);
  XsDataPacket const * packet = &m_packetBuffer.front();
  return packet;
}

// Returns empty packet on timeout
RosXsDataPacket MtwCallback::next(const std::chrono::milliseconds & timeout)
{
  RosXsDataPacket packet;

  std::unique_lock<std::mutex> lock(m_stdmutex);

  if (m_condition.wait_for(lock, timeout, [&] {return !m_rosBuffer.empty();})) {
    assert(!m_rosBuffer.empty());

    packet = m_rosBuffer.front();
    m_rosBuffer.pop_front();
  }

  return packet;
}

void MtwCallback::deleteOldestPacket()
{
  XsMutexLocker lock(m_mutex);
  m_packetBuffer.pop_front();
}

int MtwCallback::getMtwIndex() const
{
  return m_mtwIndex;
}

XsDevice const & MtwCallback::device() const
{
  assert(m_device != 0);
  return *m_device;
}

void MtwCallback::onLiveDataAvailable(XsDevice *, const XsDataPacket * packet)
{

  XsMutexLocker lock(m_mutex);

  m_packetBuffer.push_back(*packet);
  if (m_packetBuffer.size() > 300) {
    std::cout << std::endl;
    deleteOldestPacket();
  }
}
