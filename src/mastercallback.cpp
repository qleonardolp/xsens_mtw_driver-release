
#include "mastercallback.h"
#include <ros/ros.h>

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------


XsDeviceSet WirelessMasterCallback::getWirelessMTWs() const
{        
	XsMutexLocker lock(m_mutex);
	return m_connectedMTWs;
}

void WirelessMasterCallback::onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
{
	XsMutexLocker lock(m_mutex);
	switch (newState)
	{
	case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */
		
        //std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
        ROS_INFO("EVENT: MTW Disconnected -> ");
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
		ROS_INFO("EVENT: MTW Rejected -> ");
		m_connectedMTWs.erase(dev);
		break;
	case XCS_PluggedIn:			/*!< Device is connected through a cable. */
		ROS_INFO("EVENT: MTW PluggedIn -> ");
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Wireless:			/*!< Device is connected wirelessly. */
		ROS_INFO("EVENT: MTW Connected -> ");
		m_connectedMTWs.insert(dev);
		break;
	case XCS_File:				/*!< Device is reading from a file. */
		ROS_INFO("EVENT: MTW File -> ");
		m_connectedMTWs.erase(dev);
		break;
	case XCS_Unknown:			/*!< Device is in an unknown state. */
		ROS_INFO("EVENT: MTW Unknown -> ");
		m_connectedMTWs.erase(dev);
		break;
	default:
		ROS_INFO("EVENT: MTW Error -> ");
		m_connectedMTWs.erase(dev);
		break;
	}
}
