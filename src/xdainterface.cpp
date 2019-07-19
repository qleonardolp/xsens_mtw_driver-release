
//  ==> COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE <==
//  WARNING: COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
//  THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
//  FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
//  TO AN END USER LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
//  LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
//  INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
//  DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
//  IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
//  USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
//  XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
//  OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
//  COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
//  
//  THIS SOFTWARE CAN CONTAIN OPEN SOURCE COMPONENTS WHICH CAN BE SUBJECT TO 
//  THE FOLLOWING GENERAL PUBLIC LICENSES:
//  ==> Qt GNU LGPL version 3: http://doc.qt.io/qt-5/lgpl.html <==
//  ==> LAPACK BSD License:  http://www.netlib.org/lapack/LICENSE.txt <==
//  ==> StackWalker 3-Clause BSD License: https://github.com/JochenKalmbach/StackWalker/blob/master/LICENSE <==
//  ==> Icon Creative Commons 3.0: https://creativecommons.org/licenses/by/3.0/legalcode <==
//  

#include <assert.h>
#include "conio.h"
#include "xdainterface.h"
#include "findClosestUpdateRate.h"

//#include <xscontroller/idfetchhelpers.h>
#include <xsens/xsscanner.h>

#include <xsens/xscontrol.h>
#include <xsens/xsdevice.h>
#include <xsens/xsdeviceptr.h>
/*
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsdeviceptr.h>
*/
#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
//#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
//#include "messagepublishers/velocityincrementpublisher.h"

#define XS_DEFAULT_BAUDRATE (115200)

XdaInterface::XdaInterface()
	: m_device(nullptr)
{
	ROS_INFO("Creating XsControl object...");
	m_control = XsControl::construct();
	assert(m_control != 0);
}

XdaInterface::~XdaInterface()
{
	close();
	m_control->destruct();
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	//bool newDataAvailable = false;
	int nodeCounter = 0;
	for (int i = 0; i < m_mtwCallbacks.size(); i++)
	{
		if (m_mtwCallbacks[i]->dataAvailable())
		{
			//newDataAvailable = true;
			//XsDataPacket const *packet = m_mtwCallbacks[i]->getOldestPacket();
			RosXsDataPacket rosPacket = m_mtwCallbacks[i]->next(timeout);

			//rosPacket.second = packet;

			if (!rosPacket.second.empty())
			{
				for (auto &cb : m_callbacks)
				{
					nodeCounter++;
					cb->operator()(rosPacket.second, rosPacket.first);
					if((nodeCounter % topicsperMTw) == 0) 
					{
						break;
					}
				}
			}
			else
			{
				ROS_WARN("rosPacket.second is empty!");
			}
			
		}
		else
		{
			ROS_WARN("m_mtwCallbacks[i] data Unavailable");
		}
		
	}
	/*
	RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);
	if (!rosPacket.second.empty())
	{
		for (auto &cb : m_callbacks)
		{
			cb->operator()(rosPacket.second, rosPacket.first);			// operator()(const XsDataPacket &packet, ros::Time timestamp)
		}
	}
	*/
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
	m_callbacks.push_back(cb);

	// preciso criar uma lista ou vetor de m_callbacks, um pra cada MTw pois um apenas é a lista de topicos para um sensor
	/*
	std::vector<PacketCallback *> callbackMtw;
	for (int i = 0; i < mtwDevices.size(); i++)
	{
		//callbackMtw[i] = m_callbacks.push_back(cb);
	}
	*/
}

void XdaInterface::registerPublishers(ros::NodeHandle &node)
{
	bool should_publish = true;		// <- true ?

	ROS_INFO("Creating Publishers...");

	for (int i = 0; i < mtwDeviceIds.size(); i++)
	{
		topicsperMTw = 0;

		if (ros::param::get("~pub_imu", should_publish) && should_publish)
		{
			registerCallback(new ImuPublisher(node, mtwDeviceIds[i].toString().toStdString()));
			topicsperMTw++;
			ROS_INFO_STREAM(mtwDeviceIds[i].toString().toStdString() + " imuPublisher created");
		}
		if (ros::param::get("~pub_quaternion", should_publish) && should_publish)
		{
			registerCallback(new OrientationPublisher(node, mtwDeviceIds[i].toString().toStdString()));
			topicsperMTw++;
		}
		if (ros::param::get("~pub_acceleration", should_publish) && should_publish)
		{
			registerCallback(new AccelerationPublisher(node, mtwDeviceIds[i].toString().toStdString()));
			topicsperMTw++;
		}
		if (ros::param::get("~pub_angular_velocity", should_publish) && should_publish)
		{
			registerCallback(new AngularVelocityPublisher(node, mtwDeviceIds[i].toString().toStdString()));
			topicsperMTw++;
		}
		if (ros::param::get("~pub_twist", should_publish) && should_publish)
		{
			registerCallback(new TwistPublisher(node, mtwDeviceIds[i].toString().toStdString()));
			topicsperMTw++;
		}
		if (ros::param::get("~pub_free_acceleration", should_publish) && should_publish)
		{
			registerCallback(new FreeAccelerationPublisher(node, mtwDeviceIds[i].toString().toStdString()));
			topicsperMTw++;
		}
		if (ros::param::getCached("~pub_transform", should_publish) && should_publish)
		{
			registerCallback(new TransformPublisher(node, mtwDeviceIds[i].toString().toStdString()));			// INVESTIGAR
			topicsperMTw++;
		}
		if (ros::param::get("~pub_sampletime", should_publish) && should_publish)
		{
			registerCallback(new TimeReferencePublisher(node, mtwDeviceIds[i].toString().toStdString()));
			topicsperMTw++;
		}
		if (ros::param::get("~pub_temperature", should_publish) && should_publish)
		{
			registerCallback(new TemperaturePublisher(node, mtwDeviceIds[i].toString().toStdString()));
			topicsperMTw++;
		}
	}
	
	ROS_INFO("topics per MTw: %d", topicsperMTw);
	ROS_INFO("Press 'y' to proceed to the publishing spin");
	bool waitPub = true;
	while (waitPub)
	{
		if (ros::ok())
		{
			if (_kbhit())
			{
				char keypressed = (char)_getch();
				if(keypressed == 'y')
				waitPub = false;	
			}
		}
	} //do NOT works properly here

}

bool XdaInterface::connectDevice()
{	
	XsPortInfo mtPort;							// DEPRECATED, não seria m_port?

	XsPortInfoArray detectedDevices = XsScanner::scanPorts();
	XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();

	if (ros::param::has("~port"))
	{
		std::string port_name;					//rever esse if depois
		int baudrate = XS_DEFAULT_BAUDRATE;

		ros::param::get("~port", port_name);
		ros::param::get("~baudrate", baudrate);

		mtPort = XsPortInfo(port_name, XsBaud_numericToRate(baudrate));
	}
	else
	{
		ROS_INFO("Scanning for devices...");
		
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}

		if (wirelessMasterPort == detectedDevices.end())
		{
			return handleError("No wireless masters found");
		}
		
		ROS_INFO("Found a Wireless Master @ %s", wirelessMasterPort->portName().toStdString().c_str());

		/*
		for (auto const &portInfo : detectedDevices)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) //  ADDED portInfo.deviceId().isMtw(), deprecated...
			{
				mtPort = portInfo;
				break;
			}
			if (portInfo.deviceId().isWirelessMaster())
			{
				mtPort = portInfo;
				break;
			}
			
		}
		*/
	}


	//if (mtPort.empty())
	//	return handleError("No MTi/ Awinda Master device found");

	std::string deviceId;
	if (ros::param::get("~device_id", deviceId))
	{
		//if (mtPort.deviceId().toString().c_str() != deviceId)
		if (wirelessMasterPort->deviceId().toString().c_str() != deviceId)
			return handleError(std::string("Device with ID: %s not found") + deviceId);
	}
	
	ROS_INFO("Found a device with ID: %s @ port: %s, baudrate: %d", 
	wirelessMasterPort->deviceId().toString().toStdString().c_str(), 
	wirelessMasterPort->portName().toStdString().c_str(), 
	wirelessMasterPort->baudrate());
	/*
	ROS_INFO("Found a Wireless Master @ %s, ID: %s, baudrate: %d", 
	mtPort.portName().toStdString().c_str(), 
	mtPort.deviceId().toString().toStdString().c_str(), 
	mtPort.baudrate());
	*/
	
	// FUNCIONA ATÉ AQUI... AINDA...
		// teste:
	//mtPort = *wirelessMasterPort;
	//mtPort = XsPortInfo(wirelessMasterPort->portName().toStdString(),XsBaud_numericToRate(2000000));

	ROS_INFO("Opening port...");
	//if (!m_control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
	if (!m_control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		return handleError("Could not open port or Master"); 	// DA ERRO AQUI

	// m_master->setMasterDevice( m_control->device(mtPort.deviceId()) );		// promove Segmentation fault (core dumped)

	m_port = *wirelessMasterPort;

	ROS_INFO("Getting XsDevice instance for wireless master...");
	XsDevicePtr wirelessMasterDevice = m_control->device(wirelessMasterPort->deviceId());
	
	if (wirelessMasterDevice == 0){
		ROS_ERROR("Failed to construct XsDevice instance: %s", wirelessMasterPort->portName().toStdString().c_str());
		return false;
	}
	ROS_INFO("XsDevice instance for master created");

	m_device = wirelessMasterDevice;
	ROS_INFO("Attaching callback handler for master...");
	m_device->addCallbackHandler(&m_wirelessMasterCallback);
	ROS_INFO("Wireless Master Callback attached");

	/*

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	ROS_INFO("Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());

	// iihhh rapaz:
	m_device->addCallbackHandler(&m_xdaCallback);

	*/

	return true;
}

bool XdaInterface::prepare()
{
	const int desiredUpdateRate = 100;	// Use 100 Hz update rate for MTWs
	const int desiredRadioChannel = 25;	// Use radio channel 25 for wireless master.

	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	const XsIntArray supportedUpdateRates = m_device->supportedUpdateRates();

	const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);
	if (!m_device->setUpdateRate(newUpdateRate))
		return handleError("Failed to set update rate");

	ROS_INFO("Disabling radio channel if previously enabled...");
	if (m_device->isRadioEnabled())
	{
		if (!m_device->disableRadio())
		{
			handleError("Failed to disable radio channel");
		}
	}

	ROS_INFO("Setting radio channel to %d", desiredRadioChannel);
	if(!m_device->enableRadio(desiredRadioChannel))
		return handleError("Failed to set radio channel");

	ROS_INFO("Waiting for MTw to wirelessly connect...");
	bool waitForConnections = true;
	bool interruption = false;
	size_t connectedMTWCount = m_wirelessMasterCallback.getWirelessMTWs().size();

	do
	{
		XsTime::msleep(100);
		while (true)
		{
			size_t nextCount = m_wirelessMasterCallback.getWirelessMTWs().size();
			if (nextCount != connectedMTWCount)
			{
				//ROS_INFO("Number of connected MTWs: %d. Press 'y' to start measurement.", (int)nextCount);
				//ROS_INFO("Press 'q' to QUIT PROPERLY this node, wait for the 'Devices closed' message and then Ctrl+C");
				ROS_INFO_STREAM("Number of connected MTWs: " << (int)nextCount << ". Press 'y' to start measurement or 'q' to end node.");
				connectedMTWCount = nextCount;
			}
			else
			{
				break;
			}
		}
		if (ros::ok())
		{
			if (_kbhit())
			{
				char keypressed = (char)_getch();
				if(keypressed == 'y')
					waitForConnections = false;
				if(keypressed == 'q')
				{
					interruption = true;
					waitForConnections = false;
				}	
			}
		} //works properly here
	}
	while (waitForConnections);

	if(interruption)
	{
		close();
		return true;
	}

	// read EMTS and device config stored in .mtb file header.
	/*
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");
	*/

	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	ROS_INFO("Getting XsDevice instances for all MTWs...");
	allDeviceIds = m_control->deviceIds();
	
	for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
	{
		if (i->isMtw())
		{
			mtwDeviceIds.push_back(*i);
		}
	}
	
	for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
	{
		XsDevicePtr mtwDevice = m_control->device(*i);
		if (mtwDevice != 0)
		{
			mtwDevices.push_back(mtwDevice);
		}
		else
		{
			return handleError("Failed to create an MTw XsDevice instance");
		}
	}

	ROS_INFO("Attaching callback handlers to MTws...");

	m_mtwCallbacks.resize(mtwDevices.size());
	for (int i = 0; i < (int)mtwDevices.size(); i++)
	{
		m_mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
		mtwDevices[i]->addCallbackHandler(m_mtwCallbacks[i]);
	}
	
	
	std::string log_file;
	if (ros::param::get("~log_file", log_file))
	{
		if (m_device->createLogFile(log_file) != XRV_OK)
			return handleError(std::string("Failed to create a log file! (%s)") + log_file);
		else
			ROS_INFO("Created a log file: %s", log_file.c_str());

		if (!m_device->startRecording())
			return handleError("Could not start recording");
	}

	ROS_INFO("MTws callback handlers attached");
	return true;
}

void XdaInterface::close()
{
	ROS_INFO("Closing Device...");
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_wirelessMasterCallback);
		ROS_INFO("Master callback handler removed");
	}
	m_control->closePort(m_port);
	ROS_INFO("Devices closed");
}

bool XdaInterface::handleError(std::string error)
{
	ROS_ERROR("%s", error.c_str());
	return false;
}
