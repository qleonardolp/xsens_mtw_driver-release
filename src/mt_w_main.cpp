/*	Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/////////////////////////////////////////////////////////////////////////
// Adaptation by Leonardo Felipe L. S. dos Santos, 2019 (@qleonardolp) //
/////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "mastercallback.h"
#include "mtwcallback.h"
#include "findClosestUpdateRate.h"

#include <xsensdeviceapi.h> // The Xsens device API header 
#include "xstypes.h"
#include "conio.h"			// For non ANSI _kbhit() and _getch()

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

#include <xsens/xsmutex.h>

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator << (std::ostream& out, XsPortInfo const & p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", " << "ID: " << p.deviceId().toString().toStdString()
	;
	return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator << (std::ostream& out, XsDevice const & d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}

void handleError(std::string error)
{
	ROS_ERROR("%s", error.c_str());
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mt_w_driver");
	ros::NodeHandle node;

	/*

	| MTw  | desiredUpdateRate (max) |
	|------|-------------------------|
	|  1   |           150 Hz        |
	|  2   |           120 Hz        |
	|  4   |           100 Hz        |
	|  6   |            75 Hz        |
	|  12  |            50 Hz        |
	|  18  |            40 Hz        |

	*/

    const int desiredUpdateRate = 120;						// Use 120 Hz update rate for MTw, 150 Hz usually crashes!
	const int desiredRadioChannel = 25;						// Use radio channel 25 for wireless master. (try Channels 11, 15, 20 or 25)

	WirelessMasterCallback wirelessMasterCallback;			// Callback for wireless master
	std::vector<MtwCallback*> mtwCallbacks;					// Callbacks for mtw devices

    ROS_INFO("Creating XsControl object...");
    XsControl* control = XsControl::construct();
    if (control == 0)
    {
        handleError("Failed to construct XsControl instance.");
        return -1;
    }

    try
    {
        XsPortInfoArray detectedDevices = XsScanner::scanPorts();
	    XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();

        ROS_INFO("Scanning for devices...");

        while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
			throw std::runtime_error("No wireless masters found");
		}

        ROS_INFO("Found a device with ID: %s @ port: %s, baudrate: %d", wirelessMasterPort->deviceId().toString().toStdString().c_str(), 
	    wirelessMasterPort->portName().toStdString().c_str(), wirelessMasterPort->baudrate());

        ROS_INFO("Opening port...");

        if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
			std::ostringstream error;
			error << "Failed to open port " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

        ROS_INFO("Getting XsDevice instance for wireless master...");

        XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
        if (wirelessMasterDevice == 0)
		{
			std::ostringstream error;
			error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

        ROS_INFO("XsDevice instance for master created");
        ROS_INFO("Setting config mode...");

        if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

        ROS_INFO("Attaching callback handler for master...");
		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		ROS_INFO("Getting the list of the supported update rates...");
		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

        ROS_INFO_STREAM("Supported update rates: ");
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

        ROS_INFO_STREAM("Setting update rate to " << newUpdateRate << " Hz...");
		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
			std::ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

        ROS_INFO("Disabling radio channel if previously enabled...");
        if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
				std::ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice;
				throw std::runtime_error(error.str());
			}
		}

        ROS_INFO_STREAM("Setting radio channel to " << desiredRadioChannel << " and enabling radio...");
         if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
			std::ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

        ROS_INFO("Waiting for MTW to wirelessly connect...");
        bool waitForConnections = true;
	    bool interruption = false;
	    size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();

	    do
	    {
	    	XsTime::msleep(100);
	    	while (true)
	    	{
	    		size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
	    		if (nextCount != connectedMTWCount)
	    		{
	    			ROS_INFO_STREAM("Number of connected MTWs: " << (int)nextCount << ". Press 'y' to start measurement or 'q' to end node.");
	    			connectedMTWCount = nextCount;
	    		}
	    		else
	    		{
	    			break;
	    		}
	    	}
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

	    }while (waitForConnections && ros::ok());

        if (interruption)
        {
			wirelessMasterDevice->gotoConfig();
			wirelessMasterDevice->disableRadio();
            throw std::runtime_error("\naborting\n");
        }

        

        ROS_INFO("Getting XsDevice instances for all MTWs...");
	    XsDeviceIdArray allDeviceIds = control->deviceIds();
        XsDeviceIdArray mtwDeviceIds;
	    for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
	    {
	    	if (i->isMtw())
	    	{
	    		mtwDeviceIds.push_back(*i);
	    	}
	    }

        XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices.push_back(mtwDevice);
			}
			else
			{
				throw std::runtime_error("Failed to create an MTW XsDevice instance");
			}
		}

        ROS_INFO("Attaching callback handlers to MTws...");
        mtwCallbacks.resize(mtwDevices.size());
		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
			mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
		}

		// This block has no apparent effect on the measurements
		//////////////////////////////////////////////////////////////////

		XsOutputMode desiredOutput = XOM_Calibrated & XOM_Orientation;
		XsOutputSettings desiredSettings = XOS_Timestamp_PacketCounter & XOS_OrientationMode_Matrix & XOS_CalibratedMode_AccGyrOnly;

		ROS_INFO("Setting the desired OutputMode and OutputSettings... ");

		for (int i = 0; i < mtwDevices.size(); i++)
		{
			if( !control->device(mtwDeviceIds[i])->setOutputMode(desiredOutput) )
				ROS_WARN("Failed to set the desired OutputMode");

			if( !control->device(mtwDeviceIds[i])->setOutputSettings(desiredSettings) )
				ROS_WARN("Failed to set the desired OutputSettings");

			if(control->device(mtwDeviceIds[i])->isInLegacyMode())
			{
				ROS_INFO_STREAM(mtwDeviceIds[i].toString() << " is in LegacyMode");

				XsOutputMode mtw_output_mode = control->device(mtwDeviceIds[i])->outputMode();
				XsOutputSettings mtw_output_settings = control->device(mtwDeviceIds[i])->outputSettings();

				ROS_INFO_STREAM( mtw_output_mode << " " << mtw_output_settings );

			}
		}
		///////////////////////////////////////////////////////////////////

		ROS_INFO("Starting measurement...");
        if (!wirelessMasterDevice->gotoMeasurement())
		{
			std::ostringstream error;
			error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

        ROS_INFO("Publish loop starting...");

		ros::AsyncSpinner spinner( mtwDevices.size() );			// threaded spinner, one thread per MTw
		ros::V_Publisher imu_pubs;
        ros::Rate loop_rate = 2000;
		ros::Time beginning;

		/* 
		Awinda Station Hardware throttles at the desiredUpdateRate
		rostopic hz shows around 120 Hz for each MTw
		Although, echoing the /free_acc topic has less latency with higher loop_rate values
		*/ 

        for (int i = 0; i < (int)mtwDevices.size(); ++i)
        {
            std::string mtwID = mtwDeviceIds[i].toString().toStdString();

			ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("imu_" + mtwID, 10);
			imu_pubs.push_back(imu_pub);
        }
        
		ROS_INFO("Publishers started, press 's' to stop!");

		spinner.start();

		beginning = ros::Time::now();

        while (ros::ok())
        {
			if (!_kbhit())
			{

				for (size_t i = 0; i < mtwCallbacks.size(); ++i)
            	{
            	    if (mtwCallbacks[i]->dataAvailable())
            	    {
            	        XsDataPacket const * packet = mtwCallbacks[i]->getOldestPacket();

						if (packet->containsCalibratedData())
						{
							sensor_msgs::Imu imu_msg;

							imu_msg.header.frame_id = "imu_" + mtwDeviceIds[i].toString().toStdString();

							imu_msg.linear_acceleration.x = packet->calibratedAcceleration().value(0);	// [m/s²]
							imu_msg.linear_acceleration.y = packet->calibratedAcceleration().value(1);	// [m/s²]
							imu_msg.linear_acceleration.z = packet->calibratedAcceleration().value(2);	// [m/s²]
							imu_msg.linear_acceleration_covariance[0] = -1; 

							imu_msg.angular_velocity.x = packet->calibratedGyroscopeData().value(0);	// [rad/s]
							imu_msg.angular_velocity.y = packet->calibratedGyroscopeData().value(1);	// [rad/s]
							imu_msg.angular_velocity.z = packet->calibratedGyroscopeData().value(2);	// [rad/s]
							imu_msg.angular_velocity_covariance[0] = -1;

							imu_msg.orientation.x = packet->orientationQuaternion().x();				// unit quaternion
							imu_msg.orientation.y = packet->orientationQuaternion().y();				// unit quaternion
							imu_msg.orientation.z = packet->orientationQuaternion().z();				// unit quaternion
							imu_msg.orientation.w = packet->orientationQuaternion().w();				// unit quaternion
							imu_msg.orientation_covariance[0] = -1;

							imu_msg.header.stamp.fromSec(ros::Time::now().toSec() - beginning.toSec());	// [nsecs]

							imu_pubs[i].publish(imu_msg);
						}
						
            	        mtwCallbacks[i]->deleteOldestPacket();
            	    }
            	}

				beginning = ros::Time::now();	// time reference to get the messages delay after each publication loop

			}
			else
			{
				if((char)_getch() == 's')
					break;
			}
            //ros::spinOnce();
            loop_rate.sleep();
        }

		ROS_INFO("Setting config mode...");
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		ROS_INFO("Disabling radio... ");
		if (!wirelessMasterDevice->disableRadio())
		{
			std::ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}

	ROS_INFO("Closing XsControl...");
	control->close();

	ROS_INFO("Deleting mtw callbacks...");
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
	{
		delete (*i);
	}

	ROS_INFO("Successful exit.");
	return 0;
}