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

#include "myxda.h"
#include <stdio.h>
#include "console.h"
#include <xsens/xscontrol.h>
#include <xsens/xsscanner.h>
#include <xsens/xseuler.h>
#include <xsens/xsmatrix3x3.h>
#include <xsens/xssdidata.h>
#include <xsens/xsportinfoarray.h>
#include <xsens/xsdeviceidarray.h>

/*! \brief MyXda class implementation
	The MyXda class is derived from the XsControl class and indirectly from the CmtCallback class.
	It is thereby able to overload the callback functions defined in CmtCallback
*/

/*! \brief constructor, creates the XsControl object and registers the callbacks
*/
MyXda::MyXda()
	: m_xsControl(XsControl::construct())
	, m_connectCount(0)
	, m_previousDisplayMode(DM_None)
{
	m_xsControl->addCallbackHandler(this);
}

/*! \brief destructor, unregisters the callbacks and cleans up the XsControl object
*/
MyXda::~MyXda()
{
	m_xsControl->removeCallbackHandler(this);
	m_xsControl->destruct();
}

/*! \brief Opens all COM ports with either a station or an MTw conneted
*/
void MyXda::openPorts()
{
	XsPortInfoArray portinfo = XsScanner::scanPorts(XBR_Invalid, 0, true);
	for(XsPortInfoArray::const_iterator i = portinfo.begin(); i != portinfo.end(); ++i)
	{
		printf("Opening port %s @ %d baud, device Id = %s\n", i->portName().toStdString().c_str(), i->baudrate(), i->deviceId().toString().toStdString().c_str());
		xsControl()->openPort(i->portName(), i->baudrate());
	}
}

/*! \brief Repaints the screen to an empty measurement screen
*/
void MyXda::clearMeasurementScreen()
{
	clearScreen();
	printf("The system is in measurement mode (press 'q' to exit)\n");
	printf("Use keys '1' to '4' to switch between display modes\n");
}

/*! \brief Sets the display mode of incoming data
	\param displayMode : The display mode to set
*/
void MyXda::setDisplayMode(MyXda::DisplayMode displayMode)
{
	m_displayMode = displayMode;
}

/*! \brief Prints the current display mode on the screen
*/
void MyXda::printDisplayMode()
{
	gotoXY(0, 3);
	printf("Display mode = ");
	switch(m_displayMode)
	{
		case DM_OrientationEuler:
			printf("Orientation (euler)");
			break;

		case DM_OrientationQuaternion:
			printf("Orientation (quaternion)");
			break;

		case DM_Sdi:
			printf("SDI");
			break;

		case DM_OrientationMatrix:
			printf("Orientation (matrix)");
			break;

		default:
			printf("unknown");
			break;
	}
}

/*! \brief Callback function called if there is new data available from the device
	\param packet : Packet with filter output
*/
void MyXda::onDataAvailable(XsDevice*, const XsDataPacket* packet)
{
	//Get the list of devices for which packet has data
	XsDeviceIdArray ids = xsControl()->deviceIds();

	if(m_previousDisplayMode != m_displayMode)
	{
		clearMeasurementScreen();
		printDisplayMode();
		m_previousDisplayMode = m_displayMode;
	}

	int row = 0;
	for(XsDeviceIdArray::const_iterator i = ids.begin(); i != ids.end(); ++i, row++)
	{
		if (packet->deviceId() == *i)
		{
			if (packet->containsSdiData())
			{
				switch(m_displayMode)
				{
					case DM_OrientationEuler:
					{
						gotoXY(0, 4 + m_connectedDevices[i->toInt()]);
						printf("MTw %s : ", i->toString().toStdString().c_str());

						//Getting Euler angles
						XsEuler oriEuler = packet->orientationEuler();
						printf("roll: % 4.3f\t pitch: % 4.3f\t yaw: % 4.3f              ", oriEuler.roll(), oriEuler.pitch(), oriEuler.yaw());
					} break;

					case DM_OrientationQuaternion:
					{
						gotoXY(0, 4 + m_connectedDevices[i->toInt()]);
						printf("MTw %s : ", i->toString().toStdString().c_str());

						//Getting quaternions
						XsQuaternion oriQuat = packet->orientationQuaternion();
						printf("w: % 3.3f x: % 3.3f y: % 3.3f z: % 3.3f", oriQuat.w(), oriQuat.x(), oriQuat.y(), oriQuat.z());
					} break;

					case DM_Sdi:
					{
						gotoXY(0, 4 + m_connectedDevices[i->toInt()] * 3);
						printf("MTw %s : ", i->toString().toStdString().c_str());

						//Getting SDI data
						XsSdiData sdiData = packet->sdiData();
						gotoXY(0, 5 + m_connectedDevices[i->toInt()] * 3);
						printf("\tOrientation incr. :\t (% 3.3f, % 3.3f, % 3.3f, % 3.3f)\n\tVelocity incr. :\t (% 3.3f, % 3.3f, % 3.3f)",
								sdiData.orientationIncrement()[0], sdiData.orientationIncrement()[1], sdiData.orientationIncrement()[2], sdiData.orientationIncrement()[3],
								sdiData.velocityIncrement()[0], sdiData.velocityIncrement()[1], sdiData.velocityIncrement()[2]);
					} break;

					case DM_OrientationMatrix:
					{
						gotoXY(0, 4 + m_connectedDevices[i->toInt()] * 4);
						printf("MTw %s : ", i->toString().toStdString().c_str());

						gotoXY(0, 5 + m_connectedDevices[i->toInt()] * 4);
						//Getting orientation matrix
						XsMatrix3x3 oriMatrix = packet->orientationMatrix();
						printf("\t| % 3.3f   % 3.3f   % 3.3f |\n", oriMatrix.value(0, 0), oriMatrix.value(0, 1), oriMatrix.value(0, 2));
						printf("\t| % 3.3f   % 3.3f   % 3.3f |\n", oriMatrix.value(1, 0), oriMatrix.value(1, 1), oriMatrix.value(1, 2));
						printf("\t| % 3.3f   % 3.3f   % 3.3f |\n", oriMatrix.value(2, 0), oriMatrix.value(2, 1), oriMatrix.value(2, 2));
					} break;

					default:
					{
					} break;
				}
			}
			break;
		}
	}
}


