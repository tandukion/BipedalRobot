/**
 * @file XsensIMU.c
 * based on Xsens Technologies B.V. example
 *
 */

 #include <XsensIMU.h>

 /*************************************************************/
 /**               FUNCTION FOR XSENS IMU                    **/
 /*************************************************************/
 /********************************************************************/
 // init_IMU
 // Desc  : initialization of Xsens IMU by opening port with baudrate
 // Input : - portName : port of the Xsens IMU (default: /dev/ttyUSB0)
 //         - baudRate
 // Output: - *device : updated DeviceClass for the Xsens
 //         - *mtPort : updated Port name and baudrate
 /********************************************************************/
 void init_IMU(DeviceClass *device, XsPortInfo *mtPort, char *portName, int baudRate){

   XsPortInfoArray portInfoArray;

   XsPortInfo portInfo(portName, XsBaud::numericToRate(baudRate));
   portInfoArray.push_back(portInfo);

   // Use the first detected device
   *mtPort = portInfoArray.at(0);

   // Open the port with the detected device
   //std::cout << "Opening port..." << std::endl;
   device->openPort(*mtPort);
 }

 /********************************************************************/
 // config_IMU
 // Desc  : Configure the Xsens output mode (check manual or library)
 //         Enter Config State then return to Measurement State
 // Input : - outputMode
 //         - outputSettings
 // Output: - *device : updated DeviceClass for the Xsens
 //         - *mtPort : updated Port name and baudrate
 /********************************************************************/
 void config_IMU(DeviceClass *device, XsPortInfo *mtPort, XsOutputMode outputMode, XsOutputSettings outputSettings){

   // Put the device in configuration mode
   std::cout << "Putting device into configuration mode..." << std::endl;
   device->gotoConfig();

   // Request the device Id to check the device type
   mtPort->setDeviceId(device->getDeviceId());

   // Print information about detected MTi / MTx / MTmk4 device
   std::cout << "Found a device with id: " << mtPort->deviceId().toString().toStdString() << " @ port: " << mtPort->portName().toStdString() << ", baudrate: " << mtPort->baudrate() << std::endl;
   std::cout << "Device: " << device->getProductCode().toStdString() << " opened." << std::endl;

   // Configure the device. Note the differences between MTix and MTmk4
   //std::cout << "Configuring the device..." << std::endl;
   if (mtPort->deviceId().isMt9c() || mtPort->deviceId().isLegacyMtig())
     {
       /* Default Mode configuration */
       // XsOutputMode outputMode = XOM_Orientation; // output orientation data
       // XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

       /* set the device configuration with outputMode --> MTData */
       //device->setDeviceMode(outputMode, outputSettings);
     }
   else if (mtPort->deviceId().isMtMk4() || mtPort->deviceId().isFmt_X000())
     {

 			/* set the device configuration with outputMode --> MTData */
 			/*
 			XsOutputConfigurationArray configArray;
       XsOutputConfiguration none(XDI_None, 100);
       configArray.push_back(none);
       device->setOutputConfiguration(configArray);
       device->setDeviceMode(outputMode, outputSettings);
 			*/

 			/* set with OutputConfiguration --> MTData2 */
 			XsOutputConfigurationArray configArray;

 			if (outputMode == XOM_Orientation){
 				XsOutputConfiguration quat(XDI_Quaternion, 100);
 				configArray.push_back(quat);
 			}

 			else if (outputMode == XOM_Calibrated){
 				XsOutputConfiguration acc(XDI_Acceleration, 100);
 				XsOutputConfiguration gyr(XDI_RateOfTurn, 100);
 				configArray.push_back(acc);
 				configArray.push_back(gyr);
 			}

 			XsOutputConfiguration sampletime(XDI_SampleTimeFine, 100);
 			configArray.push_back(sampletime);

 			device->setOutputConfiguration(configArray);
     }

   // Put the device in measurement mode
   //std::cout << "Putting device into measurement mode..." << std::endl;
   device->gotoMeasurement();

 }

 void setOutput_IMU();

 /********************************************************************/
 // measure_IMU
 // Desc  : Measurement State, getting the data from Xsens
 // Input : - device : updated DeviceClass for the Xsens
 //         - mtPort : updated Port name and baudrate
 //         - outputMode
 //         - outputSettings
 // Output: - quaternion
 //         - euler
 /********************************************************************/

 void measure_IMU(DeviceClass *device, XsPortInfo *mtPort, XsOutputMode outputMode, XsOutputSettings outputSettings, XsQuaternion *quaternion, XsEuler *euler, XsCalibratedData *calData, unsigned int *sample_time){

   XsByteArray data;
   XsMessageArray msgs;
   bool foundAck = false;

   do {
     device->readDataToBuffer(data);
     device->processBufferedData(data, msgs);

     for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
       {
 				// Retrieve a packet
 				XsDataPacket packet;
 				if ((*it).getMessageId() == XMID_MtData) {
 					//printf("MTData\n");
 	  			LegacyDataPacket lpacket(1, false);
 	  			lpacket.setMessage((*it));
 	  			lpacket.setXbusSystem(false);
 	  			lpacket.setDeviceId(mtPort->deviceId(), 0);
 	  			lpacket.setDataFormat(outputMode, outputSettings,0);//lint !e534
 	  			XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
 	  			foundAck = true;
 				}
 				else if ((*it).getMessageId() == XMID_MtData2) {
 					//printf("MTData2\n");
 	  			packet.setMessage((*it));
 	  			packet.setDeviceId(mtPort->deviceId());
 	  			foundAck = true;
 				}

 				/*
 				if (packet.containsOrientation())
 					printf("contain Orientation\n");
 				if (packet.containsCalibratedData())
 					printf("contain calibrated Data\n");
 				if (packet.containsCalibratedAcceleration())
 					printf("contain calibrated Acc\n");
 				if (packet.containsCalibratedGyroscopeData())
 					printf("contain calibrated Gyroscope\n");
 				if (packet.containsCalibratedMagneticField())
 					printf("contain calibrated Magnetometer\n");
 				if (packet.containsSampleTimeCoarse())
 				  printf("contain SampleTimeCoarse\n");
 				*/
 				// Get the quaternion data
 					*quaternion = packet.orientationQuaternion();
 					// Convert packet to euler
 					*euler = packet.orientationEuler();
 					// Get calibrated Data
 					*calData = packet.calibratedData();

 					// Get Sample Time Coarse
 					*sample_time = packet.sampleTimeFine();
 		 	}
 	} while (!foundAck);

 }

/*! \
*/
 void test_IMU(DeviceClass *device, XsPortInfo *mtPort, XsQuaternion *quaternion, XsEuler *euler, XsCalibratedData *calData, unsigned int *sample_time){
 	std::cout << "Looping Printing by accessing function each time.." << std::endl;
 	int mode;
 	printf("Output Mode <1:Orientation><2:calibratedData> : ");
 	scanf ("%d",&mode);
 	if (mode==1){
 		XsOutputMode outputMode = XOM_Orientation;
 		XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion;
 		config_IMU(&(*device),&(*mtPort), outputMode, outputSettings);
 		while(1)
 		  {
 		  measure_IMU(&(*device),&(*mtPort), outputMode, outputSettings, &(*quaternion),&(*euler),&(*calData),&(*sample_time));
 			//printf("\n");
 			std::cout  << "\r"
 			    << "W:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion->w()
 			    << ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion->x()
 			    << ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion->y()
 			    << ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion->z()
 		    ;
 		  std::cout << ",\tRoll:" << std::setw(7) << std::fixed << std::setprecision(2) << euler->roll()
 			    << ",Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << euler->pitch()
 			    << ",Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << euler->yaw()
 			   ;
 		  }
 	}
 	else if(mode==2){
 		XsOutputMode outputMode = XOM_Calibrated;
 		XsOutputSettings outputSettings = XOS_CalibratedMode_All;

 		double acc;
 		config_IMU(&(*device),&(*mtPort), outputMode, outputSettings);
 		while(1)
 		  {
 		  measure_IMU(&(*device),&(*mtPort), outputMode, outputSettings, &(*quaternion),&(*euler),&(*calData),&(*sample_time));
 		  std::cout  << "\r"
 			   	<< "AccX:" << std::setw(7) << std::fixed << std::setprecision(4) << calData->m_acc.value(0)
 			    << ", AccY:" << std::setw(7) << std::fixed << std::setprecision(4) << calData->m_acc.value(1)
 			    << ", AccZ:" << std::setw(7) << std::fixed << std::setprecision(4) << calData->m_acc.value(2)

 			   	<< ",   GyrX:" << std::setw(7) << std::fixed << std::setprecision(4) << calData->m_gyr.value(0)
 			    << ", GyrY:" << std::setw(7) << std::fixed << std::setprecision(4) << calData->m_gyr.value(1)
 			    << ", GyrZ:" << std::setw(7) << std::fixed << std::setprecision(4) << calData->m_gyr.value(2)
 			;
 		  }

 			printf("\n");
 	}
 }
