/**
 * @file XsensIMU.h
 * based on Xsens Technologies B.V. example
 *
 * This file contains the headers for XsensIMU.c
 */

#ifndef XsensIMU_H
#define XsensIMU_H

/* For Xsens IMU */

#include <iostream>
#include <iomanip>

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "deviceclass.h"
#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

/* for IMU */
#define PORTNAME "/dev/ttyUSB0"
#define BAUDRATE 921600

#define DEFAULT_OUTPUT_MODE XOM_Orientation
#define DEFAULT_OUTPUT_SETTINGS XOS_OrientationMode_Quaternion

void init_IMU(DeviceClass *device, XsPortInfo *mtPort, char *portName, int baudRate);
void config_IMU(DeviceClass *device, XsPortInfo *mtPort, XsOutputMode outputMode, XsOutputSettings outputSettings);
void setOutput_IMU();
void measure_IMU(DeviceClass *device, XsPortInfo *mtPort, XsOutputMode outputMode, XsOutputSettings outputSettings,
                  XsQuaternion *quaternion, XsEuler *euler, XsCalibratedData *calData, unsigned int *sample_time);
void test_IMU(DeviceClass *device, XsPortInfo *mtPort, XsQuaternion *quaternion, XsEuler *euler, XsCalibratedData *calData, unsigned int *sample_time);

#endif
