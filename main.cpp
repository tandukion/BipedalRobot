/*
	Bipedal Musculoskeletal Robot Program
	2017 Dwindra Sulistyoutomo
*/

#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include "gpio.h"

#include <stdbool.h>
#include <time.h>
#include <string.h>

/* For Xsens IMU */
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

#include "deviceclass.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <sstream>

#ifdef __GNUC__
#include "conio.h" // for non ANSI _kbhit() and _getch()
#else
#include <conio.h>
#endif

/* for sensors */
#define pin_cs_sensor P9_13 // 0_31=31
#define pin_clk_sensor P9_12 // 1_28=60
#define pin_din_sensor  P9_11 // 0_30=30
#define pin_dout1_sensor P9_14 // 1_18=50
#define pin_dout2_sensor  P9_15 // 1_16=48
#define pin_dout3_sensor  P9_26 // 0_14=14
#define pin_dout4_sensor  P9_27 // 3_19=115

#define NUM_ADC 2
#define NUM_ADC_PORT 8

#define NUM_OF_SENSOR 10
#define NUM_OF_POT_SENSOR 10
#define NUM_OF_PRES_SENSOR 0

/* for valves */
#define pin_spi_sclk P9_21 // 0_3=3
#define pin_spi_mosi P9_30 // 3_16=112
#define pin_spi_cs1  P9_16 // 1_19=51
#define pin_spi_cs2  P9_42 // 0_7 =7
#define pin_spi_cs3  P9_23 // 1_17=49
#define pin_spi_cs4  P9_24 // 0_15 =15

#define pin_spi_other P9_22 // 0_2=2

#define NUM_OF_CHANNELS 16
#define NUM_DAC 2

/* for IMU */
#define PORTNAME "/dev/ttyUSB0"
#define BAUDRATE 921600

#define DEFAULT_OUTPUT_MODE XOM_Orientation
#define DEFAULT_OUTPUT_SETTINGS XOS_OrientationMode_Quaternion

/* for analog input */
#define NUM_OF_AINS 7
#define AIO_NUM 7

/* INDEX PIN for Sensor */
#define PIN_POT1 0     // Pin for Potensiometer 1
#define PIN_POT2 1     // Pin for Potensiometer 1
#define PIN_POT3 2     // Pin for Potensiometer 1
#define PIN_POT4 3     // Pin for Potensiometer 1
#define PIN_POT5 4     // Pin for Potensiometer 1
#define PIN_POT6 5     // Pin for Potensiometer 1
#define PIN_POT7 6     // Pin for Potensiometer 1
#define PIN_POT8 7     // Pin for Potensiometer 1
#define PIN_POT9 0     // Pin for Potensiometer 1
#define PIN_POT10 1     // Pin for Potensiometer 1

#define PIN_PRES_1 1    // Pin for Pressure Sensor 1
#define PIN_PRES_2 2    // Pin for Pressure Sensor 2

/* INDEX PIN for Valve */
#define NUM_OF_MUSCLE 14	// Number of muscle/valve

#define IL_R		0
#define GMAX_R	1
#define VAS_R		2
#define HAM_R		3
#define TA_R 		4
#define SOL_R 	5
#define ADD_R 	6

#define IL_L		16
#define GMAX_L	17
#define VAS_L		18
#define HAM_L		19
#define TA_L 		20
#define SOL_L 	21
#define ADD_L 	22

/* VALUE */
#define MAX_PRESSURE 0.6
#define MAX_SAMPLE_NUM 10000

/*************************************************************/
/**                   GLOBAL VARIABLES                      **/
/*************************************************************/

int SampleNum = 10000;
unsigned long SensorValue[NUM_ADC][NUM_ADC_PORT];


/* Table for muscle valve number and sensor number */
int muscle_sensor [NUM_OF_MUSCLE] = {PIN_PRES_1,PIN_PRES_2};

// list of muscle channel
int muscle_ch [NUM_OF_MUSCLE] = {IL_R,GMAX_R,VAS_R,HAM_R,TA_R,SOL_R,ADD_R,
																 IL_L,GMAX_L,VAS_L,HAM_L,TA_L,SOL_L,ADD_L};

/* Potentiometer reference for zero degree */
int Pot_straight [10] = {2128,2298,2289,3120,2336,2130,1446,1948,2131,2028};

/* Variable for IMU Data */
struct IMUDataArray{
	XsCalibratedData calData[MAX_SAMPLE_NUM];
	XsQuaternion quaternion[MAX_SAMPLE_NUM];
	XsEuler euler[MAX_SAMPLE_NUM];
	unsigned int sample_time[MAX_SAMPLE_NUM];
};

IMUDataArray IMUData;
DeviceClass device;
XsPortInfo mtPort;
XsQuaternion quaternion;
XsEuler euler;
XsCalibratedData calData;
unsigned int sample_time;

/* Xsens IMU Configuration */
char portName[20] = "/dev/ttyUSB0";
//XsOutputMode outputMode = XOM_Orientation; // output orientation data
//XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

/**** analog sensors ****/
PIN analog_pin[NUM_OF_AINS];

FILE* fd[AIO_NUM] = {};
void initAIO(void)
{
	int i;
	for (i = 0; i < AIO_NUM; i++) {
		char filename[64];
		sprintf(filename, "/sys/devices/ocp.3/helper.12/AIN%d", i);
		if ((fd[i] = fopen(filename, "r")) == NULL) {
			perror("cannot open:");
			exit(1);
		}
	}
}
void closeAIO(void)
{
	int i;
	for (i = 0; i < AIO_NUM; i++)
		fclose(fd[i]);
}
uint32_t myAnalogRead(int i)
{
	uint32_t num;
	fseek(fd[i], 0, SEEK_SET);
	fscanf(fd[i], "%d", &num);
	fflush(fd[i]);

	return num;
}

/*************************************************************/
/**                  FUNCTION FOR VALVES                    **/
/*************************************************************/

/**** SPI for valves ****/
bool clock_edge = false;
unsigned short resolution = 0x0FFF;
void set_SCLK(bool value) { digitalWrite(pin_spi_sclk, value); }
void set_OTHER(bool value) { digitalWrite(pin_spi_other, value); }
void set_MOSI(bool value) { digitalWrite(pin_spi_mosi, value); }
void setCS1(bool value){ digitalWrite(pin_spi_cs1, value); }
void setCS2(bool value){ digitalWrite(pin_spi_cs2, value); }
void setCS3(bool value){ digitalWrite(pin_spi_cs3, value); }
void setCS4(bool value){ digitalWrite(pin_spi_cs4, value); }
void set_clock_edge(bool value){ clock_edge = value; }
bool get_MISO(void) { return false; } // dummy
void wait_SPI(void){}

// value 1: Enable chipx
void chipSelect1(bool value){ setCS1(!value); wait_SPI(); wait_SPI(); }
void chipSelect2(bool value){ setCS2(!value); wait_SPI(); wait_SPI(); }
void chipSelect3(bool value){ setCS3(!value); wait_SPI(); wait_SPI(); }
void chipSelect4(bool value){ setCS4(!value); wait_SPI(); wait_SPI(); }

unsigned char transmit8bit(unsigned char output_data){
	unsigned char input_data = 0;
	int i;
	for(i = 7; i >= 0; i--){
		// MOSI - Master : write with down trigger
		//        Slave  : read with up trigger
		// MISO - Master : read before down trigger
		//        Slave  : write after down trigger
		set_SCLK(!clock_edge);
		set_MOSI( (bool)((output_data>>i)&0x01) );
		input_data <<= 1;
		wait_SPI();
		set_SCLK(clock_edge);
		input_data |= get_MISO() & 0x01;
		wait_SPI();
	}
	return input_data;
}

unsigned short transmit16bit(unsigned short output_data){
	unsigned char input_data_H, input_data_L;
	unsigned short input_data;
	input_data_H = transmit8bit( (unsigned char)(output_data>>8) );
	input_data_L = transmit8bit( (unsigned char)(output_data) );
	input_data = (((unsigned short)input_data_H << 8)&0xff00) | (unsigned short)input_data_L;
	return input_data;
}


void setDARegister(unsigned char ch, unsigned short dac_data){
	unsigned short register_data;

	if (ch < 8) {
		register_data = (((unsigned short)ch << 12) & 0x7000) | (dac_data & 0x0fff);
		chipSelect1(true);
		transmit16bit(register_data);
		chipSelect1(false);
	}
	else if ((ch >= 8) && (ch <16)) {
		register_data = (((unsigned short)(ch & 0x0007) << 12) & 0x7000) | (dac_data & 0x0fff);
		chipSelect2(true);
		transmit16bit(register_data);
		chipSelect2(false);
	}
	else if ((ch >= 16) && (ch <24)) {
		register_data = (((unsigned short)(ch & 0x0007) << 12) & 0x7000) | (dac_data & 0x0fff);
		chipSelect3(true);
		transmit16bit(register_data);
		chipSelect3(false);
	}
	else if ((ch >= 24) && (ch <32)) {
		register_data = (((unsigned short)(ch & 0x0007) << 12) & 0x7000) | (dac_data & 0x0fff);
		chipSelect4(true);
		transmit16bit(register_data);
		chipSelect4(false);
	}
}

/* SetDAResgister */
// pressure coeff: [0.0, 1.0]
// pressure coeff represent the real MPa (max 1MPa), and limited by max input pressure
void setState(unsigned int ch, double pressure_coeff)
{
	setDARegister(ch, (unsigned short)(pressure_coeff * resolution));
}

/*************************************************************/
/**                  FUNCTION FOR SENSOR                    **/
/*************************************************************/

/**** SPI for sensors ****/
void set_DIN_SENSOR(bool value) { digitalWrite(pin_din_sensor, value); }
void set_CLK_SENSOR(bool value) { digitalWrite(pin_clk_sensor, value); }
void set_CS_SENSOR(bool value) { digitalWrite(pin_cs_sensor, value); }
int get_DOUT_SENSOR(int adc_num) {
	switch (adc_num){
		case 0: digitalRead(pin_dout1_sensor); break;
		case 1: digitalRead(pin_dout2_sensor); break;
		case 2: digitalRead(pin_dout3_sensor); break;
		case 3: digitalRead(pin_dout4_sensor); break;
	}
}

/**************************************************/
/*         ADCValue to Pressure                   */
/**************************************************/

double ADCtoPressure (unsigned long ADCValue){
  /* Pressure Sensor Specification */
  double alpha = 0.0009;
  double beta = 0.04;
  double Perror = 25; //Pressure error in kPa

  /* Using error */
  //double error = Perror*1*alpha;
  /* Not using error */
  double error = 0;

  double temp;
  temp = (((double)ADCValue/4096)-error-beta)/alpha /1000;
  return temp;
}

/**************************************************/
//         ADCValue to Angle
// Desc  : Convert ADC value to Angle (Degree)
// Input : ADCValue : Current ADC Value
//				 Pot_ref	: ADC Value for 0 degree angle
// Output: Sensor Val -> 1D array of Sensor Value in 1 ADC Board
/**************************************************/

double ADCtoAngle (unsigned long ADCValue, int Pot_ref){
  double temp;
  /* Output in degree */
  temp = (((double) ADCValue - Pot_ref) /4096)*360;
  return temp;
}

/***************************************************************/
// read_sensor
// Desc  : Read ADC values from ONLY 1 ADC Board
// Input :  adc_num : the index of ADC Board
// Output: Sensor Val -> 1D array of Sensor Value in 1 ADC Board
/***************************************************************/
unsigned long *read_sensor(unsigned long adc_num,unsigned long* sensorVal){

	unsigned long pin_num=0x00;
	unsigned long sVal;
	unsigned long commandout=0x00;

	int i;

    for(pin_num=0;pin_num<NUM_ADC_PORT;pin_num++){
    	sVal=0x00;
			set_CS_SENSOR(true);
			set_CLK_SENSOR(false);
			set_DIN_SENSOR(false);
			set_CS_SENSOR(false);

    	commandout=pin_num;
    	commandout|=0x18;
    	commandout<<=3;

	    for(i=0;i<5;i++){
				if(commandout&0x80){
					set_DIN_SENSOR(true);
		  	}
		  	else{
					set_DIN_SENSOR(false);
		  	}
		  	commandout<<=1;
		  	set_CLK_SENSOR(true);
		  	set_CLK_SENSOR(false);
			}
	    for(i=0;i<2;i++){
				set_CLK_SENSOR(true);
		    set_CLK_SENSOR(false);
	    }
	    for(i=0;i<12;i++){
				set_CLK_SENSOR(true);
				sVal<<=1;
	  		if(get_DOUT_SENSOR(adc_num)){
	    		sVal|=0x01;
	    	}
		  	set_CLK_SENSOR(false);
    	}
    	sensorVal[pin_num]=sVal;
    }
    return(sensorVal);
}

/***************************************************************/
// read_sensor_all
// Desc  : Read ADC values for all available ADC Board for ONCE
// Output: Sensor Val -> 3D array of Sensor Value in all ADC Board
//         - index : index of the sample
//         - NUM_ADC : ADC board number
//         - NUM_ADC_PORT : ADC board port number
//     * There is no index here, only all the ADC Value on 1 time
/***************************************************************/
void read_sensor_all (int index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], double *Angle){
  int j,k;
  unsigned long *tmp_val0;
  unsigned long tmp_val[NUM_ADC_PORT];

  for (j = 0; j< NUM_ADC; j++){
    tmp_val0=read_sensor(j,tmp_val);
    for (k = 0; k< NUM_ADC_PORT; k++){
			// get only used ports
			if (j*NUM_ADC_PORT + k >= NUM_OF_SENSOR)
				break;

      SensorVal[index][j][k]=tmp_val0[k];
      //printf("[%d][%d] %lu\n",j,k,SensorVal[j][k]);

			// Getting Angle from Potentiometer
			if (j*NUM_ADC_PORT + k < NUM_OF_SENSOR){
				Angle[j*NUM_ADC_PORT + k] = ADCtoAngle(tmp_val0[k],Pot_straight[j*NUM_ADC_PORT + k]);
			}
    }
  }
}


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
  //std::cout << "Putting device into configuration mode..." << std::endl;
  device->gotoConfig();

  // Request the device Id to check the device type
  mtPort->setDeviceId(device->getDeviceId());

  // Print information about detected MTi / MTx / MTmk4 device
  //std::cout << "Found a device with id: " << mtPort->deviceId().toString().toStdString() << " @ port: " << mtPort->portName().toStdString() << ", baudrate: " << mtPort->baudrate() << std::endl;
  //std::cout << "Device: " << device->getProductCode().toStdString() << " opened." << std::endl;

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
			//XsOutputConfiguration quat(XDI_Quaternion, 100);
			XsOutputConfiguration acc(XDI_Acceleration, 100);
			XsOutputConfiguration gyr(XDI_RateOfTurn, 100);
			XsOutputConfiguration sampletime(XDI_SampleTimeFine, 100);

			//configArray.push_back(quat);
			configArray.push_back(acc);
			configArray.push_back(gyr);
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


void test_IMU(){
	std::cout << "Looping Printing by accessing function each time.." << std::endl;
	int mode;
	printf("Output Mode <1:Orientation><2:calibratedData> : ");
	scanf ("%d",&mode);
	if (mode==1){
		XsOutputMode outputMode = XOM_Orientation;
		XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion;
		config_IMU(&device,&mtPort, outputMode, outputSettings);
		while(1)
		  {
		  measure_IMU(&device,&mtPort, outputMode, outputSettings, &quaternion,&euler,&calData,&sample_time);
			printf("\n");
			std::cout  << "\r"
			    << "W:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.w()
			    << ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.x()
			    << ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.y()
			    << ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.z()
		    ;
		  std::cout << ",Roll:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.roll()
			    << ",Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.pitch()
			    << ",Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.yaw()
			   ;
		  }
	}
	else if(mode==2){
		XsOutputMode outputMode = XOM_Calibrated;
		XsOutputSettings outputSettings = XOS_CalibratedMode_All;

		double acc;
		config_IMU(&device,&mtPort, outputMode, outputSettings);
		while(1)
		  {
		  measure_IMU(&device,&mtPort, outputMode, outputSettings, &quaternion,&euler,&calData,&sample_time);
		  std::cout  << "\r"
			   	<< "AccX:" << std::setw(7) << std::fixed << std::setprecision(4) << calData.m_acc.value(0)
			    << ", AccY:" << std::setw(7) << std::fixed << std::setprecision(4) << calData.m_acc.value(1)
			    << ", AccZ:" << std::setw(7) << std::fixed << std::setprecision(4) << calData.m_acc.value(2)

			   	<< ",   GyrX:" << std::setw(7) << std::fixed << std::setprecision(4) << calData.m_gyr.value(0)
			    << ", GyrY:" << std::setw(7) << std::fixed << std::setprecision(4) << calData.m_gyr.value(1)
			    << ", GyrZ:" << std::setw(7) << std::fixed << std::setprecision(4) << calData.m_gyr.value(2)
			;
		  }

			printf("\n");
	}
}

/*************************************************************/
/**                    INIT FUNCTIONS                       **/
/*************************************************************/
void init_pins()
{
	set_SCLK(LOW);
	set_MOSI(LOW);
	set_OTHER(LOW);
	setCS1(HIGH);
	setCS2(HIGH);

/*	set_SCLK(HIGH);
	set_MOSI(HIGH);
	set_OTHER(HIGH);
	setCS1(HIGH);
	setCS2(HIGH);
*/

	analog_pin[0] = P9_33;
	analog_pin[1] = P9_35;
	analog_pin[2] = P9_36;
	analog_pin[3] = P9_37;
	analog_pin[4] = P9_38;
	analog_pin[5] = P9_39;
	analog_pin[6] = P9_40;
}

void init_DAConvAD5328(void) {
	set_clock_edge(false);// negative clock (use falling-edge)

	// initialize chip 1
	chipSelect1(true);
	transmit16bit(0xa000);// synchronized mode
	chipSelect1(false);

	chipSelect1(true);
	transmit16bit(0x8003);// Vdd as reference
	chipSelect1(false);

	// initialize chip 2
	chipSelect2(true);
	transmit16bit(0xa000);// synchronized mode
	chipSelect2(false);

	chipSelect2(true);
	transmit16bit(0x8003);// Vdd as reference
	chipSelect2(false);
}

void init_sensor(void) {
	set_DIN_SENSOR(false);
	set_CLK_SENSOR(false);
	set_CS_SENSOR(false);
}



/*************************************************************/
/**                    FILE WRITING                         **/
/*************************************************************/


/***************************************************************/
// Desc   : logging for saving data (sensor data) to txt file
// Input  : mode [0] start logging. create and open file
//               [1] input data
//               [2] close file
// Output :
/***************************************************************/
int logging (int mode, const char *message, int  index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], XsCalibratedData *calData){
//int logging(int mode, const char *message){
  static FILE *fp;
  char str[256];

  int i,j,k;
  // int SampleNum; //local SampleNum

  if (mode==0) {
    /* Creating Log File with format YYYYMMDD_HHMM.csv in /log directory */
    time_t now = time(NULL);
    struct tm *pnow = localtime (&now);

    sprintf (str,"log/%d%02d%02d_%02d%02d",
	   pnow->tm_year+1900,   // year start from 1900
	   pnow->tm_mon+1,  // month [0,11]
	   pnow->tm_mday,
	   pnow->tm_hour,
	   pnow->tm_min);

    if (message!=""){
      strcat (str,"_");
      strcat (str,message);
    }
    strcat(str,".csv");

    fp = fopen(str,"w");  // "w" : create file for writing
    if (fp == NULL){
      printf("File open error\n");
      return 0;
    }
    printf("File created. Start logging\n");
  }


  else if (mode==1){
    sprintf(str, "%d",index);//TimeData[i]);
    fputs(str, fp);
    for (j = 0; j< NUM_ADC; j++){
      for (k = 0; k< NUM_ADC_PORT; k++){
				//sprintf(str, "%10lu\t", SensorVal[index][j][k]);
				sprintf(str, ",%d", SensorVal[index][j][k]);
				fputs(str, fp);
      }
    }

		// converting float to string
		for (j=0;j<3;j++){
			std::string strs = ","+ std::to_string(calData[index].m_acc.value(j));
			//sprintf(str, "%d\t", calData.m_acc.value(j));
			fputs(strs.c_str(), fp);
		}
		for (j=0;j<3;j++){
			std::string strs = ","+ std::to_string(calData[index].m_gyr.value(j));
			//sprintf(str, "%d\t", calData.m_gyr.value(j));
			fputs(strs.c_str(), fp);
		}
    sprintf(str, "\n");
    fputs(str, fp);
  }


  else if(mode==2){
    printf("End logging\n");
    fclose(fp);
  }
}

/* start and only open file for log */
void startlog(const char* message){
	XsCalibratedData *dummy;
  logging(0,message,NULL,NULL,dummy);
}

/* input log on specific index */
void entrylog(int  index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], XsCalibratedData *calData){
  logging(1,"",index,SensorVal,calData);
}

/* close file */
void endlog() {
	XsCalibratedData *dummy;
  logging(2,"",NULL,NULL,dummy);
}

/* */
void fulllog(const char* message, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], XsCalibratedData *calData){
  int i;
  startlog(message);
  for (i=0;i<SampleNum;i++){
    entrylog (i,SensorVal,calData);
  }
  endlog();
}

/*************************************************************/
/**                    TESTING FUNCTION                         **/
/*************************************************************/

void Reset_Valve (){
	int i;
	for (i=0;i< (NUM_DAC*NUM_OF_CHANNELS) ;i++){
		if (i%16 < (NUM_OF_MUSCLE/2)){
			setState(i,0.0);
			printf("Valve #%d off\n",i);
		}
	}
}



/*===== Running test to print all the Sensor Data ======*/
/* Read and print only ONCE for all sensor */
void test_sensor (int SampleNum){
  int index,j,k;
  static unsigned long Value[MAX_SAMPLE_NUM][NUM_ADC][NUM_ADC_PORT];
	double Angle[NUM_OF_POT_SENSOR];

  //startlog("test_sensor");

  for(index=0;index<SampleNum;index++){
    read_sensor_all(index,Value,Angle);
    /**/
    for (j = 0; j< NUM_ADC; j++){
      for (k = 0; k< NUM_ADC_PORT; k++){
	printf("[%d][%d][%d] %lu\n",index,j,k, Value[index][j][k]);
      }
    }
    printf ("-------------------\n");
    /**/
    // logging into file
    //entrylog(index,Value);
  }
  //endlog();

  //return (Value);
}

/*======  Test one Muscle with Specific Pressure =======*/
void test_valve (){

  int valve_num;
  static int i=0;
  double val,sensorval;
  static unsigned long Value[MAX_SAMPLE_NUM][NUM_ADC][NUM_ADC_PORT];

  val=0;

  printf ("Testing muscle with %1lf pressure coef\n", val);

  while(1){
    printf("Input valve number: ");
    scanf ("%d",&valve_num);
    printf ("Testing muscle number %d\n", valve_num);
    printf("Input pressure coef : ");
    scanf ("%lf",&val);
    //printf ("%lf\n",val);
    if ((val>=0)&&(val<=1)){
      setState(valve_num,val);
      usleep(500000);

      //read_sensor_all(i,Value);
      //sensorval = ADCtoPressure (Value[i][0][1]);
      //printf("%lf\n",sensorval);
    }
    printf("--------\n");
    i++;
  }
}



/**********************************************************************************/

int main(int argc, char *argv[]) {

	init(); // from gpio.h
	init_pins(); // ALL 5 pins are HIGH except for GND
	init_DAConvAD5328();
	init_sensor();

	//init_IMU(&device,&mtPort,PORTNAME,BAUDRATE);
	// Config IMU with default settings
	//config_IMU(&device,&mtPort, DEFAULT_OUTPUT_MODE, DEFAULT_OUTPUT_SETTINGS);

	XsOutputMode outputMode = XOM_Calibrated;
	XsOutputSettings outputSettings = XOS_CalibratedMode_All;

	int i,j,k;
	unsigned int ch_num;
	// Valve initialization
	Reset_Valve();


	/* Variable for ADC Board Data */
	unsigned long SensorData[SampleNum][NUM_ADC][NUM_ADC_PORT];
	unsigned long ***SensorArray;

	double JointAngle[NUM_OF_POT_SENSOR];

	clock_t TimeData[SampleNum];
	//int ValveNum = 1;


	clock_t laps1,laps2;

	if (argc==2){
	  switch (*argv[1]){
	  case '1':
	    printf("Testing Sensor\n");
		  printf("Sample \tPot1 \tPot2 \tPot3 \tPot4 \tPot5 \tPot6 \tPot7 \tPot8 \tPot9 \tPot10 \n");
	    /* this is for using function */
	    //test_sensor(SampleNum);

	    /* this is for direct reading from main, to test reading at once in main loop */
	    /**/
	    // config_IMU(&device,&mtPort, outputMode, outputSettings);
			laps1 = clock();


			setState(0,0.1);
			setState(1,0.7);

	    for (i=0;i<SampleNum;i++){
	      read_sensor_all(i,SensorData,JointAngle);
	      //		measure_IMU(&device,&mtPort, outputMode, outputSettings, &quaternion,&euler,&calData,&sample_time);

				IMUData.calData[i] = calData;
				IMUData.sample_time[i] = sample_time;

				laps2 = clock();
				double elapsed = double(laps2 - laps1) / CLOCKS_PER_SEC;
				laps1 = laps2;

				int delta_t;
				// printing
				printf("\r");
				printf("[%d]\t",i);

	      for (j = 0; j< NUM_ADC; j++){
					for (k = 0; k< NUM_ADC_PORT; k++){
						// show only used port
						if (j*NUM_ADC_PORT + k >= NUM_OF_SENSOR)
							break;
					  printf("%4lu\t", SensorData[i][j][k]);
					}
	      }

				for (j = 0; j<NUM_OF_POT_SENSOR;j++){
					printf("%.1f\t", JointAngle[j]);
				}
	      /*
				for (j=0;j<3;j++){
					printf("%.3f\t", IMUData.calData[i].m_acc.value(j));
				}
				for (j=0;j<3;j++){
					printf("%.3f\t", IMUData.calData[i].m_gyr.value(j));
				}

				printf("%d\t",IMUData.sample_time[i]);
				if (i>0){
					delta_t = IMUData.sample_time[i]-IMUData.sample_time[i-1];
				}
				else{
					delta_t = 0;
				}
				printf("%.5f\t",elapsed);
				//printf("\n");
	     */
	    }
			printf ("\n");
	    printf ("-------------------\n");
	    // fulllog("adc_acc_gyr",SensorData,IMUData.calData);
	    /**/
			// Saving test data
			printf ("Save data (y/n)? ");
			char in,msg[20];
	    scanf ("%c",&in);
			if (in=='y'){
				printf ("filename message: ");
		    scanf ("%s",&msg);
				fulllog(msg,SensorData,IMUData.calData);
			}
			printf("%c, %s\n",in, msg);
			/**/
	    break;
		case '2':
			test_IMU();
			break;
	  case '3':
	    printf("Testing Valve\n");
	    test_valve();
	    break;
	  case '4':
	    printf("Testing Preset Pressure\n");

			double muscle_val[32]= {0};


			double muscle_zero_deg_value [32] = { 0 };

			muscle_val[IL_R] = 0.05;
			muscle_val[IL_L] = 0.1;
			muscle_val[GMAX_R] = 0.6;
			muscle_val[GMAX_L] = 0.6;
			//muscle_val[VAS_R] = 0.8;
			//muscle_val[VAS_L] = 0.8;
			//muscle_val[HAM_R] = 0.5;
			//muscle_val[HAM_L] = 0.5;
			//muscle_val[TA_R] = 0.44;
			//muscle_val[TA_L] = 0.44;
			//muscle_val[SOL_R] = 0.31;
			//muscle_val[SOL_L] = 0.31;
			muscle_val[ADD_R] = 0.3;
			muscle_val[ADD_L] = 0.3;

			for (i=0;i< (NUM_DAC*NUM_OF_CHANNELS) ;i++){
				if (i%16 < (NUM_OF_MUSCLE/2)){
					printf("%d\t",i);
					printf("%.2lf\n", muscle_val[i]);
					setState(i,muscle_val[i]);
				}

			}
			printf ("\n");

			printf("Sensor Data\n");
			printf("Sample \tPot1 \tPot2 \tPot3 \tPot4 \tPot5 \tPot6 \tPot7 \tPot8 \tPot9 \tPot10 \n");

			for (i=0;i<2000000;i++){

				read_sensor_all(i,SensorData,JointAngle);
				printf("\r");
				printf("[%d]\t",i);
				for (j = 0; j<NUM_OF_POT_SENSOR;j++){
					printf("%.1f\t", JointAngle[j]);
				}

			}

			Reset_Valve();
			printf ("Done\n");
	    break;
	  }
	}
	else{
	  printf("Please start program with an argument:\n");
	  printf("1 : Testing Sensor\n");
		printf("2 : Testing IMU Sensor\n");
	  printf("3 : Testing a desired Muscle/Valve with desired pressure\n");
	  printf("4 : Testing all Muscle/Valves with preset pressure\n");
	  printf("5 : \n");
	}


	return 0;
}
