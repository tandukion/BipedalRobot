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
#include <chrono>

#include "deviceclass.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <sstream>
#include <fstream>

#ifdef __GNUC__
#include "conio.h" // for non ANSI _kbhit() and _getch()
#else
#include <conio.h>
#endif

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

/* Arduino PID Library */
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <PIDAutotuner.h>

/* for sensors */
#define pin_cs_sensor P9_13 // 0_31=31
#define pin_clk_sensor P9_12 // 1_28=60
#define pin_din_sensor  P9_11 // 0_30=30
#define pin_dout1_sensor P9_14 // 1_18=50
#define pin_dout2_sensor  P9_15 // 1_16=48
#define pin_dout3_sensor  P9_26 // 0_14=14
#define pin_dout4_sensor  P9_27 // 3_19=125

#define NUM_ADC 4
#define NUM_ADC_PORT 8

#define NUM_OF_POT_SENSOR 10
#define NUM_OF_PRES_SENSOR 22
#define NUM_OF_SENSOR NUM_OF_POT_SENSOR+NUM_OF_PRES_SENSOR

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
#define NUM_OF_MUSCLE 22	// Number of muscle/valve

#define IL_R	0
#define GMAX_R	1
#define VAS_R	2
#define HAM_R	3
#define TA_R 	4
#define SOL_R 	5
#define ADD_R 	6
#define ABD_R 	7
#define TP_R	8
#define FB_R	9
#define RF_R	10
#define IL_L	11
#define GMAX_L	12
#define VAS_L	13
#define HAM_L	14
#define TA_L 	15
#define SOL_L 	16
#define ADD_L 	17
#define ABD_L 	18
#define TP_L	19
#define FB_L	20
#define RF_L	21

#define IL_R_CH			0
#define GMAX_R_CH		1
#define VAS_R_CH		2
#define HAM_R_CH		3
#define TA_R_CH 		4
#define SOL_R_CH 		5
#define ADD_R_CH	 	6
#define ABD_R_CH	 	7
#define TP_R_CH			8
#define FB_R_CH			9
#define RF_R_CH			10
#define IL_L_CH			16
#define GMAX_L_CH		17
#define VAS_L_CH		18
#define HAM_L_CH		19
#define TA_L_CH 		20
#define SOL_L_CH	 	21
#define ADD_L_CH	 	22
#define ABD_L_CH	 	23
#define TP_L_CH			24
#define FB_L_CH			25
#define RF_L_CH			26

/* VALUE */
#define MAX_PRESSURE 0.8
#define MIN_PRESSURE 0
// Default Pressure
#define PRES_DEF 0.3

#define MAX_SAMPLE_NUM 100000
#define SAMPLE_NUM 5000		// default 20000

/* muscle pair for a joint */
#define muscle_pair_num 12

/*************************************************************/
/**                   GLOBAL VARIABLES                      **/
/*************************************************************/

long SampleNum = SAMPLE_NUM; //20000;
unsigned long SensorValue[NUM_ADC][NUM_ADC_PORT];
unsigned long SensorData[SAMPLE_NUM][NUM_ADC][NUM_ADC_PORT];
double JointAngle[SAMPLE_NUM][NUM_OF_POT_SENSOR];
double SetPoint_Angle[NUM_OF_POT_SENSOR] ={0};
double AngleError[muscle_pair_num] ={0};
double MusclePressure[NUM_DAC*NUM_OF_CHANNELS];


/* for sampling time */
std::chrono::system_clock::time_point StartTimePoint, EndTimePoint;
double TimeStamp[MAX_SAMPLE_NUM];

/* Table for muscle valve number and sensor number */
struct MuscleDataArray{
	int channel;			// channel for the valve
	double value;	// pressure value
	double p0;    // P0 for antagonistic muscle (starting pressure)
	double dP;				// delta P for control output
} muscle [NUM_OF_MUSCLE], muscle_dummy; //dummy for muscle without pair: RF

int muscle_sensor [NUM_OF_MUSCLE] = {PIN_PRES_1,PIN_PRES_2};

/* Table for muscle valve channel */
int muscle_ch [NUM_OF_MUSCLE] = {IL_R_CH,GMAX_R_CH,VAS_R_CH,HAM_R_CH,TA_R_CH,SOL_R_CH,ADD_R_CH,
				 ABD_R_CH,TP_R_CH,FB_R_CH,RF_R_CH,
				 IL_L_CH,GMAX_L_CH,VAS_L_CH,HAM_L_CH,TA_L_CH,SOL_L_CH,ADD_L_CH,
				 ABD_L_CH,TP_L_CH,FB_L_CH,RF_L_CH};

// +2 for extra biarticular muscle RF
int muscle_pair [muscle_pair_num][2] = {{IL_R,GMAX_R}, {IL_L,GMAX_L},
					{ABD_R,ADD_R}, {ABD_L,ADD_L},
					{VAS_R,HAM_R}, {VAS_L,HAM_L},
					{TA_R,SOL_R}, {TA_L,SOL_L},
					{FB_R,TP_R}, {FB_L,TP_L},
					{RF_R,HAM_R}, {RF_L,HAM_L},// for biarticular,can't use NULL=0
};


/* Potentiometer reference for zero degree */
// motion capture
//int Pot_straight [10] = {2128,2346,2305,3090,2336,2167,1446,1876,2118,2010};
// recalibration
//int Pot_straight [10] = {2056,2432,2255,3008,2464,2040,1360,1904,2167,1984};
//int Pot_straight [10] = {2000,2464,2232,3024,2544,2023,1344,1920,2143,1984};			// working for 66s in mode 7
//int Pot_straight [10] = {2000,2464,2132,2970,2544,2023,1344,1920,2143,1984};			// recalibrate
//int Pot_straight [10] = {2000,2464,2132,2970,2544,2023,1414,1890,2117,2080};			// manual calibration
//Pot 3 Broken! Shaft changed again

//int Pot_straight [10] = {2000,2464,1984,2975,2544,2023,1344,1920,2143,1984};			// manual calibration on studio
//int Pot_straight [10] = {2000,2464,2018,2975,2544,2023,1344,1920,1950,1984};			// PI stand 06/04
// Pot 4 got impact
//int Pot_straight [10] = {2000,2464,2018,2099,2544,2023,1344,1920,1950,1984};			// jump_take9 06/14
//int Pot_straight [10] = {2000,2464,2084,2746,2544,2023,1344,1920,1950,1984};			// doublejump 06/16

// Pot 7 8 changed
//int Pot_straight [10] = {2000,2464,1639,2695,2544,2023,1344,1920,1950,2128};			// new 06/23, doublejump 11-14
//int Pot_straight [10] = {2000,2464,1639,2640,2500,2023,1344,1920,1950,2128};			// new 06/24, new method, jumptake32
//int Pot_straight [10] = {2000,2464,1539,2540,2500,2023,1344,1920,1950,2128};			// retake to correct posture
//int Pot_straight [10] = {2000,2464,1539,2540,2500,2023,1344,1920,1950,2378};			// 06/27, start from jumpstep11

//Pot 4 got Impact again!
//int Pot_straight [10] = {2000,2464,1539,2150,2500,2023,1344,1920,1950,2378};			// 06/27, start from jumpstep15
																																									// good for jumpstep22, jumpstep35
//int Pot_straight [10] = {2000,2464,1589,2200,2500,2023,1344,1920,1950,2378};			// 06/29,  jumpstep36

//Pot 3 crash!
int Pot_straight [10] = {2000,2464,1057,2223,2500,2023,1344,1920,1950,2378};			// 06/29,  jumpstep36

// Angle on same pressure p=0.3
int Pot_Psame 	[10] = {1820,2665,2383,3034,2641,1817,1573,1759,2199,2134};
double Angle_Psame [10] = {27.1,28.0,2.3,4.9,-26.8,-30.8,-11.2,-10.3, -7.1,-6.9};

/* Variable for IMU Data */
struct IMUDataArray{
	XsCalibratedData calData; //[MAX_SAMPLE_NUM];
	XsQuaternion quaternion; //[MAX_SAMPLE_NUM];
	XsEuler euler; //[MAX_SAMPLE_NUM];
	unsigned int sample_time; //[MAX_SAMPLE_NUM];
};

IMUDataArray IMUData[MAX_SAMPLE_NUM];
DeviceClass device;
XsPortInfo mtPort;
XsOutputMode outputMode = DEFAULT_OUTPUT_MODE; //XOM_Calibrated;
XsOutputSettings outputSettings = DEFAULT_OUTPUT_SETTINGS; //XOS_CalibratedMode_All;

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

void setMuscle(MuscleDataArray muscle)
{
	setState(muscle.channel,muscle.value);
}
void setMusclenewVal(MuscleDataArray muscle, double value)
{
	setState(muscle.channel, value);
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

/**************************************************/
//         ADCValue to Pressure
// Desc  : Convert ADC value to Pressure (MPa)
// Input : ADCValue : Current ADC Value
/**************************************************/
double ADCtoPressure (unsigned long ADCValue){
  /* Pressure Sensor Specification */
  double alpha = 0.0009;
  double beta = 0.04;
  // XFGM-6001MPGSR
  //double Perror = 25; //Pressure error in kPa
  // AG206-001MG
  double Perror = 15; //Pressure error in kPa

  /* Using error */
  double error = Perror*1*alpha;
  /* Not using error */
  //double error = 0;

  double temp;
  temp = (((double)ADCValue/4096)+error-beta)/alpha /1000;
  return temp;
}

/**************************************************/
//         ADCValue to Angle
// Desc  : Convert ADC value to Angle (Degree)
// Input : ADCValue : Current ADC Value
//	   Pot_ref  : ADC Value for 0 degree angled
/**************************************************/
double ADCtoAngle (unsigned long ADCValue, int Pot_ref){
  double temp;
  /* Output in degree */
  temp = (((double) ADCValue - Pot_ref) /4096)*360;
	if(temp>180)
		temp-=360;
	else if(temp<-180)
		temp+=360;
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
void read_sensor_all (int index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], double Angle[][NUM_OF_POT_SENSOR], double *Pressure){
  int j,k,index_adc;
  unsigned long *tmp_val0;
  unsigned long tmp_val[NUM_ADC_PORT];

  for (j = 0; j< NUM_ADC; j++){
    tmp_val0=read_sensor(j,tmp_val);
    for (k = 0; k< NUM_ADC_PORT; k++){
      index_adc = j*NUM_ADC_PORT + k;

      // get only used ports
      if (index_adc >= NUM_OF_SENSOR)
			break;

      SensorVal[index][j][k]=tmp_val0[k];
      //printf("[%d][%d] %lu\n",j,k,SensorVal[j][k]);

      // Getting Angle from Potentiometer
      if (index_adc < NUM_OF_POT_SENSOR){
				Angle[index][index_adc] = ADCtoAngle(tmp_val0[k],Pot_straight[index_adc]);
      }
      // Getting Pressure from Pressure SENSOR
      else if(index_adc >= NUM_OF_POT_SENSOR){
				Pressure[index_adc-NUM_OF_POT_SENSOR] = ADCtoPressure(tmp_val0[k]);

	// for first board (Right) valve 0-10
	//if ((index_adc-NUM_OF_POT_SENSOR) < (NUM_OF_MUSCLE/2))
	//  Pressure[index_adc-NUM_OF_POT_SENSOR] = ADCtoPressure(tmp_val0[k]);
	// second board (Left) valve 16-26
	//else
	//  Pressure[index_adc-NUM_OF_POT_SENSOR - (NUM_OF_MUSCLE/2) + NUM_OF_CHANNELS] = ADCtoPressure(tmp_val0[k]);
      }
    }
  }
  // Adjusting Angle direction
  Angle[index][0] = -1* Angle[index][0];
  Angle[index][3] = -1* Angle[index][3];
  Angle[index][4] = -1* Angle[index][4];
  Angle[index][6] = -1* Angle[index][6];
  Angle[index][8] = -1* Angle[index][8];
}

void printSensorVal (int i, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT]){
  int j,k;

  for (j = 0; j< NUM_ADC; j++){
    for (k = 0; k< NUM_ADC_PORT; k++){
      // show only used port
      if (j*NUM_ADC_PORT + k >= NUM_OF_SENSOR)
	break;
      printf("%4lu\t", SensorVal[i][j][k]);
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

//void measure_IMU(DeviceClass *device, XsPortInfo *mtPort, XsOutputMode outputMode, XsOutputSettings outputSettings, XsQuaternion *quaternion, XsEuler *euler, XsCalibratedData *calData, unsigned int *sample_time){
void measure_IMU(DeviceClass *device, XsPortInfo *mtPort, XsOutputMode outputMode, XsOutputSettings outputSettings, IMUDataArray *IMUData){

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
	IMUData->quaternion = packet.orientationQuaternion();
	// Convert packet to euler
	IMUData->euler = packet.orientationEuler();
	// Get calibrated Data
	IMUData->calData = packet.calibratedData();

	// Get Sample Time Coarse
	IMUData->sample_time = packet.sampleTimeFine();
      }
  } while (!foundAck);

}


void test_IMU(){
  std::cout << "Looping Printing by accessing function each time.." << std::endl;
  int mode;
  int i=0;
  printf("Output Mode <1:Orientation><2:calibratedData> : ");
  scanf ("%d",&mode);
  if (mode==1){
    XsOutputMode outputMode = XOM_Orientation;
    XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion;
    config_IMU(&device,&mtPort, outputMode, outputSettings);
    while(i<MAX_SAMPLE_NUM)
      {
	//measure_IMU(&device,&mtPort, outputMode, outputSettings, &quaternion,&euler,&calData,&sample_time);
	measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
	//printf("\n");
	std::cout  << "\r"
		   << "W:" << std::setw(5) << std::fixed << std::setprecision(2) << IMUData[i].quaternion.w()
		   << ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << IMUData[i].quaternion.x()
		   << ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << IMUData[i].quaternion.y()
		   << ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << IMUData[i].quaternion.z()
	  ;
	std::cout << ",\tRoll:" << std::setw(7) << std::fixed << std::setprecision(2) << IMUData[i].euler.roll()
		  << ",Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << IMUData[i].euler.pitch()
		  << ",Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << IMUData[i].euler.yaw()
	  ;
	i++;
      }
  }
  else if(mode==2){
    XsOutputMode outputMode = XOM_Calibrated;
    XsOutputSettings outputSettings = XOS_CalibratedMode_All;

    double acc;
    config_IMU(&device,&mtPort, outputMode, outputSettings);
    while(i<MAX_SAMPLE_NUM)
      {
	//measure_IMU(&device,&mtPort, outputMode, outputSettings, &quaternion,&euler,&calData,&sample_time);
	measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
	std::cout  << "\r"
		   << "AccX:" << std::setw(7) << std::fixed << std::setprecision(4) << IMUData[i].calData.m_acc.value(0)
		   << ", AccY:" << std::setw(7) << std::fixed << std::setprecision(4) << IMUData[i].calData.m_acc.value(1)
		   << ", AccZ:" << std::setw(7) << std::fixed << std::setprecision(4) << IMUData[i].calData.m_acc.value(2)

		   << ",   GyrX:" << std::setw(7) << std::fixed << std::setprecision(4) << IMUData[i].calData.m_gyr.value(0)
		   << ", GyrY:" << std::setw(7) << std::fixed << std::setprecision(4) << IMUData[i].calData.m_gyr.value(1)
		   << ", GyrZ:" << std::setw(7) << std::fixed << std::setprecision(4) << IMUData[i].calData.m_gyr.value(2)
	  ;
	i++;
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
// Input  : mode [1] start logging. create and open file
//               [2] input data
//               [3] close file
// Output :
/***************************************************************/
int logging (int mode, const char *message, int  index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT],
	     double *SetPoint_Angle, IMUDataArray *IMUData){
  //int logging(int mode, const char *message){
  static FILE *fp;
  char str[256];

  int i,j,k;
  // int SampleNum; //local SampleNum

  if (mode==1) {
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


  else if (mode==2){
    // Column #0 : index
    sprintf(str, "%d",index);
    fputs(str, fp);

    // All ADC Value, Column #1~#32
    for (j = 0; j< NUM_ADC; j++){
      for (k = 0; k< NUM_ADC_PORT; k++){
	// show only used port
	if (j*NUM_ADC_PORT + k >= NUM_OF_SENSOR)
	  break;
	//sprintf(str, "%10lu\t", SensorVal[index][j][k]);
	sprintf(str, ",%d", SensorVal[index][j][k]);
	fputs(str, fp);
      }
    }

    // Angle Set Point Value, Column #33~#42
    for (j = 0; j<NUM_OF_POT_SENSOR;j++){
      // converting float to string
      std::string strs = ","+ std::to_string(SetPoint_Angle[j]);
      fputs(strs.c_str(), fp);
    }


    // IMU roll pitch yaw, Column #43~45
    std::string strs = ","+ std::to_string(IMUData[index].euler.roll());
    fputs(strs.c_str(), fp);
    strs = ","+ std::to_string(IMUData[index].euler.pitch());
    fputs(strs.c_str(), fp);
    strs = ","+ std::to_string(IMUData[index].euler.yaw());
    fputs(strs.c_str(), fp);

    /*
    // Accelerometer Value, Column #21~#23
    for (j=0;j<3;j++){
    std::string strs = ","+ std::to_string(calData[index].m_acc.value(j));
    fputs(strs.c_str(), fp);
    }
    // Gyroscope Value, Column #24~#26
    for (j=0;j<3;j++){
    std::string strs = ","+ std::to_string(calData[index].m_gyr.value(j));
    fputs(strs.c_str(), fp);
    }
    */
    // Time Data in milliseconds, last Column
    strs = ","+ std::to_string(TimeStamp[index]);
    fputs(strs.c_str(), fp);

    sprintf(str, "\n");
    fputs(str, fp);
  }


  else if(mode==3){
    printf("End logging\n");
    fclose(fp);
  }
}

/* start and only open file for log */
int startlog(const char* message){
	int mode = 1;
	IMUDataArray *dummy;
	logging(mode,message,NULL,NULL,NULL,dummy);
	return mode;
}

/* input log on specific index */
int entrylog(int  index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], double *SetPoint_Angle,
							IMUDataArray *IMUData){
	int mode = 2;
	logging(mode,"",index,SensorVal,SetPoint_Angle,IMUData);
	return mode;
}

/* close file */
int endlog() {
	int mode = 3;
	IMUDataArray *dummy;
	logging(mode,"",NULL,NULL,NULL,dummy);
	return mode;
}

/* */
void fulllog(const char* message, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT],
		 double *SetPoint_Angle, IMUDataArray *IMUData, int DataMark){
  int i;
  startlog(message);
  for (i=0;i<DataMark;i++){
    entrylog (i,SensorVal,SetPoint_Angle,IMUData);
  }
  endlog();
}

/***************************************************************/
// Desc   : Saving PID Tuning data to file
// Input  :
// Output :
/***************************************************************/
void saveTunings(double Kp, double Ki, double Kd){
	std::string line;
	std::ofstream fp;			// ofstream, stream class to read

	fp.open("src/PIDTuningData.txt");
	if (fp.is_open()){
		fp << Kp << "\n";
		fp << Ki << "\n";
		fp << Kd << "\n";
	}
	else
		std::cout << "File Open Error\n";

}

/***************************************************************/
// Desc   : Loading PID Tuning data from file
// Input  :
// Output :
/***************************************************************/
void loadTunings(double *Kp, double *Ki, double *Kd){
	double temp[3];
	int i=0;
	std::string line;
	std::ifstream fp;			// fstream, stream class to read/write

	fp.open("src/PIDTuningData.txt");
	if (fp.is_open()){
		while (getline(fp,line)){
			temp[i] = std::stod(line);   // string to double
			i++;
		}
	}
	else
		std::cout << "No File. Please check for PID Tuning file\n";
	*Kp = temp[0];
	*Ki = temp[1];
	*Kd = temp[2];
}

/*************************************************************/
/**                    TESTING FUNCTION                         **/
/*************************************************************/

void ResetAllValve (){
  int i;
  for (i=0;i< (NUM_DAC*NUM_OF_CHANNELS) ;i++){
    if (i%16 < (NUM_OF_MUSCLE/2)){
      setState(i,0.0);
      //printf("Valve #%d off\n",i);
    }
  }
}
template<typename T, size_t N>
void ResetValve (T (&mus)[N]){
  int i;
  for (i=0;i< N ;i++){
    setState(mus[i],0.0);
  }
}
void SetAllValve (double value){
  int i;
  for (i=0;i< (NUM_DAC*NUM_OF_CHANNELS) ;i++){
    if (i%16 < (NUM_OF_MUSCLE/2)){
      setState(i,value);
    }
  }
}
template<typename T, size_t N>
void SetValve (T (&mus)[N], double value){
  int i;
  for (i=0;i< N ;i++){
    setState(mus[i],value);
  }
}

template<typename T, size_t N>
void SetValveEach (T (&mus)[N], double *value){
  int i;
  for (i=0;i< N ;i++){
    setState(mus[i],value[i]);
  }
}

void SetFrontalPlaneMuscle (double value){
  int mus[] = {ADD_R_CH,ADD_L_CH,ABD_R_CH,ABD_L_CH,TP_R_CH,TP_L_CH,FB_R_CH,FB_L_CH};
  SetValve(mus,value);
}

/*===== Running test to print all the Sensor Data ======*/
/* Read and print only ONCE for all sensor */
void test_sensor (int SampleNum){
  int index,j,k;
  static unsigned long Value[MAX_SAMPLE_NUM][NUM_ADC][NUM_ADC_PORT];
	double Angle[SAMPLE_NUM][NUM_OF_POT_SENSOR];
	double Pressure[NUM_DAC*NUM_OF_CHANNELS];

  for(index=0;index<SampleNum;index++){
    read_sensor_all(index,Value,Angle,Pressure);
    /**/
    for (j = 0; j< NUM_ADC; j++){
      for (k = 0; k< NUM_ADC_PORT; k++){
	printf("[%d][%d][%d] %lu\n",index,j,k, Value[index][j][k]);
      }
    }
    printf ("-------------------\n");
  }
}

/*======  Test one Muscle with Specific Pressure =======*/
void test_valve (){

  int valve_count, valve_num[NUM_OF_MUSCLE];
  static int i=0;
	int j;
  double val [NUM_OF_MUSCLE],sensorval;
  static unsigned long Value[MAX_SAMPLE_NUM][NUM_ADC][NUM_ADC_PORT];
	double Angle[SAMPLE_NUM][NUM_OF_POT_SENSOR];
	double Pressure[NUM_DAC*NUM_OF_CHANNELS];

  while(1){
    printf("Num of valve to test: ");
    scanf ("%d",&valve_count);
		if (valve_count >0){
			for (j=0;j<valve_count;j++){
				printf("Input valve number: ");   scanf ("%d",&valve_num[j]);
		    printf("Input pressure coef : "); scanf ("%lf",&val[j]);
			}

			for (j=0;j<valve_count;j++){
		    if ((val[j]>=0)&&(val[j]<=1)){
					printf("Valve %d, %.3lf\t", valve_num[j], val[j]);
		      setState(valve_num[j],val[j]);
		    }
			}
			printf("\n");

			while(!_kbhit()){
	      read_sensor_all(i,Value,Angle,Pressure);

				for (j = 0; j<valve_count;j++){
					if (valve_num[j]<16)
						printf("Valve %d: %.3f\t", valve_num[j], Pressure[valve_num[j]]);
					else
						printf("Valve %d: %.3f\t", valve_num[j], Pressure[11+(valve_num[j]%NUM_OF_CHANNELS) ]);
				}
				printf("\n");
				i++;
			}
		}
		else
			break;
  }
}

// Output Saturation
void OutputSaturation (double *MuscleVal, double Pres_0){
	if (*MuscleVal > Pres_0*2)
		*MuscleVal = Pres_0*2;
	else if (*MuscleVal < MIN_PRESSURE)
		*MuscleVal = MIN_PRESSURE;
}

void ControlSignalSaturation (double *sig,double Pres_0){
	if (*sig > (Pres_0*2 - Pres_0))
		*sig = (Pres_0*2 - Pres_0);
}

// Bang-Bang Controller Test
int BangBang (double SetPoint, double RefPoint, MuscleDataArray *MusUp, MuscleDataArray *MusDown){

	double error = 2; // error compensation
	//double dP;
	int temp;

	//dP = MusUp->dP;
	if (RefPoint < SetPoint - error){
		//printf("Below SetPoint");
		//if (dP < MAX_PRESSURE)
		//	dP=0.02;
		if (MusUp->dP < MAX_PRESSURE)
			MusUp->dP = 0.02;
		if (MusDown->dP < MAX_PRESSURE)
			MusDown->dP = 0.02;

		// flag for stability
		temp=-1;
	}
	else if (RefPoint > SetPoint + error){
		//printf("Above SetPoint");
		//if (abs(dP) < MAX_PRESSURE)
		//	dP=-0.02;
		if (abs(MusUp->dP) < MAX_PRESSURE)
			MusUp->dP=-0.02;
		if (abs(MusDown->dP) < MAX_PRESSURE)
			MusDown->dP=-0.02;

		// flag for stability
		temp=-1;
	}
	else{
		//printf("Reach SetPoint");
		//dP=0;
		MusUp->dP=0;
		MusDown->dP=0;
		temp=1;
	}

	// Output Saturation
	MusUp->value += MusUp->dP;
	MusDown->value -= MusDown->dP;
	OutputSaturation(&(MusUp->value),MusUp->p0);
	OutputSaturation(&(MusDown->value),MusDown->p0);
	return temp;
}

// Global variable for Controller
double errorComp = 2; //0.5;  // error compensation,  2 works for 66s
//double Pgain = 0.001; // Kp
double Pgain = 0.003; // Kp
double Igain = 0.002; // Ki * Ts

double Kb = 1;  // Kb, back-calculation constant for anti windup

// P Controller Test
//int Controller (double SetPoint, double RefPoint, MuscleDataArray *MusUp, MuscleDataArray *MusDown){
int Controller (double SetPoint, double RefPoint, double *Err, MuscleDataArray *MusUp, MuscleDataArray *MusDown){
	//double errorComp = 0.5; // error compensation,  2 works for 66s
	//double Pgain = 0.001;
	double dP;			// controller output
	int temp;

	double error = SetPoint - RefPoint;
	double lasterror = *Err;
	double lastdP = MusUp->dP;

	double sat_error = MusUp->value - MusUp->p0;   // if value+dP < MAX_PRESSURE, sat_error = dP;

	// controller work outside error compensation band
	if (abs(error) > errorComp){
		//dP = error * Pgain;				// P controller
		dP = lastdP + Pgain * (error - lasterror) + Igain * (error);  // PI controller, discrete
		//dP = lastdP + Pgain * (error - lasterror) + Igain * (error) + Kb * sat_error;  // PI controller + anti-windup, discrete
		*Err = error;							// error update

		//*Err += Igain * error;
		//dP = Pgain * error + *Err;  // I controller
		temp = 0;

		MusUp->dP = dP;
		MusDown->dP = dP;

		// Output Saturation
		//MusUp->value += MusUp->dP;
		//MusDown->value -= MusDown->dP;
		MusUp->value = MusUp->p0 + MusUp->dP;
		MusDown->value = MusDown->p0 - MusDown->dP;
		OutputSaturation(&(MusUp->value),MusUp->p0);
		OutputSaturation(&(MusDown->value),MusDown->p0);
	}
	else
		temp = 1;  // return 1 for compensated stability
	return temp;
}

//template<typename T, size_t N>
//int JointAngleControl (T (&jointlist)[N],unsigned long *index){
int JointAngleControl (int *jointlist, int N,unsigned long *index){
	int temp,j,k,l,mus1,mus2,state[N],statecount=0,joint;
	unsigned long i;
	i = *index;

	for (l=0;l<N;l++){
		joint = jointlist[l];
		j = joint-1;

		//while (!_kbhit())
		{
			read_sensor_all(i,SensorData,JointAngle,MusclePressure);
			measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);

			EndTimePoint = std::chrono::system_clock::now();
			TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
			i++;

			printf("Joint %2d. ", j+1);
			printf("Act: %5.1f  Set: %5.1f\t ", JointAngle[i-1][j], SetPoint_Angle[j]);

			mus1= muscle_pair[j][0];
			mus2= muscle_pair[j][1];

			state[l]= Controller(SetPoint_Angle[j],JointAngle[i-1][j],&AngleError[j],&muscle[mus1],&muscle[mus2]);
			setMuscle(muscle[mus1]);
			setMuscle(muscle[mus2]);

			if (state[l]!=1)
			{
				printf("P%2d: %.2f  P%2d: %.2f\t", mus1, muscle[mus1].value, mus2,muscle[mus2].value);
				//printf("dP%2d: %5.2f  dP%2d: %5.2f\t", mus1,muscle[mus1].dP, mus2,muscle[mus2].dP);
				//printf("P%2d: %.2f  P%2d: %.2f\t", mus1, MusclePressure[mus1], mus2,MusclePressure[mus2]);
			}
			//usleep(20000);


			// case for RF, works for joint 1,2,5,6
			/*
			if ((joint==1)||(joint==2)||(joint==5)||(joint==6)){
				k=j;
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);

				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;

				mus1= muscle_pair[k%2+10][0];
				mus2= muscle_pair[k%2+10][1];
				state[l]= Controller(SetPoint_Angle[j],JointAngle[i-1][j],&AngleError[j],&muscle[mus1],&muscle[mus2]);
				setMuscle(muscle[mus1]);
				setMuscle(muscle[mus2]);

				if (state[l]!=1)
				{
					printf("P%2d: %.2f  P%2d: %.2f\t", mus1, muscle[mus1].value, mus2,muscle[mus2].value);
					//printf("dP%2d: %5.2f  dP%2d: %5.2f\t", mus1,muscle[mus1].dP, mus2,muscle[mus2].dP);
					//printf("P%2d: %.2f  P%2d: %.2f\t", mus1, MusclePressure[mus1], mus2,MusclePressure[mus2]);
				}
				//usleep(20000);
			}
			*/

			statecount += state[l];
			printf("\n");
		}
	}
	*index = i;
	return statecount;
}

int AllAngleControl (unsigned long *index, int doRF){
	int temp=0, j,k,mus1,mus2,state[NUM_OF_POT_SENSOR],statecount=0;
	unsigned long i;

	i = *index;

	//while(!_kbhit())
	{
		for (j = 0; j<NUM_OF_POT_SENSOR;j++){
			read_sensor_all(i,SensorData,JointAngle,MusclePressure);
			measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);

			EndTimePoint = std::chrono::system_clock::now();
			TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
			i++;

			printf("%5d ", i);
			printf("Joint%2d ", j+1);
			printf("Act:%5.1f Set:%5.1f\t ", JointAngle[i-1][j], SetPoint_Angle[j]);

			mus1= muscle_pair[j][0];
			mus2= muscle_pair[j][1];

			state[j]= Controller(SetPoint_Angle[j],JointAngle[i-1][j],&AngleError[j],&muscle[mus1],&muscle[mus2]);
			setMuscle(muscle[mus1]);
			setMuscle(muscle[mus2]);

			if (state[j]!=1)
			{
				printf("P%2d: %.2f  P%2d: %.2f\t", mus1, muscle[mus1].value, mus2,muscle[mus2].value);
				//printf("dP%2d: %5.2f  dP%2d: %5.2f\t", mus1,muscle[mus1].dP, mus2,muscle[mus2].dP);
				//printf("Err%2d: %5.2f Err%2d: %5.2f\t", mus1,AngleError[k%2+10], mus2,AngleError[k%2+10]);
				printf("dP: %5.2f\t", muscle[mus2].dP);
				printf("Err: %5.2f\t", AngleError[k%2+10]);
			}
			//usleep(5000);
			printf("\n");

			statecount +=state[j];
		}

		/**/
		// only for RF
		if (doRF>0){
		for (k=0;k<4;k++){
			if(k<2)
				j=k;
			else
				j=k+2;
			read_sensor_all(i,SensorData,JointAngle,MusclePressure);
			measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);

			EndTimePoint = std::chrono::system_clock::now();
			TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
			i++;

			printf("%5d ", i);
			printf("Joint%2d ", j+1);
			printf("Act:%5.1f Set:%5.1f\t ", JointAngle[i-1][j], SetPoint_Angle[j]);

			mus1= muscle_pair[k%2+10][0];
			mus2= muscle_pair[k%2+10][1];

			state[j]= Controller(SetPoint_Angle[j],JointAngle[i-1][j],&AngleError[k%2+10],&muscle[mus1],&muscle[mus2]);
			setMuscle(muscle[mus1]);
			setMuscle(muscle[mus2]);


			if (state[j]!=1)
			{
				printf("P%2d: %.2f  P%2d: %.2f\t", mus1, muscle[mus1].value, mus2,muscle[mus2].value);
				//printf("dP%2d: %5.2f  dP%2d: %5.2f\t", mus1,muscle[mus1].dP, mus2,muscle[mus2].dP);
				//printf("Err%2d: %5.2f Err%2d: %5.2f\t", mus1,AngleError[k%2+10], mus2,AngleError[k%2+10]);
				printf("dP: %5.2f\t", muscle[mus2].dP);
				printf("Err: %5.2f\t", AngleError[k%2+10]);
			}
			//usleep(5000);
			printf("\n");

			statecount +=state[j];
		}
		}
		/**/
		printf("\n");
	}
	*index = i;

	if (statecount>=10)
		temp = 1;
	else if((statecount>=8)&&(statecount<10))
		temp = 0;
	else
		temp = -1;

	return temp;
}


/**********************************************************************************/

int main(int argc, char *argv[]) {

	init(); // from gpio.h
	init_pins(); // ALL 5 pins are HIGH except for GND
	init_DAConvAD5328();
	init_sensor();

	unsigned long i,j,k,l;
	unsigned int ch_num;

	clock_t laps1,laps2;
	int joint, lastjoint=0;
	int jointnum, *jointlist;
	char in,in2,msg[20];
	int logflag=0;
	int temp,state;
	int mus1,mus2;


	for (i=0;i<NUM_OF_MUSCLE;i++)
		//muscle[i].p0 = PRES_DEF;
		muscle[i].p0 = MAX_PRESSURE/2;
	muscle[TP_L].p0 = PRES_DEF-0.08;
	muscle[TP_R].p0 = PRES_DEF-0.08;
	muscle[FB_L].p0 = PRES_DEF-0.08;
	muscle[FB_R].p0 = PRES_DEF-0.08;/*
	muscle[ABD_R].p0 = MAX_PRESSURE/2;
	muscle[ADD_R].p0 = MAX_PRESSURE/2;
	muscle[ABD_L].p0 = MAX_PRESSURE/2;
	muscle[ADD_L].p0 = MAX_PRESSURE/2;
	muscle[TA_R].p0 = MAX_PRESSURE/2;
	muscle[TA_L].p0 = MAX_PRESSURE/2;
	muscle[SOL_R].p0 = MAX_PRESSURE/2;
	muscle[SOL_L].p0 = MAX_PRESSURE/2;*/

	for (i=0;i<NUM_OF_MUSCLE;i++){
		muscle[i].channel = muscle_ch[i];
		muscle[i].dP = 0;
		muscle[i].value = muscle[i].p0;
	}

	// Config IMU with default settings
	init_IMU(&device,&mtPort,PORTNAME,BAUDRATE);
	config_IMU(&device,&mtPort, outputMode, outputSettings);
	printf("IMU no problem\n");


	/* PID library */
	double SetPoint, Input, Output;
	double Kp=0,Ki=0,Kd=0;
	PID myPID(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);
  myPID.SetMode(AUTOMATIC);

	PID_ATune PIDTune(&Input,&Output);

	PIDAutotuner tuner = PIDAutotuner();

	int tuneflag=0;



	// Valve initialization
	//ResetAllValve();
	usleep(1000);

	if (argc==2){
		// converting command from argument
		std::string s(argv[1]);
		int cmd = std::stoi(s);

		switch (cmd){
	  case 1: {
	    printf("Testing Sensor\n");
		  printf("1. All ADC Data\n");
	    printf("2. Angle\n");
	    printf("3. Pressure\n");
	    printf("4. IMU\n");
			std::cout<< "Data to display "; std::cin >> in2;

			double valuetest;

			if (in2=='1'){
				printf("Sample \t");
				for (i=0;i<NUM_OF_SENSOR;i++)
					printf("%d\t",i);
			}
			else if (in2=='2')
				printf("Sample \tPot1 \tPot2 \tPot3 \tPot4 \tPot5 \tPot6 \tPot7 \tPot8 \tPot9 \tPot10 \n");
			else if (in2=='3'){
				//std::cout<< "Set Value on all Valve: "; std::cin >> valuetest;
				//SetAllValve(valuetest);
				printf("Sample \tR00 \tR01 \tR02 \tR03 \tR04 \tR05 \tR06 \tR07 \tR08 \tR09 \tR10 \tL00 \tL01 \tL02 \tL03 \tL04 \tL05 \tL06 \tL07 \tL08 \tL09 \tL10 \n");
			}

			laps1 = clock();
			StartTimePoint = std::chrono::system_clock::now();


	    for (i=0;i<SampleNum;){
	      read_sensor_all(i,SensorData,JointAngle,MusclePressure);
	      //measure_IMU(&device,&mtPort, outputMode, outputSettings, &quaternion,&euler,&calData,&sample_time);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);

				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;

				laps2 = clock();
				double elapsed = double(laps2 - laps1) / CLOCKS_PER_SEC;
				laps1 = laps2;

				int delta_t;
				// printing
				printf("\r");
				printf("[%d]\t",i);

				if (in2=='1'){
		      for (j = 0; j< NUM_ADC; j++){
						for (k = 0; k< NUM_ADC_PORT; k++){
							// show only used port
							if (j*NUM_ADC_PORT + k >= NUM_OF_SENSOR)
								break;
						  printf("%4lu\t", SensorData[i-1][j][k]);
						}
		      }
				}
				else if (in2=='2'){
					for (j = 0; j<NUM_OF_POT_SENSOR;j++){
						printf("%.1f\t", JointAngle[i-1][j]);
					}
				}
				else if (in2=='3'){
					for (j = 0; j<NUM_OF_PRES_SENSOR;j++){
						printf("%.3f\t", MusclePressure[j]);
						//printf("%.3f\t", MusclePressure[muscle_ch[j]]);
					}
				}
				else if (in2=='4'){
					printf("Roll : %5.2f\t", IMUData[i-1].euler.roll());
					printf("Pitch : %5.2f\t", IMUData[i-1].euler.pitch());
					printf("Yaw : %5.2f\t", IMUData[i-1].euler.yaw());
				}

	    }
			printf ("\n");
	    printf ("-------------------\n");
	    // fulllog("adc_acc_gyr",SensorData,SetPoint_Angle,IMUData.calData,i);

			std::cout<< "Save data (y/n)? "; std::cin >> in;
			//printf ("Save data (y/n)? ");
	    //scanf ("%c",&in);
			if (in=='y'){
				printf ("filename message: ");
		    scanf ("%s",&msg);
				fulllog(msg,SensorData,SetPoint_Angle,IMUData,i);
			}
			printf("%c, %s\n",in, msg);

	    break;
		}
		case 2:
			test_IMU();
			break;
	  case 3:{
	    printf("Testing Valve\n");
	    test_valve();
			//ResetAllValve();
	    break;
		}
	  case 4:{
	    printf("Calibrate Potentiometer\n");
			int add;

			while (1){
				std::cout<< "\nJoint No. : "; std::cin >> joint;
				std::cout<< "Current Pot ref : " << Pot_straight[joint-1] << "\n";

				std::cout<< "Correction (11.38/deg): "; std::cin >> add;
				Pot_straight[joint-1] += add;
				std::cout<< "Pot ref : " << Pot_straight[joint-1] << "\n";

				//checking
				j = joint-1;
				while(!_kbhit()){
					read_sensor_all(i,SensorData,JointAngle,MusclePressure);
					i++;
					mus1= muscle_pair[j][0];
					mus2= muscle_pair[j][1];
					state= Controller(0,JointAngle[i-1][j],&AngleError[j],&muscle[mus1],&muscle[mus2]);
					setMuscle(muscle[mus1]);
					setMuscle(muscle[mus2]);

					printf("\r");
					printf("Angle: %5.1f", JointAngle[i-1][j]);
					usleep(50000);
				}
			}
			printf ("Done\n");
	    break;
		}
		case 5:{
			printf("Testing Bang-bang\n");

			while(1){
				std::cout<< "Joint No. : "; std::cin >> joint;
				if (lastjoint!=joint){
					lastjoint = joint;
					i=0;
					StartTimePoint = std::chrono::system_clock::now();

					// ending log if joint changed, not done if it hasn't been logged
					if (logflag==2)
						logflag = endlog();

					if (joint!=0){
						std::cout<< "Saved File (y/n)? : "; std::cin >> in;
						if (in=='y'){
							std::cout<< "Message : "; std::cin >> msg;
							logflag = startlog(msg);
						}
						else
							std::cout << "Not saving file\n";
					}
				}

				if (joint>0){
					printf("Setting Joint #%d . Muscle %d\t%d\n", joint, muscle_pair[joint-1][0],muscle_pair[joint-1][1]);
					std::cout<< "SetPoint : "; std::cin >> SetPoint_Angle[joint-1];
					while(!_kbhit()){
						EndTimePoint = std::chrono::system_clock::now();
						TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();

						read_sensor_all(i,SensorData,JointAngle,MusclePressure);
						i++;
						printf("\n");
						//printf("\r");
						Input = JointAngle[i-1][joint-1];
						printf("%.1f\t%.1f\t", SetPoint_Angle[joint-1], Input);


						mus1= muscle_pair[joint-1][0];
						mus2= muscle_pair[joint-1][1];
						printf("Setting Muscle %d\t%d\t", mus1,mus2);

						BangBang(SetPoint_Angle[joint-1],Input,&muscle[mus1],&muscle[mus2]);
						setMuscle(muscle[mus1]);
						setMuscle(muscle[mus2]);

						printf("P%2d: %.2f\t P%2d: %.2f\t", mus1,muscle[mus1].value, mus2,muscle[mus2].value);
						printf("dP%2d: %5.2f\t dP%2d: %5.2f\t", mus1,muscle[mus1].dP, mus2,muscle[mus2].dP);
						printf("P%2d: %.2f\t P%2d: %.2f\t", mus1,MusclePressure[mus1], mus2,MusclePressure[mus2]);

						usleep(50000);

						// if logging
						if ((logflag==1)||(logflag==2))
							logflag = entrylog(i,SensorData,SetPoint_Angle,IMUData);
					}
					printf("\n");
				}
				else
					break;
			}
			ResetAllValve();
			break;
		}
		case 6:{
			printf ("BangBang Controller for all joints\n");

			int mode,lastmode;


			while(1){
				printf("Set SetPoint for Angle\n");
			  printf("0 : All ZERO\n");
				printf("1 : Predefined 1\n");
				printf("2 : Manually set\n");
			  printf("99 : Exit\n");
				std::cout<< "Mode : "; std::cin >> mode;
				if (mode!=99){

					// for logging
					std::cout<< "Saved File (y/n)? : "; std::cin >> in;

					StartTimePoint = std::chrono::system_clock::now();
					i=0;

					// mode for All ZERO
					if (mode==0){
						for (j = 0; j<NUM_OF_POT_SENSOR;j++){
							SetPoint_Angle[j] = 0;
						}
					}
					else if (mode==1){
						SetPoint_Angle[0] = 5;
						SetPoint_Angle[1] = 5;
						SetPoint_Angle[4] = -10;
						SetPoint_Angle[5] = -10;
						SetPoint_Angle[6] = 5;
						SetPoint_Angle[7] = 5;
					}
					else if (mode==2){
						for (j = 0; j<NUM_OF_POT_SENSOR;j++){
							std::cout<< "Joint #" << j+1 << " : ";
							std::cin >> SetPoint_Angle[j];
						}
					}

					//loop
					while(!_kbhit()){
						printf("\r");
						for (j = 0; j<NUM_OF_POT_SENSOR;j++){
							read_sensor_all(i,SensorData,JointAngle,MusclePressure);
							measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);

							EndTimePoint = std::chrono::system_clock::now();
							TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
							i++;

							printf("Joint %2d. ", j+1);
							printf("Act: %5.1f\t SetPoint: %5.1f\t ", JointAngle[i-1][j], SetPoint_Angle[j]);

							mus1= muscle_pair[j][0];
							mus2= muscle_pair[j][1];

							state= BangBang(SetPoint_Angle[j],JointAngle[i-1][j],&muscle[mus1],&muscle[mus2]);
							setMuscle(muscle[mus1]);
							setMuscle(muscle[mus2]);

							if (state!=1)
							{
								printf("P%2d: %.2f\t P%2d: %.2f\t", mus1, muscle[mus1].value, mus2,muscle[mus2].value);
								printf("dP%2d: %5.2f\t dP%2d: %5.2f\t", mus1,muscle[mus1].dP, mus2,muscle[mus2].dP);
								printf("P%2d: %.2f\t P%2d: %.2f\t", mus1, MusclePressure[mus1], mus2,MusclePressure[mus2]);
							}


							usleep(20000);
							printf("\n");
						}

						// only for RF
						for (k=0;k<4;k++){
							if(k<2)
								j=k;
							else
								j=k+2;

							read_sensor_all(i,SensorData,JointAngle,MusclePressure);
							measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);

							EndTimePoint = std::chrono::system_clock::now();
							TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
							i++;

							printf("Joint %2d. ", j+1);
							printf("Act: %5.1f\t SetPoint: %5.1f\t ", JointAngle[i-1][j], SetPoint_Angle[j]);

							mus1= muscle_pair[k%2+10][0];
							mus2= muscle_pair[k%2+10][1];

							state= BangBang(SetPoint_Angle[j],JointAngle[i-1][j],&muscle[mus1],&muscle[mus2]);
							setMuscle(muscle[mus1]);
							setMuscle(muscle[mus2]);


							if (state!=1)
							{
								printf("P%2d: %.2f\t P%2d: %.2f\t", mus1, muscle[mus1].value, mus2,muscle[mus2].value);
								printf("dP%2d: %5.2f\t dP%2d: %5.2f\t", mus1,muscle[mus1].dP, mus2,muscle[mus2].dP);
								printf("P%2d: %.2f\t P%2d: %.2f\t", mus1, MusclePressure[mus1], mus2,MusclePressure[mus2]);
							}

							usleep(20000);
							printf("\n");
						}

						printf("\n");

						usleep(1000);
					}

					// if logging
					//if ((logflag==1)||(logflag==2))
						//logflag = endlog();

					// full log version to minimize time
					if (in=='y'){
						std::cout<< "Message : "; std::cin >> msg;
						fulllog(msg,SensorData,SetPoint_Angle,IMUData,i);
					}
					else
						std::cout << "Not saving file\n";

					printf("\n");
				}
				else
					break;
			}

			//ResetAllValve();
			break;
		}
		case 7:{
			printf("Testing Joint Control\n");
			while(1){
				std::cout<< "Num of Joints : "; std::cin >> jointnum;
				if (jointnum>0){
					jointlist = new int[jointnum];
					for (j=0;j<jointnum;j++){
						std::cout<< "Joint #" << j+1 << " No: "; std::cin >> joint;
						jointlist[j] = joint;
						std::cout<< "SetPoint : "; std::cin >> SetPoint_Angle[joint-1];
					}

					StartTimePoint = std::chrono::system_clock::now();
					i=0;

					while (!_kbhit()){
						JointAngleControl(jointlist,jointnum,&i);
					}

					// for logging
					std::cout<< "Saved File (y/n)? : "; std::cin >> in;
					if (in=='y'){
						std::cout<< "Message : "; std::cin >> msg;
						fulllog(msg,SensorData,SetPoint_Angle,IMUData,i);
					}
					else
						std::cout << "Not saving file\n";

					printf("\n");
				}
				else
					break;
			}
			break;
		}
		case 8:{
			printf ("Controller Test\n");

			int mode,lastmode;

			while(1){
				printf("Set SetPoint for Angle\n");
			  printf("0 : All ZERO\n");
				printf("1 : Predefined 1\n");
				printf("2 : Manually set\n");
			  printf("99 : Exit\n");
				std::cout<< "Mode : "; std::cin >> mode;
				if (mode!=99){

					// for logging
					std::cout<< "Saved File (y/n)? : "; std::cin >> in;

					StartTimePoint = std::chrono::system_clock::now();
					i=0;

					// mode for All ZERO
					if (mode==0){
						for (j = 0; j<NUM_OF_POT_SENSOR;j++){
							SetPoint_Angle[j] = 0;
						}
					}
					else if (mode==1){
						SetPoint_Angle[0] = 5;
						SetPoint_Angle[1] = 5;
						SetPoint_Angle[4] = -10;
						SetPoint_Angle[5] = -10;
						SetPoint_Angle[6] = 5;
						SetPoint_Angle[7] = 5;
					}
					else if (mode==2){
						for (j = 0; j<NUM_OF_POT_SENSOR;j++){
							std::cout<< "Joint #" << j+1 << " : ";
							std::cin >> SetPoint_Angle[j];
						}
					}

					//loop
					setMuscle(muscle[RF_R]);
					setMuscle(muscle[RF_L]);
					while (!_kbhit()){
						AllAngleControl(&i,1);	// 0:w/o RF 1:w/ RF
					}

					// if logging
					//if ((logflag==1)||(logflag==2))
						//logflag = endlog();

					// full log version to minimize time
					if (in=='y'){
						std::cout<< "Message : "; std::cin >> msg;
						fulllog(msg,SensorData,SetPoint_Angle,IMUData,i);
					}
					else
						std::cout << "Not saving file\n";

					printf("\n");
				}
				else
					break;
			}

			//ResetAllValve();
			break;
		}
		// ==================================================================================================
		// 												JUMPING
		// ==================================================================================================
		case 9:{
			printf ("Jumping after Standing\n");
			//usleep(2000000);
			int sw = 0;
			int sw_angle = -30;//-25; //25 default
			state = 0;
			int standflag =0;
			int flagged = 0;

			// ==============================================================================
			// ==================== SETUP ===================================================

			//  ----- JUMPING Activation ----
			int active1 [6] = {GMAX_R, GMAX_L, VAS_R, VAS_L, RF_R, RF_L};
			int counter1[4] = {IL_R, IL_L, HAM_R, HAM_L};
			int active2 [2] = {SOL_R, SOL_L};//, TP_R, TP_L, FB_R, FB_L};
			int counter2[2] = {TA_R, TA_L};

			int musjumprelease[]= {IL_L_CH,IL_R_CH,GMAX_L_CH,GMAX_R_CH,VAS_L_CH,VAS_R_CH,HAM_L_CH,HAM_R_CH,
									TA_L_CH,TA_R_CH,SOL_L_CH,SOL_R_CH,RF_L_CH,RF_R_CH};

			// ------ STEPPING Activation ----
			/**/
			// Right Support Left Swing
			// ------- THRUST -----
			int thrust1[] = {RF_R,GMAX_R};
			int nthrust1[] ={HAM_R,IL_R};
			int thrust2[] = {TA_R};
			int nthrust2[]=	{SOL_R,TP_R,FB_R};
			// ------- SWING ----------
			int gait1 [2] = {IL_L, HAM_L};
			int gait2 [3] = {RF_L,VAS_L,TA_L};
			int gait3 [3] = {GMAX_L,HAM_L,RF_L};

			int mussteprelease[]= {IL_L_CH,GMAX_L_CH,VAS_L_CH,HAM_L_CH,TA_L_CH,SOL_L_CH,RF_L_CH,};
			/**/
			/*
			// Left Support Right Swing
			// ------- THRUST -----
			int thrust1[] = {RF_L,GMAX_L};
			int nthrust1[] ={HAM_L,IL_L};
			int thrust2[] = {TA_L};
			int nthrust2[]=	{SOL_L,TP_L,FB_L};
			// ------- SWING ----------
			int gait1 [2] = {IL_R, HAM_R};
			int gait2 [3] = {RF_R,VAS_R,TA_R};
			int gait3 [3] = {GMAX_R,HAM_R,RF_R};

			int mussteprelease[]= {IL_R_CH,GMAX_R_CH,VAS_R_CH,HAM_R_CH,TA_R_CH,SOL_R_CH,RF_R_CH,};
			*/


			jointnum=2;
			jointlist = new int[jointnum];
			jointlist[0] = 1; jointlist[1] = 2;

			// ===== initial posture ========================================================
			for (j = 0; j<NUM_OF_POT_SENSOR;j++){
				SetPoint_Angle[j] = 0;
			}

			StartTimePoint = std::chrono::system_clock::now();
			i=0;
			setMuscle(muscle[RF_R]);
			setMuscle(muscle[RF_L]);
			//while (!_kbhit()){
			while (sw<50){
				sw+=AllAngleControl(&i,1); // 0:w/o RF 1:w/ RF
				if (sw<0)
					sw=0;
			}
			sw=0; //reset flag

			//loop for preparation before standing
			while(!_kbhit()){
			}

			// ===================================================================================
			// ==================== 1st MOTION ===================================================

			// ======= PRE JUMP #1 ============================================================
			//releasing muscle tension
			ResetValve(musjumprelease);

			// activation switch based on joint angle. knee < sw_angle 25

			while (!sw){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;

				sw = ((JointAngle[i-1][4] < sw_angle) || (JointAngle[i-1][5] < sw_angle));

				printf ("\r");
				printf("Knee Joint5: %.2f\t Joint6: %.2f\t", JointAngle[i-1][4], JointAngle[i-1][5]);
			}

			// ======= JUMP #1 ============================================================

			// Jumping activation set
			std::cout << "\n" << i <<"\tJUMP!\n";

			for (j=0; j<6; j++){
				muscle[active1[j]].value = MAX_PRESSURE;
				if (j<2){
					muscle[active2[j]].value = 0.6;
					muscle[counter2[j]].value = 0.1;
				}
			}
			muscle[counter1[0]].value = MAX_PRESSURE/4; //IL_R
			muscle[counter1[1]].value = MAX_PRESSURE/4; //IL_L
			muscle[counter1[2]].value = MAX_PRESSURE/4; //HAM_R
			muscle[counter1[3]].value = MAX_PRESSURE/4; //HAM_L

			// ACTIVE #1
			for (j=0; j<6; j++){
				setMuscle(muscle[active1[j]]);
				if (j<4)
					setMuscle(muscle[counter1[j]]);
			}

			//Delay 10ms  -----> may affect jumping performance (?)
			for (j=0; j<1; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}

			// ACTIVE #2
			for (j=0; j<2; j++){
				setMuscle(muscle[active2[j]]);
				setMuscle(muscle[counter2[j]]);
			}

			//Delay 300ms
			// need 30seq ~300ms to pressurozed muscle to jump
			for (j=0; j<20; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}

			/**/
			// ======= POST JUMP #1 ============================================================
			// Let the robot jump first then prepare for flexing knee
			// 1seq ~ 10ms

			std::cout << "\n" << i << "\n";

			//set RF=0,GMAX =0.5, IL HAM = 0.6
			// Knee Flexing
			setMusclenewVal(muscle[active1[4]],0);		//RF
			setMusclenewVal(muscle[active1[5]],0);
			setMusclenewVal(muscle[counter1[2]],0.2);	//HAM
			setMusclenewVal(muscle[counter1[3]],0.2);
			// Ankle Right flexion
			setMusclenewVal(muscle[SOL_R],0);
			setMusclenewVal(muscle[TA_R],0.6);
			// Hip right flexion ===> for landing position
			setMusclenewVal(muscle[IL_R],0.2);
			setMusclenewVal(muscle[GMAX_R],0.2);

			/**/

			for (j=0; j<20; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}

			// ==================== DOUBLE JUMP ==================================================

			/*
			// ======= PRE JUMP #2 ============================================================
			// Landing Preparation
			// delay 100ms
			for (j=0; j<10; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}
			ResetValve(musjumprelease);

			// 2nd Jumping activation
			// activation switch based on joint angle
			// needed time from release->jump =~ 400ms. break when already jump.
			flagged = 0;
			sw=0;
			while (!sw){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;

				sw = ((JointAngle[i-1][4] < sw_angle) || (JointAngle[i-1][5] < sw_angle));

				printf ("\r");
				printf("Joint5: %.2f\t Joint6: %.2f\t", JointAngle[i-1][4], JointAngle[i-1][5]);
			}


			// ======= JUMP #2 ============================================================

			// Jumping activation set
			std::cout << "2nd JUMP!\n";
			muscle[active1[0]].value = 0.5;  // GMAX reduced for inclining forward
			muscle[active1[1]].value = 0.5;  // GMAX reduced for inclining forward

			// ACTIVE #1
			for (j=0; j<6; j++){
				setMuscle(muscle[active1[j]]);
				if (j<4)
					setMuscle(muscle[counter1[j]]);
			}

			//Delay 50ms  -----> may affect jumping performance (?)
			//Delay 80ms --> since there is no delay on activating muscle
			for (j=0; i<5; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}

			// ACTIVE #2
			for (j=0; j<2; j++){
				setMuscle(muscle[active2[j]]);
			}
			*/

			/**/
			// ===================================================================================
			// ==================== TRANSITION ===================================================

			// Landing Preparation
			// activation switch based on ankle joint  ===> impact on landing
			flagged = 0;
			sw=0;

			//while (!sw){
				//read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				//measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				//EndTimePoint = std::chrono::system_clock::now();
				//TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				//i++;
				//
				//sw = ((JointAngle[i-1][6] > JointAngle[i-2][6]) || (JointAngle[i-1][7] > JointAngle[i-2][6]));
				//printf ("\r");
				//printf("Ankle Joint7: %.2f\t Joint8: %.2f\t", JointAngle[i-1][6], JointAngle[i-1][7]);
			//}


			// ===================================================================================
			// ==================== 2nd MOTION  ==================================================
			// ======= PRE STEP ================================
			//releasing muscle tension for swing leg

			//ResetValve(mussteprelease);
			//
			//for (j=0; j<10; j++){
				//read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				//measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				//EndTimePoint = std::chrono::system_clock::now();
				//TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				//i++;
			//}

			setMusclenewVal(muscle[GMAX_L],0);
			setMusclenewVal(muscle[HAM_L],0);

			// ===================================================================================
			// ======= STEP  ====================================
			std::cout << "\n" << i <<"\tSTEP!\n";
			// SWING


			// -- GAIT1 --
			muscle[gait1[0]].value=0.8;
			muscle[gait1[1]].value=0;
			for(j=0;j<2;j++){
				setMuscle(muscle[gait1[j]]);
			}

			//THRUST
			//muscle[thrust1[0]].value=0.8;
			//muscle[thrust1[1]].value=0.8;
			//muscle[nthrust1[0]].value=0;
			//muscle[nthrust1[1]].value=0.2;
			//muscle[thrust2[0]].value=0.8;
			//muscle[nthrust2[0]].value=0;
			//muscle[nthrust2[1]].value=0;
			//muscle[nthrust2[2]].value=0;
			//for(j=0;j<2;j++){
				//setMuscle(muscle[nthrust1[j]]);
				//setMuscle(muscle[thrust1[j]]);
			//}
			//for(j=0;j<3;j++){
				//setMuscle(muscle[nthrust1[j]]);
			//}
			//setMuscle(muscle[thrust2[0]]);


			setMusclenewVal(muscle[RF_R],0);
			setMusclenewVal(muscle[HAM_R],0.2);
			setMusclenewVal(muscle[ADD_R],0.8);

			// -- GAIT2 --
			//delay
			for (j=0; j<5; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}
			//THRUST with SOL
			muscle[thrust2[0]].value=0;	//TA
			muscle[nthrust2[0]].value=0.8;
			muscle[nthrust2[1]].value=0.4;
			muscle[nthrust2[2]].value=0.4;
			setMuscle(muscle[thrust2[0]]);
			for(j=0;j<3;j++){
				setMuscle(muscle[nthrust2[j]]);
			}
			for (j=0; j<5; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}
			// SWING
			setMusclenewVal(muscle[gait1[1]],0);
			muscle[gait2[0]].value=0.2;
			muscle[gait2[1]].value=0.4;
			muscle[gait2[2]].value=0.3;
			for(j=0;j<3;j++){
				setMuscle(muscle[gait2[j]]);
			}

			// -- GAIT3 --
			//delay
			// 30 for BEST on jumpstep36
			for (j=0; j<20; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}
			// SWING
			setMusclenewVal(muscle[gait1[0]],0.2);
			setMusclenewVal(muscle[gait2[0]],0);
			muscle[gait3[0]].value=0.4;
			muscle[gait3[1]].value=0.2;
			muscle[gait3[2]].value=0.2;
			for(j=0;j<3;j++){
				setMuscle(muscle[gait3[j]]);
			}
			/**/

			// ===================================================================================
			// ==================== END OF MOTION=================================================
			//data reading for the rest
			printf("%d\n", i);
			for (j=i; i<SampleNum; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
				printf ("\r");
				printf("%d", i);
			}

			std::cout<< "Save data (y/n)? "; std::cin >> in;
			//printf ("Save data (y/n)? ");
	    //scanf ("%c",&in);
			if (in=='y'){
				printf ("filename message: ");
		    scanf ("%s",&msg);
				fulllog(msg,SensorData,SetPoint_Angle,IMUData,i);
			}

			//ResetAllValve();
			break;
		}
		// ======================================================================================================================================================
		// 												STEPPING
		// ======================================================================================================================================================
		case 10:{
			printf("Stepping Gait\n");

			int sw = 0;
			int sw_angle = -30;//-25; //25 default
			state = 0;
			int standflag =0;

			// ==============================================================================
			// ==================== SETUP ===================================================

			// ------ STEPPING Activation ----
			/**/
			// Right Support Left Swing
			// ------- THRUST -----
			int thrust1[] = {RF_R,GMAX_R};
			int nthrust1[] ={HAM_R,IL_R};
			int thrust2[] = {TA_R};
			int nthrust2[]=	{SOL_R,TP_R,FB_R};
			// ------- SWING ----------
			int gait1 [2] = {IL_L, HAM_L};
			int gait2 [3] = {RF_L,VAS_L,TA_L};
			int gait3 [3] = {GMAX_L,HAM_L,RF_L};

			int mussteprelease[]= {IL_L_CH,GMAX_L_CH,VAS_L_CH,HAM_L_CH,TA_L_CH,SOL_L_CH,RF_L_CH,};
			/**/
			/*
			// Left Support Right Swing
			// ------- THRUST -----
			int thrust1[] = {RF_L,GMAX_L};
			int nthrust1[] ={HAM_L,IL_L};
			int thrust2[] = {TA_L};
			int nthrust2[]=	{SOL_L,TP_L,FB_L};
			// ------- SWING ----------
			int gait1 [2] = {IL_R, HAM_R};
			int gait2 [3] = {RF_R,VAS_R,TA_R};
			int gait3 [3] = {GMAX_R,HAM_R,RF_R};

			int mussteprelease[]= {IL_R_CH,GMAX_R_CH,VAS_R_CH,HAM_R_CH,TA_R_CH,SOL_R_CH,RF_R_CH,};
			*/

			jointnum=2;
			jointlist = new int[jointnum];
			jointlist[0] = 1; jointlist[1] = 2;

			// ===== initial posture ========================================================
			for (j = 0; j<NUM_OF_POT_SENSOR;j++){
				SetPoint_Angle[j] = 0;
			}

			StartTimePoint = std::chrono::system_clock::now();
			i=0;
			setMuscle(muscle[RF_R]);
			setMuscle(muscle[RF_L]);
			//while (!_kbhit()){
			while (sw<50){
				sw+=AllAngleControl(&i,1); // 0:w/o RF 1:w/ RF
				if (sw<0)
					sw=0;
			}
			sw=0; //reset flag

			//loop for preparation before standing
			while(!_kbhit()){
			}

			// ======= PRE STEP ============================================================
			//releasing muscle tension
			/*
			ResetValve(mussteprelease);

			for (j=0; j<10; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}
			*/

			setMusclenewVal(muscle[GMAX_L],0);
			setMusclenewVal(muscle[HAM_L],0);


			// ===================================================================================
			// ======= STEP  ====================================
			std::cout << "\n" << i <<"\tSTEP!\n";
			// SWING


			// -- GAIT1 --
			muscle[gait1[0]].value=0.8;
			muscle[gait1[1]].value=0;
			for(j=0;j<2;j++){
				setMuscle(muscle[gait1[j]]);
			}
			/*
			//THRUST
			muscle[thrust1[0]].value=0.8;
			muscle[thrust1[1]].value=0.8;
			muscle[nthrust1[0]].value=0;
			muscle[nthrust1[1]].value=0.2;
			muscle[thrust2[0]].value=0.8;
			muscle[nthrust2[0]].value=0;
			muscle[nthrust2[1]].value=0;
			muscle[nthrust2[2]].value=0;
			for(j=0;j<2;j++){
				setMuscle(muscle[nthrust1[j]]);
				setMuscle(muscle[thrust1[j]]);
			}
			for(j=0;j<3;j++){
				setMuscle(muscle[nthrust1[j]]);
			}
			setMuscle(muscle[thrust2[0]]);
			*/

			setMusclenewVal(muscle[RF_R],0);
			setMusclenewVal(muscle[HAM_R],0.2);
			// for after jumping
			//setMusclenewVal(muscle[ADD_R],0.8);
			// for standing
			setMusclenewVal(muscle[ABD_R],0.8);
			//setMusclenewVal(muscle[FB_R],0.8);
			setMusclenewVal(muscle[TA_R],0);
			setMusclenewVal(muscle[SOL_R],0);


			// -- GAIT2 --
			//delay
			for (j=0; j<10; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}

			//THRUST with SOL
			muscle[thrust2[0]].value=0;	//TA
			muscle[nthrust2[0]].value=0.8;
			muscle[nthrust2[1]].value=0.4;
			muscle[nthrust2[2]].value=0.4;
			setMuscle(muscle[thrust2[0]]);
			for(j=0;j<3;j++){
				setMuscle(muscle[nthrust2[j]]);
			}
			// SWING
			setMusclenewVal(muscle[gait1[1]],0);
			muscle[gait2[0]].value=0.2;
			muscle[gait2[1]].value=0.4;
			muscle[gait2[2]].value=0.3;
			for(j=0;j<3;j++){
				setMuscle(muscle[gait2[j]]);
			}

			// -- GAIT3 --
			//delay
			for (j=0; j<30; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
			}
			// SWING
			setMusclenewVal(muscle[gait1[0]],0.2);
			setMusclenewVal(muscle[gait2[0]],0);
			muscle[gait3[0]].value=0.4;
			muscle[gait3[1]].value=0.2;
			muscle[gait3[2]].value=0.2;
			for(j=0;j<3;j++){
				setMuscle(muscle[gait3[j]]);
			}

			// ===================================================================================
			// ==================== END OF MOTION=================================================
			//data reading for the rest
			printf("%d\n", i);
			for (j=i; i<SampleNum; j++){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;
				printf ("\r");
				printf("%d", i);
			}

			std::cout<< "Save data (y/n)? "; std::cin >> in;
			if (in=='y'){
				printf ("filename message: ");
		    scanf ("%s",&msg);
				fulllog(msg,SensorData,SetPoint_Angle,IMUData,i);
			}

			break;
		}

		case 98:{
			printf("Check sensor reading time\n");
			StartTimePoint = std::chrono::system_clock::now();
			i=0;
			read_sensor_all(i,SensorData,JointAngle,MusclePressure);
			measure_IMU(&device,&mtPort, outputMode, outputSettings, &IMUData[i]);
			EndTimePoint = std::chrono::system_clock::now();
			TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
			i++;
			std::cout<< "\nDuration time: " << TimeStamp[i-1] << "ms\n";
			break;
		}
		case 99:{
			printf("Testing update Kp Ki Kd\n");
			while (1){
				std::cout<< "Kp :"; std::cin >> Kp;
				std::cout<< "Ki :"; std::cin >> Ki;
				std::cout<< "Kd :"; std::cin >> Kd;
				saveTunings(Kp,Ki,Kd);
				Kp=0;Ki=0;Kd=0;
				std::cout << Kp << "\t" << Ki << "\t" << Kd << "\n";
				loadTunings(&Kp,&Ki,&Kd);
				std::cout << Kp << "\t" << Ki << "\t" << Kd << "\n";
			}
			break;
		}
		case 0:{
			printf("Reseting All Valves\n");
			ResetAllValve();
			break;
		}
		case 100:{
			printf("Reseting All Valves\n");
			setMusclenewVal(muscle[SOL_R_CH],0);
			setMusclenewVal(muscle[TP_R_CH],0);
			setMusclenewVal(muscle[FB_R_CH],0);
			setMusclenewVal(muscle[TA_R_CH],0);
			usleep(500000);
			setMusclenewVal(muscle[TA_R_CH],0.8);
			break;
		}
	  }
	}
	else{
	  printf("Please start program with an argument:\n");
	  printf("1 : Testing Sensor\n");
		printf("2 : Testing IMU Sensor\n");
	  printf("3 : Testing a desired Muscle/Valve with desired pressure\n");
	  printf("4 : Calibrate Potentiometer\n");
	  printf("5 : Testing Bang-bang 1 joint\n");
	  printf("6 : Bang-bang Controller\n");
	  printf("7 : Testing 1 Joint Control\n");
	  printf("8 : P Controller\n");
	  printf("9 : Jump!\n");
	  printf("10: A\n");
	  printf("99: Testing saving loading file\n");
	  printf("99: Testing saving loading file\n");
	  printf("0 : Reset Valve\n");
	}


	return 0;
}
