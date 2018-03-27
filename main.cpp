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
#include "XsensIMU.h"
/*
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
*/

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

#define IL_R		0
#define GMAX_R	1
#define VAS_R		2
#define HAM_R		3
#define TA_R 		4
#define SOL_R 	5
#define ADD_R 	6
#define ABD_R 	7
#define TP_R		8
#define FB_R		9
#define RF_R		10

#define IL_L		16
#define GMAX_L	17
#define VAS_L		18
#define HAM_L		19
#define TA_L 		20
#define SOL_L 	21
#define ADD_L 	22
#define ABD_L 	23
#define TP_L		24
#define FB_L		25
#define RF_L		26

/* VALUE */
#define MAX_PRESSURE 0.8
#define MIN_PRESSURE 0
// Default Pressure
#define PRES_DEF 0.3

#define MAX_SAMPLE_NUM 100000


/*************************************************************/
/**                   GLOBAL VARIABLES                      **/
/*************************************************************/

long SampleNum = 20000;
unsigned long SensorValue[NUM_ADC][NUM_ADC_PORT];

/* for sampling time */
std::chrono::system_clock::time_point StartTimePoint, EndTimePoint;
double TimeStamp[MAX_SAMPLE_NUM];

/* Table for muscle valve number and sensor number */
int muscle_sensor [NUM_OF_MUSCLE] = {PIN_PRES_1,PIN_PRES_2};

/* Table for muscle valve channel */
int muscle_ch [NUM_OF_MUSCLE] = {IL_R,GMAX_R,VAS_R,HAM_R,TA_R,SOL_R,ADD_R,
																	ABD_R,TP_R,FB_R,RF_R,
																 IL_L,GMAX_L,VAS_L,HAM_L,TA_L,SOL_L,ADD_L,
															 		ABD_L,TP_L,FB_L,RF_L};

/* muscle pair for a joint */
#define muscle_pair_num 12
// +2 for extra biarticular muscle RF
int muscle_pair [muscle_pair_num][2] = {{IL_R,GMAX_R}, {IL_L,GMAX_L},
																					{ABD_R,ADD_R}, {ABD_L,ADD_L},
																					{VAS_R,HAM_R}, {VAS_L,HAM_L},
																					{TA_R,SOL_R}, {TA_L,SOL_L},
																					{FB_R,TP_R}, {FB_L,TP_L},
																					{RF_R,99}, {RF_L,99},			// for biarticular, can not use NULL=0
																				};


/* Potentiometer reference for zero degree */
int Pot_straight [10] = {2128,2346,2305,3090,2336,2167,1446,1876,2118,2010};

// Angle on same pressure p=0.3
int Pot_Psame 		[10] = {	1820,	2665,	2383,	3034,	2641,	1817,	1573,	1759,	2199,	1931};
double Angle_Psame [10] = {	27.1,	28.0,	2.3,	4.9, -26.8,-30.8,-11.2,-10.3, -7.1,	-6.9};

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
//				 Pot_ref	: ADC Value for 0 degree angled
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
void read_sensor_all (int index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], double *Angle, double *Pressure){
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
				Angle[index_adc] = ADCtoAngle(tmp_val0[k],Pot_straight[index_adc]);
			}
			// Getting Pressure from Pressure SENSOR
			else if(index_adc >= NUM_OF_POT_SENSOR){

				// for first board (Right) valve 0-10
				if ((index_adc-NUM_OF_POT_SENSOR) < (NUM_OF_MUSCLE/2))
					Pressure[index_adc-NUM_OF_POT_SENSOR] = ADCtoPressure(tmp_val0[k]);
				// second board (Left) valve 16-26
				else
					Pressure[index_adc-NUM_OF_POT_SENSOR - (NUM_OF_MUSCLE/2) + NUM_OF_CHANNELS] = ADCtoPressure(tmp_val0[k]);

			}
    }
  }
	// Adjusting Angle direction
	Angle[0] = -1* Angle[0];
	Angle[3] = -1* Angle[3];
	Angle[4] = -1* Angle[4];
	Angle[6] = -1* Angle[6];
	Angle[8] = -1* Angle[8];
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
	 						double *SetPoint_Angle, XsCalibratedData *calData){
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

		// Potentiometer Value, Column #1~#10
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

		// Angle Set Point Value, Column #11~#20
		for (j = 0; j<NUM_OF_POT_SENSOR;j++){
			// converting float to string
			std::string strs = ","+ std::to_string(SetPoint_Angle[j]);
			fputs(strs.c_str(), fp);
		}

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
		std::string strs = ","+ std::to_string(TimeStamp[index]);
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
	XsCalibratedData *dummy;
  logging(mode,message,NULL,NULL,NULL,dummy);
	return mode;
}

/* input log on specific index */
int entrylog(int  index, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT], double *SetPoint_Angle,
							XsCalibratedData *calData){
	int mode = 2;
	logging(mode,"",index,SensorVal,SetPoint_Angle,calData);
	return mode;
}

/* close file */
int endlog() {
	int mode = 3;
	XsCalibratedData *dummy;
  logging(mode,"",NULL,NULL,NULL,dummy);
	return mode;
}

/* */
void fulllog(const char* message, unsigned long SensorVal[][NUM_ADC][NUM_ADC_PORT],
						 double *SetPoint_Angle, XsCalibratedData *calData){
  int i;
  startlog(message);
  for (i=0;i<SampleNum;i++){
    entrylog (i,SensorVal,SetPoint_Angle,calData);
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

void Reset_Valve (){
	int i;
	for (i=0;i< (NUM_DAC*NUM_OF_CHANNELS) ;i++){
		if (i%16 < (NUM_OF_MUSCLE/2)){
			setState(i,0.0);
			//printf("Valve #%d off\n",i);
		}
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

void SetFrontalPlaneMuscle (double value){
	setState(ADD_R,value);
	setState(ADD_L,value);
	setState(ABD_R,value);
	setState(ABD_L,value);
	setState(TP_R,value);
	setState(TP_L,value);
	setState(FB_R,value);
	setState(FB_L,value);
}

/*===== Running test to print all the Sensor Data ======*/
/* Read and print only ONCE for all sensor */
void test_sensor (int SampleNum){
  int index,j,k;
  static unsigned long Value[MAX_SAMPLE_NUM][NUM_ADC][NUM_ADC_PORT];
	double Angle[NUM_OF_POT_SENSOR];
	double Pressure[NUM_DAC*NUM_OF_CHANNELS];

  //startlog("test_sensor");

  for(index=0;index<SampleNum;index++){
    read_sensor_all(index,Value,Angle,Pressure);
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

  int valve_count, valve_num[NUM_OF_MUSCLE];
  static int i=0;
	int j;
  double val [NUM_OF_MUSCLE],sensorval;
  static unsigned long Value[MAX_SAMPLE_NUM][NUM_ADC][NUM_ADC_PORT];
	double Angle[NUM_OF_POT_SENSOR];
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
				//for (j = 0; j<NUM_OF_POT_SENSOR;j++){
				//	printf("%.1f\t", Angle[j]);
				//}

				for (j = 0; j<valve_count;j++){
					printf("Valve %d: %.3f\t", valve_num[j], Pressure[valve_num[j]]);
				}
				/*
				for (j = 0; j<valve_count;j++){
					if (valve_num[j]< (NUM_OF_MUSCLE/2))
						printf("Right Valve %d: %.3f\t", valve_num[j], Pressure[valve_num[j]]);
					else
						printf("Left Valve %d: %.3f\t", valve_num[j], Pressure[valve_num[j]- NUM_OF_CHANNELS + (NUM_OF_MUSCLE/2)]);
				*/
				printf("\n");
				i++;
			}
		}
		else
			break;
  }
}

// Output Saturation
void OutputSaturation (double *MuscleVal){
	if (*MuscleVal > MAX_PRESSURE)
		*MuscleVal = MAX_PRESSURE;
	else if (*MuscleVal < MIN_PRESSURE)
		*MuscleVal = MIN_PRESSURE;
}

// Bang-Bang Controller Test
int BangBang (double SetPoint, double RefPoint, double *UpMuscleVal, double *DownMuscleVal){

	double error = 2; // error compensation
	static double dP =0;
	int temp;
	static double lastCh1, lastCh2;

	if (!((lastCh1== *UpMuscleVal)&&(lastCh2== *DownMuscleVal))){
		//printf("differ\n");
		lastCh1 = *UpMuscleVal;
		lastCh2 = *DownMuscleVal;
		dP=0;
	}

	if (RefPoint < SetPoint - error){
		//printf("Below SetPoint");
		//if (PRES_DEF+dP < MAX_PRESSURE)
		if (dP < MAX_PRESSURE)
			dP+=0.02;

		// flag for stability
		temp=-1;
	}
	else if (RefPoint > SetPoint + error){
		//printf("Above SetPoint");
		//if (PRES_DEF-dP > MIN_PRESSURE)
		if (abs(dP) < MAX_PRESSURE)
			dP-=0.02;

		// flag for stability
		temp=-1;
	}
	else{
		//printf("Reach SetPoint");
		temp=1;
	}

	/*
	setState(UpMuscleChannel,PRES_DEF+dP);
	setState(DownMuscleChannel,PRES_DEF-dP);
	usleep(50000);
	*/

	//printf("dP: %.1f\t", dP);

	// Output Saturation
	if ((*UpMuscleVal+dP < MAX_PRESSURE)&&(*UpMuscleVal+dP > MIN_PRESSURE))
		*UpMuscleVal += dP;
	//if (*UpMuscleVal > MAX_PRESSURE)
		//*UpMuscleVal = MAX_PRESSURE;
	//else if (*UpMuscleVal < MIN_PRESSURE)
		//*UpMuscleVal = MIN_PRESSURE;

	if ((*DownMuscleVal-dP < MAX_PRESSURE)&&(*DownMuscleVal-dP > MIN_PRESSURE))
		*DownMuscleVal -= dP;
	//if (*DownMuscleVal > MAX_PRESSURE)
		//*DownMuscleVal = MAX_PRESSURE;
	//else if (*DownMuscleVal < MIN_PRESSURE)
		//*DownMuscleVal = MIN_PRESSURE;
	return temp;
}



/**********************************************************************************/

int main(int argc, char *argv[]) {

	init(); // from gpio.h
	init_pins(); // ALL 5 pins are HIGH except for GND
	init_DAConvAD5328();
	init_sensor();



	init_IMU(&device,&mtPort,PORTNAME,BAUDRATE);
	// Config IMU with default settings
	//config_IMU(&device,&mtPort, DEFAULT_OUTPUT_MODE, DEFAULT_OUTPUT_SETTINGS);

	XsOutputMode outputMode = XOM_Calibrated;
	XsOutputSettings outputSettings = XOS_CalibratedMode_All;

	unsigned long i,j,k,l;
	unsigned int ch_num;

	/* Variable for ADC Board Data */
	// ==============>>>>>>.  Problem here for a lot of Data
	unsigned long SensorData[SampleNum][NUM_ADC][NUM_ADC_PORT];

	double JointAngle[NUM_OF_POT_SENSOR];
	double SetPoint_Angle[NUM_OF_POT_SENSOR] ={0};

	double MusclePressure[NUM_DAC*NUM_OF_CHANNELS];

	// muscle pressure val init
	double muscle_pair_val [muscle_pair_num][2];
	for (j=0;j<muscle_pair_num;j++){
		for (k=0;k<2;k++){
			//muscle_pair_val [j][k] = PRES_DEF;
			muscle_pair_val [j][k] = 0;
		}
	}

	/* PID library */
	double SetPoint, Input, Output;
	double Kp=0,Ki=0,Kd=0;
	PID myPID(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);
  myPID.SetMode(AUTOMATIC);

	PID_ATune PIDTune(&Input,&Output);

	PIDAutotuner tuner = PIDAutotuner();

	int tuneflag=0;

	clock_t laps1,laps2;
	int joint, lastjoint=0;

	// Valve initialization
	Reset_Valve();
	usleep(1000);

	char in,in2,msg[20];
	int logflag=0;

	if (argc==2){
	  switch (*argv[1]){
	  case '1': {
	    printf("Testing Sensor\n");
			double valuetest;
			//printf("Set Value on all Valve: "); scanf ("%lf",&valuetest);
			std::cout<< "Set Value on all Valve: "; std::cin >> valuetest;
			SetAllValve(valuetest);

			//printf("Sample \tPot1 \tPot2 \tPot3 \tPot4 \tPot5 \tPot6 \tPot7 \tPot8 \tPot9 \tPot10 \n");
			printf("Sample \tR00 \tR01 \tR02 \tR03 \tR04 \tR05 \tR06 \tR07 \tR08 \tR09 \tR10 \tL00 \tL01 \tL02 \tL03 \tL04 \tL05 \tL06 \tL07 \tL08 \tL09 \tL10 \n");
	    /* this is for using function */
	    //test_sensor(SampleNum);

	    /* this is for direct reading from main, to test reading at once in main loop */
	    /**/
	    // config_IMU(&device,&mtPort, outputMode, outputSettings);
			laps1 = clock();
			StartTimePoint = std::chrono::system_clock::now();


			//setState(IL_L,0.2);
			//setState(GMAX_L,0.5);

	    for (i=0;i<SampleNum;i++){
	      read_sensor_all(i,SensorData,JointAngle,MusclePressure);
	      //		measure_IMU(&device,&mtPort, outputMode, outputSettings, &quaternion,&euler,&calData,&sample_time);

				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();


				IMUData.calData[i] = calData;
				IMUData.sample_time[i] = sample_time;

				laps2 = clock();
				double elapsed = double(laps2 - laps1) / CLOCKS_PER_SEC;
				laps1 = laps2;

				int delta_t;
				// printing
				printf("\r");
				printf("[%d]\t",i);

				/*
	      for (j = 0; j< NUM_ADC; j++){
					for (k = 0; k< NUM_ADC_PORT; k++){
						// show only used port
						if (j*NUM_ADC_PORT + k >= NUM_OF_SENSOR)
							break;
					  printf("%4lu\t", SensorData[i][j][k]);
					}
	      }
				*/
				/*
				for (j = 0; j<NUM_OF_POT_SENSOR;j++){
					printf("%.1f\t", JointAngle[j]);
				}
				*/
				/**/
				for (j = 0; j<NUM_OF_PRES_SENSOR;j++){
					//printf("%.3f\t", MusclePressure[j]);
					printf("%.3f\t", MusclePressure[muscle_ch[j]]);
				}
				/**/
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
				//printf("\n");
	     */
	    }
			printf ("\n");
	    printf ("-------------------\n");
	    // fulllog("adc_acc_gyr",SensorData,SetPoint_Angle,IMUData.calData);
	    /**/
			std::cout<< "Save data (y/n)? "; std::cin >> in;
			//printf ("Save data (y/n)? ");
	    //scanf ("%c",&in);
			if (in=='y'){
				printf ("filename message: ");
		    scanf ("%s",&msg);
				fulllog(msg,SensorData,SetPoint_Angle,IMUData.calData);
			}
			printf("%c, %s\n",in, msg);
			/**/
	    break;
		}
		case '2':
			test_IMU();
			break;
	  case '3':{
	    printf("Testing Valve\n");
	    test_valve();
			Reset_Valve();
	    break;
		}
	  case '4':{
	    printf("Testing Preset Pressure\n");

			double muscle_val[32]= {0};


			double muscle_zero_deg_value [32] = { 0 };

			/*
			muscle_val[IL_R] = 0.1;
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
			*/

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

				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
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
		case '5':{
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
						printf("\r");
						//printf("\r");
						Input = JointAngle[joint-1];
						printf("%.1f\t%.1f\t", SetPoint_Angle[joint-1], Input);
						printf("Error: %.1f\t", SetPoint_Angle[joint-1] - Input);
						printf("time: %.1f\t", TimeStamp[i]);

						BangBang(SetPoint_Angle[joint-1],Input,&muscle_pair_val[joint-1][0],&muscle_pair_val[joint-1][1]);
						setState(muscle_pair[joint-1][0],muscle_pair_val[joint-1][0]);
						setState(muscle_pair[joint-1][1],muscle_pair_val[joint-1][1]);
						usleep(50000);

						printf("P1: %.2f\t P2: %.2f\t", muscle_pair_val[joint-1][0], muscle_pair_val[joint-1][1]);

						// if logging
						if ((logflag==1)||(logflag==2))
							logflag = entrylog(i,SensorData,SetPoint_Angle,IMUData.calData);
						i++;
					}
					printf("\n");
				}
				else
					break;
			}
			Reset_Valve();
			break;
		}
		case '6':{
			printf ("BangBang Controller for all joints\n");
			printf ("ADD ABD TP FB are set to default\n");
			//SetFrontalPlaneMuscle (PRES_DEF);

			int mode,lastmode;
			int flag[NUM_OF_POT_SENSOR] = {0};


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
						// ====== One by One check. ========
						// ====> stuck one 1 joints if not reached
						/*
						for (j = 0; j<NUM_OF_POT_SENSOR;j++){
							if ((j==0)||(j==1)||(j==4)||(j==5)||(j==6)||(j==7)){
								while(flag[j]<10){
									read_sensor_all(i,SensorData,JointAngle,MusclePressure);

									EndTimePoint = std::chrono::system_clock::now();
									TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();

									// if logging
									//if ((logflag==1)||(logflag==2))
										//logflag = entrylog(i,SensorData,SetPoint_Angle,IMUData.calData);
									i++;

									printf("\rJoint %d. ", j+1);
									printf("Act: %.1f\t SetPoint: %.1f\t ", JointAngle[j], SetPoint_Angle[j]);

									flag[j] +=BangBang(SetPoint_Angle[j],JointAngle[j],&muscle_pair_val[j][0],&muscle_pair_val[j][1]);
									setState(muscle_pair[j][0],muscle_pair_val[j][0]);
									setState(muscle_pair[j][1],muscle_pair_val[j][1]);
									usleep(50000);

									printf("P1: %.2f\t P2: %.2f\t", muscle_pair_val[j][0], muscle_pair_val[j][1]);

									if (flag[j]<0)
										flag[j] = 0;
								}
								//reset flag
								flag[j] = 0;
							}
							else{
								read_sensor_all(i,SensorData,JointAngle,MusclePressure);
								EndTimePoint = std::chrono::system_clock::now();
								TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
								i++;

								printf("\rJoint %d: ", j+1);
								printf("%.1f\t", JointAngle[j]);
							}
							printf("\n");
						}
						printf("\n");
						*/

						// ======= only once corrected ========
						printf("\r");
						for (j = 0; j<NUM_OF_POT_SENSOR;j++){
							read_sensor_all(i,SensorData,JointAngle,MusclePressure);
							EndTimePoint = std::chrono::system_clock::now();
							TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
							i++;

							printf("Joint %2d. ", j+1);
							printf("Act: %5.1f\t SetPoint: %5.1f\t ", JointAngle[j], SetPoint_Angle[j]);

							BangBang(SetPoint_Angle[j],JointAngle[j],&muscle_pair_val[j][0],&muscle_pair_val[j][1]);
							setState(muscle_pair[j][0],muscle_pair_val[j][0]);
							setState(muscle_pair[j][1],muscle_pair_val[j][1]);
							usleep(20000);

							printf("P%2d: %.2f\t P%2d: %.2f\t", j,muscle_pair_val[j][0], j,muscle_pair_val[j][1]);
							printf("P%2d: %.2f\t P%2d: %.2f\t", j,MusclePressure[muscle_pair[j][0]], j,MusclePressure[muscle_pair[j][1]]);
							printf("\n");
						}


						// only for RF
						for (k=0;k<4;k++){
							if(k<2)
								j=k;
							else
								j=k+2;
							read_sensor_all(i,SensorData,JointAngle,MusclePressure);
							EndTimePoint = std::chrono::system_clock::now();
							TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
							i++;

							printf("Joint %2d. ", j+1);
							printf("Act: %5.1f\t SetPoint: %5.1f\t ", JointAngle[j], SetPoint_Angle[j]);

							BangBang(SetPoint_Angle[j],JointAngle[j],&muscle_pair_val[k%2+10][0],&muscle_pair_val[k%2+10][1]);
							setState(muscle_pair[k%2+10][0],muscle_pair_val[k%2+10][0]);
							setState(muscle_pair[k%2+10][1],muscle_pair_val[k%2+10][1]);
							usleep(20000);

							printf("P%2d: %.2f\t P%2d: %.2f\t", k%2+10,muscle_pair_val[k%2+10][0], k%2+10,muscle_pair_val[k%2+10][1]);
							printf("P%2d: %.2f\t P%2d: %.2f\t", k%2+10,MusclePressure[muscle_pair[k%2+10][0]], k%2+10,MusclePressure[muscle_pair[k%2+10][1]]);
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
						fulllog(msg,SensorData,SetPoint_Angle,IMUData.calData);
					}
					else
						std::cout << "Not saving file\n";

					printf("\n");
				}
				else
					break;
			}
			Reset_Valve();
			break;

		}
		case '7':{
			printf("Testing PID\n");
			printf ("ADD ABD are set to default\n");
			setState(ADD_R,PRES_DEF);
			setState(ADD_L,PRES_DEF);

			joint = 1;

			std::cout<< "Used saved parameters? (y/n):"; std::cin >> in;
			if (in=='y')
				loadTunings(&Kp,&Ki,&Kd);
			else{
				std::cout<< "Kp :"; std::cin >> Kp;
				std::cout<< "Ki :"; std::cin >> Ki;
				std::cout<< "Kd :"; std::cin >> Kd;
			}
			myPID.SetTunings(Kp,Ki,Kd);
			std::cout<< "SetPoint : "; std::cin >> SetPoint_Angle[joint-1];
			SetPoint = SetPoint_Angle[joint-1];

			std::cout << Kp << "\t" << Ki << "\t" << Kd << "\t" << SetPoint << "\n";
			//for (i=0;i<SampleNum;i++){


			if (joint>0){
				std::cout<< "Saved File (y/n)? : "; std::cin >> in;
				StartTimePoint = std::chrono::system_clock::now();
			}
			i=0;

			OutputSaturation(&muscle_pair_val[joint-1][0]);
			OutputSaturation(&muscle_pair_val[joint-1][1]);

			while(!_kbhit()){
				read_sensor_all(i,SensorData,JointAngle,MusclePressure);
				EndTimePoint = std::chrono::system_clock::now();
				TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
				i++;

				//printf("\n");
				printf("\r");
				printf("%5.2f\t", JointAngle[0]);

				Input = JointAngle[0];
				myPID.Compute();
				//std::cout << myPID.GetKp() << "\t" << myPID.GetKi() << "\t" << myPID.GetKd() << "\t";

				muscle_pair_val[joint-1][0] += Output;
				muscle_pair_val[joint-1][1] -= Output;

				//muscle_pair_val[joint-1][0] = PRES_DEF + Output;
				//muscle_pair_val[joint-1][1] = PRES_DEF - Output;

				// Output OutputSaturation
				OutputSaturation(&muscle_pair_val[joint-1][0]);
				OutputSaturation(&muscle_pair_val[joint-1][1]);

				setState(muscle_pair[joint-1][0],muscle_pair_val[joint-1][0]);
				setState(muscle_pair[joint-1][1],muscle_pair_val[joint-1][1]);
				usleep(50000);

				printf("Out: %5.2f\t", Output);
				printf("P1: %.2f\t P2: %.2f\t", muscle_pair_val[joint-1][0], muscle_pair_val[joint-1][1]);
			}


			// full log version to minimize time
			if (in=='y'){
				std::cout<< "Message : "; std::cin >> msg;
				fulllog(msg,SensorData,SetPoint_Angle,IMUData.calData);
			}
			else
				std::cout << "Not saving file\n";

			Reset_Valve();
			break;
		}

		case '8':{
			printf("PID Tuning\n");
			printf ("ADD ABD are set to default\n");
			setState(ADD_R,PRES_DEF);
			setState(ADD_L,PRES_DEF);

			while(1){
				std::cout<< "Joint No. : "; std::cin >> joint;
				if (lastjoint!=joint){
					lastjoint = joint;

					if (joint>0){
						std::cout<< "Saved File (y/n)? : "; std::cin >> in;
						StartTimePoint = std::chrono::system_clock::now();
					}
				}
				i=0;


				if (joint>0){
					printf("Setting Joint #%d . Muscle %d\t%d\n", joint, muscle_pair[joint-1][0],muscle_pair[joint-1][1]);
					std::cout<< "SetPoint : "; std::cin >> SetPoint_Angle[joint-1];

					// VER 2
					/*

					tuner.setTargetInputValue(SetPoint_Angle[joint-1]);
					double loopInterval = 10;
	    		tuner.setLoopInterval(loopInterval);
	    		tuner.setOutputRange(-0.8, 0.8);
	    		tuner.setZNMode(PIDAutotuner::znModeBasicPID);
					tuner.startTuningLoop();


					std::chrono::system_clock::time_point tick1, tick2;
					long tickval;

					while (!tuner.isFinished()) {
		        // This loop must run at the same speed as the PID control loop being tuned
		        tick1 = std::chrono::system_clock::now();


						printf("\n");
		        // Get input value here (temperature, encoder position, velocity, etc)
						read_sensor_all(i,SensorData,JointAngle,MusclePressure);
						Input = JointAngle[joint-1];
						printf("%.1f\t",  Input);

		        // Call tunePID() with the input value
		        Output = tuner.tunePID(Input);
						printf("output:%d\t",  tuner.getoutputstate());
						printf("ku:%4.2f\t",  tuner.getku());
						printf("tu:%4.2f\t",  tuner.gettu());
						printf("OutVal: %5.2f\t", Output);

		        // Set the output - tunePid() will return values within the range configured
		        // by setOutputRange(). Don't change the value or the tuning results will be
		        // incorrect.

						muscle_pair_val[joint-1][0] += Output;
						muscle_pair_val[joint-1][1] -= Output;

						// Output Saturation
						OutputSaturation(&muscle_pair_val[joint-1][0]);
						OutputSaturation(&muscle_pair_val[joint-1][1]);

						setState(muscle_pair[joint-1][0],muscle_pair_val[joint-1][0]);
						setState(muscle_pair[joint-1][1],muscle_pair_val[joint-1][1]);
						usleep(50000);
						printf("P1: %.2f\t P2: %.2f\t", muscle_pair_val[joint-1][0], muscle_pair_val[joint-1][1]);

        		// This loop must run at the same speed as the PID control loop being tuned
        		while ( tickval < loopInterval){
							tick2 = std::chrono::system_clock::now();
							tickval = std::chrono::duration_cast<std::chrono::microseconds> (tick2-tick1).count();
							usleep(1);
						}
    			}

					Kp = tuner.getKp();
					Ki = tuner.getKi();
					Kd = tuner.getKd();
					*/

					// VER 1
					/**/

					PIDTune.SetControlType(1); // mode 0 = PI, 1 = PID
					PIDTune.SetOutputStep(0.005);
					PIDTune.SetNoiseBand(0.5);
					PIDTune.SetLookbackSec(5);


					while(!_kbhit()){
						read_sensor_all(i,SensorData,JointAngle,MusclePressure);
						EndTimePoint = std::chrono::system_clock::now();
						TimeStamp[i] =  std::chrono::duration_cast<std::chrono::milliseconds> (EndTimePoint-StartTimePoint).count();
						i++;

						//printf("\r");
						printf("\n");
						Input = JointAngle[joint-1];
						printf("%.1f\t",Input);

						tuneflag= PIDTune.Runtime();
						if (tuneflag!=0){
							printf ("Tuning Done\t");
							break;
						}
						else
							printf ("Tuning\t");


						printf("%d %d\t",  PIDTune.getjustchanged(), PIDTune.getpeakCount());
						//printf("%d %d\t",  PIDTune.getisMax(), PIDTune.getisMin());

						printf("Out: %5.2f\t", Output);
						muscle_pair_val[joint-1][0] += Output;
						muscle_pair_val[joint-1][1] -= Output;

						//muscle_pair_val[joint-1][0] = PRES_DEF + Output;
						//muscle_pair_val[joint-1][1] = PRES_DEF - Output;

						// Output Saturation
						OutputSaturation(&muscle_pair_val[joint-1][0]);
						OutputSaturation(&muscle_pair_val[joint-1][1]);

						setState(muscle_pair[joint-1][0],muscle_pair_val[joint-1][0]);
						setState(muscle_pair[joint-1][1],muscle_pair_val[joint-1][1]);
						usleep(50000);
						printf("P1: %.2f\t P2: %.2f\t", muscle_pair_val[joint-1][0], muscle_pair_val[joint-1][1]);
					}

					Kp = PIDTune.GetKp();
					Ki = PIDTune.GetKi();
					Kd = PIDTune.GetKd();
					/**/

					std::cout << "\n" << Kp << "\t" << Ki << "\t" << Kd << "\n";

					saveTunings(Kp,Ki,Kd);

					// full log version to minimize time
					if (in=='y'){
						std::cout<< "Message : "; std::cin >> msg;
						fulllog(msg,SensorData,SetPoint_Angle,IMUData.calData);
					}
					else
						std::cout << "Not saving file\n";
				}
				else
					break;
			}
			Reset_Valve();
			break;
		}
		case '99':{
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
	  }
	}
	else{
	  printf("Please start program with an argument:\n");
	  printf("1 : Testing Sensor\n");
		printf("2 : Testing IMU Sensor\n");
	  printf("3 : Testing a desired Muscle/Valve with desired pressure\n");
	  printf("4 : Testing all Muscle/Valves with preset pressure\n");
	  printf("5 : Testing Bang-bang 1 joint\n");
	  printf("6 : Bang-bang Controller\n");
	  printf("7 : Testing PID Controller\n");
	  printf("7 : PID Tuning\n");
	  printf("99 : Testing saving loading file\n");
	}


	return 0;
}
