/**
 * @file Sensor.cpp
 *
 */

#include <sensor.h>

/**** analog sensors ****/
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
