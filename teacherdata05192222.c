#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "gpio.h"
#include <stdbool.h>
#include <time.h>
#include <math.h>

/*The two muscles set to 0.3 in turn,periodic movement*/

// for valves
//#define pin_spi_cs1  P9_28 // 3_17=113
//#define pin_spi_other P9_29 // 3_16=111
//#define pin_spi_mosi P9_30 // 3_15=112
//#define pin_spi_sclk P9_31 // 3_14=110
//#define pin_spi_cs2  P9_42 // 0_7 =7
//#define NUM_OF_CHANNELS 16

// for valves
#define pin_spi_cs1  P9_16 // 1_19=51
#define pin_spi_other P9_22 // 0_2=2
#define pin_spi_mosi P9_30 // 3_15=112
#define pin_spi_sclk P9_21 // 0_3=3
#define pin_spi_cs2  P9_42 // 0_7 =7
#define NUM_OF_CHANNELS 16

// for sensors
#define pin_din_sensor  P9_11 // 0_30=30
#define pin_clk_sensor P9_12 // 1_28=60
#define pin_cs_sensor P9_13 // 0_31=31
#define pin_dout1_sensor P9_14 // 1_18=50
#define pin_dout2_sensor  P9_15 // 1_19=51
#define NUM_ADC_PORT 8
#define NUM_ADC 2

// for analog input
#define NUM_OF_AINS 7
#define AIO_NUM 7

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

/**** SPI for valves ****/
bool clock_edge = false;
unsigned short resolution = 0x0FFF;
void set_SCLK(bool value) { digitalWrite(pin_spi_sclk, value); }
void set_OTHER(bool value) { digitalWrite(pin_spi_other, value); }
void set_MOSI(bool value) { digitalWrite(pin_spi_mosi, value); }
void setCS1(bool value){ digitalWrite(pin_spi_cs1, value); }
void setCS2(bool value){ digitalWrite(pin_spi_cs2, value); }
void set_clock_edge(bool value){ clock_edge = value; }
bool get_MISO(void) { return false; } // dummy
void wait_SPI(void){}

// value 1: Enable chipx
void chipSelect1(bool value){ setCS1(!value); wait_SPI(); wait_SPI(); }
void chipSelect2(bool value){ setCS2(!value); wait_SPI(); wait_SPI(); }

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
	else if (ch >= 8) {
		register_data = (((unsigned short)(ch & 0x0007) << 12) & 0x7000) | (dac_data & 0x0fff);
		chipSelect2(true);
		transmit16bit(register_data);
		chipSelect2(false);
	}
}

// pressure coeff: [0.0, 1.0]
void setState(unsigned int ch, double pressure_coeff)
{
	setDARegister(ch, (unsigned short)(pressure_coeff * resolution));
}

/**** SPI for sensors ****/
void set_DIN_SENSOR(bool value) { digitalWrite(pin_din_sensor, value); }
void set_CLK_SENSOR(bool value) { digitalWrite(pin_clk_sensor, value); }
void set_CS_SENSOR(bool value) { digitalWrite(pin_cs_sensor, value); }
int get_DOUT_SENSOR(int adc_num) { 
	if(adc_num==0){
		digitalRead(pin_dout1_sensor); 
	}
	else{
		digitalRead(pin_dout2_sensor); 
	}
}

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

/*******************************************/
/*              Init Functions              /
/*******************************************/
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

/*******************************************/

int main() {

	init();
	init_pins(); // ALL 5 pins are HIGH except for GND
	init_DAConvAD5328();
	init_sensor();

	int i,j,k,count;
	float state0[20000],state1[20000];
	double sankaku;
	unsigned int ch_num;
	for (i = 0; i < NUM_OF_CHANNELS; i++) 
		setState(i, 0.0); 

	int SampleNum = 2000;
	int Sample = 20000;
	unsigned long SensorData[Sample][NUM_ADC_PORT];
	clock_t TimeData[SampleNum];
	//int ValveNum = 1;
	


	/*	//========File Reading=========//
	FILE *fo;
	//	flot statechange0[10000],statechange1[100000];
	int n1;
	char s[100];
	int ret;
	float f0[20000],f1[20000], f2[20000],f3[20000],f4[20000],f5[20000],f6[20000],f7[20000],f8[20000],f9[20000],f10[20000],f11[20000],f12[20000],f13[20000],f14[20000],f15[20000] ;
	int line_num=0;

	fo = fopen("data/periodic160416_1.txt","r");
	if (fo == NULL){
	  printf("File open error\n");
	  return;
	}

	//printf( "¥n-- fgets() --¥n" );
	//while( fgets( s, 100, fo ) != NULL ){
	//printf( "%s", s );
	//}

		printf("\n-- fgets() --\n");
		for (n1=0;n1<20000;n1++){
		  fscanf(fo,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f",
			 &f0[n1],&f1[n1],&f2[n1],&f3[n1],&f4[n1],
			 &f5[n1],&f6[n1],&f7[n1],&f8[n1]);
		   printf("%d %f %f\n",line_num, f0[n1],f1[n1]);
		  //printf("loop\n");
		  line_num++;
	  
		}
	fclose(fo);
	//=========filter===============//
	for(n1 = 0;n1<20000;n1++){
	  if(n1 <100)
 	    f2[n1]= f1[n1];
	  else{
	    f2[n1]=0;
	    for(i=0;i<100;i++){
	      f2[n1]+=((f1[n1-i])/100.0);
	    }
	  }
	  }*/

	//=============================//

	unsigned long *tmp_val0;
	unsigned long tmp_val[NUM_ADC_PORT];
	float f1[20000], f2[20000],f3[20000],f4[20000],f5[20000],fd[20000],theta0[20000],theta1[20000],k1=1/100000.0,kd=1/10000000.0;
	int n1=0;
	for(n1=0;n1<20000;n1++){
	 
	  tmp_val0=read_sensor(0,tmp_val);
	  for (k = 0; k< 8; k++){
	    SensorData[n1][k]=tmp_val0[k];
	    //printf("%d\n", tmp_val0[k]);
	  }
	    f1[n1]= SensorData[n1][0]-31.5;
	    f3[n1]= SensorData[n1][3]-73.0;
	    theta0[n1]=(3220.0-SensorData[n1][5])*333.3/4095.0;
	    theta1[n1]=(1540.0-SensorData[n1][4])*333.3/4095.0;
	   
	    // printf("%f\t%f\n",theta1[n1],SensorData[n1][5]);
	    //	    printf("%f,%f\n",f1[n1],f3[n1]);//force sensor input check
	//=====online filter===========//
       
	    if(n1 <100){
 	    f2[n1]= f1[n1];
	    f4[n1]= f3[n1];
	    }
	  else{
	    f2[n1]=0;
	    f4[n1]=0;
	    for(i=0;i<100;i++){
	      f2[n1]+=((f1[n1-i])/100.0);
	      f4[n1]+=((f3[n1-i])/100.0);
	    }
	  }
	//=============================//
	    state0[0]=0.1;
	    state1[0]=0.1;
	    fd[0]=0;
	   
	    // for (i = 0; i < SampleNum; i++){ 
	    // if (i > 1000){
	      
	    // fd[n1] =(f2[n1]-f4[n1])-(f2[n1-1]-f4[n1-1]); 
	      // state0[n1+1] = fd[n1]*kd+(f2[n1]-f4[n1])*k1+state0[n1];
	      // f5[n1+1]=state0[n1+1];
	      // if(state0[n1+1] < 0.1)
	      // state0[n1+1] =0.1;
	      // else if(state0[n1+1] > 0.4)
	      // state0[n1+1] = 0.4;
	       // printf("%f\n",state0[n1]);
	      // state1[n1+1] = state1[n1]-(f2[n1]-f4[n1])*k1-fd[n1]*kd;
	    sankaku= (3.14/1000.0)*n1;
	    state0[n1]= 0.3*sin(sankaku);
	    state1[n1]= 0.3-state0[n1];
	   
	      
	       if(state1[n1+1]< 0.1)
		  state1[n1+1] =0.1;
	       else if(state1[n1+1] > 0.3)
		 state1[n1+1] = 0.3;
	
	      setState(0,state0[n1]); 
	      setState(1,state1[n1]);
	      setState(2,state0[n1]);
	      setState(3,state1[n1]);
	      // printf("%d,%f,%f\n",n1,state0[n1],state1[n1]);
	       //  }
	    
	    
	        
	     /*  else{
	        state0 = -f2[(count)*2000+i];
	      if(state0 < 0)
		state0 =0;
	      else if(state0 > 0.5)
		state0 = 0.5;
	    
	      state1 = 0.2+f2[(count)*2000+i];
           
	      if(state1 < 0)
		state1 =0;
	      else if(state1 > 0.5)
		state1 = 0.5;
	      printf("%d,%d,%f,%f\n",count,i,state0,state1);
	      setState(0, 0.2); 
	      setState(1,0);
	      setState(2,state0);
	      setState(3,state1);
		  
	      }*/
	    
		  // printf("%10lu\n", SensorData[count][i][0]);
	  TimeData[i] = clock();
	  usleep(100);	   	  
   	}
	setState(0,0);
	setState(1,0);
	setState(2,0);
	setState(3,0);
	
	// ============= File Writing =========
	FILE *fp;
	char str[100];

	fp = fopen("data/full160420_1.txt","w");
	if (fp == NULL){
	  printf("File open error\n");
	  return;
	}
	for(n1=0;n1<20000;n1++){
	  // sprintf(str, "%d\t", count);
	  // fputs(str,fp);
	 
	     sprintf(str, "%d\t", n1); ///TimeData[i]);
	    fputs(str, fp);
	    
		for (k = 0; k< NUM_ADC_PORT; k++){
		  sprintf(str, "%10lu\t", SensorData[n1][k]);
		  fputs(str, fp);
		}
	    
	    sprintf(str, "\n");
	    fputs(str, fp);
	 
	  // sprintf(str,"\n");
	  // fputs(str,fp);
	}

	fclose(fp);
	

	// ============= File Writing filter =========
	FILE *fp2;
	char str1[100],str2[100],str3[100],str4[100],str5[100];
	fp2 = fopen("data/teachdata.txt","w");
	if (fp2 == NULL){
	  printf("File open error\n");
	  return;
	}
	for(n1 = 0;n1 < 20000;n1++){
	  // sprintf(str, "%d\t", count);
	  // fputs(str,fp
	  // sprintf(str1,str2,str3,str4 ,"%f\t%f\t%f\t%f\t%f\t", f2[n1],f4[n1],(f2[n1]-f4[n1]),theta0[n1],theta1[n1]);
	  // printf("%f\t%f\n",f2[n1],f4[n1]);
	   fprintf(fp2,"%f\t%f\t%f\t%f\t%f\t%f\t%f\n", f2[n1],f4[n1],(f2[n1]-f4[n1]),f5[n1],state0[n1],theta0[n1],theta1[n1]);	
	    
	 
	  }


	fclose(fp2);

	return 0;
}


