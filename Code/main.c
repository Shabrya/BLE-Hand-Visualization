/*
 * Final.c
 *
 * Created: 5/9/2015 7:12:58 PM
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <util/twi.h>
#include "i2c.h"
#include <math.h>
#include <string.h>


#define MPU60501  0xD0     // (0x68 << 1) I2C slave address

unsigned char ret;            // return value

char outs[50];

//********************************//



void uart_init (void)
{
	//asynchronous uart, transmit 8-bit data
	UCSR0C = ((1<<UCSZ01)|(1<<UCSZ00));
	//9600 Baud Rate
	UBRR0L = 0x33;
	UCSR0B = (1<<TXEN0);	//enable transmitter
}
void uart_tx_string (char *data)
{
	while((*data!='\0')){
		while(!(UCSR0A&(1<<UDRE0)));			//wait until transmit register is empty
		UDR0 = *data;
		data++;
	}
}


void MPU6050_writereg(uint8_t accel, uint8_t reg, uint8_t val)
{
	i2c_start(accel+I2C_WRITE);
	i2c_write(reg);  // go to register e.g. 106 user control
	i2c_write(val);  // set value e.g. to 0100 0000 FIFO enable
	i2c_stop();        // set stop condition = release bus
}


uint16_t MPU6050_readreg(uint8_t accel, uint8_t reg)//read unsigned 16 bits
{
	i2c_start_wait(accel+I2C_WRITE); // set device address and write mode
	i2c_write(reg);                                  // ACCEL_OUT
	i2c_rep_start(accel+I2C_READ);    // set device address and read mode
	int raw = i2c_readAck();                    // read one intermediate byte
	raw = (raw<<8) | i2c_readNak();        // read last byte
	i2c_stop();
	return raw;
}

int16_t MPU6050_signed_readreg(uint8_t accel, uint8_t reg)//read signed 16 bits
{
	i2c_start_wait(accel+I2C_WRITE); // set device address and write mode
	i2c_write(reg);                                  // ACCEL_OUT
	i2c_rep_start(accel+I2C_READ);    // set device address and read mode
	char raw1 = i2c_readAck();                    // read one intermediate byte
	int16_t raw2 = (raw1<<8) | i2c_readNak();        // read last byte
	i2c_stop();
	return raw2;
}



void Init_MPU6050(uint8_t accel)
{

	ret = i2c_start(accel+I2C_WRITE);       // set device address and write mode
	if ( ret )
	{

		snprintf(outs,sizeof(outs),"failed to issue start condition \n\r");
		uart_tx_string(outs);
		i2c_stop();
	}
	else
	{
		/* issuing start condition ok, device accessible */
		MPU6050_writereg(accel, 0x6B, 0x00); // reg 107 set value to 0000 0000 and wake up sensor
		MPU6050_writereg(accel, 0x19, 0x07); // reg 25 sample rate divider set value to 0000 1000 for 1000 Hz
		MPU6050_writereg(accel, 0x1C, 0x18); // reg 28 acceleration configuration set value to 0001 1000 for 16g
		MPU6050_writereg(accel, 0x23, 0xF8); // reg 35 FIFO enable set value to 1111 1000 for all sensors not slave
		MPU6050_writereg(accel, 0x37, 0x10); // reg 55 interrupt configuration set value to 0001 0000 for logic level high and read clear
		MPU6050_writereg(accel, 0x38, 0x01); // reg 56 interrupt enable set value to 0000 0001 data ready creates interrupt
		MPU6050_writereg(accel, 0x6A, 0x40); // reg 106 user control set value to 0100 0000 FIFO enable
		snprintf(outs,sizeof(outs),"done start \n\r");
		uart_tx_string(outs);
	}
	i2c_stop();
}



int main(){
	int16_t xi1 = 0;
	int16_t yi1 = 0;
	int16_t zi1 = 0;
	int xa1, ya1, za1;
	DDRD = 0xF0;
	DDRC = 0x00;
	//declare average calibrated accelerometer values
	//initialize calibarition values
	//declare accelerometer value strings

	uart_init();//initialize uart
	i2c_init();     // init I2C interface
	_delay_ms(200);  // Wait for 200 ms.
	Init_MPU6050(MPU60501);    // sensor init
	_delay_ms(200);     // Wait for 200 ms.
	snprintf(outs,sizeof(outs),"6050 initialized \n\r");
	uart_tx_string(outs);
	for(int i = 0; i<10; i++)//get values for initial calibration
	{
			// read raw X acceleration from fifo
			xi1 += MPU6050_signed_readreg(MPU60501,0x3B);   
			// read raw Y acceleration from fifo
			yi1 += MPU6050_signed_readreg(MPU60501,0x3D);   
			// read raw Z acceleration from fifo
			zi1 += MPU6050_signed_readreg(MPU60501,0x3F);   			
	}
	//average values for calibration
	xi1 = xi1/10;
	yi1 = yi1/10;
	zi1 = zi1/10;
	
	//Start infinite loop
	while(1){

		//grab 3 values, average, subtract calibration value, and divide by MSB
		// read raw X acceleration from fifo
		xa1 = MPU6050_signed_readreg(MPU60501,0x3B)+MPU6050_signed_readreg(MPU60501,0x3B)+MPU6050_signed_readreg(MPU60501,0x3B);   		
		xa1 = ((xa1/3)-xi1)/2048.00;
		// read raw Y acceleration from fifo
		ya1 = MPU6050_signed_readreg(MPU60501,0x3D)+MPU6050_signed_readreg(MPU60501,0x3D)+MPU6050_signed_readreg(MPU60501,0x3D);   
		ya1 = ((ya1/3)-yi1)/2048.00;
		// read raw Z acceleration from fifo
		za1 = MPU6050_signed_readreg(MPU60501,0x3F)+MPU6050_signed_readreg(MPU60501,0x3F)+MPU6050_signed_readreg(MPU60501,0x3F);   
		za1 = ((za1/3)-zi1)/2048.00;

		//convert doubles to printable strings
		//print out the values in a special format required
		//by the Android App. Format:"EValue1,Value2,Value3.....\n"
		
		snprintf(outs,sizeof(outs),"E%i, ", xa1); //send x-value
		uart_tx_string(outs);
		snprintf(outs,sizeof(outs),"%i, ", ya1);  //send y-value
		uart_tx_string(outs);
		snprintf(outs,sizeof(outs),"%i\n", za1);  //send z-value	
		uart_tx_string(outs);

	} 
	
	return 0;
} 