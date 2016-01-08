/*
File: usb_io.h
Date: 11/23/2013
Author: Splicer
Description:
	Ported version of hk_usb_io.py as c file
*/

#ifndef usb_io
#define usb_io

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include "hidapi.h"

#define TRUE 1
#define FALSE 0

//instructions
#define _mod_ver  "0.51x0.01"	// python HKUSBIO module version
#define _mod_date "2/25/2013x11/16/2013/"	// module date
#define u_ad0 0x37	// read ADC value from RA0
#define u_ad1 0x38	// read ADC value from RA1
#define u_i2c_init 0x40	// i2c_init(void)
#define u_i2c_idle 0x41	// i2c_idle(void)
#define u_i2c_strt 0x42	// i2c_start(uchar)
#define u_i2c_stop 0x43	// i2c_stop(void)
#define u_i2c_slak 0x44	// uchar i2c_slave_ack(void)
#define u_i2c_writ 0x45	// void i2c_write(uchar)
#define u_i2c_mack 0x46	// void i2c_master_ack(uchar)
#define u_i2c_read 0x47	// uchar i2c_read(void)
#define u_i2c_dtrd 0x48	// uchar i2c_isdatardy(void)
#define u_spi_init 0x50	// void spi_init(mode, baud, sample)
#define u_spi_tran 0x51	// uchar spi_transfer(regAddr)
#define u_spi_cs 0x52	// void spi_cs(enable|disable)
#define u_rom 0x85	// get PIC rom version
#define u_led 0x80	// toggle LED 
#define u_swc 0x81	// get switch pressed or not
#define u_gpd 0x84	// configure GPIO direction on a pin
#define u_gpi 0x82	// read value on GPIO pin
#define u_gpo 0x83	// write value to GPIO pin
#define u_uss 0x86	// send a string to the UART
#define u_tst 0x87	// test if UART has a char available
#define u_urc 0x88	// read a single char from UART
#define u_usc 0x89	// send a single char to the UART
#define h_getr 0x98	// SFR register to read
#define h_setr 0x99	// SFR register to set
#define h_getb 0x9a	// SFR read register bit
#define h_setb 0x9b	// SFR set register bit

//registers
#define INTCON 0xf2   
#define INTCON2 0xf1  
#define INTCON3 0xf0 
#define SSP1CON3 0xcb 
#define SSP1MSK 0xca  
#define SSP1BUF 0xc9  
#define SSP1ADD 0xc8 
#define SSP1STAT 0xc7
#define SSP1CON1 0xc6 
#define SSP1CON2 0xc5 
#define TMR2 0xbc    
#define T2CON 0xba    
#define PSTR1CON 0xb9 
#define BAUDCON1 0xb8 
#define PWM1CON 0xb7 
#define SPBRGH1 0xb0
#define TRISE 0x96    
#define TRISD 0x95    
#define TRISC 0x94   
#define TRISB 0x93    
#define TRISA 0x92    
#define LATE 0x8d     
#define LATD 0x8c    
#define LATC 0x8b 
#define LATB 0x8a     
#define LATA 0x89     
#define PORTE 0x84   
#define PORTD 0x83    
#define PORTC 0x82   
#define PORTB 0x81    
#define PORTA 0x80   
#define ANSELE 0x5f
#define ANSELD 0x5e   
#define ANSELC 0x5d   
#define ANSELB 0x5c  
#define ANSELA 0x5b   
#define GP_RAM1 0x52  
#define GP_RAM2 0x51  
#define GP_RAM3 0x50 
#define GP_RAM4 0x4f
#define SPBRG1 0xaf   
#define CCPTMRS 0x59  
#define CCP1CON 0xbd 
#define CCPR1H 0xbf   
#define CCPR1L 0xbe   
#define T2CON 0xba    
#define PR2 0xbb 
#define VREFCON1 0x7c
#define VREFCON2 0x7b 
#define VREFCON0 0x7d 
#define ADCON0  0xc2

//bits
#define SSP1STAT_SMP 7 
#define SSP1STAT_CKE 6 
#define SSP1STAT_BF 0 
#define SSP1CON1_WCOL 7 
#define SSP1CON1_SSPOV 6 
#define SSP1CON1_SSPEN 5 
#define SSP1CON1_CKP 4
#define VREFCON1_DACEN 7 
#define VREFCON1_DACLPS 6 
#define VREFCON1_DACOE 5 
#define VREFCON1_DACNSS 0 
#define VREFCON1_DACPSS1 3 
#define VREFCON1_DACPSS0 2

//constants
#define rd4 1		// GPIO pin rd4	def=input
#define rd5 2		// GPIO pin rd5	def=input
#define rd6 3		// GPIO pin rd6	def=output
#define rd7 4		// GPIO pin rd7	def=output
#define pwm_rd5 2
#define pwm_rd6 3
#define pwm_rd7 4
#define pwm_rc1 5
#define dir_output 0	// control GPIO pin direction
#define dir_input 1
#define SPI_LOW_BAUD 0// 750khz
#define SPI_MED_BAUD 1// 3mhz
#define SPI_HI_BAUD  2// 12mhz
#define SPI_SAMP_MID 0// sample input in middle data input time
#define SPI_SAMP_END 1// sample input at end of data input
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3
#define SPI_CS_ENABLE 0
#define SPI_CS_DISABLE 1
#define I2C_DATA_ACK 0	// i2c constants
#define I2C_DATA_NOACK 1
#define I2C_WRITE_CMD 0
#define I2C_READ_CMD 1
#define I2C_START_CMD 0
#define I2C_REP_START_CMD 1
#define I2C_REQ_ACK 0
#define I2C_REQ_NOACK 0

//buffer
#define MAX_STR 65

hid_device* dev_open() {

	hid_device *device;

	device = hid_open(0x04D8, 0x003F, NULL);

	if(device) {
		printf("HID Device opened\n");
		hid_set_nonblocking(device, TRUE);
	} else
		printf("HID Device open failed\n");

	return device;
}

void dev_close(hid_device *device) {
	hid_close(device);
}

void checkmSec(int msec) {
	struct timeval ts, te;
	long tdiff, sec, ms;

	gettimeofday(&ts, NULL);

	while(1) {
		gettimeofday(&te, NULL);

		tdiff = (1000000*(te.tv_sec - ts.tv_sec)) + (te.tv_usec - ts.tv_usec);

		ms = tdiff / 1000;

		if(ms > msec)
			break;
	}
}

//init() see beginning of main() in main.c 

//o_buf : send to PIC
//i_buf : recived from PIC

//other gpio functions over sfr

const char * module_version() {
	static char  a[80];
	strcpy(a, "Module Version: ");
	strcat(a, _mod_ver);
	strcat(a, " Date: ");
	strcat(a, _mod_date);
	return &a[0];
} 

const char * rom_version(hid_device *device) {	// get PIC ROM version
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	// read ROM version
	o_buf[0] = u_rom; //0 oder 1 ?
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	static char a[80];
	strcpy(a, "ROM v");
	strncat(a, &i_buf[1], 1);
	strcat(a,".");
	strncat(a, &i_buf[2], 2); 
	//return &i_buf[0];
	return &a[0];
}

void toggle_led(hid_device *device) {	// toggle LED
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0] = u_led;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

char read_switch(hid_device *device) {	// read switch press
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	char ret;
	o_buf[0] = u_swc;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	if (i_buf[1]==0) {ret=TRUE;}
	else {ret=FALSE;}
	return ret;
}

void gpio_init(hid_device *device,unsigned char pin, unsigned char pdir) {	// set GPIO direction on pin
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_gpd;
	o_buf[1]=pin;
	o_buf[2]=pdir;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

void gpio_out(hid_device *device,char pin) {	// otuput a value on GPIO pin
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_gpo;
	o_buf[1]=pin;
	o_buf[2]=1;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

unsigned char gpio_in(hid_device *device,char pin) {	// read value on GPIO pin
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_gpi;
	o_buf[1]=pin;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}	
	return i_buf[1];	
}

unsigned int adc_ra0(hid_device *device) { // do ADC conversion on RA0
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	unsigned int a;
	o_buf[0]=u_ad0;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}	
	a =i_buf[2] << 8;
	a|=i_buf[1];	
	return a;
}

unsigned int adc_ra1(hid_device *device) {	// do ADC conversion on RA1
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	unsigned int a;
	o_buf[0]=u_ad1;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}	
	a =i_buf[2] << 8;
	a|=i_buf[1];	
	return a;
}

unsigned char ser_test(hid_device *device) {	// check if a char available on serial port
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_tst;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];
}

void ser_putc(hid_device *device,char schar) {	// send a char to the serial port
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_usc;
	o_buf[1]=schar;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}	
}

void ser_puts(hid_device *device, const char * strval) {	// send a string to the serial port
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_uss;
	strcat(o_buf, strval);
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}	
}

unsigned char ser_getc(hid_device *device) { // get a single char from the serial port
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_urc;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];
}

unsigned char sfr_get_reg(hid_device *device, unsigned char reg) {	// get a SFR register
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=h_getr;
	o_buf[10]=reg;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];
}

unsigned char sfr_set_reg(hid_device *device, unsigned char reg, unsigned char rval) {	// set a SFR register
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=h_setr;
	o_buf[10]=reg;		// register to select
	o_buf[11]=rval;	// value to set
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];
}

unsigned char sfr_get_regbit(hid_device *device, unsigned char reg, unsigned char bval) {	// get a SFR register bit
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=h_getb;
	o_buf[10]=reg;		// register to select
	o_buf[11]=bval;	// bit value to get
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];
}

unsigned char sfr_set_regbit(hid_device *device, unsigned char reg, unsigned char rbit, unsigned char bval) {	// set a SFR register bit
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=h_setb;
	o_buf[10]=reg;		// register to select
	o_buf[11]=rbit;	// bit to set
	o_buf[12]=bval;	// bit value to set
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];
}

void i2c_init(hid_device *device) {	// init i2c
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_i2c_init;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

void i2c_idle(hid_device *device) {	// i2c idle
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_i2c_idle;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

void i2c_start(hid_device *device, char cval) {	// i2c start	
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_i2c_strt;
	o_buf[1]=cval;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

void i2c_stop(hid_device *device) {	// i2c stop
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_i2c_stop;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

unsigned char i2c_slave_ack(hid_device *device) {	// i2c slave ack
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_i2c_slak;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];	// 1=no ack, 0=ack
}

void i2c_write(hid_device *device, char cval) {	// i2c write
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_i2c_writ;
	o_buf[1]=cval;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

void i2c_master_ack(hid_device *device, char cval) {	// 1=nack, 0=ack
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_i2c_mack;
	o_buf[1]=cval;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

unsigned char i2c_read(hid_device *device) {	// i2c read
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_i2c_read;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];	//i2c_read char

}

unsigned char i2c_isdatardy(hid_device *device) {	// check if i2c char avail
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];	
	o_buf[0]=u_i2c_dtrd;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1]; //i2c_read char
}

void spi_init(hid_device *device, unsigned char mode, unsigned char baud, unsigned char sample) {	// SPI init
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_spi_init;
	o_buf[1]=mode;
	o_buf[2]=baud;
	o_buf[3]=sample;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

unsigned char spi_transfer(hid_device *device, unsigned char value) {	// SPI transfer 
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_spi_tran;
	o_buf[1]=value;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
	checkmSec(50);
	if (hid_read(device, i_buf, sizeof(i_buf)) == -1) {dev_close(device);}
	return i_buf[1];
}

void spi_cs(hid_device *device, unsigned char select) {	// enable or disable SPI CS
	unsigned char o_buf[MAX_STR];
	unsigned char i_buf[MAX_STR];
	o_buf[0]=u_spi_cs;
	o_buf[1]=select;
	if (hid_write(device, o_buf, sizeof(o_buf)) == -1) {dev_close(device);}
}

#endif
