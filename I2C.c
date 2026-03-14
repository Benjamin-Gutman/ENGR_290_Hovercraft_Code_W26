#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

#define MPU6050_ADDR 0x68

void I2C_init(void){ // This function initializes the status and bit rate register to set I2C at about 100 kHz if CPU at 16MHz
	TWSR = 0;
	TWBR = 72;
}

void I2C_start(void){//Sends the start signal to the slave (IMU)
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR&(1<<TWINT)));
}

void I2C_stop(void){ //Sends the stop signal to the slave (IMU)
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void I2C_write(uint8_t Data){ // Sends the data byte to the slave through the data register
	TWDR = Data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR&(1<<TWINT)));
}

void I2C_write_reg(uint8_t Data, uint8_t Register){ //Combine all the prior functions to write to a register
	I2C_start();
	//I2C slave address is 8 bits, bit 0 is the read/write flag and 1-7 are the address
    I2C_write(MPU_ADDR << 1);
    //State the register value
	I2C_write(Register);
	//Send the data
	I2C_write(Data);
	I2C_stop();
}

uint8_t I2C_read_ack(void){//Read the data and tell the IMU you want more data
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}

uint8_t I2C_read_nack(void){// Read the data and tell the IMU you're done
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}

uint8_t I2C_read_reg(uint8_t Register){//read 1 byte off a register
	I2C_start();
	I2C_write(MPU_ADDR << 1);
	I2C_write(Register);

	I2C_start();
	I2C_write(MPU_ADDR << 1|1);
	uint8_t Data = I2C_read_nack();

	I2C_stop();

	return Data;
}


void I2C_read_multiple_reg(uint8_t Base_reg, uint8_t* values, int range){//Read multiple bytes in a register
	I2C_start();
	I2C_write(MPU_ADDR << 1);
	I2C_write(Base_reg);

	I2C_start();
	I2C_write(MPU_ADDR << 1|1);
	//This assumes the register addresses are next to each other. This is good for something like IMU acceleration values which
	//are all within 6 registers of each other
	for (int i = 0; i < range; i++){
		if (i < range-1){
			values[i] = I2C_read_ack();
		}
		else{
			values[i] = I2C_read_nack();
		}
	}

	I2C_stop();
}


