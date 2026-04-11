#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU_ADDR 0x68

	void I2C_init(void);
	void I2C_start(void);
	void I2C_stop(void);
	void I2C_write(uint8_t Data);
	void I2C_write_reg(uint8_t Data, uint8_t Register);
	uint8_t I2C_read_ack(void);
	uint8_t I2C_read_nack(void);
	uint8_t I2C_read_reg(uint8_t Register);
	void I2C_read_multiple_reg(uint8_t Base_reg, uint8_t* values, int range);

#ifdef __cplusplus
}
#endif

#endif