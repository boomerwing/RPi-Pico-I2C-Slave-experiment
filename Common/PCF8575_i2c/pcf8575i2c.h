/**
 * RP2040 FreeRTOS seven_seg.h
 *
 * @copyright 2022, Calvin McCarthy
 * @version   1.0.0
 * @license   MIT
 *
 */
#ifndef _PCF8575_I2C_H_
#define _PCF8575_I2C_H_
#endif      // _PCF8575_I2C_H_

#define GPIO4 4
#define GPIO5 5
#define I2C0_ADDR 0x16
#define I2C1_ADDR 0x20
#define I2C0_SDA_GPIO 4  // pin 6 Pico  SDA
#define I2C0_SCL_GPIO 5  // pin 7 Pico  SCL
#define I2C1_SDA_GPIO 2  // pin 4 Pico  SDA
#define I2C1_SCL_GPIO 3  // pin 5 Pico  SCL

#define P00 0
#define P01 1
#define P02 2
#define P03 3
#define P04 4
#define P05 5
#define P06 6
#define P07 7
#define PSW1  P00 
#define PSW2  P01
#define PSW3  P02
#define PSW4  P03


/**
 * PROTOTYPES
 */
void pcf8575_init();
uint8_t setBit_High(uint8_t,uint8_t);
uint8_t setBit_Low(uint8_t,uint8_t);
uint8_t readBit(uint8_t,uint8_t);
uint8_t readBits(uint8_t,uint8_t);

#ifdef __cplusplus
}           // extern "C"
#endif      // _PCF8575_I2C_H_
