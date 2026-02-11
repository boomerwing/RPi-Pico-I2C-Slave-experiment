/**
 * RP2040 FreeRTOS Template
 *   
 */

#include <stdio.h>
#include "pico/stdlib.h" 
#include "hardware/i2c.h"
#include "pcf8575i2c.h"

/**
 * @brief pcf8575 init
 */
void pcf8575_init(){
    i2c_init(i2c1, 100 * 1000);  // 100000 b/s
    gpio_set_function(I2C1_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_GPIO);  // pin 4
    gpio_pull_up(I2C1_SCL_GPIO);  // pin 5
}
     
/**
 * @brief Set (High) new port Byte value modified by OR mask
 */
uint8_t setBit_High(uint8_t outbuffer, uint8_t bitnum){
    const uint8_t high_mask[8]={0B00000001,0B00000010,0B00000100,0B00001000,0B00010000,0B00100000,0B01000000,0B10000000};
    return (outbuffer |= high_mask[bitnum]);  // OR mask with data to set portnum bit HIGH
}
    
/**
 * @brief Clear (Low) new port Byte value modified by AND mask
 *
 */
uint8_t setBit_Low(uint8_t outbuffer, uint8_t bitnum){
    const uint8_t low_mask[8]={0B11111110,0B11111101,0B11111011,0B11110111,0B11101111,0B11011111,0B10111111,0B01111111};
    return (outbuffer &= low_mask[bitnum]);  // AND mask with data to set portnum bit LOW

}
/**
 * @brief Isolate bit value by OR mask to read one port value
 * 
 * Read PCF8576 with:
 *      i2c_read_blocking(i2c0, I2C_ADDR, pcfbuffer, 2, false);
 *      sw_state = (pcfbuffer[0]);  // if all eight switches read on P0 to P7
 *  Then apply Mask to isolate individual bits.
 */
uint8_t readBit(uint8_t inbuffer, uint8_t bitnum){
    uint8_t value;
    const uint8_t isolate_mask[8]={0B00000001,0B00000010,0B00000100,0B00001000,0B00010000,0B00100000,0B01000000,0B10000000};
    value = (inbuffer &= isolate_mask[bitnum]); // AND mask to isolate portnum bit
    if(!value)  {value = 0;}
    else {value = 1;}
    return value; // 1 or 0 output
}
/**
 * @brief Isolate bit value by OR mask to read one port value
 * 
 * Read PCF8576 with:
 *      i2c_read_blocking(i2c0, I2C_ADDR, pcfbuffer, 2, false);
 *      sw_state = (pcfbuffer[0]);  // if all eight switches read
 *  Then apply Mask to isolate individual bits.
 * 
 */
uint8_t readBits(uint8_t inbuffer, uint8_t mask){
    uint8_t value;
    value = (inbuffer &= mask); // AND mask to isolate portnum bits
    return value; // 
}

    
