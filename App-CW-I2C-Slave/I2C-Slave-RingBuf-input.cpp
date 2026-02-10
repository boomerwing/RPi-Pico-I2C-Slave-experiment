/**
extern int wiringPiI2CRead           (int fd) ;
extern int wiringPiI2CReadReg8       (int fd, int reg) ;
extern int wiringPiI2CReadReg16      (int fd, int reg) ;
extern int wiringPiI2CReadBlockData  (int fd, int reg, uint8_t *values, uint8_t size);  //Interface 3.3
extern int wiringPiI2CRawRead        (int fd, uint8_t *values, uint8_t size);           //Interface 3.3

extern int wiringPiI2CWrite          (int fd, int data) ;
extern int wiringPiI2CWriteReg8      (int fd, int reg, int data) ;
extern int wiringPiI2CWriteReg16     (int fd, int reg, int data) ;
extern int wiringPiI2CWriteBlockData (int fd, int reg, const uint8_t *values, uint8_t size);  //Interface 3.3
extern int wiringPiI2CRawWrite       (int fd, const uint8_t *values, uint8_t size);           //Interface 3.3

extern int wiringPiI2CSetupInterface (const char *device, int devId) ;
extern int wiringPiI2CSetup          (const int devId) ;

    cd ~/a_test
    
    cd ~/a_test
    g++ -o pico_ring_test I2C-Slave-RingBuf-input.cpp I2CDevice.cpp -lwiringPi
    
    sudo scp pico_ring_test pi2@10.0.0.197:./Play
    
    sudo scp Nutcracker.txt pi2@10.0.0.197:./Play
    
    sudo scp Gardeners_Grandmother.txt pi2@10.0.0.197:./Play
    
    sudo scp test_Grand.txt pi2@10.0.0.197:./Play
    
    ./pico_test MaxBrand.txt
    ./pico_ring_test Nutcracker.txt
    ./pico_ring_test Gardeners_Grandmother.txt

   
    i2cdetect -y -r 1
     i2cdump -y 1 0x16 b
     
     This Raspberry Pi code reads a TXT file and transmits the characters one by one to a
     Raspberry Pi Pico Slave Mode Morse Code Practice sender.
*/

#include <stdio.h>
#include <iostream>
#include <thread>
#include <wiringPiI2C.h>
#include "I2CDevice.h"


#define DEVICE_ID  0x16
#define FLAG_REG   0xF0
#define INDEX_IN   0xF2
#define INDEX_OUT  0xF4
#define START      0x00
#define END        0x2F

int main (int argc, char **argv)
{
    uint8_t flag;
    uint8_t old_flag;
    uint8_t count;
    uint8_t full;
    uint8_t next = START;
    char inchar;
    
    // Setup I2C communication
    int fd = wiringPiI2CSetup(DEVICE_ID);
    if (fd == -1) {
        std::cout << "Failed to init I2C communication.\n";
        return 0;
    }
    std::cout << "I2C communication successfully setup.\n";


    FILE *fp;
    fp =  fopen(argv[1], "r");  //  ./pico_test MaxBrand.txt
     if(!fp){
         printf("Failure to Open file\n");
         return 0;
        }; 

    next = START;
    
            flag = 0x30;    // flag
            while(flag == 0x30){
                flag = wiringPiI2CReadReg8(fd, 0xF0);    // flag
            }
            old_flag = flag;

    while (1) {
                
                flag = wiringPiI2CReadReg8(fd, 0xF0);    // flag
                if(flag == 0x31)
                    printf("\n flag: %3x ", flag);
            
                if(flag == 0x31){
                    inchar = fgetc(fp);
                    if(feof(fp)) break;
                     
                    if(inchar > 0x7F)  inchar = 0x20;
                    if(inchar == 0x0A) inchar = 0x20;
                    
                    wiringPiI2CWriteReg8(fd,next,inchar);
                     next++;
                    if(next > END) next = START;  // Ring Buffer Boundary
                    
                    
                  printf("  count: %3x  char: %c", next, inchar);
                   
                }
            
            for(int i=0;i<20000000;i++);
    }
    fclose(fp);
    return 0;
}
