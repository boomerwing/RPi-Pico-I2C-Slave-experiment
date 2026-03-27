/**

////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    
    g++ -o RPi-Text-Reader-Sp RPi-Text-Reader-Sp.cpp I2CDevice.cpp -lwiringPi
    
    sudo scp RPi-Text-Reader-Sp pi2@10.0.0.197:./Play
    
    sudo scp Gardeners_Grandmother.txt pi2@10.0.0.197:./Play
    sudo scp Nutcracker.txt pi2@10.0.0.197:./Play
    sudo scp Code-Groups.txt pi2@10.0.0.197:./Play
    sudo scp Words-Shuffle.txt pi2@10.0.0.197:./Play
    sudo scp Short-Words-Shuffle.txt pi2@10.0.0.197:./Play
    sudo scp Numbers-Punct.txt pi2@10.0.0.197:./Play
    
    ./RPi-Text-Reader-Sp Nutcracker.txt
    ./RPi-Text-Reader-Sp Gardeners_Grandmother.txt
    ./RPi-Text-Reader-Sp Code-Groups.txt
    ./RPi-Text-Reader-Sp Words-Shuffle.txt
    ./RPi-Text-Reader-Sp tiny-Code-Groups.txt
   
    i2cdetect -y -r 1
     i2cdump -y 1 0x16 b
   
//////////////////////////////////////////////////////////////////////////////////////
     
     This Raspberry Pi code reads a TXT file and transmits the characters one by one to a
     Raspberry Pi Pico I2C Slave Mode Morse Code Practice sender.
*/

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "I2CDevice.h"


#define DEVICE_ID  0x16
#define FLAG_REG   0xF0
#define INDEX_IN   0xF2
#define INDEX_OUT  0xF4
#define SPD_INDEX  0xF6 
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
    char titleString[10];
    const uint8_t speed[] =     {120,109,100,92,86,80,75,70,66,63,60,57,54,52,50,48};
    const uint8_t showspeed[] = { 10, 11, 12,13,14,15,16,17,18,19,20,21,22,23,24,25};
    uint8_t wpm;
    uint8_t speed_out;
    uint8_t speed_out2;
    
    // Setup I2C communication
    int fd = wiringPiI2CSetup(DEVICE_ID);
    if (fd == -1) {
        std::cout << "Failed to init I2C communication.\n";
        return 0;
    }
    std::cout << "I2C communication successfully setup.\n";


    FILE *fp;
    fp =  fopen(argv[1], "r");  // Open with the name of the Text File to Read
     if(!fp){
        std::cout << "Failure to Open file\n";
         return 0;
        };
    //   Read first line of the Text file to pick up Code WPM     
     while(fgetc(fp) != ' ');    // look for Space char
     fgets(titleString, 4, fp);  //  read number as ASCII characters
     wpm = atoi(titleString);    // Convert ASCII number to int
     speed_out = speed[wpm-10];  // Pick Timer delay value out of speed[] array
     wiringPiI2CWriteReg8(fd,SPD_INDEX,speed_out);  // Write into I2C Slave Ring Buffer 
        delay(25); // 25 ms WiringPi Delay
    
    flag = 0x30;    // flag
    while(flag == 0x30){
        flag = wiringPiI2CReadReg8(fd, 0xF0);    // read flag from I2C Slave
        delay(25); // 25 ms WiringPi Delay
        }
            
    next = START;
          
    while (1) {  // Only send characters to Pico I2C Slave if flag is 0x31
                
                if(flag == 0x31){
            
                    inchar = fgetc(fp);
                    if(feof(fp)) break;  // Close App when EOF found
                     
                    if(inchar > 0x7F)  inchar = 0x20;
                    if(inchar == 0x0A) inchar = 0x20;
                    
                    wiringPiI2CWriteReg8(fd,next,inchar);  // Write into I2C Slave Ring Buffer 
                     next++;  // Increment Ring Buffer Position
                    if(next > END) next = START;  // Ring Buffer Boundary
                    
                    
                  printf("\n  Ring Position: %3x  char: %c", next, inchar);
                   
                }  // end if(flag == 0x31)
                delay(50); // 50 ms WiringPi Delay
          //  I2C Slave asks for more char when the Ring Buffer is almost empty (flag == 0X31)
                flag = wiringPiI2CReadReg8(fd, 0xF0); // check flag before getting next character
            
    }  // end while (1)
    fclose(fp);
    return 0;
}
