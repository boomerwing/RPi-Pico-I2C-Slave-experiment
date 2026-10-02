/**

////////////////////////////////////////////////////////////////////////////////////////////////////////

    cd ~/a_test
    
    g++ -o RPi-Text-Reader-Cmd RPi-Text-Reader-Cmd-6.cpp I2CDevice.cpp -lwiringPi
    
    sudo scp RPi-Text-Reader-Cmd pi2@10.0.0.197:./Play
    
    
    ../RPi-Text-Reader-Cmd Isaiah-55.txt
   
     i2cdetect -y -r 1
     i2cdump -y 1 0x10 b
   
    minicom -b 115200 -o -D  /dev/ttyACM0

    ../RPi-Text-Reader-Cmd Code-Groups.txt

     //////////////////////////////////////////////////////////////////////////////////////
     
     This Raspberry Pi code reads a TXT file and transmits the characters one by one to a
     Raspberry Pi Pico I2C Slave Mode Morse Code Practice sender.  Select the Code speed
     from a menu, (Speed from 10 WPM to 25 WPM)
     Before you start this program with "./RPI-Text-Reader-Cmd Textfile-name.txt" you must reset 
     the RPI Pico. The Pico will wait for the Code Speed to be set.  Start this program, 
     select the code speed, Characters will be read by this code and sent
     to the Pico.

      //////////////////////////////////////////////////////////////////////////////////////
*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "I2CDevice.h"


#define DEVICE_ID       0x10
#define FLAG_REG        0xF0
#define INDEX_IN        0xF2
#define WPM_INDEX       0xF4
#define SPD_INDEX       0xF6 
#define START           0x00
#define END             0x2F
#define SW0_INDEX       0xE0
#define SW1_INDEX       0xE1
#define SW2_INDEX       0xE2
#define SW3_INDEX       0xE3
#define SW4_INDEX       0xE4
#define SW5_INDEX       0xE5
#define SW6_INDEX       0xE6
#define SW7_INDEX       0xE7
#define ADC_INDEX       0xE8
#define SEVSEG_INDEX    0xE9

using namespace std;

void setWPM(uint8_t &speed_setting, uint8_t &speed_out);

/** setWPM function ************************************************************************
 * A menu of WPM choices are displayed. Enter Code speed you want to use.  
 * 
 * */

//void setWPM(uint8_t &speed_setting, uint8_t &speed_out){
void setWPM(uint8_t &speed_setting,uint8_t &speed_out){
    const uint8_t dotspeed[] =   {120,109,100,92,86,80,75,70,66,63,60,57,54,52,50,48};
    const uint8_t showspeed[] =  { 10, 11, 12,13,14,15,16,17,18,19,20,21,22,23,24,25};
    char command;
    uint8_t WPMcommand;
    uint8_t commandd;
    uint8_t check;
    char number_string[10];
 
   //  First Print Command menu 
    printf("\n");
    printf(" 10 WPM     14 WPM     18 WPM     22 WPM\n");
    printf(" 11 WPM     15 WPM     19 WPM     23 WPM\n");
    printf(" 12 WPM     16 WPM     20 WPM     24 WPM\n");
    printf(" 13 WPM     17 WPM     21 WPM     25 WPM\n");
    
    do{     //  Prompt for the Command, checking range of input values
   
        printf("\x1B[%i;%if",13,1);  // place curser
        printf( "  Enter WPM (10-25):     ");  // write prompt
        printf("\x1B[%i;%if",13,22);  // place curser
        fgets(number_string, 4,stdin);
        command = atoi(number_string);
       }                      
    while((command > 25) || (command < 10));  //  END do while()
         
    speed_setting = dotspeed[command-10]; // finally, once command is certified
    speed_out = command;

}

int main (int argc, char **argv)
{
    uint8_t speed_setting;
    uint8_t speed_out;
    uint8_t flag;
    uint8_t next;
    uint8_t ch_input;
    
    printf("\x1B[H\x1B[2J");
    printf("\n   RPi-Text-Reader-Cmd\n  **********************");

    // Setup I2C communication
    int fd = wiringPiI2CSetup(DEVICE_ID);
    if (fd == -1) {
        cout << endl << "Failed to init I2C communication." << endl ;
        return 0;
    }
    cout << endl << " I2C communication successfully setup.";

    // Open Text File
    FILE *fp;
    fp =  fopen(argv[1], "r");  // Open with the name of the Text File to Read
     if(!fp){
        cout << endl << " Failure to Open file" << endl ;
         return 0;
        };

    cout << endl << endl << " " << argv[1] << endl; // Display Text File name


     wiringPiI2CWriteReg8(fd,SPD_INDEX,0);  // Write 0x00 into I2C Slave SPD_INDEX register 
    delay(125); // 125 ms WiringPi Delay
     
    // Enter Words per Minute (WPM) desired - See SWAN 202 for passing variables with ref parameters
    setWPM(speed_setting, speed_out);
    
     wiringPiI2CWriteReg8(fd,SPD_INDEX,speed_setting);  // Write into I2C Slave Ring Buffer 
        delay(100); // 100 ms WiringPi Delay
     wiringPiI2CWriteReg8(fd,WPM_INDEX,speed_out);  // Write into I2C Slave Ring Buffer 
        delay(100); // 100 ms WiringPi Delay
    
    flag = 0x30;    // flag
    while(flag == 0x30){
        flag = wiringPiI2CReadReg8(fd, FLAG_REG);    // read flag from I2C Slave
        delay(125); // 125 ms WiringPi Delay
        }
                   
    next = START;
    
    while (1) {  // Only send characters to Pico I2C Slave while flag is 0x31
                 // If flag is 0x30, just check for 0x31.  Do not send characters
                if(flag == 0x31){
            
                    ch_input = (uint8_t)fgetc(fp);
                    if(feof(fp)) break;  // Close App when EOF found
                    if(ch_input > 0x7F)  ch_input = 0x20;
                    if(ch_input < 0x20)  ch_input = 0x20;
                    
                    wiringPiI2CWriteReg8(fd,next,ch_input);  // Write into I2C Slave Ring Buffer 
                     next++;  // Increment Ring Buffer Position
                    if(next > END) next = START;  // Ring Buffer Boundary
                   
                }  // end if(flag == 0x31) -- one Character has been sent
//                printf(" \x1B[%i;%if Ring Position: %3x | ASCII: %3x | char: %c ", 15,1,next, ch_input, ch_input);
        delay(100); // 100 ms WiringPi Delay
          //  I2C Slave asks for more char when the Ring Buffer is almost empty (flag == 0X31)
        flag = wiringPiI2CReadReg8(fd, FLAG_REG); // Is flag commanding stop or requesting characters?
            
    }  // end while (1)
    fclose(fp); 
    return 0;
}
