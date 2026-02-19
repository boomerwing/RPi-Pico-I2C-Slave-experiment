 /**
 * RP2040 FreeRTOS Template: Send CW with Timer trigger
 * Timer is monostable, creating delay between text sent
 * Also learning to use structures in Queue
 * Sending Morse Code text
 * 
 * cd ~/FreeRTOS-CW-Play/build/App-I2C-Slave
 * 
 * 
 * minicom -b 115200 -o -D  /dev/ttyACM0 -C "Shuffle_Out.txt"
 * 
 * minicom -b 115200 -o -D  /dev/ttyACM0
 *  
    sudo scp Gardeners_Grandmother.txt pi2@10.0.0.197:./Play
    sudo scp Nutcracker.txt pi2@10.0.0.197:./Play
    sudo scp Code-Groups.txt pi2@10.0.0.197:./Play
    
    ./RPi_Text_Reader Nutcracker.txt
    ./RPi_Text_Reader Gardeners_Grandmother.txt
    ./RPi_Text_Reader Code-Groups.txt
   
 * 
 * built on 
 * @copyright 2022, Tony Smith (@smittytone)  
 * @version   1.2.0
 * @licence   MIT
 *
 */
 
#include <stdio.h>
#include "main.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "../Common/pico_i2c_slave/i2c_slave/include/i2c_slave.h"

#define DOT_PERIOD 65
#define TONE_FREQ 500
#define TONE_GPIO  21        // pin 27
#define CW_GPIO    17        // pin 22 
#define CW_LED_GPIO   0      // pin  1
#define I2C_LED0_GPIO 1      // pin  2
#define I2C0_SDA_GPIO 4      // pin  6
#define I2C0_SCL_GPIO 5      // pin  7
#define I2C0_IRQ    23       // I2C0 IRQ
#define FLAG_REG    0xF0      // Flag
#define IN_INDEX    0xF2      // Index of incoming Character
#define OUT_INDEX   0xF4
#define START       0x00
#define END         0x2F

/* 
 * GLOBALS
 */
    // FROM 1.0.1 Record references to the tasks
TaskHandle_t cw_task_handle = NULL;
TaskHandle_t i2c_task_handle = NULL;
TaskHandle_t sw1_task_handle = NULL;
TaskHandle_t latch_task_handle = NULL;
// TaskHandle_t select_phrase_task_handle = NULL;

    // These are the Send CW timers
volatile TimerHandle_t cwd_timer;

    // These are the inter-task queues
volatile QueueHandle_t xQdit = NULL;
volatile QueueHandle_t xQChar = NULL;
volatile QueueHandle_t xQsw1 = NULL;
volatile QueueHandle_t xQadc = NULL;
volatile QueueHandle_t xQlatch = NULL;

/*
 * FUNCTIONS
 */

void switch1(uint8_t);
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event);



// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred. Reading is done
// sequentially from the current memory address.
static struct
{
    uint8_t mem[0xFF];
    uint8_t mem_address;  // Index into the mem string
    uint8_t mem_address_saved;
    bool mem_address_written;
} context;

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// such as printing to stdio may interfere with interrupt handling.
// This slave handler just reads the registers and puts values in the 
// registers.  These registers in Global Variable space can then be accessed
// by functions in the microprocessor workspace as needed.

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: // master has written some data
            if (!context.mem_address_written) {
                // writes always start with the memory address
                i2c_read_raw_blocking(i2c,&context.mem_address,1); // pico SDK I2C
                context.mem_address_written = true;
            } else {
                // save into memory
                 i2c_read_raw_blocking(i2c,&context.mem[context.mem_address],1 ); // pico SDK I2C
                context.mem_address_saved = context.mem_address;
            }
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
            // load from memory
           i2c_write_raw_blocking(i2c,&context.mem[context.mem_address],1); // pico SDK I2C
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            context.mem_address_written = false;
            break;
        default:
            break;
        }
}



void i2c_task(void* unused_arg) {
    UBaseType_t xStatus;
    uint8_t sw_state;
    int counter_LED_state = 0;
    int8_t flag;
    int8_t end_flag = pdFALSE;
    int8_t out_flag = pdFALSE;
    int8_t char_out; 
    int8_t out_ring;
    int8_t in_adr = 0x00;;
    int8_t save_in_adr;
    int8_t CharOut_Count;
    
    int8_t difference;
      
    context.mem_address = 0;
    context.mem_address_written = false;

      // ****************************************************************   
    sw_state = 1;
    CharOut_Count= 0;
    // First look for Switch Change to send a signal to RPi 
    // to start text transmission
    context.mem[FLAG_REG] = 0x30;
    xQueuePeek(xQsw1,&sw_state, 0);  // extra activity to show Task alive
    gpio_put(I2C_LED0_GPIO, sw_state); 
    printf("Start Pico, start RPi Zero, then Press Switch\n");
    while(sw_state){
        xQueuePeek(xQsw1,&sw_state, 0);
//        gpio_put(I2C_LED0_GPIO, sw_state); 
       vTaskDelay(ms_delay100);  // Break for other tasks 
    }
    context.mem[FLAG_REG] = 0x31;
    gpio_put(I2C_LED0_GPIO, sw_state); 
     
     // ****************************************************************  
       vTaskDelay(ms_delay200);  // Break for other tasks 
      out_ring = 0x00;
        
      
   while (1) {
        // Allow at least 5 characters to be entered into the Ring Buffer before characters are
        // taken to be output as CW code.
            in_adr = context.mem_address_saved ;
            if(in_adr > 0x04 && out_flag == 0) {
                out_flag = 1;
            }
            
            if(in_adr > out_ring) difference =  ( (END +1) - in_adr) + out_ring;
            else if((in_adr) < out_ring) difference = out_ring - in_adr;
            
            if (((int8_t)difference) < 0x5 ){  // if Ring almost Full, stop Master Tx
                 context.mem[IN_INDEX] = in_adr;  // save in_adr 
                 flag = (int8_t)0x30;
                 context.mem[FLAG_REG] = flag;
                gpio_put(I2C_LED0_GPIO, 0);
                }
            else if(((uint8_t)difference > (END - 0x08)) &&  (context.mem[FLAG_REG] == 0x30)){
                 flag = (int8_t)0x31;   // if Ring almost empty, start Master Tx
                 context.mem[IN_INDEX] = in_adr;  // use saved in_adr
                 context.mem[FLAG_REG] = flag;
                gpio_put(I2C_LED0_GPIO, 1);
                }
          // Output to CW Queue - Only if I2C Server has already sent a number of characters      
            if(out_flag == 1){
                if(uxQueueSpacesAvailable( xQChar)){  // keep Queue full
                    char_out = context.mem[out_ring];  // take char from tail of Ring Buffer
                    xStatus = xQueueSendToBack(xQChar,&char_out,portMAX_DELAY);

                    out_ring++; 
                    if(out_ring > END) out_ring = START;
                }
            }
       vTaskDelay(ms_delay100);  // Break for other tasks
        if(out_flag == 1){
            if(in_adr == out_ring){  // EOF found in Text, stop CW, Blink LED
                end_flag = 1;
                while(1){
                    gpio_put(I2C_LED0_GPIO, end_flag);
                    end_flag = !end_flag;
                    vTaskDelay(ms_delay500);  // Break for other tasks
                }
            }
        }
                
    }  // end of while(true)
}

/**
 * @brief Send CW, blinking RX LED, switching audio tone ON and OFF 
 *        Characters are sent through the I2C Task Queue to regulate between
 *        the speed of the I2C source and the speed needed for CW transmission.
 * 
 */
  
void cw_task(void* unused_arg) {
    UBaseType_t uxMessagesWaiting  = 0;
    UBaseType_t xStatus;

    uint8_t i  = 1;
    uint8_t char_count  = 0;
    uint8_t this_char = 0;
    vTaskDelay(ms_delay50);
    
   while (true) {
           
           //************* Bring Character in from I2C ****************************************  

            if(uxQueueMessagesWaiting(xQChar)){
                xStatus = xQueueReceive(xQChar,&this_char, portMAX_DELAY); 
                
           //*********************************************************************************  

                send_CW((char)this_char);
           //************* Format Character Print output *************************************  
                char_count++;
                    if ((char_count > 60) && (this_char == 0x20)){
                         char_count = 0;
                         printf("\n");
                     }
                printf("%c",this_char);
           //*********************************************************************************  
            } // end of if(...
    }  // end of while(true)
} 


/**
 * @brief CW print a Morse character from a supplied ASCII character.
 * Morse char 'F' is ". . - ." converted to 0x1175 or Binary 1000101110101
 * This Binary number is shifted RIGHT one bit at a time.  Each Morse letter ends
 * with three '0's (making one space following the letter). The leftmost '1' is
 * the flag to indicate that the letter is completely sent.
 *
 */
void send_CW(char ascii_in) {
    const uint32_t masknumber = 0x00000001;
    uint8_t this_char = 0;
    uint32_t bit_out = 1;
    uint32_t morse_out = 0;
    UBaseType_t xStatus;
    UBaseType_t uxNumberOfQSpaces = 1;
    struct op
    {
        uint32_t t_number;
        uint32_t t_state;
    }; 
    struct op tdit_info = {1,1};

    
// **** Full Speed  with punctuation ***********************************************************     
//                        A      B      C     D    E      F      G     H     I     J
    uint32_t morse[] = {0x11D,0x1157,0x45D7,0x457,0x11,0x1175,0x1177,0x455,0x45,0x11DDD,
//                        0      1      2     3    4      5      6     7     8     9
// *********************************************************************************************     
//        K      L     M     N      O      P      Q      R     S     T 
       0X11D7,0X115D,0X477,0X117,0X4777,0X22DD,0X11D77,0X45D,0X115,0X47,
//       10     11    12    13     14     15     16     17    18    19
// *********************************************************************************************     
//        U      V      W     X       Y       Z       0       1        2       3  
       0X475,0X11D5,0X11DD,0X4757,0X11DD7,0X4577,0X477777,0x11DDDD,0X47775,0X11DD5,
//       20     21     22    23      24      25      26      27       28      29  
// *********************************************************************************************     
//        4      5      6      7        8       9     SP  SPlg    .        , 
       0X4755,0X1155,0X4557,0X11577,0X45777,0X117777,0X10,0X100,0X11D75D,0X477577,
//       30     31     32     33       34      35     36   37     38      39 
// *********************************************************************************************     
//        ?       /        '       !
       0x45775,0X11757,0X15DDDD,0X4775D7};
//        40      41       42      43
// *********************************************************************************************     

        this_char = ascii_in;
       
        if (this_char == 44) { // look for COMMA char
            this_char = 39;    // index to Morse char
            }
        else if (this_char == 46) { // look for PERIOD char
            this_char = 38;    // index to Morse char
            }
        else if (this_char == 63) { // look for QUESTION char
            this_char = 40;    // index to Morse char
            }
        else if (this_char == 47) { // look for FORWARD SLASH char
            this_char = 41;    // index to Morse char
            }
        else if (this_char == 39) { // look for APOSTROPHY char
            this_char = 42;    // index to Morse char
            }
        else if (this_char == 33) { // look for Exclamation char
            this_char = 43;    // index to Morse char
            }
        else if (this_char == 0x10) { // look for LONG SPACE char
            this_char = 37;    // index to Morse char
            }
        else if (this_char == 0x20) { // look for SPACE char
            this_char = 36;    // index to Morse char
            }
        else if (isdigit(this_char)) {    // look for Numbers
            this_char = this_char - 22 ;  // index to Morse char
            }
        else if(islower(this_char)) {     // look for Lower Case
            this_char = toupper(this_char);
            this_char = this_char - 65;   // index to Morse char
            }
        else if(isalpha(this_char)){
            this_char = this_char - 65;   // index to Morse char
            }
        else  {
            this_char = 36; // SPACE for any unidentified characters
         }   
        morse_out = morse[this_char];     // get Morse char from array
/**
 // ************* Code Speed Adjustment from ADC ********************************       
        last_dot_period = dotperiod;
        xQueuePeek(xQadc,&dotperiod, 0); 
        if(last_dot_period != dotperiod){
            xTimerChangePeriod( cwd_timer,dotperiod,0 );
        }
// ******************************************************************************       
*/        
        
    while (morse_out != 1) {              // send Morse bits              
        bit_out = (morse_out & masknumber);  // isolate bit to send
        gpio_put(CW_LED_GPIO, bit_out);  //  Dots need High to turn ON  
        gpio_put(CW_GPIO, bit_out);    // 
        
        morse_out /= 2;                   // divide by 2 to shift Right
        if (cwd_timer != NULL) {
            xTimerStart(cwd_timer, 0);  // define dot length 
            }
         xStatus = xQueueReceive(xQdit, &tdit_info, portMAX_DELAY); // xQueueReceive empties queue
       }

} 



/**
 * @brief Callback actioned when the CW timer fires.  Sends trigger to
 * initiate a new CW String TX.
 *
 * @param timer: The triggering timer.
 */
void cwd_timer_fired_callback(TimerHandle_t timer) {
    UBaseType_t xStatus;
    UBaseType_t uxNumberOfQSpaces = 1;
    struct op
    {
        uint32_t t_number;
        uint32_t t_state;
    }; 
    struct op timer_info = {2,0};

    if (timer == cwd_timer)  // define dot length 
         // The timer fired so trigger cw ID in cw task
        xQueueOverwrite(xQdit, &timer_info);
}

/**
 * @brief Switch Debounce Repeat check of SW, send result adc task to 
 * show seven segment LED decimal or hex number
 * Measures sw state, compares NOW state with PREVIOUS state. If states are different
 * sets count == 0 and looks for two states the same.  It then looks for five or more (MIN_COUNT)
 * in a row or more where NOW and PREVIOUS states are the same. Then Switch state is used
 * as a control signal, passed to an action function by a Queue.
 */
 void sw1_task(void* unused_arg) {
    UBaseType_t xStatus;
    UBaseType_t uxNumberOfQSpaces = 1;
    uint8_t now = 1;            // initialize sw1_state
    uint8_t last = 1;   // initialize sw1_previous_state
    uint8_t count = 1;   // initialize sw1_final_state

    while (true) {
        // Measure SW and add the switch state
        // to the FreeRTOS xQUEUE
        last = now; // save last sw_state
        if(gpio_get(SW01)) now = 1;
        else now = 0;
        
        if(last == now) {
            count++;
            if(count >10) count = 3;
            vTaskDelay(ms_delay50);  // check switch state every 50 ms
            }
        else {
             // previous state != sw_state, switch has changed
             count = 0;  // Need at least MIN_COUNT consecutive same states
             while(count <3){
                last = now;  // Need at least MIN_COUNT consecutive same states
                if(gpio_get(SW01)) now = 1;
                else now = 0;
                if(last == now){
                    count++;
                }
                else{
                    count = 0;
                }
                vTaskDelay(ms_delay5);  // check switch state every 50 ms
            }  //  End if (count>3)
            switch1(now);
       }
    }  // End while (true)    
}

/*
 * Switch1(int)
 *  Input Switch state value.  The function will Mask out its Switch value 
 * from sw_state and will send Qmessage to task requiring
 * switch change. The switch value is expected to remain in the gueue until
 * changed.
 */
void switch1(uint8_t sw_state) {
    uint8_t now = 0 ;
    
    if(sw_state){
        now = 1;
     }
     else {
         now = 0;
     }
        xQueueOverwrite(xQsw1, &now);
}

/* 
 * @brief Switch Latch Task 
 * This receives a switch Queue output and makes an ON/OFF Latch
 * It watches for a HiGH to LOW state change of now.  When the Low state
 * is seen it watches for a LOW to HIGH state change then changes the
 * stete of the Latch variable and puts the Latch variable on the 
 * xQlatch Queue.
 */

 void latch_task(void* unused_arg) {

    uint8_t now = 1;
    uint8_t latch = 1;
    
    while (true) {
        // Read switch input
            xQueuePeek(xQsw1,&now, 0);
            while ( !now ) { // now is LOW
                xQueuePeek(xQsw1,&now, 0);
                if ( now ) {  //  now is back to HIGH so Latch changes state
                    if (latch == 0 ) latch = 1;
                    else  latch = 0;
                xQueueOverwrite(xQlatch, &latch);                    
                }  // end if(now)
            vTaskDelay(ms_delay75);  // check switch value every 75 ms
            }  // end while(!now)
    
        vTaskDelay(ms_delay500);  // check SW1 value every 200 ms
    }  // End while (true)    
}

/**
 * @brief Initialize GPIO Pin for input and output.
 *
 */
void configure_gpio(void) {
    uint32_t pico_led_state = 1;

    // Enable STDIO
    stdio_init_all();

    stdio_usb_init();
    // Pause to allow the USB path to initialize
    sleep_ms(2000);
    
    // Configure PICO_LED_PIN for Initialization failure warning
    gpio_init(PICO_LED_PIN);
    gpio_disable_pulls(PICO_LED_PIN);  // remove pullup and pulldowns
    gpio_set_dir(PICO_LED_PIN, GPIO_OUT);
    gpio_put(PICO_LED_PIN, pico_led_state);  // set initial state to OFF

    // Configure I2C_LED0_GPIO
    gpio_init(I2C_LED0_GPIO);
    gpio_disable_pulls(I2C_LED0_GPIO);  // remove pullup and pulldowns
    gpio_set_dir(I2C_LED0_GPIO, GPIO_OUT);

    // Configure CW_PIN
    gpio_init(CW_GPIO);
    gpio_disable_pulls(CW_GPIO);  // remove pullup and pulldowns
    gpio_set_dir(CW_GPIO,GPIO_OUT);

      // Configure CW_LED_PIN
    gpio_init(CW_LED_GPIO);
    gpio_disable_pulls(CW_LED_GPIO);  // remove pullup and pulldowns
    gpio_set_dir(CW_LED_GPIO, GPIO_OUT);

    // Configure SW1 
    gpio_init(SW01);  
    gpio_pull_up(SW01);  // pullup for switches
    gpio_set_dir(SW01, GPIO_IN);

    // Configure i2c0 on GPIO 4,5 Pins 6,7
    i2c_init(i2c0, 100 * 1000);
    i2c_set_slave_mode(i2c0,1,0);
    i2c_slave_init(i2c0, 0x16,  &i2c_slave_handler);
    irq_set_enabled(I2C0_IRQ,1);  // pins 6 and 7
    
    gpio_init(I2C0_SDA_GPIO);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    
    gpio_init(I2C0_SCL_GPIO);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SCL_GPIO);
    
    
    
    // Configure GPIOCLOCK0 to blink LED on GPIO 21 using System PLL clock source
    clock_gpio_init(D21_P27, 
                    CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                     0X3C000); //   500 Hz
                    
    

}
 
/*
 * RUNTIME START
 */
int main() {
    uint32_t error_state = 0;
    uint32_t pico_led_state = 0;
    struct op
    {
        uint32_t sw_number;
        uint32_t sw_state;
    }; 
    struct op sw_info = {0,0};

    configure_gpio();
    
     // label Program Screen
    printf("\x1B[2J");  // Clear Screen
    printf("\x1B[%i;%iH",2,3);  // place curser
    puts("*** CW Sending Program - Text In with I2C Slave ***");
    printf("\x1B[%i;%iH",4,1);  // place curser
    puts("0****^****1****^****2****^****3****^****4****^****5****^*****6****^****7****^*****8");
    printf("\x1B[%i;%ir",6,18);  // set window top and bottom lines
 

// Timer creates dot length
    cwd_timer = xTimerCreate("CWD_TIMER", 
                            DOT_PERIOD,
                            pdFALSE,
                            (void*)DOT_TIMER_ID,
                            cwd_timer_fired_callback);
        if (cwd_timer == NULL) {
            error_state  += 1;
            }
            

    // Set up tasks
    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes

     BaseType_t i2c_status = xTaskCreate(i2c_task, 
                                         "I2C_TASK", 
                                         1024, 
                                         NULL, 
                                         9,     // Task priority
                                         &i2c_task_handle);
        if (i2c_status != pdPASS) {
            error_state  += 1;
            }
   
    BaseType_t sw1_status = xTaskCreate(sw1_task, 
                                         "SW1_TASK", 
                                         256, 
                                         NULL, 
                                         8,     // Task priority
                                         &sw1_task_handle);
        if (sw1_status != pdPASS) {
           error_state  += 1;
            }
    
    BaseType_t cw_status = xTaskCreate(cw_task, 
                                         "CW_TASK", 
                                         256, 
                                         NULL, 
                                         7, 
                                         &cw_task_handle);
        if (cw_status != pdPASS) {
            error_state  += 1;
            }
            
     BaseType_t latch_status = xTaskCreate(latch_task,             
                                         "LATCH_TASK", 
                                         256, 
                                         NULL, 
                                         6,     // Task priority
                                         &latch_task_handle);
        if (latch_status != pdPASS) {
            error_state  += 1;
            }
             
          
   // Set up the event queue
    xQdit = xQueueCreate(1, sizeof(sw_info)); 
    if ( xQdit == NULL ) error_state += 1;

    xQsw1 = xQueueCreate(1, sizeof(uint8_t)); 
    if ( xQsw1 == NULL ) error_state += 1; 

    xQlatch = xQueueCreate(1, sizeof(uint8_t));
    if ( xQlatch == NULL ) error_state += 1;
    
    xQChar = xQueueCreate(8, sizeof(uint8_t)); 
    if ( xQChar == NULL ) error_state += 1; 
    
    // Start the FreeRTOS scheduler 
    // FROM 1.0.1: Only proceed if no tasks signal error in setup
    if (error_state == 0) {
        vTaskStartScheduler();
    }
    else {   // if tasks don't initialize, pico board led will light   
        pico_led_state = 1;
        gpio_put(PICO_LED_PIN, pico_led_state);
    }
    
    // We should never get here, but just in case...
    while(true) {
        // NOP
    };
}
