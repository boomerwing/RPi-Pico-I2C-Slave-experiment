 /**
 * RP2040 FreeRTOS Template: Send CW with Timer trigger
 * Timer is monostable, creating delay between dits sent
 * 
 * The Clock function provides a 500 Hz Square wave
 * The Clock output and CW output are two inputs to a Common Collector NOR Gate
 * A CW LOW (GPIO 20) blocks the PIO Square wave output (GPIO 17)
 *  
 * An i2c task on I2C port I2C0 is introduced to allow the Pico to be used as an I2C
 * Slave. An IRQ0 Interrupt reads the I2C interrupt line (IRQ 23) to sense when the 
 * Master wants to communicate.  The I2C Slave registers are defined in "Global" memory
 * to be accessible from various tasks.
 * 
 * This program uses characters input through and I2C interface.  A program running
 * on a Raspberry Pi Zero reads a TXT file and sends it to the Pico I2C Slave.
 * The Pico software receives the Characters sent in a Ring buffer.  As the ring fills
 * it monitors how close it is to full and sends a Stop flag back to the I2C Master.
 * The Pico software then empties the Ring Buffer until it is worth filling it again.
 * A Send Flag starts the Master sending Charactrs again.
 * 
 * The original CW Function continues to be active, being fed with characters through
 * the I2C Master - Slave Function.
 * 
 * built on code provided by 
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.2.0
 * @licence   MIT
 * 
 * cd ~/FreeRTOS-CW-I2C/build/App-CW-I2C-Slave
 * 
 * minicom -b 115200 -o -D  /dev/ttyACM0
 * 
 * I2C0_IRQ 23  
 * I2C1_IRQ 24  
 * 
 * ./pico_ring_test Nutcracker.txt
 *
 */
 
#include <stdio.h>
#include "main.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "../Common/pico_i2c_slave/i2c_slave/include/i2c_slave.h"


#define PAUSE_PERIOD 50
#define DOT_PERIOD 68
#define SPTOOSIZE 130
#define TONE_FREQ 500
#define CW_GPIO  17          // pin 22 for CW Tone output
#define CW_LED_GPIO 11       // pin  15
#define I2C_LED0_GPIO 13     // pin  17
#define I2C_LED1_GPIO 14     // pin  19
#define I2C1_SDA_GPIO 2      // pin  4
#define I2C1_SCL_GPIO 3      // pin  5
#define I2C0_SDA_GPIO 4      // pin  6
#define I2C0_SCL_GPIO 5      // pin  7
#define I2C1_IRQ    24      
#define I2C0_IRQ    23      
#define FLAG_REG   0xF0      // Flag
#define IN_INDEX   0xF2      // Index of incoming Character
#define OUT_INDEX  0xF4
#define START      0x00
#define END        0x2F

/* 
 * GLOBALS
 */
    // FROM 1.0.1 Record references to the tasks
TaskHandle_t cw_task_handle = NULL;
TaskHandle_t sw1_task_handle = NULL;
TaskHandle_t i2c_task_handle = NULL;

    // These are the Send CW timers
volatile TimerHandle_t cwd_timer;

    // These are the inter-task queues
volatile QueueHandle_t xQdit = NULL;
volatile QueueHandle_t xQtoggle = NULL;
volatile QueueHandle_t xQsw1 = NULL;
volatile QueueHandle_t xQchar = NULL;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event);



// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred. Reading is done
// sequentially from the current memory address.
static struct
{
    uint8_t mem[255];
    uint8_t mem_address;
    uint8_t mem_address_saved;
    bool mem_address_written;
} context;

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// such as printing to stdio may interfere with interrupt handling.
// This slave handler just reads the registers and puts values in the 
// registers.  These registers in Global Variable space can then be accessed
// by functions in the microprocessor workspaceas needed.

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
            context.mem_address+1;
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            context.mem_address_written = false;
            break;
        default:
            break;
        }
}


/*
 * FUNCTIONS
 */
/**  i2c Task - This task is just a Dummy Task to show Read and Write values in variables.
 * The I2C Slave function is set up to have 256 registers in Global Variable space.
 * These registers are available to the I2C IRQ as well as any task in the application.
 * This task monitors the context.mem I2C registers and shows an LED indication
 * of the change of value of the register.
 * A practical use is to accept I2C input of text from a TXT file and send characters to a
 * Morse Code generator.
*/
void i2c_task(void* unused_arg) {
    UBaseType_t xStatus;
    uint8_t led_state;
    int counter_LED_state = 0;
    int8_t flag;
    int8_t char_out; 
    int8_t out_ring;
    int8_t in_adr = 0x00;;
    int8_t save_in_adr;
    int8_t CharOut_Count;
    
    int8_t difference;
      
    context.mem_address = 0;
    context.mem_address_written = false;

      // ****************************************************************   
    led_state = 1;
    CharOut_Count= 0;
    xQueuePeek(xQsw1,&led_state, 0);  // extra activity to show Task alive
    printf("\n Press Switch to send Start Flag to RPi Zero to start text");
    while(xQueuePeek(xQsw1,&led_state, 0)){
        context.mem[FLAG_REG] = 0x30;
    }
    context.mem[FLAG_REG] = 0x31;
    gpio_put(I2C_LED0_GPIO, led_state); 
     
     // ****************************************************************  
       vTaskDelay(ms_delay50);  // Break for other tasks 
      out_ring = 0x00;
        
//        in_adr = context.mem_address;
      
   while (true) {
            if(uxQueueSpacesAvailable( xQchar)){
                char_out = context.mem[out_ring];  // tail of Ring Buffer
                xStatus = xQueueSendToBack(xQchar,&char_out,portMAX_DELAY);

                out_ring++; 
                if(out_ring > END) out_ring = START;
            

            in_adr = context.mem_address_saved ;
            
            if(in_adr > out_ring) difference =  ( (END +1) - in_adr) + out_ring;
            else if((in_adr) < out_ring) difference = out_ring - in_adr;
            
            if (((int8_t)difference) < 0x10){
                 context.mem[IN_INDEX] = in_adr;
                 flag = (int8_t)0x30;
                 context.mem[FLAG_REG] = flag;
                gpio_put(I2C_LED0_GPIO, 0);
                }
            else if(((uint8_t)difference > (END - 0x08)) &&  (context.mem[FLAG_REG] == 0x30)){
                 flag = (int8_t)0x31;
                 context.mem[IN_INDEX] = in_adr;
                 context.mem[FLAG_REG] = flag;
                gpio_put(I2C_LED0_GPIO, 1);
                }
                
//            printf("\n  flag: %3x  in_adr: %3x  out_ring%3x  difference: %3x  Char Out: %c ",context.mem[FLAG_REG], in_adr, out_ring, difference, char_out);
            }
       vTaskDelay(ms_delay100);  // Break for other tasks
    }  // end of while(true)
}

/**
 * @brief Send CW, blinking RX LED, switching audio tone ON and OFF 
 *        xQtimer1 triggers TX of phrase sent from Phrases task.
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

            if(uxQueueMessagesWaiting(xQchar)){
                xStatus = xQueueReceive(xQchar,&this_char, portMAX_DELAY); 
                
           //*********************************************************************************  

                send_CW((char)this_char);
                char_count++;
                    if ((char_count > 60) && (this_char == 0x20)){
                         char_count = 0;
                         printf("\n");
                     }
                printf("%c",this_char);
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
    uint32_t masknumber = 0x00000001;
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

// ***********Normal pause between letters *****************************************************   
//                              A      B      C     D    E      F      G     H     I     J
    const uint32_t morse[] = {0x11D,0x1157,0x45D7,0x457,0x11,0x1175,0x1177,0x455,0x45,0x11DDD,
//                              0      1      2     3    4      5      6     7     8     9
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
       0X4755,0X1155,0X4557,0X11577,0X45777,0X117777,0X10,0X100,0X101D75D,0X4077577,
//       30     31     32     33       34      35     36   37     38      39 
// *********************************************************************************************     
//        ?       /        '        !
       0x4057751,0X11757,0X15DDDD,0X4775D7};
//        40      41       42       43
// *********************************************************************************************     


        this_char = ascii_in;
        
        if (this_char == 32) { // look for space char
            this_char = 36;    // index to Morse char
            }
        else if (this_char == 0x10) { // look for space char
            this_char = 37;    // index to Morse char
            }
        else if (this_char == 46) { // look for PERIOD char
            this_char = 38;         // index to Morse char
            }
        else if (this_char == 44) { // look for COMMA char
            this_char = 39;    // index to Morse char
            }
        else if (this_char == 63) { // look for QUESTION char
            this_char = 40;         // index to Morse char
            }
        else if (this_char == 47) { // look for FORWARD SLASH char
            this_char = 41;         // index to Morse char
            }
        else if (this_char == 39) { // look for APOSTROPHY char
            this_char = 42;         // index to Morse char
            }
        else if (this_char == 33) { // look for EXCLAMATION char
            this_char = 43;    // index to Morse char
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
            this_char = 37; // SPACE long
            }      

        morse_out = morse[this_char];     // get Morse char from array
        
    while (morse_out != 1) {              // send Morse bits              
        bit_out = (morse_out & masknumber);  // isolate bit to send
        gpio_put(CW_GPIO, bit_out);       //  Output to Audio Gate
        gpio_put(DOTL, !bit_out);         //  Dots need High to turn ON  
        
        morse_out /= 2;                   // divide by 2 to shift Right
        if (cwd_timer != NULL) {
            xTimerStart(cwd_timer, 0);  // define dot length 
            }
         xStatus = xQueueReceive(xQdit, &tdit_info, portMAX_DELAY); // xQueueReceive empties queue
       }

} 


/**
 * @brief Switch Debounce Repeat check of SW, send result adc task to 
 * show seven segment LED decimal or hex number
 * Measures sw state, compares NOW state with PREVIOUS state. If states are different
 * sets count == 0 and looks for two states the same.  It then looks for five or more (MIN_COUNT)
 * in a row or more where NOW and PREVIOUS states are the same. Then Switch state is used
 * as a control signal, passed to an action function by a Queue.
 */
 void sw1_debounce(void* unused_arg) {
    UBaseType_t xStatus;
    UBaseType_t uxNumberOfQSpaces = 1;
    uint8_t now = 1;            // initialize sw1_state
    uint8_t last = 1;           // initialize sw1_previous_state
    uint32_t count = 1;         // initialize sw1_final_state
    uint8_t toggle = 0;         // initialize Toggle_final_state
    uint32_t toggle_count = 0;  // initialize sw1_final_state
    uint8_t out = 1;            // initialize sw1_state

    while (true) {
        // Measure SW and add the switch state
        // to the FreeRTOS xQUEUE
        last = now; // save last sw_state
        if(gpio_get(SW1)) now = 1;
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
                if(gpio_get(SW1)) now = 1;
                else now = 0;
                if(last == now){
                    count++;
                }
                else{
                    count = 0;
                }
            vTaskDelay(ms_delay5);  // check switch state every 50 ms
            }  //  End if (count>3)

            if( now ) {
                out = 1;
            }
            else {
                out = 0;
            }
//            printf("\n sw1: %3x", out);
           xQueueOverwrite(xQsw1, &out);  //  CW Start
           
        // ******************   Toggle Output ************************     
            if( out == 0 && toggle_count == 0){
                 toggle_count++;
             }
            else if( out == 1 && toggle_count != 0) {
                toggle = !toggle;
                toggle_count = 0;
            }
            xQueueOverwrite(xQtoggle, &toggle);  // for CW Speed Control
        // **********************************************************     
            
        }
    }  // End while (true)    
}



/*
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


/*
 * @brief Initialize GPIO Pin for input and output.
 *
 */
void configure_gpio(void) {
    uint32_t pico_led_state = 0;

    // Enable STDIO
    stdio_init_all();

    stdio_usb_init();
    // Pause to allow the USB path to initialize
    sleep_ms(2000);
    
    // Configure PICO_LED_PIN for Initialization failure warning
    gpio_init(PICO_LED_PIN);
    gpio_disable_pulls(PICO_LED_PIN);  // remove pullup and pulldowns
    gpio_set_dir(PICO_LED_PIN, GPIO_OUT);

    // Configure CW_LED_PIN
    gpio_init(CW_LED_GPIO);
    gpio_disable_pulls(CW_LED_GPIO);  // remove pullup and pulldowns
    gpio_set_dir(CW_LED_GPIO, GPIO_OUT);

    // Configure I2C_LED0_PIN
    gpio_init(I2C_LED0_GPIO);
    gpio_disable_pulls(I2C_LED0_GPIO);  // remove pullup and pulldowns
    gpio_set_dir(I2C_LED0_GPIO, GPIO_OUT);

    // Configure I2C_LED1_PIN
    gpio_init(I2C_LED1_GPIO);
    gpio_disable_pulls(I2C_LED1_GPIO);  // remove pullup and pulldowns
    gpio_set_dir(I2C_LED1_GPIO, GPIO_OUT);

    // Configure CW_PIN
    gpio_init(CW_GPIO);
    gpio_disable_pulls(CW_GPIO);  // remove pullup and pulldowns
    gpio_set_dir(CW_GPIO, GPIO_OUT);

    // Configure SW1 
    gpio_init(SW1);
    gpio_pull_up(SW1);  // pullup for switches
    gpio_set_dir(SW1, GPIO_IN);

    // Configure i2c on GPIO 0,1 Pins 1,2
    i2c_init(i2c0, 100 * 1000);
    gpio_init(I2C0_SDA_GPIO);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    
    gpio_init(I2C0_SCL_GPIO);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SCL_GPIO);
    
    i2c_set_slave_mode(i2c0,1,0);
    i2c_slave_init(i2c0, 0x16,  &i2c_slave_handler);

    irq_set_enabled(I2C0_IRQ,1);  // pins 6 and 7
 
    // Enable board LED
    gpio_put(PICO_LED_PIN, pico_led_state);  // set initial state to OFF
    
    // Configure GPIOCLOCK0 to blink LED on GPIO 21 using System PLL clock source
    clock_gpio_init(D21_P27, 
                    CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                     0X3C000); //   500 Hz
                  
    context.mem[FLAG_REG] = 0x30;
}
 
/*
 * RUNTIME START
 */
int main() {
    uint32_t error_state = 0;
    uint32_t pico_led_state = 0;
    uint8_t now = 1;
    char pph[SPTOOSIZE];
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
    puts("*** CW Sending  Program ***");
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

    BaseType_t cw_status = xTaskCreate(cw_task, 
                                         "CW_LED_PIN_TASK", 
                                         1024, 
                                         NULL, 
                                         7, 
                                         &cw_task_handle);
        if (cw_status != pdPASS) {
            error_state  += 1;
            }
            
    BaseType_t sw1_status = xTaskCreate(sw1_debounce, 
                                         "SW1_TASK", 
                                         1024, 
                                         NULL, 
                                         6,     // Task priority
                                         &sw1_task_handle);
        if (sw1_status != pdPASS) {
           error_state  += 1;
            }
    
     BaseType_t i2c_status = xTaskCreate(i2c_task, 
                                         "I2C_TASK", 
                                         1024, 
                                         NULL, 
                                         8,     // Task priority
                                         &i2c_task_handle);
        if (i2c_status != pdPASS) {
            error_state  += 1;
            }
   
   // Set up the event queue
    xQdit = xQueueCreate(1, sizeof(sw_info)); 
    if ( xQdit == NULL ) error_state += 1;

    xQsw1 = xQueueCreate(1, sizeof(uint8_t)); 
    if ( xQsw1 == NULL ) error_state += 1; 

    xQtoggle = xQueueCreate(1, sizeof(uint8_t)); 
    if ( xQtoggle == NULL ) error_state += 1; 
    
    xQchar = xQueueCreate(8, sizeof(uint8_t)); 
    if ( xQchar == NULL ) error_state += 1; 
    
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
        }
}
