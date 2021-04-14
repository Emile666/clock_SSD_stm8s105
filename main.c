#include "main.h"
#include "delay.h"
#include "scheduler.h"
#include "i2c_bb.h"
#include "i2c_ds3231_bb.h"
#include "uart.h"
#include "eep.h"

extern uint32_t t2_millis;         // Updated in TMR2 interrupt

char     rs232_inbuf[UART_BUFLEN]; // buffer for RS232 commands
uint8_t  rs232_ptr     = 0;        // index in RS232 buffer
char     ssd_clk_ver[] = "Clock SSD v0.41\n";
uint8_t  ssd[10] = {0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B}; // 0abcdefg

uint8_t led_r[NR_LEDS];          // Array with 8-bit red colour for all WS2812
uint8_t led_g[NR_LEDS];          // Array with 8-bit green colour for all WS2812
uint8_t led_b[NR_LEDS];          // Array with 8-bit blue colour for all WS2812
uint8_t enable_test_pattern = 1; // 1 = enable WS2812 test-pattern
uint8_t watchdog_test       = 0; // 1 = watchdog test modus
uint8_t led_intensity;           // Intensity of WS2812 LEDs
bool    dst_active  = false;     // true = Daylight Saving Time active
Time    dt;                      // Struct with time and date values, updated every sec.
bool    real_binary = false;
bool    powerup     = true;

uint8_t blank_begin_h  = 23;
uint8_t blank_begin_m  = 30;
uint8_t blank_end_h    =  8;
uint8_t blank_end_m    = 30;

uint8_t  tmr3_std = STATE_IDLE;
uint16_t rawbuf[100]; // buffer with clock-ticks
uint8_t  rawlen     = 0;
uint16_t prev_ticks = 0;
uint32_t ir_result  = 0;
bool     ir_cont    = false;
bool     ir_rdy     = false;
uint8_t  ir_cmd_std = IR_CMD_IDLE;
uint8_t  ir_cmd_tmr = 0;

//----------------------------------------------------------------------------
// These values are stored directly into EEPROM
//----------------------------------------------------------------------------
__root __eeprom const int eedata[] = 
{
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Not used
       LED_INTENSITY, /* Intensity of WS2812 leds */
       0,             /* 1 = Daylight Saving Time active */
       23,            /* Blanking begin hours */
       30,            /* Blanking begin minutes */
       8,             /* Blanking end hours */
       30,            /* Blanking end minutes */
       0,             /* not in use yet */
       0              /* not in use yet */
}; // eedata[]

/*-----------------------------------------------------------------------------
  Purpose  : This is external interrupt routine 5 for PORTC
             It is connected to the IR output of the VS1838B.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = EXTI2_vector
__interrupt void PORTC_IRQHandler(void)
{
    uint16_t diff_ticks;
    uint16_t ticks = tmr3_val(); // counts st f = 31.25 kHz, T = 32 usec.
        
    if (ticks < prev_ticks)
         diff_ticks = ~prev_ticks + ticks;
    else diff_ticks = ticks  - prev_ticks;
    if (IR_RCVb) // copy IR-signal to debug output
         IRQ_LEDb = 1;
    else IRQ_LEDb = 0; 
    switch (tmr3_std)
    {
        case STATE_IDLE:
            if (!IR_RCVb)
            {   // falling edge
                rawbuf[0] = ticks;
                rawlen    = 1;
                tmr3_std  = STATE_MARK;
            } // if
            break;
        case STATE_MARK: // A mark is a 0 for the VS1838B
            if (IR_RCVb) 
            {   // rising edge, end of mark
                if (rawlen < 99)
                {
                    rawbuf[rawlen++] = diff_ticks;
                    tmr3_std         = STATE_SPACE;
                } // if
                else 
                {   // overflow
                    ir_rdy   = true;
                    tmr3_std = STATE_STOP;
                } // else
            } // if
            break;
        case STATE_SPACE:
            if (!IR_RCVb) 
            {   // falling edge, end of space
                if (diff_ticks > 625) // 625 = 20 msec. min. gap between transmissions
                {   // long space received, ready to process everything
                    ir_rdy    = true;
                    tmr3_std  = STATE_STOP;
                } // if
                else if (rawlen < 99)
                {
                    rawbuf[rawlen++] = diff_ticks;
                    tmr3_std         = STATE_MARK;
                } // if
                else 
                {   // overflow
                    ir_rdy   = true;
                    tmr3_std = STATE_STOP;
                } // else
            } // if
            break;
        case STATE_STOP:
            // remain in this state unless ir_task() resets this
            break;
    } // switch
    prev_ticks = ticks; // save ticks value
} // PORTC_IRQHandler()

/*------------------------------------------------------------------
  Purpose  : This function checks if the IR bit timing is between a
             lower and a upper limit. All values are in clock-ticks,
             a clock-tick (timer 3) is 32 usec (f = 31.25 kHz)
  Variables: -
  Returns  : true = success ; false = error
  ------------------------------------------------------------------*/
bool check_ticks(uint16_t val, uint16_t low, uint16_t high)
{
    return ((val >= low) && (val <= high));
} // check_ticks()

/*------------------------------------------------------------------
  Purpose  : This function decodes the bits from rawbuf and stores
             the result in ir_result. It is called from ir_task().
  Variables: -
  Returns  : true = success ; false = error
  ------------------------------------------------------------------*/
bool ir_decode_nec(void)
{
    int16_t  offset = 1;  // Index in to results; Skip first entry!?
    char     s[20];
    
    ir_result = 0; // We decode in to here; Start with nothing
    // Check we have the right amount of data (68). 
    // The +4 is for initial gap, start bit mark and space + stop bit mark.
    if ((rawlen < 68) && (rawlen != 4)) 
    {
        sprintf(s,"len (%d) err\n",rawlen);
        uart_printf(s);
        return false;
    } // if
    
    if (!check_ticks(rawbuf[offset], HDR_MARK_LTICKS, HDR_MARK_HTICKS))  
    {   // Check header "mark" this must be done for repeat and data
        uart_printf("hdr mark err\n");
        return false;
    } // if
    offset++;
    
    if (rawlen == 4)
    {   // Check for repeat - this has a different header space length
        uart_printf("rpt hdr ");
        if (check_ticks(rawbuf[offset  ], RPT_SPACE_LTICKS, RPT_SPACE_HTICKS) &&
            check_ticks(rawbuf[offset+1], BIT_MARK_LTICKS , BIT_MARK_HTICKS))
        {
            uart_printf("ok\n");
            return true;
        } // if
        uart_printf("err\n");
        return false; // wrong repeat header
    } // if 
    
    if (!check_ticks(rawbuf[offset], HDR_SPACE_LTICKS, HDR_SPACE_HTICKS)) 
    {   // Check command header space
        uart_printf("hdr space err\n");
        return false; // Header space length is wrong
    } // if
    offset++;
    
    // Build the data
    while (offset < rawlen-1)
    {   // Check data "mark"
        if (!check_ticks(rawbuf[offset], BIT_MARK_LTICKS, BIT_MARK_HTICKS)) 
        {
            sprintf(s,"mark %d err\n",offset);
            uart_printf(s);
            return false;
        } // if
        offset++;
        // Suppend this bit
        if      (check_ticks(rawbuf[offset], ONE_SPACE_LTICKS , ONE_SPACE_HTICKS ))  ir_result = (ir_result << 1) | 1 ;
        else if (check_ticks(rawbuf[offset], ZERO_SPACE_LTICKS, ZERO_SPACE_HTICKS))  ir_result = (ir_result << 1) | 0 ;
        else 
        {
            sprintf(s,"space %d err\n",offset);
            uart_printf(s);
            return false;
        } // else
        offset++;
    } // while
    return true; // success
} // ir_decode_nec()

/*------------------------------------------------------------------
  Purpose  : This function retrieves the proper key from the IR code
  Variables: global variable ir_result is used. It is called from ir_task().
  Returns  : code for key found
  ------------------------------------------------------------------*/
uint8_t ir_key(void)
{
    char    s[10];
    uint8_t key;
    
    switch (ir_result)
    {
        case IR_CODE_UP      : key = IR_UP      ; break;
        case IR_CODE_DOWN    : key = IR_DOWN    ; break;
        case IR_CODE_LEFT    : key = IR_LEFT    ; break;
        case IR_CODE_RIGHT   : key = IR_RIGHT   ; break;
        case IR_CODE_OK	     : key = IR_OK      ; break;
        case IR_CODE_ASTERISK: key = IR_ASTERISK; break;
        case IR_CODE_HASH    : key = IR_HASH    ; break;
        case IR_CODE_0       : key = IR_0       ; break;
        case IR_CODE_1       : key = IR_1       ; break;
        case IR_CODE_2       : key = IR_2       ; break;
        case IR_CODE_3       : key = IR_3       ; break;
        case IR_CODE_4       : key = IR_4       ; break;
        case IR_CODE_5       : key = IR_5       ; break;
        case IR_CODE_6       : key = IR_6       ; break;
        case IR_CODE_7       : key = IR_7       ; break;
        case IR_CODE_8       : key = IR_8       ; break;
        case IR_CODE_9       : key = IR_9       ; break;
        case IR_CODE_REPEAT  : key = IR_REPEAT  ; break;
        default              : key = IR_NONE    ; break;
    } // switch
    sprintf(s,"IR[%c]\n",IR_CHARS[key]);
    uart_printf(s); // output to terminal screen
    return key;
} // ir_key()

/*-----------------------------------------------------------------------------
  Purpose  : This function is called every 100 msec. and initiates all actions
             derived from IR remote keys
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void handle_ir_command(uint8_t key)
{
    if (key == IR_NONE)
    {   // increment no-action timer
        if (++ir_cmd_tmr > 100)
        {   // back to idle after 10 seconds
            ir_cmd_std = IR_CMD_IDLE; 
            return; // exit
        } // if
    } // if
    else ir_cmd_tmr = 0; // reset timer if IR key is pressed
    
    switch (ir_cmd_std)
    {
        case IR_CMD_IDLE:
            if (key == IR_0)      ir_cmd_std = IR_CMD_0;
            else if (key == IR_1) ir_cmd_std = IR_CMD_1;
            break;
        case IR_CMD_0:
            break;
        case IR_CMD_1:
            break;
        default:
            ir_cmd_std = IR_CMD_IDLE;
            break;
    } // switch   
} // handle_ir_command()

/*-----------------------------------------------------------------------------
  Purpose  : This is the 100 msec. task from the task-scheduler and it controls
             the IR remote controller. If the PORTC IRQ handler signals a new
             IR signal, it set ir_rdy high. This function then decodes the
             IR signal into a key pressed and resets the PORTC IRQ handler for
             reception of a new IR signal.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void ir_task(void)
{
    char s[25];
    uint8_t i, key = IR_NONE;
    
    if (ir_rdy)
    {
       sprintf(s,"[%d]",rawlen);
       uart_printf(s);
       for (i = 0; i < rawlen; i++)
       {
           sprintf(s,"%u,",rawbuf[i]);
           uart_printf(s);
       } // for i
       uart_putc('\n');
       if (ir_decode_nec()) 
       {
           sprintf(s,"result=0x%lx\n",ir_result);
           uart_printf(s);
           key = ir_key(); // find the key pressed
           handle_ir_command(key);
       } // if
       for (i = 0; i < 99; i++) rawbuf[i] = 0; // clear buffer
       tmr3_std = STATE_IDLE; // reset state for next IR code
       ir_rdy   = false;      // done here
    } // if
    handle_ir_command(key);   // run this every 100 msec.
} // ir_task()

/*-----------------------------------------------------------------------------
  Purpose  : This is external interrupt routine 7 for PORTE
             It is connected to the SQW output of the DS3231, but not used now.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = EXTI4_vector
__interrupt void PORTE_IRQHandler(void)
{
} // PORTE_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This is interrupt routine 13 for the Timer 2 Overflow handler.
             It runs at 1 kHz and drives the scheduler and the multiplexer.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = TIM2_OVR_UIF_vector
__interrupt void TIM2_UPD_OVF_IRQHandler(void)
{
    scheduler_isr();  // Run scheduler interrupt function
    t2_millis++;      // update milliseconds timer
    TIM2_SR1_UIF = 0; // Reset the interrupt otherwise it will fire again straight away.
} // TIM2_UPD_OVF_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This is interrupt routine 15 for the Timer 3 Overflow handler.
             It is currently not used.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = TIM3_OVR_UIF_vector
__interrupt void TIM3_UPD_OVF_IRQHandler(void)
{
    //IRQ_LEDb = !IRQ_LEDb;
    //ir_isr();
    TIM3_SR1_UIF = 0; // Reset the interrupt otherwise it will fire again straight away.
} // TIM3_UPD_OVF_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises the system clock to run at 16 MHz.
             It uses the internal HSI oscillator.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void initialise_system_clock(void)
{
    CLK_ICKR       = 0;           //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;           //  Enable the HSI.
    CLK_ECKR       = 0;           //  Disable the external clock.
    while (CLK_ICKR_HSIRDY == 0); //  Wait for the HSI to be ready for use.
    CLK_CKDIVR     = 0;           //  Ensure the clocks are running at full speed.
 
    // The datasheet lists that the max. ADC clock is equal to 6 MHz (4 MHz when on 3.3V).
    // Because fMASTER is now at 16 MHz, we need to set the ADC-prescaler to 4.
    ADC_CR1_SPSEL  = 0x02;        //  Set prescaler to 4, fADC = 4 MHz
    CLK_SWIMCCR    = 0;           //  Set SWIM to run at clock / 2.
    CLK_SWR        = 0xe1;        //  Use HSI as the clock source.
    CLK_SWCR       = 0;           //  Reset the clock switch control register.
    CLK_SWCR_SWEN  = 1;           //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);  //  Pause while the clock switch is busy.
} // initialise_system_clock()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises Timer 2 to generate a 1 kHz interrupt.
             16 MHz/(16 * 1000) = 1000 Hz (1000 = 0x03E8), which is the main
             time-base for the scheduler.
             Furthermore, set the interrupt priority to level 1 (medium), so
             that it only can be interrupted by the IR-interrupt (Timer 3).
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_timer2(void)
{
    TIM2_PSCR    = 0x04;    //  Prescaler = 16
    TIM2_ARRH    = 0x03;    //  High byte of 1000
    TIM2_ARRL    = 0xE8;    //  Low  byte of 1000
    
    ITC_SPR4_VECT13SPR = 1; // IRQ 13 (Timer 2 update/overflow) to priority 1
    TIM2_IER_UIE       = 1; //  Enable the update interrupts
    TIM2_CR1_CEN       = 1; //  Finally enable the timer
} // setup_timer2()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises Timer 3 to generate a 31.25 kHz 32 usec.
             counter. 16 MHz/(2^9) = 31.25 kHz. This counter is used for the
             VS1838b IR remote controller.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_timer3(void)
{
    TIM3_PSCR    = 0x09;    //  Prescaler = 512, freq. = 31.25 kHz, T = 32 usec.
    //TIM3_ARRH    = 0x00;    //  High byte of 50
    //TIM3_ARRL    = 0x32;    //  Low  byte of 50
    
    //ITC_SPR4_VECT15SPR = 0; // IRQ 15 (Timer 3 update/overflow) to priority 2
    //TIM3_IER_UIE       = 1; //  Enable the update interrupts
    TIM3_CR1_CEN       = 1; //  Finally enable the timer
} // setup_timer3()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises all the GPIO pins of the STM8 uC.
             See binary_clock.h for a detailed description of all pin-functions.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_output_ports(void)
{
  PB_ODR     |=  (I2C_SCL | I2C_SDA);   // Must be set here, or I2C will not work
  PB_DDR     |=  (I2C_SCL | I2C_SDA);   // Set as outputs
  PB_CR2     &= ~(I2C_SCL | I2C_SDA);   // O: Set speed to 2 MHz, I: disable IRQ
  
  PC_DDR     |= DI_3V3;     // Set as output
  PC_CR1     |= DI_3V3;     // Set to Push-Pull
  PC_ODR     &= ~(DI_3V3);  // Turn off outputs
  PC_ODR     |=  IR_RCV;
  PC_DDR     &= ~IR_RCV;    // Set as input
  PC_CR1     &= ~IR_RCV;    // Enable pull-up
  PC_CR2     |=  IR_RCV;    // Enable external interrupt
  EXTI_CR1_PCIS = 0x03;     // PORTC external interrupt to rising & falling edge
  ITC_SPR2_VECT5SPR = 0;    // PORTC external interrupt to priority 2 (higher)
  
  PD_DDR     |= TX;         // Set as output
  PD_CR1     |= TX;         // Set to Push-Pull
  PD_ODR     |= TX;         // Set TX high
  PD_DDR     &= ~RX;        // Set UART1-RX as input
  PD_CR1     &= ~RX;        // Set to floating
  
  PE_DDR     &= ~SQW;       // Set as input
  PE_CR1     |=  SQW;       // Enable pull-up
  
  //PE_CR2     |=  SQW;       // Enable external interrupt
  //EXTI_CR2_PEIS = 0x01;     // PORTE external interrupt to rising edge only
  PE_DDR     |=  IRQ_LED;
  PE_CR1     |=  IRQ_LED;
  PE_ODR     &= ~IRQ_LED;
} // setup_output_ports()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends one byte to the WS2812B LED-string.
  Variables: bt: the byte to send
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812b_send_byte(uint8_t bt)
{
    uint8_t i,x = 0x80; // Start with MSB first
    
    for (i = 0; i < 8; i++)
    {
        if (bt & x)
        {    // Send a 1   
             ws2812b_send_1;
        } // if
        else 
        {   // Send a 0
            ws2812b_send_0;
        } // else
        x >>= 1; // Next bit
    } // for i
} // ws2812b_send_byte()

/*-----------------------------------------------------------------------------
  Purpose  : This routine clears all WS2812B LEDs.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void clear_all_leds(void)
{
    for (uint8_t i = 0; i < NR_LEDS; i++)
    {
        led_g[i] = led_r[i] = led_b[i] = 0x00;
    } // for i
 } // clear_all_leds()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends a test pattern to all WS2812B LEDs. It is 
             called by pattern_task() every 100 msec.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void test_pattern(void)
{
    static uint8_t cntr_b = 0, tmr_b = 0;
    uint8_t i;

    if (++tmr_b >= 20)
    {   // change colour every 2 seconds
        tmr_b = 0;
        switch (cntr_b)
        {
            case 0: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_b[i] = led_intensity;
                    led_g[i] = led_r[i] = 0x00;
                } // for
                cntr_b = 1; // next colour
                break;
            case 1: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_g[i] = led_intensity;
                    led_b[i] = led_r[i] = 0x00;
                } // for
                cntr_b = 2;
                break;
            case 2: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_r[i] = led_intensity;
                    led_b[i] = led_g[i] = 0x00;
                } // for
                cntr_b = 0;
                break;
        } // switch
    } // if
} // test_pattern()

/*------------------------------------------------------------------------
  Purpose  : Encode a byte into 2 BCD numbers.
  Variables: x: the byte to encode
  Returns  : the two encoded BCD numbers
  ------------------------------------------------------------------------*/
uint8_t encode_to_bcd(uint8_t x)
{
    uint8_t temp;
    uint8_t retv = 0;
    
    temp   = x / 10;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    temp   = x - temp * 10;
    retv  |= (temp & 0x0F);
    return retv;
} // encode_to_bcd()

/*-----------------------------------------------------------------------------
  Purpose  : This routine creates a pattern for the LEDs and stores it in
             the arrays led_r, led_g and led_b
             It uses the global variables seconds, minutes and hours and is
             called every 100 msec. by the scheduler.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void pattern_task(void)
{
    uint8_t x,xl,xm;
    
    if (!watchdog_test)   
    {   // only refresh when watchdog_test == 0 (X0 command)
        IWDG_KR = IWDG_KR_KEY_REFRESH; // Refresh watchdog (reset after 500 msec.)
    } // if
    if (enable_test_pattern)
    {   // WS2812 test-pattern
	test_pattern(); 
    } // if
    else
    {
        if (blanking_active() || powerup) 
        {
            clear_all_leds();
            return;
        } // if
        // check summertime change every minute
        if (dt.sec == 0) check_and_set_summertime(); 
    	x  = encode_to_bcd(dt.sec);
        xm = (x >> 4) & 0x0F; // msb seconds
        xl = x & 0x0F;        // lsb seconds
        // There's at least 1 board, send seconds
        led_r[NR_LEDS- 1] = 0x00; // no decimal-point
        led_r[NR_LEDS- 2] = led_r[NR_LEDS- 3] = 
        led_r[NR_LEDS- 4] = led_r[NR_LEDS- 5] = (ssd[xl] & SEG_F) ? led_intensity : 0x00;
        led_r[NR_LEDS- 6] = led_r[NR_LEDS- 7] = 
        led_r[NR_LEDS- 8] = led_r[NR_LEDS- 9] = (ssd[xl] & SEG_A) ? led_intensity : 0x00;
        led_r[NR_LEDS-10] = led_r[NR_LEDS-11] = 
        led_r[NR_LEDS-12] = led_r[NR_LEDS-13] = (ssd[xl] & SEG_B) ? led_intensity : 0x00;
        led_r[NR_LEDS-14] = led_r[NR_LEDS-15] = 
        led_r[NR_LEDS-16] = led_r[NR_LEDS-17] = (ssd[xl] & SEG_G) ? led_intensity : 0x00;
        led_r[NR_LEDS-18] = led_r[NR_LEDS-19] = 
        led_r[NR_LEDS-20] = led_r[NR_LEDS-21] = (ssd[xl] & SEG_C) ? led_intensity : 0x00;
        led_r[NR_LEDS-22] = led_r[NR_LEDS-23] = 
        led_r[NR_LEDS-24] = led_r[NR_LEDS-25] = (ssd[xl] & SEG_D) ? led_intensity : 0x00;
        led_r[NR_LEDS-26] = led_r[NR_LEDS-27] = 
        led_r[NR_LEDS-28] = led_r[NR_LEDS-29] = (ssd[xl] & SEG_E) ? led_intensity : 0x00;
#if NR_BOARDS > 1
        led_r[NR_LEDS-30] = 0x00; // no decimal-point
        led_r[NR_LEDS-31] = led_r[NR_LEDS-32] = 
        led_r[NR_LEDS-33] = led_r[NR_LEDS-34] = (ssd[xm] & SEG_F) ? led_intensity : 0x00;
        led_r[NR_LEDS-35] = led_r[NR_LEDS-36] = 
        led_r[NR_LEDS-37] = led_r[NR_LEDS-38] = (ssd[xm] & SEG_A) ? led_intensity : 0x00;
        led_r[NR_LEDS-39] = led_r[NR_LEDS-40] = 
        led_r[NR_LEDS-41] = led_r[NR_LEDS-42] = (ssd[xm] & SEG_B) ? led_intensity : 0x00;
        led_r[NR_LEDS-43] = led_r[NR_LEDS-44] = 
        led_r[NR_LEDS-45] = led_r[NR_LEDS-46] = (ssd[xm] & SEG_G) ? led_intensity : 0x00;
        led_r[NR_LEDS-47] = led_r[NR_LEDS-48] = 
        led_r[NR_LEDS-49] = led_r[NR_LEDS-50] = (ssd[xm] & SEG_C) ? led_intensity : 0x00;
        led_r[NR_LEDS-51] = led_r[NR_LEDS-52] = 
        led_r[NR_LEDS-53] = led_r[NR_LEDS-54] = (ssd[xm] & SEG_D) ? led_intensity : 0x00;
        led_r[NR_LEDS-55] = led_r[NR_LEDS-56] = 
        led_r[NR_LEDS-57] = led_r[NR_LEDS-58] = (ssd[xm] & SEG_E) ? led_intensity : 0x00;
#endif        
#if NR_BOARDS > 2
    	x  = encode_to_bcd(dt.min);
        xm = (x >> 4) & 0x0F; // msb minutes
        xl = x & 0x0F;        // lsb minutes
        led_g[NR_LEDS-59] = 0x00; // no decimal-point
        led_g[NR_LEDS-60] = led_g[NR_LEDS-61] = 
        led_g[NR_LEDS-62] = led_g[NR_LEDS-63] = (ssd[xl] & SEG_F) ? led_intensity : 0x00;
        led_g[NR_LEDS-64] = led_g[NR_LEDS-65] = 
        led_g[NR_LEDS-66] = led_g[NR_LEDS-67] = (ssd[xl] & SEG_A) ? led_intensity : 0x00;
        led_g[NR_LEDS-68] = led_g[NR_LEDS-69] = 
        led_g[NR_LEDS-70] = led_g[NR_LEDS-71] = (ssd[xl] & SEG_B) ? led_intensity : 0x00;
        led_g[NR_LEDS-72] = led_g[NR_LEDS-73] = 
        led_g[NR_LEDS-74] = led_g[NR_LEDS-75] = (ssd[xl] & SEG_G) ? led_intensity : 0x00;
        led_g[NR_LEDS-76] = led_g[NR_LEDS-77] = 
        led_g[NR_LEDS-78] = led_g[NR_LEDS-79] = (ssd[xl] & SEG_C) ? led_intensity : 0x00;
        led_g[NR_LEDS-80] = led_g[NR_LEDS-81] = 
        led_g[NR_LEDS-82] = led_g[NR_LEDS-83] = (ssd[xl] & SEG_D) ? led_intensity : 0x00;
        led_g[NR_LEDS-84] = led_g[NR_LEDS-85] = 
        led_g[NR_LEDS-86] = led_g[NR_LEDS-87] = (ssd[xl] & SEG_E) ? led_intensity : 0x00;
#endif        
#if NR_BOARDS > 3
        led_g[NR_LEDS- 88] = 0x00; // no decimal-point
        led_g[NR_LEDS- 89] = led_g[NR_LEDS- 90] = 
        led_g[NR_LEDS- 91] = led_g[NR_LEDS- 92] = (ssd[xm] & SEG_F) ? led_intensity : 0x00;
        led_g[NR_LEDS- 93] = led_g[NR_LEDS- 94] = 
        led_g[NR_LEDS- 95] = led_g[NR_LEDS- 96] = (ssd[xm] & SEG_A) ? led_intensity : 0x00;
        led_g[NR_LEDS- 97] = led_g[NR_LEDS- 98] = 
        led_g[NR_LEDS- 99] = led_g[NR_LEDS-100] = (ssd[xm] & SEG_B) ? led_intensity : 0x00;
        led_g[NR_LEDS-101] = led_g[NR_LEDS-102] = 
        led_g[NR_LEDS-103] = led_g[NR_LEDS-104] = (ssd[xm] & SEG_G) ? led_intensity : 0x00;
        led_g[NR_LEDS-105] = led_g[NR_LEDS-106] = 
        led_g[NR_LEDS-107] = led_g[NR_LEDS-108] = (ssd[xm] & SEG_C) ? led_intensity : 0x00;
        led_g[NR_LEDS-109] = led_g[NR_LEDS-110] = 
        led_g[NR_LEDS-111] = led_g[NR_LEDS-112] = (ssd[xm] & SEG_D) ? led_intensity : 0x00;
        led_g[NR_LEDS-113] = led_g[NR_LEDS-114] = 
        led_g[NR_LEDS-115] = led_g[NR_LEDS-116] = (ssd[xm] & SEG_E) ? led_intensity : 0x00;
#endif        
#if NR_BOARDS > 4
    	x  = encode_to_bcd(dt.hour);
        xm = (x >> 4) & 0x0F; // msb hours
        xl = x & 0x0F;        // lsb hours
        led_b[NR_LEDS-117] = 0x00; // no decimal-point
        led_b[NR_LEDS-118] = led_b[NR_LEDS-119] = 
        led_b[NR_LEDS-120] = led_b[NR_LEDS-121] = (ssd[xl] & SEG_F) ? led_intensity : 0x00;
        led_b[NR_LEDS-122] = led_b[NR_LEDS-123] = 
        led_b[NR_LEDS-124] = led_b[NR_LEDS-125] = (ssd[xl] & SEG_A) ? led_intensity : 0x00;
        led_b[NR_LEDS-126] = led_b[NR_LEDS-127] = 
        led_b[NR_LEDS-128] = led_b[NR_LEDS-129] = (ssd[xl] & SEG_B) ? led_intensity : 0x00;
        led_b[NR_LEDS-130] = led_b[NR_LEDS-131] = 
        led_b[NR_LEDS-132] = led_b[NR_LEDS-133] = (ssd[xl] & SEG_G) ? led_intensity : 0x00;
        led_b[NR_LEDS-134] = led_b[NR_LEDS-135] = 
        led_b[NR_LEDS-136] = led_b[NR_LEDS-137] = (ssd[xl] & SEG_C) ? led_intensity : 0x00;
        led_b[NR_LEDS-138] = led_b[NR_LEDS-139] = 
        led_b[NR_LEDS-140] = led_b[NR_LEDS-141] = (ssd[xl] & SEG_D) ? led_intensity : 0x00;
        led_b[NR_LEDS-142] = led_b[NR_LEDS-143] = 
        led_b[NR_LEDS-144] = led_b[NR_LEDS-145] = (ssd[xl] & SEG_E) ? led_intensity : 0x00;
#endif        
#if NR_BOARDS > 5
        led_b[NR_LEDS-146] = 0x00; // no decimal-point
        led_b[NR_LEDS-147] = led_b[NR_LEDS-148] = 
        led_b[NR_LEDS-149] = led_b[NR_LEDS-150] = (ssd[xm] & SEG_F) ? led_intensity : 0x00;
        led_b[NR_LEDS-151] = led_b[NR_LEDS-152] = 
        led_b[NR_LEDS-153] = led_b[NR_LEDS-154] = (ssd[xm] & SEG_A) ? led_intensity : 0x00;
        led_b[NR_LEDS-155] = led_b[NR_LEDS-156] = 
        led_b[NR_LEDS-157] = led_b[NR_LEDS-158] = (ssd[xm] & SEG_B) ? led_intensity : 0x00;
        led_b[NR_LEDS-159] = led_b[NR_LEDS-160] = 
        led_b[NR_LEDS-161] = led_b[NR_LEDS-162] = (ssd[xm] & SEG_G) ? led_intensity : 0x00;
        led_b[NR_LEDS-163] = led_b[NR_LEDS-164] = 
        led_b[NR_LEDS-165] = led_b[NR_LEDS-166] = (ssd[xm] & SEG_C) ? led_intensity : 0x00;
        led_b[NR_LEDS-167] = led_b[NR_LEDS-168] = 
        led_b[NR_LEDS-169] = led_b[NR_LEDS-170] = (ssd[xm] & SEG_D) ? led_intensity : 0x00;
        led_b[NR_LEDS-171] = led_b[NR_LEDS-172] = 
        led_b[NR_LEDS-173] = led_b[NR_LEDS-174] = (ssd[xm] & SEG_E) ? led_intensity : 0x00;
#endif     
    } // else
} // pattern_task()    
        
/*-----------------------------------------------------------------------------
  Purpose  : This routine sends the RGB-bytes for every LED to the WS2812B
             LED string. It is called every 100 msec. by the scheduler.
  Variables: 
      led_g: the (global) green byte array
      led_r: the (global) red byte array
      led_b: the (global) blue byte array
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812_task(void)
{
    __disable_interrupt();   // disable IRQ for time-sensitive LED-timing
    for (uint8_t i = 0; i < NR_LEDS; i++)
    {
        ws2812b_send_byte(led_g[i]); // Send one byte of Green
        ws2812b_send_byte(led_r[i]); // Send one byte of Red
        ws2812b_send_byte(led_b[i]); // Send one byte of Blue
    } // for i
    __enable_interrupt(); // enable IRQ again
} // ws2812_task()

/*------------------------------------------------------------------------
Purpose  : This task is called every minute by pattern_task(). It checks 
           for a change from summer- to wintertime and vice-versa.
           To start DST: Find the last Sunday in March  : @2 AM advance clock to 3 AM.
           To stop DST : Find the last Sunday in October: @3 AM set clock back to 2 AM (only once!).
Variables: p: pointer to time-struct
Returns  : -
------------------------------------------------------------------------*/
void check_and_set_summertime(void)
{
    uint8_t        hr,day,lsun03,lsun10,dst_eep;
    static uint8_t advance_time = 0;
    static uint8_t revert_time  = 0;
    char           s[20];
    
    if (dt.mon == 3)
    {
        day    = ds3231_calc_dow(31,3,dt.year); // Find day-of-week for March 31th
        lsun03 = 31 - (day % 7);                // Find last Sunday in March
        sprintf(s,"lsun03=%d\n",lsun03); 
        uart_printf(s);
        switch (advance_time)
        {
        case 0: if ((dt.day == lsun03) && (dt.hour == 2) && (dt.min == 0))
                {   // At 2:00 AM advance time to 3 AM, check for one minute
                    advance_time = 1;
                } // if
                else if (dt.day < lsun03) dst_active = false;
                else if (dt.day > lsun03) dst_active = true;
                else if (dt.hour < 2)     dst_active = false;
                break;
        case 1: // Now advance time, do this only once
                ds3231_settime(3,0,dt.sec); // Set time to 3:00, leave secs the same
                advance_time = 2;
                dst_active   = true;
                eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x01); // set DST in eeprom
                break;
        case 2: 
                if (dt.min > 0) advance_time = 0; // At 3:01:00 back to normal
                dst_active = true;
        break;
        } // switch
    } // if
    else if (dt.mon == 10)
    {
        day    = ds3231_calc_dow(31,10,dt.year); // Find day-of-week for October 31th
        lsun10 = 31 - (day % 7);                 // Find last Sunday in October
        sprintf(s,"lsun10=%d\n",lsun10); 
        uart_printf(s);
        switch (revert_time)
        {
            case 0: if ((dt.day == lsun10) && (dt.hour == 3) && (dt.min == 0))
                    {   // At 3:00 AM revert time back to 2 AM, check for one minute
                        revert_time = 1;
                    } // if
                    else if (dt.day > lsun10) dst_active = false;
                    else if (dt.day < lsun10) dst_active = true;
                    else if (dt.hour < 3)     dst_active = true;
                    break;
            case 1: // Now revert time, do this only once
                    ds3231_settime(2,0,dt.sec); // Set time back to 2:00, leave secs the same
                    revert_time = 2;
                    dst_active  = false;
                    eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x00); // reset DST in eeprom
                    break;
            case 2: // make sure we passed 3 AM in order to prevent multiple reverts
                    if (dt.hour > 3) revert_time = 0; // at 4:00:00 back to normal
                    dst_active = false;
                    break;
        } // switch
    } // else if
    else if ((dt.mon < 3) || (dt.mon > 10)) dst_active = false;
    else                                    dst_active = true;

    //------------------------------------------------------------------------
    // If, for some reason, the clock was powered-off during the change to
    // summer- or winter-time, the eeprom value differs from the actual 
    // dst_active value. If so, set the actual sommer- and winter-time.
    //------------------------------------------------------------------------
    dst_eep = (uint8_t)eeprom_read_config(EEP_ADDR_DST_ACTIVE);
    if (dst_active && !dst_eep)
    {   // It is summer-time, but clock has not been advanced yet
        hr = (dt.hour >= 23) ? 0 : dt.hour + 1;
        ds3231_settime(hr,dt.min,dt.sec); // Set summer-time to 1 hour later
        eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x01); // set DST in eeprom
    } // if
    else if (!dst_active && dst_eep)
    {   // It is winter-time, but clock has not been moved back yet
        hr = (dt.hour > 0) ? dt.hour - 1 : 23;
        ds3231_settime(hr,dt.min,dt.sec); // Set summer-time to 1 hour earlier
        eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x00); // set DST in eeprom
    } // if
} // check_and_set_summertime()

/*-----------------------------------------------------------------------------
  Purpose  : This routine reads the date and time info from the DS3231 RTC and
             stores this info into the global variables seconds, minutes and
             hours.
  Variables: 
    seconds: global variable [0..59]
    minutes: global variable [0..59]
      hours: global variable [0..23]
  Returns  : -
  ---------------------------------------------------------------------------*/
void clock_task(void)
{
    ds3231_gettime(&dt);
    powerup = false;
} // clock_task()

/*-----------------------------------------------------------------------------
  Purpose  : This routine prints the day-of-week to the uart
  Variables: 
        dow: [0..7], 1=Monday, 7 = Sunday
  Returns  : -
  ---------------------------------------------------------------------------*/
void print_dow(uint8_t dow)
{
    char day[8][4] = {"???","Mon","Tue","Wed","Thu","Fri","Sat","Sun"};

    uart_printf(day[dow]);
} // print_dow()

/*-----------------------------------------------------------------------------
  Purpose  : This routine reads the time and date from the DS3231 RTC and 
             prints this info to the uart.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void print_date_and_time(void)
{
    char s2[40]; // Used for printing to UART

    check_and_set_summertime();
    sprintf(s2," %d-%d-%d, %d:%d.%d",
               dt.day , dt.mon, dt.year,
               dt.hour, dt.min, dt.sec);
    uart_printf(s2);
    sprintf(s2," dow:%d, dst:%d, blanking:%d\n",
               dt.dow, dst_active, blanking_active());
    uart_printf(s2);
} // print_date_and_time()

/*------------------------------------------------------------------------
  Purpose  : This function converts hours and minutes to minutes.
  Variables: h  : hours of actual time
             min: minutes of actual time
  Returns  : time in minutes
  ------------------------------------------------------------------------*/
uint16_t cmin(uint8_t h, uint8_t m)
{
    return (uint16_t)h * 60 + m;
} // cmin()

/*------------------------------------------------------------------------
  Purpose  : This function decides if the current time falls between the
             blanking time for the LEDs.
  Variables: -
  Returns  : true: blanking is true, false: no blanking
  ------------------------------------------------------------------------*/
bool blanking_active(void)
{
	uint16_t x = cmin(dt.hour      , dt.min);
	uint16_t b = cmin(blank_begin_h, blank_begin_m);
	uint16_t e = cmin(blank_end_h  , blank_end_m);
	
	// (b>=e): Example: 23:30 and 05:30, active if x>=b OR  x<=e
	// (b< e): Example: 02:30 and 05:30, active if x>=b AND x<=e
	return (b >= e) && ((x >= b) || (x <= e)) || ((x >= b) && (x < e)); 
} // blanking_active()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the USB serial terminal:
  Variables: 
          s: the string that contains the command from RS232 serial port 0
  Returns  : -
  ---------------------------------------------------------------------------*/
void execute_single_command(char *s)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   char     s2[40]; // Used for printing to RS232 port
   char     *s1;
   uint8_t  d,m,h,sec;
   uint16_t i,y;
   int16_t  temp;
   const char sep[] = ":-.";
   
   switch (s[0])
   {
        case 'd': // Set Date, 1 = Get Date
		 switch (num)
		 {
                    case 0: // Set Date
			    s1 = strtok(&s[3],sep);
                            d  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            y  = atoi(s1);
                            uart_printf("Date: ");
                            print_dow(ds3231_calc_dow(d,m,y));
                            sprintf(s2," %d-%d-%d\n",d,m,y);
                            uart_printf(s2);
                            ds3231_setdate(d,m,y); // write to DS3231 IC
                            break;
                    case 1: // Set Time
                            s1      = strtok(&s[3],sep);
                            h       = atoi(s1);
                            s1      = strtok(NULL ,sep);
                            m       = atoi(s1);
                            s1      = strtok(NULL ,sep);
                            sec     = atoi(s1);
                            sprintf(s2,"Time: %d:%d:%d\n",h,m,sec);
                            uart_printf(s2);
                            ds3231_settime(h,m,sec); // write to DS3231 IC
                            break;
                    case 2: // Get Date & Time
                            print_date_and_time(); 
                            sprintf(s2,"Blanking: %d:%d - %d:%d\n",
                                       blank_begin_h, blank_begin_m,
                                       blank_end_h  , blank_end_m);
                            uart_printf(s2);
                            break;
                    case 3: // Get Temperature
                            temp = ds3231_gettemp();
                            sprintf(s2,"DS3231: %d.",temp>>2);
                            uart_printf(s2);
                            switch (temp & 0x03)
                            {
				case 0: uart_printf("00 C\n"); break;
				case 1: uart_printf("25 C\n"); break;
				case 2: uart_printf("50 C\n"); break;
				case 3: uart_printf("75 C\n"); break;
                            } // switch
                            break;
                    case 4: // Set Start-Time for blanking Nixies
                            s1 = strtok(&s[3],sep);
                            h  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            if ((h < 24) && (m < 60))
                            {
                                blank_begin_h = h;
                                blank_begin_m = m;
                                eeprom_write_config(EEP_ADDR_BBEGIN_H,blank_begin_h);
                                eeprom_write_config(EEP_ADDR_BBEGIN_M,blank_begin_m);
                            } // if
                            break;
                    case 5: // Set End-Time for blanking Nixies
                            s1 = strtok(&s[3],sep);
                            h  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            if ((h < 24) && (m < 60))
                            {
                                blank_end_h = h;
                                blank_end_m = m;
                                eeprom_write_config(EEP_ADDR_BEND_H,blank_end_h);
                                eeprom_write_config(EEP_ADDR_BEND_M,blank_end_m);
                            } // if
                            break;		 
                   default: break;
                 } // switch
                 break;
  
	case 'i': // Set intensity of WS2812 LEDs between 1..255
		 if (num > 0)
                 {
                     led_intensity = num;
                     eeprom_write_config(EEP_ADDR_INTENSITY,led_intensity);
                 } // if
		 break;

	case 's': // System commands
		 switch (num)
		 {
                    case 0: // revision
                            uart_printf(ssd_clk_ver);
                            break;
                    case 1: // List all tasks
                            list_all_tasks(); 
                            break;
                    case 2: // I2C-scan
			    uart_printf("I2C-scan: ");
			    for (i = 0x02; i < 0xff; i+=2)
			    {
				if (i2c_start_bb(i) == I2C_ACK)
				{
					sprintf(s2,"0x%x, ",i);
		  		    	uart_printf(s2);
				} // if
				i2c_stop_bb();
			    } // for
			    uart_putc('\n');
                            break;
                   default: break;
                 } // switch
		 break;
                                        
	case 'w': // WS2812 test-pattern command
		 enable_test_pattern = num; // 1 = enable test-pattern
                 if (!num)
                 {  // clear all leds when finished with test-pattern
                    clear_all_leds();
                 } // if
		 break;

        default: break;
   } // switch
} // execute_single_command()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking RS232 command-handler via the USB port
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void rs232_command_handler(void)
{
  char    ch;
  static uint8_t cmd_rcvd = 0;
  
  if (!cmd_rcvd && uart_kbhit())
  { // A new character has been received
    ch = tolower(uart_getc()); // get character as lowercase
    switch (ch)
	{
            case '\n': break;
            case '\r': cmd_rcvd  = 1;
		       rs232_inbuf[rs232_ptr] = '\0';
		       rs232_ptr = 0;
                       uart_putc('\n');
                       break;
            default  : if (rs232_ptr < UART_BUFLEN)
                       {   
                           rs232_inbuf[rs232_ptr++] = ch;
                           uart_putc(ch);
                       } // if
                       else rs232_ptr = 0;
                       break;
	} // switch
  } // if
  if (cmd_rcvd)
  {
	  cmd_rcvd = 0;
	  execute_single_command(rs232_inbuf);
  } // if
} // rs232_command_handler()

/*-----------------------------------------------------------------------------
  Purpose  : This functions initializes the independent watchdog (IWDG) and 
             sets the watchdog timeout to the maximum of T = 512 msec.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void init_watchdog(void)
{
	IWDG_KR  = IWDG_KR_KEY_ENABLE;  // start the IWDG
	IWDG_KR  = IWDG_KR_KEY_ACCESS;  // enable access to IWDG_PR and IWDG_RLR registers
	IWDG_PR  = 0x05;                // prescaler divider 128
	IWDG_RLR = 0xFF;	        // Reload register to maximum
	IWDG_KR  = IWDG_KR_KEY_REFRESH; // reset the IWDG
} // init_watchdog()

/*-----------------------------------------------------------------------------
  Purpose  : This is the main entry-point for the program
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
int main(void)
{
    uint8_t i2c_err;
	
    __disable_interrupt();
    initialise_system_clock(); // Set system-clock to 16 MHz
    setup_output_ports();      // Init. needed output-ports for LED and keys
    setup_timer2();            // Set Timer 2 to 1 kHz for scheduler
    setup_timer3();            // Set Timer 3 to 20 kHz for IRDA
    uart_init();               // Init. UART-peripheral

    led_intensity = (uint8_t)eeprom_read_config(EEP_ADDR_INTENSITY);
    if (!led_intensity)
    {   // First time power-up: eeprom value is 0x00
        led_intensity = LED_INTENSITY;
    } // if
    blank_begin_h = (uint8_t)eeprom_read_config(EEP_ADDR_BBEGIN_H);
    blank_begin_m = (uint8_t)eeprom_read_config(EEP_ADDR_BBEGIN_M);
    blank_end_h   = (uint8_t)eeprom_read_config(EEP_ADDR_BEND_H);
    blank_end_m   = (uint8_t)eeprom_read_config(EEP_ADDR_BEND_M);
    
    // Initialise all tasks for the scheduler
    scheduler_init();                         // clear task_list struct
    add_task(pattern_task, "PTRN"  , 0, 100); // every 100 msec.
    add_task(ws2812_task , "WS2812",25, 500); // every 100 msec.
    add_task(ir_task     , "IR"    ,50, 100); // every 100 msec.
    add_task(clock_task  , "CLK"   ,75,1000); // every second
    init_watchdog();                          // init. the IWDG watchdog
    __enable_interrupt();

    i2c_err = i2c_reset_bus(); // Init. I2C-peripheral
    uart_printf(ssd_clk_ver);  // Print welcome message
    uart_printf("i2c_reset_bus:");
    if (i2c_err) uart_printf("err\n");
    else         uart_printf("ok\n");
    
    if (ds3231_write_register(REG_CTRL,0x00)) // Enable 1 Hz SQW output
    {   //1 = error, DS3231 RTC not found
        uart_printf("DS3231 not found\n");
        dt.hour = dt.min = dt.sec = 0; // init. 00:00:00 if no RTC present
        dt.day  = dt.mon = 1;          // init. to 01-01-2021 if no RTC present
        dt.year = 2021;
        dt.dow  = FRIDAY;
    } // if
    else ds3231_gettime(&dt); // Read time from DS3231 RTC
    print_date_and_time();    // and output to UART

    while (1)
    {   // background-processes
        dispatch_tasks();        // Run task-scheduler()
        rs232_command_handler(); // run command handler continuously
    } // while
} // main()
