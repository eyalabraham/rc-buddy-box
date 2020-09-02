/* buddy-box.c
 *
 * AVR code for ATtiny84 PPM generator.
 * Device replaces salvaged transmitter-electronics
 * to generate a PPM signal as a buddy-box controller.
 *
 * Port A bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'i' ADC0 Channel 1 (Rudder/Ailerons)
 *  |  |  |  |  |  |  +------ 'i' ADC1 Channel 2 (Elevator)
 *  |  |  |  |  |  +--------- 'i' ADC2 Channel 3 (Throttle)
 *  |  |  |  |  +------------ 'i' ADC3 Channel 4 (Rudder)
 *  |  |  |  +--------------- 'i' ADC4 Channel 5 (Flaps)
 *  |  |  +------------------ 'i' PA5  Channel 1 reverse
 *  |  +--------------------- 'i' PA6  Channel 2 reverse
 *  +------------------------ 'i' PA7  Channel 4 reverse
 *
 * Port B bit assignment
 *
 *              b3 b2 b1 b0
 *              |  |  |  |
 *              |  |  |  +--- 'o' PB0 Test point 1 - scope sync
 *              |  |  +------ 'o' PB1 PPM signal output
 *              |  +--------- 'i' PB2 Calibration mode
 *              +------------ 'i' ^Reset
 *
 * note: all references to data sheet are for ATtiny84 Rev. 8006K–AVR–10/10
 *
 */

#include    <stdint.h>

#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>
#include    <util/delay.h>

// IO ports initialization
#define     PA_DDR_INIT     0b00000000  // port data direction
#define     PA_PUP_INIT     0b11100000  // port input pin pull-up
#define     PA_INIT         0b00000000  // port initial values

#define     PB_DDR_INIT     0b00000011  // port data direction
#define     PB_PUP_INIT     0b00000100  // port input pin pull-up
#define     PB_INIT         0b00000010  // port initial values

// Timer0 initialization
#define     TCCR0A_INIT     0b00000010  // Clear on compare with OCR0A
#define     TCCR0B_INIT     0b00000011  // Mode=2, Fclk/64
#define     OCR0A_INT_ENA   0b00000010  // Interrupt on OCR0A compare match
#define     OCR0A_INT_CLR   0b00000010  // Clear interrupt on OCR0A compare match

// Timer1 initialization
#define     TCCR1A_INIT     0b00000000  // Clear on compare with OCR1A
#define     TCCR1B_INIT     0b00001011  // Mode=4. Fclk/64
#define     OCR1A_INT_ENA   0b00000010  // Interrupt on OCR1A compare match
#define     OCR1A_INT_CLR   0b00000010  // Clear interrupt on OCR1A compare match

// ADC initialization
#define     ADMUX_INIT      0b00100000  // Vcc reference, converter to AGND
#define     ADCSRA_INIT     0b10000110  // Enable ADC, no interrupts, Fclk/64
#define     ADCSRB_INIT     0b00010000  // Result is left shifted into ADCH
#define     DIDR0_INIT      0b00011111  // Disable digital input on ADC pins

#define     ADC_CHAN_MASK   0b00111111  // ADC channel number mask
#define     ADC0            0
#define     ADC1            1
#define     ADC2            2
#define     ADC3            3
#define     ADC4            4

#define     ADC_START       0b01000000  // Start conversion bit in ADCSRA
#define     ADC_CONV_COMP   0b00010000  // Conversion complete bit

// Input/output pin masks
#define     PPM_SYNC        0b00000001
#define     PPM_OUTPUT_BIT  0b00000010
#define     CHAN_CALIB      0b00000100

#define     CHAN_REV_DIPSW  0b11100000
#define     CHAN1_REV       0b00100000
#define     CHAN2_REV       0b01000000
#define     CHAN4_REV       0b10000000

// System definitions
#define     PPM_CHANNELS    8           // 8 PPM channels intervals
#define     MAX_CHANNELS    5           // System has 5 channels
#define     PPM_INTERVALS   (PPM_CHANNELS+1) //  + 1 filler to 20mSec

// Timing constants for OCxA
// Calculated for 8Mhz clock divided by 64
#define     FRAME_20MSEC    2440        // 20mSec frame
#define     SHORT_PULSE     45          // 370uSec channel pulse
#define     MIN_CHAN        75          // Minimum channel 1.0mSec with short-pulse
#define     MAX_CHAN        195         // Maximum channel 2.0mSec with short-pulse
#define     MID_CHAN        135         // Middle range 1.5mSec with short pulse
#define     CHAN_RANGE      (MAX_CHAN-MIN_CHAN) // ** never 'zero' **

/****************************************************************************
  Function prototypes
****************************************************************************/
void ioinit(void);
void stick_calibration(void);
void retrieve_calibration(void);
void save_calibration(void);

// This function is called upon a HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

/****************************************************************************
  Globals
****************************************************************************/
volatile int      channel = 0;
volatile uint16_t channel_duration[PPM_INTERVALS] =
    {
        MID_CHAN,   // Rudder/Ailerons
        MID_CHAN,   // Elevator
        MIN_CHAN,   // Throttle
        MID_CHAN,   // Rudder
        MID_CHAN,   // Flaps
        60,         // (dummy)
        60,         // (dummy)
        60,         // (dummy)
        1180        // (filler for 20mSec)
    };

/* '0' indicate normal channel movement, and
 * a '1' indicates reversed channel to stick movement.
 * Only 3 channels can be reversed: 1, 2, and 4
 */
uint16_t    channel_reverse[PPM_INTERVALS] =
    {
        0,          // Rudder/Ailerons
        0,          // Elevator
        0,
        0,          // Rudder
        0,
        0,
        0,
        0,
        0,
    };

/* These arrays hold the maximum and minimum ADC conversion
 * values read during calibration. The stick potentiometers
 * movement range can be different between sticks, and is not
 * a 0-255 min-max value. See usage during ADC scan.
 * These values are made persistent between power cycles by
 * saving them in EEPROM.
 */
uint8_t     channel_calib_high[PPM_INTERVALS] =
    {
        255,
        255,
        255,
        255,
        255,
        255,
        255,
        255,
        255,
    };

uint8_t     channel_calib_low[PPM_INTERVALS] =
    {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 */
int main(void)
{
    float       temp;
    uint16_t    duration_sum;
    uint8_t     analog_readout;
    float       analog_range;
    uint8_t     channel_reverse_sw;

    uint8_t     i = 0;
    int         refresh_flag = 1;

    /* Initialize hardware
     */
    ioinit();

    _delay_ms(1000);

    /* Perform stick calibration if DIP switch is ON.
     * This check is only performed once at power-on,
     * therefore calibration will only be entered if the calibration
     * switch is ON before the system is powered up.
     */
    retrieve_calibration();

    if ( PINB & CHAN_CALIB )
    {
        stick_calibration();
        save_calibration();
    }

    /* Enable interrupts
     */
    sei();

    /* Start PPM output with short duration pulse
     */
    PORTB &= ~PPM_OUTPUT_BIT;
    TCNT0  = 0;
    OCR0A  = SHORT_PULSE;
    TIMSK0 = OCR0A_INT_ENA;

    /* Loop forever to sample stick potentiometers,
     * and update PPM pulse duration.
     */
    while ( 1 )
    {
        /* This segment of code scans the ADC inputs and calculates
         * pulse duration for channel 1 through 5.
         * It runs only when the PPM output is idle between
         * pulse bursts. Measured at 1.2mSec
         */
        if ( channel == 8 && refresh_flag )
        {
            // Update channel reverse settings
            channel_reverse_sw = PINA;
            if ( channel_reverse_sw & CHAN1_REV )
                channel_reverse[0] = 1;
            else
                channel_reverse[0] = 0;

            if ( channel_reverse_sw & CHAN2_REV )
                channel_reverse[1] = 1;
            else
                channel_reverse[1] = 0;

            if ( channel_reverse_sw & CHAN4_REV )
                channel_reverse[3] = 1;
            else
                channel_reverse[3] = 0;

            // ADC scan
            for ( i = 0; i < MAX_CHANNELS; i++)
            {
                ADMUX &= ~ADC_CHAN_MASK;
                ADMUX |= i;
                ADCSRA |= ADC_START;
                while ( bit_is_clear(ADCSRA, ADIF) ) {};
                analog_readout = ADCH;

                ADCSRA |= ADC_CONV_COMP;

                // Adjust analog readout to stick range
                if ( channel_calib_low[i] > analog_readout )
                    analog_readout = 0;
                else
                    analog_readout -= channel_calib_low[i];

                analog_range = channel_calib_high[i] - channel_calib_low[i];
                if ( analog_range == 0 )
                    analog_range = 1;

                temp = (float)analog_readout;
                temp = temp * (CHAN_RANGE / analog_range);

                if ( channel_reverse[i] )
                    temp = MAX_CHAN - temp;
                else
                    temp = MIN_CHAN + temp;

                if ( temp > MAX_CHAN )
                    temp = MAX_CHAN;

                if ( temp < MIN_CHAN )
                    temp = MIN_CHAN;

                channel_duration[i] = (uint8_t)temp;
            }

            // Idle-time pulse duration calculation
            duration_sum = 0;
            for ( i = 0; i < PPM_CHANNELS; i++)
            {
                duration_sum += channel_duration[i];
            }
            duration_sum += (9 * SHORT_PULSE);
            channel_duration[8] = FRAME_20MSEC - duration_sum;

            refresh_flag = 0;
        }

        /* Arm the refresh flag and prevent ADC scanning
         * when producing the PPM pulse burst.
         */
        else if ( channel < 8 )
        {
            refresh_flag = 1;
        }

    } /* endless while loop */

    return 0;
}

/* ----------------------------------------------------------------------------
 * ioinit()
 *
 *  Initialize IO interfaces
 *  Timer and data rates calculated based on internal oscillator
 *
 */
void ioinit(void)
{
    // Reconfigure system clock scaler to 8MHz
    CLKPR = 0x80;   // change clock scaler (sec 6.5.2 p.31)
    CLKPR = 0x00;

    // Initialize Timer1 for servo PWM (see sec 11.9.3 Fast PWM Mode)
    // 20mSec pulse interval, with 1mSec to 2mSec variable pulse width
    // Fast PWN mode with Non-inverting Compare Output mode to clear output
    TCNT1  = 0;
    OCR1A  = channel_duration[0];
    TCCR1A = TCCR1A_INIT;
    TCCR1B = TCCR1B_INIT;
    TCCR1C = 0;

    // Initialize Timer0 to produce the 500uSec timing for the PPM
    // short pulse
    TCCR0A = TCCR0A_INIT;
    TCCR0B = TCCR0B_INIT;

    // ADC initialization
    ADMUX  = ADMUX_INIT;
    ADCSRA = ADCSRA_INIT;
    ADCSRB = ADCSRB_INIT;
    DIDR0  = DIDR0_INIT;

    // Initialize IO pins
    DDRA  = PA_DDR_INIT;            // PA pin directions
    PORTA = PA_INIT | PA_PUP_INIT;  // initial value and pull-up setting

    DDRB  = PB_DDR_INIT;            // PB pin directions
    PORTB = PB_INIT | PB_PUP_INIT;  // initial value and pull-up setting
}

/* ----------------------------------------------------------------------------
 * stick_calibration()
 *
 *  Entered if calibration DIP switch is set to 'ON' before power-up.
 *  Calibration mode is exited when the calibration DIP switch is set
 *  to the OFF position.
 *
 */
void stick_calibration(void)
{
    int     i;
    uint8_t analog_readout;

    for ( i = 0; i < MAX_CHANNELS; i++ )
    {
        channel_calib_high[i] = 128;
        channel_calib_low[i] = 128;
    }

    while ( PINB & CHAN_CALIB )
    {
        /* ADC scan and update calibration matrixes
         * until calibration DIP switch is turned OFF
         */
        for ( i = 0; i < MAX_CHANNELS; i++)
        {
            ADMUX &= ~ADC_CHAN_MASK;
            ADMUX |= i;
            ADCSRA |= ADC_START;
            while ( bit_is_clear(ADCSRA, ADIF) ) {};
            analog_readout = ADCH;

            ADCSRA |= ADC_CONV_COMP;

            if ( analog_readout > channel_calib_high[i] )
                channel_calib_high[i] = analog_readout;

            if ( analog_readout < channel_calib_low[i] )
                channel_calib_low[i] = analog_readout;
        }
    }
}

/* ----------------------------------------------------------------------------
 * retrieve_calibration()
 *
 *  Retrieve calibration data from EEPROM
 *
 */
void retrieve_calibration(void)
{
    uint8_t     i;
    uint8_t     start_address;

    EEARH = 0;
    EEARL = 0;

    /* Make sure not writing is in progress
     * should not happen!
     */
    while ( bit_is_set(EECR, EEPE) ) {};

    /* Read calibration data
     */
    start_address = 0;
    for ( i = 0; i < PPM_CHANNELS; i++ )
    {
        EEARL = start_address + i;
        EECR |= 0b00000001; // Set EERE to read EEPROM
        channel_calib_high[i] = EEDR;
    }

    start_address += PPM_CHANNELS;
    for ( i = 0; i < PPM_CHANNELS; i++ )
    {
        EEARL = start_address + i;
        EECR |= 0b00000001; // Set EERE to read EEPROM
        channel_calib_low[i] = EEDR;
    }
}

/* ----------------------------------------------------------------------------
 * save_calibration()
 *
 *  Store calibration data to EEPROM
 *
 *  [data sheet] Using Atomic Byte Programming is the simplest mode. When writing a byte to the EEPROM,
 *  the user must write the address into register EEAR and data into register EEDR. If the EEPMn bits
 *  are zero, writing EEPE (within four cycles after EEMPE is written) will trigger the erase/write
 *  operation. Both the erase and write cycle are done in one operation. The EEPE bit remains set
 *  until the erase and write operations are completed. While the device is busy with programming,
 *  it is not possible to do any other EEPROM operations.
 *
 */
void save_calibration(void)
{
    uint8_t     i;
    uint8_t     start_address;

    EEARH = 0;
    EEARL = 0;

    /* Write calibration data
     */
    start_address = 0;
    for ( i = 0; i < PPM_CHANNELS; i++ )
    {
        EEARL = start_address + i;
        EEDR  = channel_calib_high[i];
        EECR &= 0b11001111; // Atomic write
        EECR |= 0b00000100; // Set EEMPE
        EECR |= 0b00000010; // Set EEPE to start programming
        while ( bit_is_set(EECR, EEPE) ) {};
    }

    start_address += PPM_CHANNELS;
    for ( i = 0; i < PPM_CHANNELS; i++ )
    {
        EEARL = start_address + i;
        EEDR  = channel_calib_low[i];
        EECR &= 0b11001111; // Atomic write
        EECR |= 0b00000100; // Set EEMPE
        EECR |= 0b00000010; // Set EEPE to start programming
        while ( bit_is_set(EECR, EEPE) ) {};
    }
}

/* ----------------------------------------------------------------------------
 * reset()
 *
 *  Clear SREG_I on hardware reset.
 *  source: http://electronics.stackexchange.com/questions/117288/watchdog-timer-issue-avr-atmega324pa
 */
void reset(void)
{
     cli();
    // Note that for newer devices (any AVR that has the option to also
    // generate WDT interrupts), the watchdog timer remains active even
    // after a system reset (except a power-on condition), using the fastest
    // prescaler value (approximately 15 ms). It is therefore required
    // to turn off the watchdog early during program startup.
    MCUSR = 0; // clear reset flags
    wdt_disable();
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when Timer0 compare match
 * at the end of the 500uSec interval.
 *
 */
ISR(TIM0_COMPA_vect)
{
    /* Disable Timer0 interrupt
     */
    TIMSK0 &= ~OCR0A_INT_ENA;

    /* Assert PPM line
     */
    PORTB  |= PPM_OUTPUT_BIT;

    /* Initialize channel interval with
     * Timer1 interrupt
     */
    TIFR1  |= OCR1A_INT_CLR;
    TCNT1   = 0;
    OCR1A   = channel_duration[channel];
    TIMSK1 |= OCR1A_INT_ENA;
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when Timer1 compare match
 * at the end of the variable per-channel interval.
 *
 */
ISR(TIM1_COMPA_vect)
{
    /* Disable Timer1 interrupt
     */
    TIMSK1 &= ~OCR1A_INT_ENA;

    /* Assert PPM scope sync test point
     */
    if ( channel == PPM_CHANNELS )
        PORTB |= PPM_SYNC;

    /* Clear PPM line
     */
    PORTB &= ~PPM_OUTPUT_BIT;

    /* Initialize channel interval with
     * Timer0 interrupt
     */
    TIFR0  |= OCR0A_INT_CLR;
    TCNT0  = 0;
    OCR0A  = SHORT_PULSE;
    TIMSK0 |= OCR0A_INT_ENA;

    /* Advance channel interval pointer
     */
    channel++;
    if ( channel == PPM_INTERVALS )
        channel = 0;

    /* Clear PPM scope sync test point
     */
    if ( channel == PPM_CHANNELS )
        PORTB &= ~PPM_SYNC;
}
