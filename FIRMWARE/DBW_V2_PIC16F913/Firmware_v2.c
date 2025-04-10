#include <xc.h>
#include <stdbool.h>

// Define the oscillator frequency
#ifndef _XTAL_FREQ
    #define _XTAL_FREQ 4000000 // 4 MHz
#endif

// Configuration bits
#pragma config FOSC = HS       // High-speed Oscillator
#pragma config WDTE = OFF      // Watchdog Timer Disabled
#pragma config PWRTE = ON      // Power-up Timer Enabled
#pragma config MCLRE = ON      // Power-up Timer Enabled
#pragma config CPD = OFF       // Data EEPROM Memory Code Protection Disabled
#pragma config CP = ON        // Flash Program Memory Code Protection bits
#pragma config BOREN = ON      // Brown-out Reset Enabled
#pragma config IESO = ON
#pragma config FCMEN = ON 

// Constants
#define VEHICLE_SPEED_LIMIT  100
#define CIRCUMFERENCE 22
#define PPR_RATIO 53
#define MIN_VOLTAGE_LOW 801
#define LIMIT_VOLTAGE_LOW 1100
#define VEHICLE_TYPE 2
#define UPHILL_ADJUST   7
#define DOWNHILL_ADJUST 4

#define ONE_SEC_OVERFLOW_COUNT 1953


// Splitting 16-bit values into high and low bytes
#define MIN_VOLTAGE_LOW_H ((MIN_VOLTAGE_LOW >> 8) & 0xFF)  // High byte
#define MIN_VOLTAGE_LOW_L (MIN_VOLTAGE_LOW & 0xFF)         // Low byte
#define LIMIT_VOLTAGE_LOW_H ((LIMIT_VOLTAGE_LOW >> 8) & 0xFF)  // High byte
#define LIMIT_VOLTAGE_LOW_L (LIMIT_VOLTAGE_LOW & 0xFF)         // Low byte

// EEPROM data memory
__EEPROM_DATA(VEHICLE_SPEED_LIMIT, CIRCUMFERENCE, PPR_RATIO,MIN_VOLTAGE_LOW_H, MIN_VOLTAGE_LOW_L,LIMIT_VOLTAGE_LOW_H, LIMIT_VOLTAGE_LOW_L,VEHICLE_TYPE);
__EEPROM_DATA(UPHILL_ADJUST, DOWNHILL_ADJUST, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);

#define EEPROM_SPEED_LIMIT_OFFSET     0
#define EEPROM_CIRC_OFFSET            1
#define EEPROM_PPR_OFFSET             2
#define EEPROM_MINVOLT_LOW_OFFSET     3
#define EEPROM_MINVOLT_HIGH_OFFSET    4
#define EEPROM_LIMVOLT_LOW_OFFSET     5
#define EEPROM_LIMVOLT_HIGH_OFFSET    6
#define EEPROM_VEHICLE_TYPE_OFFSET    7
#define EEPROM_UPHILL_OFFSET          8
#define EEPROM_DOWNHILL_OFFSET        9

// Other constants
#define DAC_VOLTAGE_STEP 21 //25 mv
#define EEPROM_SIZE_SID 10
#define RELAY_SPEED_OFFSET 5
#define ISUZU_MAXCOUNT 4176 //5.1v
#define UPHILL_OFFSET 3 //3
#define DOWNHILL_OFFSET 3//3
#define RELEASE_PEDAL_OFFSET 5//5

// Pin definitions
#define CS      PORTCbits.RC3 // Chip Select
#define SCK     PORTCbits.RC2 // Serial Clock
#define SDI     PORTCbits.RC1 // Serial Data Input
#define LDAC    PORTCbits.RC0 // Load DAC
#define CS_TRIS TRISCbits.TRISC3
#define SCK_TRIS TRISCbits.TRISC2
#define SDI_TRIS TRISCbits.TRISC1
#define LDAC_TRIS TRISCbits.TRISC0
#define CHANNEL_A  1
#define CHANNEL_B  2

// Function prototypes
void spi_write_16bit_mode_0(unsigned int data);
void dac_init_spi();
void dac_write_channel(unsigned char channel, unsigned int value);
void InitADC(void);
void transmit_pulse_pedal_voltage ();
uint16_t ReadADC(uint8_t channel);
uint16_t VoltageToDACCount(float voltage);
float ADCCountToVoltage(uint16_t adcCount);
void InitUART(void);
void SendByteSerially(unsigned char Byte);
void dac_pedal_control_voltage();
void write_eeprom_params();
void read_eeprom_params();
void transmit_device_params();
bool verify_eeprom(void);
void validateSIDParameters();
bool isInvalidSIDParam(uint8_t param);
bool isInvalidSIDParam16(uint16_t param);
void error_led (int i);
void ok_led(int i);

// Global variables
float circum_1;
float ppr_1;
uint16_t min_voltage_low;//SID PARAM
uint16_t limiting_voltage_low;//SID PARAM
uint16_t limiting_voltage_high;
uint16_t pedal_dac_count_B;
uint16_t pedal_dac_count_A;
uint16_t dac_output_count_B;
uint16_t dac_output_count_A;
int limitvolt_dac_count_B;
int limitvolt_dac_count_A;
int min_dac_count;
int sid_write_index;
int sid_read_index;
volatile int overflow_count=0;
volatile int pulse_count;
int pulse_copy_count;
int speed_vehicle_int=0;
uint8_t speed_limit=0;//SID PARAM
uint8_t circumference=0;//SID PARAM
uint8_t ppr_ratio=0;//SID PARAM
uint8_t vehicle_type=0;//SID PARAM
uint8_t uphill_adjust=0;//SID PARAM
uint8_t downhill_adjust=0;//SID PARAM
uint8_t relay_speed_limit=0;
unsigned char sid_write_val[EEPROM_SIZE_SID];
unsigned char sid_read_val[EEPROM_SIZE_SID];
unsigned char relay_strike_count=0;
volatile unsigned char timer_flag=0;
volatile char rx_char;
bool rcvd_char=false;
bool sid_req=0;
bool dac_control=0;

void main(void) {
    // Wait for oscillator to stabilize
    __delay_ms(10);
    uint16_t adcresult;
    float speed_vehicle;
    float voltage_ADC;
    unsigned char i;



  

    // Initialize ADC
    InitADC();

    // Initialize DAC
    dac_init_spi();  // Mode 0

    // Initialize UART
    InitUART();

    while (1) {
        // Send device details request from SID
        if (sid_req == 1) {
            sid_req = 0;
            ok_led (2);
            transmit_device_params();  // * <data> #
        }

        // Set device config from SID
        if (rcvd_char == true) {
            rcvd_char = false;

            write_eeprom_params();
            read_eeprom_params();

            if (verify_eeprom() == false) {
                while (1) {
                    //Flash Error
                    error_led(10);
                }
            }
            else
            {
                ok_led(5);
            }
            // Send back to Device For confirmation
            transmit_device_params();

           
        }
       if (timer_flag) 
            {
            // One second elapsed if this flag is true
            timer_flag = 0;
            INTCONbits.GIE = 0;
            pulse_copy_count = pulse_count;
            pulse_count = 0;
            INTCONbits.GIE = 1;

           speed_vehicle = ((pulse_copy_count * circum_1) / ppr_1) * 3.6;
           speed_vehicle_int = speed_vehicle;

            if (speed_vehicle > relay_speed_limit) {
                if (relay_strike_count == 0) {
                    PORTBbits.RB1 = 1;
                } else {
                    PORTBbits.RB1 = 0;
                }
                relay_strike_count++;
                if (relay_strike_count > 1) {
                    relay_strike_count = 0;
                }
            } else {
                PORTBbits.RB1 = 0;
            }

            transmit_pulse_pedal_voltage();
        }  // timer flag

        adcresult = ReadADC(0);
        voltage_ADC = ADCCountToVoltage(adcresult);
        pedal_dac_count_B = VoltageToDACCount(voltage_ADC);
        __delay_ms(1);
 
        adcresult = ReadADC(1);
        voltage_ADC = ADCCountToVoltage(adcresult);
        pedal_dac_count_A = VoltageToDACCount(voltage_ADC);

        if (speed_vehicle < speed_limit) {
            PORTBbits.RB2 = 0;  // Buzzer OFF
            PORTBbits.RB3 = 0;  // RED LED OFF

            if (pulse_copy_count > 2) {
                PORTBbits.RB4 = !PORTBbits.RB4;//GREEN LED TOGGLE
            } else {
                PORTBbits.RB4 = 1;  // GREEN LED ON
            }

            if (dac_control == 0) {  // normal
                dac_output_count_B = pedal_dac_count_B;
                dac_output_count_A = pedal_dac_count_A;
            } else {
                // Precaution for RPM increase
                if (pedal_dac_count_B < limitvolt_dac_count_B) {
                    dac_output_count_B = pedal_dac_count_B;  // release to pedal
                    dac_output_count_A = pedal_dac_count_A;
                    dac_control = 0;
                } else {
                    if (speed_vehicle < (speed_limit - UPHILL_OFFSET)) {
                        // For handling Up hill
                        dac_output_count_B = limitvolt_dac_count_B + (DAC_VOLTAGE_STEP * uphill_adjust);
                        dac_output_count_A = limitvolt_dac_count_A + (DAC_VOLTAGE_STEP * uphill_adjust);
                    } else {
                        dac_output_count_B = limitvolt_dac_count_B;
                        dac_output_count_A = limitvolt_dac_count_A;
                    }
                }

                if (speed_vehicle < (speed_limit - RELEASE_PEDAL_OFFSET)) {
                    // Just for Precaution
                    dac_output_count_B = pedal_dac_count_B;  // release to pedal
                    dac_output_count_A = pedal_dac_count_A;
                    dac_control = 0;
                }
            }
        } else {
            PORTBbits.RB4 = 0;  // GREEN LED OFF
            PORTBbits.RB3 = 1;  // BUZZER ON
            PORTBbits.RB2 = 1;  // RED LED ON

            // For downhill control
            if (speed_vehicle > (speed_limit + DOWNHILL_OFFSET)) {
                if (pedal_dac_count_B < limitvolt_dac_count_B) {
                    // Release control to Pedal
                    dac_output_count_B = pedal_dac_count_B;
                    dac_output_count_A = pedal_dac_count_A;
                } else {
                    // Down Hill
                    dac_output_count_B = limitvolt_dac_count_B - (DAC_VOLTAGE_STEP * downhill_adjust);
                    dac_output_count_A = limitvolt_dac_count_A - (DAC_VOLTAGE_STEP * downhill_adjust);
                }
            } else {
                // Normal terrain
                if (pedal_dac_count_B < limitvolt_dac_count_B) {
                    dac_output_count_B = pedal_dac_count_B; // release to pedal
                    dac_output_count_A = pedal_dac_count_A;
                } else {
                    dac_output_count_B = limitvolt_dac_count_B;
                    dac_output_count_A = limitvolt_dac_count_A;
                }
            }

            dac_control = 1;
        }

        // Determine DAC voltage and pedal voltage to be sent
        dac_pedal_control_voltage();

        if(speed_limit == 72) {
            //Pedal Voltage
            dac_write_channel(CHANNEL_B, pedal_dac_count_B);
            dac_write_channel(CHANNEL_A, pedal_dac_count_A);
        } else {
            //can be pedal or DAC voltage
            dac_write_channel(CHANNEL_B, dac_output_count_B);
            dac_write_channel(CHANNEL_A, dac_output_count_A);
        }
    }  // while loop end
}
void dac_pedal_control_voltage()
{
    switch (vehicle_type) 
    {
        case 1:
            dac_output_count_A = dac_output_count_B * 2;
            pedal_dac_count_A = pedal_dac_count_B * 2;
            break;

        case 2:
            dac_output_count_A = min_dac_count + dac_output_count_B;
            pedal_dac_count_A = min_dac_count + pedal_dac_count_B;
            break;

        case 3:
            dac_output_count_A = ISUZU_MAXCOUNT - dac_output_count_B;
            pedal_dac_count_A =  ISUZU_MAXCOUNT - pedal_dac_count_B;
            break;

        default:
            // Handle invalid vehicle_type (e.g., reset values to default or trigger an error)
            break;
    }
}

void __interrupt() ISR(void) {
    if (INTCONbits.INTF) { // External interrupt on RB0
        INTCONbits.INTF = 0; // Clear the interrupt flag for next interrupt
        pulse_count++;
    }  
    if (INTCONbits.TMR0IF) { // Timer0 overflow interrupt flag
        INTCONbits.TMR0IF = 0; // Clear the timer overflow flag
        overflow_count++;
        
        if (overflow_count >= ONE_SEC_OVERFLOW_COUNT) { // 4
            overflow_count = 0;
            timer_flag = 1; // 1 second has elapsed
        }
    }
 
    
    if (RCIF) {  // If UART Rx Interrupt
        if (OERR) {  // If overrun error, reset the receiver
            CREN = 0;
            CREN = 1;
        }

        rx_char = RCREG;

        // SID requesting DBW data or starting/ending characters
        if (rx_char == '@') {
            sid_req = 1;
        } 
        else if (rx_char == '*' || rx_char == '#') { // handle start and end characters
            if (rx_char == '*') {
                sid_write_index = 0;  // Prepare for next character
            } else {  // rx_char == '#'
                rcvd_char = true;
                sid_write_index = 0;
            }
        } 
        else {  // Handle data characters
            sid_write_val[sid_write_index++] = rx_char;
        }

        RCIF = 0; // Clear the UART receive interrupt flag 
    }
}


void InitUART(void)
{
    //UART configurations
    SPBRG=25;
    RCSTA = 0x00;
    TXSTA = 0x00;
    CSRC  = 0;
	BRGH  = 1;                   	// Fast baudrate
	SYNC  = 0;						// Asynchronous
	SPEN  = 1;						// Enable serial port pins
	CREN  = 1;						// Enable reception
	SREN  = 0;						// No effect
	TXIE  = 0;						// Disable tx interrupts
	RCIE  = 1;						// Enable rx interrupts
	TX9   = 0;						// 8-bit transmission
	RX9   = 0;						// 8-bit reception
	TXEN  = 0;						// Reset transmitter
	TXEN  = 1;						// Enable the transmitter
}


void InitADC(void) {
    TRISA0 = 1; // Set RA2 as input (ADC channel 0)
    TRISA1 = 1; // Set RA3 as input (ADC channel 1)
    
    ADCON0bits.ADFM = 1; // Right Justified
    ADCON0bits.ADON = 1; // ADC Enable bit
}

// Function to read the ADC value from the specified channel
uint16_t ReadADC(uint8_t channel) {
        
    // Select the ADC channel
    ADCON0bits.CHS = channel; // Set the ADC channel
    __delay_us(10); // Wait for the channel to settle

    // Start the conversion
    ADCON0bits.GO_nDONE = 1;

    // Wait for the conversion to complete
    while (ADCON0bits.GO_nDONE) {
        // Optionally, add a timeout here to prevent infinite loops
    }

    // Read the result from the ADC result registers
    uint16_t adc_result = (ADRESH << 8) | ADRESL; // Combine the high and low bytes

    return adc_result;
}

void dac_init_spi() {
    CS_TRIS = 0;  // Set RC3 (CS) as output
    SCK_TRIS = 0; // Set RC2 (SCK) as output
    SDI_TRIS = 0; // Set RC1 (SDI) as output
    LDAC_TRIS = 0; // Set RC0 (LDAC) as output
    CS = 1;
    LDAC = 1;
    SCK = 0;
}   

void spi_write_16bit_mode_0(unsigned int data) {
    unsigned char i;
    CS = 0;

    for (i = 0; i < 16; i++) { // MCP4922 requires 16 bits
        if (data & 0x8000) {
            SDI = 1;
        } else {
            SDI = 0;
        }

        SCK = 1; // Clock HIGH // Data Sampling phase
        SCK = 0; // Data shifting phase Clock LOW
        
        data <<= 1; // Shift data left
    }
  
    CS = 1;
    LDAC = 0;
    LDAC = 1;
}


void dac_write_channel(unsigned char channel, unsigned int value) {
    unsigned int control_bits;
    
    if (channel == CHANNEL_A) { // Channel A
        control_bits = 0x3000; // Control bits for channel A
    } else 
    { // Channel B
        control_bits = 0xB000; // Control bits for channel B
    } 
    // Combine control bits with data, ensuring only the lower 12 bits of value are used
    unsigned int data_to_send = control_bits | (value & 0x0FFF);
    
        spi_write_16bit_mode_0(data_to_send);
    
}
 void transmit_pulse_pedal_voltage()
{
    
        SendByteSerially('%');
        __delay_ms(1);
        SendByteSerially((pedal_dac_count_B&0xFF00)>>8);//PEDAL LOW
        __delay_ms(1);
        SendByteSerially(pedal_dac_count_B&0xFF);
        __delay_ms(1);

        SendByteSerially((pedal_dac_count_A&0xFF00)>>8);//PEDAL HIGH
        __delay_ms(1);
        SendByteSerially(pedal_dac_count_A&0xFF);
        __delay_ms(1);
        
         SendByteSerially( (pulse_copy_count&0xFF00)>>8);//PULSE
        __delay_ms(1);
        SendByteSerially(pulse_copy_count&0xFF);
        __delay_ms(1);

        SendByteSerially( (speed_vehicle_int&0xFF00)>>8);//PULSE
        __delay_ms(1);
        SendByteSerially(speed_vehicle_int&0xFF);
        __delay_ms(1);

        SendByteSerially('$');
    
}        

void SendByteSerially(unsigned char Byte)  // Writes a character to the serial port
{
	while(!TRMT);
    TXREG = Byte;

}

bool verify_eeprom(void) {
    uint8_t *w = sid_write_val, *r = sid_read_val;
    uint8_t i = EEPROM_SIZE_SID;  
    while (i--) {
        if (*w++ != *r++) return false;
    }
    return true;
}

void read_eeprom_params()
{
    unsigned char i;
    for (i = 0; i < EEPROM_SIZE_SID; i++) 
    {
        sid_read_val[i] = eeprom_read(i);
        __delay_ms(100);
    }
   
   speed_limit=sid_read_val[EEPROM_SPEED_LIMIT_OFFSET];
   vehicle_type =sid_read_val[EEPROM_VEHICLE_TYPE_OFFSET];
   uphill_adjust=sid_read_val[EEPROM_UPHILL_OFFSET];
   downhill_adjust=sid_read_val[EEPROM_DOWNHILL_OFFSET];
   
   circumference=sid_read_val[EEPROM_CIRC_OFFSET] ;
   ppr_ratio =sid_read_val[EEPROM_PPR_OFFSET] ;
   min_voltage_low = (sid_read_val[EEPROM_MINVOLT_LOW_OFFSET] << 8) | sid_read_val[EEPROM_MINVOLT_HIGH_OFFSET];
   limiting_voltage_low = (sid_read_val[EEPROM_LIMVOLT_LOW_OFFSET] << 8) | sid_read_val[EEPROM_LIMVOLT_HIGH_OFFSET];
   
   //Re calculate   
   circum_1 = (float)circumference / 10.0;
   ppr_1 = (float)ppr_ratio / 10.0; 
   relay_speed_limit=speed_limit+RELAY_SPEED_OFFSET;
  
   
   if (vehicle_type == 1) 
    {
        limiting_voltage_high = limiting_voltage_low * 2;//Type-1 (Urvan, Sentra ,H1,Victory)
    }
   else if (vehicle_type == 2) 
    {
        limiting_voltage_high = min_voltage_low + limiting_voltage_low;//Type-2 (Hiace)
    }
   else //vehicle_type =3
    {
        limiting_voltage_high = ISUZU_MAXCOUNT - limiting_voltage_low;//Type-3 (Isuzu)
   }
   
    //DAC counts
    min_dac_count=VoltageToDACCount((float)min_voltage_low / 1000);
    limitvolt_dac_count_B = VoltageToDACCount((float)limiting_voltage_low / 1000);
    limitvolt_dac_count_A = VoltageToDACCount((float)limiting_voltage_high / 1000);
    
}


void write_eeprom_params() {
    uint8_t i;
    // Write sid_write_val to EEPROM
    for (i = 0; i < EEPROM_SIZE_SID; i++) {
        eeprom_write(i, sid_write_val[i]);
        __delay_ms(100);
    }
}



void transmit_device_params()
{
    SendByteSerially('*');
    __delay_ms(1);

    for (uint8_t i = 0; i < EEPROM_SIZE_SID; i++)  // Use a local loop variable instead of global `sid_read_index`
    {
        SendByteSerially(sid_read_val[i]);
       __delay_ms(1);
    }

    SendByteSerially('#');
}


uint16_t VoltageToDACCount(float voltage) {
    return (uint16_t)((voltage / 5.0) * 4095); // 4095 = 2^12 - 1
}

float ADCCountToVoltage(uint16_t adcCount) {
    // Calculate the voltage
    float voltage = ((float)adcCount / 1023.0) * 5.0;
    
    return voltage;
}

bool isInvalidSIDParam(uint8_t param) {
    return (param == 0 || param == 0xFF);
}

bool isInvalidSIDParam16(uint16_t param) {
    return (param == 0 || param == 0xFFFF);
}

/**
 * @brief Validates all SID parameters and handles errors.
 */
void validateSIDParameters() {
    if (isInvalidSIDParam(speed_limit) || 
        isInvalidSIDParam(circumference) || 
        isInvalidSIDParam(ppr_ratio) || 
        isInvalidSIDParam(vehicle_type) || 
        isInvalidSIDParam(uphill_adjust) || 
        isInvalidSIDParam(downhill_adjust) ||
        isInvalidSIDParam16(min_voltage_low) || 
        isInvalidSIDParam16(limiting_voltage_low)) 
    {  
        error_led(10);
       
    } 
    else 
    {
        // No action needed for now
           // Indicate green LED to let know it is passed
            PORTBbits.RB4 = 1;
            __delay_ms(1000);
            PORTBbits.RB4 = 0;
            __delay_ms(1000);
    }
}
void error_led (int i)
{
       PORTBbits.RB4 = 0;
       //int i=0;
        while (i>0) {
            // GLOW RED LED TO INDICATE ERROR
            PORTBbits.RB3 = 1;
            __delay_ms(100);
            PORTBbits.RB3 = 0;
            __delay_ms(100);
            i--;
        }
}  

void ok_led (int i)
{
     PORTBbits.RB4 = 0;
     while (i>0) {
            // GLOW RED LED TO INDICATE ERROR
            PORTBbits.RB3 = 1;
            __delay_ms(100);
            PORTBbits.RB3 = 0;
            __delay_ms(100);
            i--;
        }
     //PORTBbits.RB4 = 0;
    
}