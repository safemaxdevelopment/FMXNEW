#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define _XTAL_FREQ 4000000 // 4 MHz
#define DEBOUNCE_DELAY 50  // 50 ms debounce delay
#define DEBOUNCE_DELAY_SELECT_KEY 20
#define SCREEN_START_DELAY 300  // 50 ms debounce delay

#define SPEED_MIN_VALUE 1
#define SPEED_MAX_VALUE 600

#define PPR_MIN_VALUE   1
#define PPR_MAX_VALUE   20

#define PULSE_START_DISP_VALUE 150
#define PULSE_MIN_VALUE   1
#define PULSE_MAX_VALUE   5000


#define SPEED_START_DISP_VALUE 5
#define PPR_START_DISP_VALUE 5

#define BAUDRATE 9600  //bps

//__CONFIG (FOSC_INTOSCIO & WDTE_OFF & PWRTE_ON & MCLRE_ON & BOREN_ON & LVP_OFF & CPD_OFF & CP_OFF);
#pragma config FOSC = INTOSCIO  // Internal oscillator, I/O on RA6/OSC2/CLKOUT pin
#pragma config WDTE = OFF       // Watchdog Timer disabled
#pragma config PWRTE = ON      // Power-up Timer disabled
#pragma config MCLRE = ON     // MCLR pin function is digital input
#pragma config BOREN = ON      // Brown-out Reset disabled
#pragma config LVP = OFF        // Low-Voltage Programming disabled
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection disabled
#pragma config CP = ON         // Flash Program Memory Code Protection disabled

volatile bool buttonStateChanged = false;
volatile uint8_t buttonState = 0; // Bitwise state for buttons (0: released, 1: pressed)
volatile uint8_t lastButtonState = 0; // Previous button state for debouncing

volatile bool screen1_done = false;
volatile bool screen2_done = false;
volatile bool screen3_done = false;
volatile bool screen4_done = false;
volatile bool screen5_done = false;
volatile bool screen6_done = false;
volatile bool screen7_done = false;

bool speed_seq = false;
bool pulse_seq = false;
bool speed_seq_blank = false;
bool pulse_seq_blank = false;
bool speed_pulse_value = false;

volatile bool direct1_speed = false;
volatile bool direct1_pulse = false;


volatile uint16_t pulse_value = PULSE_START_DISP_VALUE;
volatile uint16_t receivedValue = 0;

volatile uint16_t speed_value = SPEED_START_DISP_VALUE; // Initialize speed_value
volatile uint16_t ppr_value = PPR_START_DISP_VALUE; // Initialize ppr_value

unsigned char Dev_stSel = 0;
unsigned char Dev_stReqSent = 0;
unsigned char Rx_complete;
unsigned char Dev_stRespTOut = 0;
unsigned char Dev_stInit = 1;
unsigned int Rx_elapsed_ticks;

volatile unsigned char highByte = 0;
volatile unsigned char lowByte = 0;
volatile unsigned char receivedByteCount = 0;
volatile char rx_char=0;
static bool first_recv=0;







#define RS RA4
#define EN  RA7
#define D4 RA0
#define D5 RA1
#define D6 RA2
#define D7 RA3

#define LCD_EN           RB6                                 
#define LCD_RS           RB7                                 
#define LCD_DATA4        RA0                                    
#define LCD_DATA5        RA1                                    
#define LCD_DATA6        RA2                                    
#define LCD_DATA7        RA3

#define SELECT           RB4
#define UP               RB5
#define DOWN             RB3

#define LCD_STROBE   	((LCD_EN = 1),(LCD_EN = 0))

void InitHardware(void);
void Lcd_SAFEMAX();
void Lcd_SIDID();
void IntToStr(uint16_t num, char *buffer);

void lcd_init(void);
void lcd_puts(const char *);
void lcd_putInt(int);
void lcd_clear(void);
void lcd_init(void);
void lcd_write(unsigned char);
void lcd_write_ascii(unsigned char);
void lcd_goto(unsigned char line, unsigned char pos);
void clearLine(char line);
void SendSpeedValue_word(uint16_t value);


void Lcd_Screen_Speed();
void Lcd_Screen_Pulse();
void Lcd_Screen_Save();
void Lcd_Screen_Sent();

//Function Prototypes
void disp_init(void);

void InitUART(void);
void SendByteSerially(unsigned char);
unsigned char ReceiveByteSerially(void);
void SendStringSerially(const unsigned char*);


volatile bool rcvd_char=false;


uint16_t DBW_Device_Value[8]  = {0, 100, 20, 95, 370, 500, 1,0};//start value for pulse
unsigned int min_val[8]   = {0,  50, 15, 40, 300, 300, 1,0};//min val 1 for pulse
unsigned int max_val[8]   = {0, 1000,40, 100,5000,5000,5,0};//max val 5000 for pulse
unsigned const char * units[7]= {"","KM/H","MTR","PS","mV","mV","TY",""};

unsigned char sid_val[12];
int sid_index=0;


void itoa(int num, char *str, int width) {
    int i = width - 1;
    str[width] = '\0';  // Null-terminate the string

    // Handle the case of 0 separately
    if (num == 0) {
        str[i--] = '0';
    }

    // Process the digits of the number
    while (num > 0 && i >= 0) {
        str[i--] = (num % 10) + '0';  // Get the next digit and convert to char
        num /= 10;  // Remove the last digit
    }

    // Fill remaining positions with '0' if necessary
    while (i >= 0) {
        str[i--] = '0';
    }
}


// Function to convert DAC count back to voltage in millivolts
int DACCountToVoltageMillivolts(uint16_t dacCount) {
    // Convert to voltage in millivolts (1 volt = 1000 millivolts)
    return (int)(((float)dacCount / 4095.0) * 5000); // 5000 = 5.0V * 1000
}





void SendSpeedValue_word(uint16_t value) 
{
    unsigned char highByte = (value >> 8) & 0xFF; // Get the higher byte
    unsigned char lowByte = value & 0xFF;         // Get the lower byte
    
    SendByteSerially(highByte);  // Send the higher byte first
    __delay_ms(10);
    SendByteSerially(lowByte);   // Send the lower byte
    __delay_ms(10);
}

void SendSpeedValue_byte(unsigned char value) 
{
    SendByteSerially(value);  // Send the higher byte first
}

void Lcd_SAFEMAX() {
    lcd_clear();
    lcd_goto(1,0);
    lcd_puts("SAFEMAX");
    lcd_goto(2,1);
    lcd_puts("AKHI");
 }

void Lcd_SIDID() {
    lcd_clear();
    lcd_goto(1,2);
    lcd_puts("DBW");
    lcd_goto(2,3);
    lcd_puts("SID");
}

void screen1() 
{
   char formattedValue[5]; 
   
   itoa(DBW_Device_Value[3], formattedValue, 4);
   lcd_goto(1,0);
   lcd_puts(formattedValue);

   lcd_goto(1,5);
   lcd_puts("PU");
   
   itoa(DBW_Device_Value[6], formattedValue, 4);
   lcd_goto(2,0);
   lcd_puts(formattedValue);

   lcd_goto(2,5);
   lcd_puts("SP");
    //lcd_clear();
#if 0    
   lcd_goto(1,1);
   lcd_puts("PULSE");
   
   char formattedValue[5];  // Array to hold the formatted value (4 digits + null terminator)
   
   itoa(DBW_Device_Value[3], formattedValue, 4);

   lcd_goto(2,0);
   lcd_puts(formattedValue);

   lcd_goto(2,5);
   lcd_puts("PS");
#endif   

}

void screen2() 
{
   char formattedValue[5]; 
   
   itoa(DBW_Device_Value[1], formattedValue, 4);
   lcd_goto(1,0);
   lcd_puts(formattedValue);

   lcd_goto(1,5);
   lcd_puts("VL");
   
   itoa(DBW_Device_Value[2], formattedValue, 4);
   lcd_goto(2,0);
   lcd_puts(formattedValue);

   lcd_goto(2,5);
   lcd_puts("VH");

    
}


void InitHardware(void) {  
#if 1  
    //Oscillator configuration
    PCON    = 0b00001000; //     OSCF(4Mhz) PORBOR ---- 1-0x 27
    //Push Button Settings 
    TRISBbits.TRISB3 = 1; // Set RB3 as input (DOWN button)
    TRISBbits.TRISB4 = 1; // Set RB4 as input (SELECT button)
    TRISBbits.TRISB5 = 1; // Set RB5 as input (UP button)
    
    //LCD Settings 
    TRISAbits.TRISA0 = 0; //Set RA0 as output (LCD_DATA4) 
    TRISAbits.TRISA1 = 0; //Set RA1 as output (LCD_DATA5) 
    TRISAbits.TRISA2 = 0; //Set RA2 as output (LCD_DATA6) 
    TRISAbits.TRISA3 = 0; //Set RA3 as output (LCD_DATA7) 
    TRISBbits.TRISB6 = 0; //Set RB6 as output (LCD_EN)
    TRISBbits.TRISB7 = 0; //Set RB7 as output  (LCD_RS)

    RBIF = 0; // Clear PORTB change interrupt flag

    // Enable interrupts
   GIE = 1; // Enable global interrupts
   PEIE = 1; // Enable peripheral interrupts
#endif    
   
}

#if 1 //SID settings
void lcd_write(unsigned char c)
{
   __delay_us(40);   
   PORTA = c >> 4;
   LCD_STROBE;
   PORTA = c;
   LCD_STROBE;
}

void lcd_write_ascii(unsigned char s)
{
   LCD_RS = 1;   // write characters
   lcd_write(s);
   __delay_ms(5);
}

/* Clear and home the LCD */
void lcd_clear()
{
   LCD_RS = 0;
   lcd_write(0x1);
   __delay_ms(2);
}

/* write a string of chars to the LCD */
void lcd_puts(const char *s)
{
   LCD_RS = 1;   // write characters
   while(*s)
   lcd_write(*s++);
   __delay_ms(5);
}

void clearLine(char line)
{
    lcd_goto(line,0);
    lcd_puts("        ");
}

void clearPosition(char line, char position)
{
    lcd_goto(line, position);  // Go to the specific line and position
    lcd_putc(' ');             // Clear the character at this position with a space
}

void lcd_putInt(int num)
{
    LCD_RS = 1;
    unsigned char t1,i,wrote;
    unsigned int k;
    
    wrote=0;
    for (i=4;i>=1;i--)
        {
        switch(i){
        case 4: k=10000;
            break;
        case 3: k=1000;
            break;
        case 2: k=100;
            break;
        case 1: k=10;
        }
        t1=num/k;
        if((wrote)||(t1!=0))
            {
            lcd_write(t1+'0');
            wrote=1;
            }
        num-=(t1*k);
        }
        
    lcd_write(num+'0');
}

/* Go to the specified position */
void lcd_goto(unsigned char line, unsigned char pos)
{
   LCD_RS = 0;
   if(line == 1)
    {
        lcd_write(0x80+pos);
    }
    else
    {
        lcd_write(0xC0+pos);        
    }
}

void lcd_init(void)
{
   LCD_RS = 0;   // write control bytes
   __delay_us(15);   // power on delay
   PORTA = 0x3;   // Return home
   LCD_STROBE;
   __delay_ms(5);
   LCD_STROBE;
   __delay_ms(100);
   LCD_STROBE;
   __delay_ms(5);
   PORTA = 0x2;   // set 4 bit mode
   LCD_STROBE;
   __delay_ms(40);
   lcd_write(0x28);   // 4 bit mode, 1/16 duty, 5x8 font
   __delay_ms(40);
   lcd_write(0x08);   // display off
   __delay_ms(40);
   lcd_write(0x0C);   // display on
   __delay_ms(40);
   lcd_write(0x06);   // entry mode
   __delay_ms(40);
}

void disp_init()
{
    lcd_init();
    lcd_clear();
}

#endif

void IntToStr(uint16_t num, char *buffer) {
    char temp[6]; // Temporary buffer to hold digits
    int i = 0, j = 0;
    
    if (num == 0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }
    
    // Extract digits in reverse order
    while (num > 0) {
        temp[i++] = (num % 10) + '0'; // Convert digit to character
        num /= 10;
    }
    
    // Reverse the digits to get the correct order
    while (i > 0) {
        buffer[j++] = temp[--i];
    }
    
    buffer[j] = '\0'; // Null-terminate the string
}



void main() {
    InitHardware(); // Initialize buttons
    
    disp_init();
    
    	//Timer 2 configurations
	PR2 = 25; //Timer2 Period Register 1111 1111 107,227
	T2CON = 0b00000000; // TOUTPS3 TOUTPS2 TOUTPS1 TOUTPS0 TMR2ON T2CKPS1 T2CKPS0 -000 0000 108,226
	TOUTPS3 = 1;
	TOUTPS2 = 0;
	TOUTPS1 = 0;
	TOUTPS0 = 1;
	T2CKPS1 = 0;
	T2CKPS0 = 1;
    
    TMR2IE = 1;
    
    PEIE = 1;
	GIE = 1;

    InitUART();	
    // Initial screen display
    Lcd_SAFEMAX();
    __delay_ms(SCREEN_START_DELAY);
    Lcd_SIDID();
    // __delay_ms(100);
    //lcd_clear();
    
    Dev_stInit=0;
    
    DBW_Device_Value[1]=2222;
DBW_Device_Value[2]=3333;
DBW_Device_Value[3]=1111;


    //DBW_Device_Value[1]=2020;
    //DBW_Device_Value[2]=4040;
    //DBW_Device_Value[3]=135;
            

    while (1) {
        
        if(rcvd_char==1)
        {
                        rcvd_char=0;
                        sid_index=0;
                        
                        //Pedal voltage
                        DBW_Device_Value[4]=(sid_val[0] << 8) | sid_val[1];//VL
                        DBW_Device_Value[5]=(sid_val[2] << 8) | sid_val[3];//VH
                        DBW_Device_Value[3]=(sid_val[4] << 8) | sid_val[5];//PULSE
                        DBW_Device_Value[6]=(sid_val[6] << 8) | sid_val[7];//SPEED
                        
                        DBW_Device_Value[1]=DACCountToVoltageMillivolts(DBW_Device_Value[4]);
                        DBW_Device_Value[2]=DACCountToVoltageMillivolts(DBW_Device_Value[5]);
                        
                switch(Dev_stSel)
				{
					case 1://Low and High
					{
                        screen1();
						break;
					}
                    
                    case 2://Low and Pulse
					{
						screen2();
						break;
					}
    
					default:
                        if(first_recv==0)
                        {
                            lcd_clear();
                            first_recv=1;
                        }
                        //lcd_clear();
                        screen1();
						break;
					break;
				}


         }                
                        
    
        buttonState = 0;
        
         // SELECT KEY
   // if (SELECT == 0) {
        
       // buttonStateChanged = true;
        //buttonState |= 0x02;
     //}
        
      if (UP == 1 && DOWN == 0) {
        bool stable = true;
        for (int i = 0; i < 3; i++) {  // Check for 3 cycles
            __delay_ms(DEBOUNCE_DELAY / 2); // Shorter delay in between checks
            if (UP != 1 || DOWN != 0) {
                stable = false;
                break;
            }
        }
        if (stable) {
            buttonStateChanged = true;
            buttonState |= 0x01;
        }
    }   
        
     else if (UP == 0 && DOWN == 1) {
        bool stable = true;
        for (int i = 0; i < 3; i++) {  // Check for 3 cycles
            __delay_ms(DEBOUNCE_DELAY / 2); // Shorter delay in between checks
            if (UP != 0 || DOWN != 1) {
                stable = false;
                break;
            }
        }
        if (stable) {
            buttonStateChanged = true;
            buttonState |= 0x04;
        }
    }   
    // SELECT KEY
    if (buttonStateChanged) {
        buttonStateChanged = false; // Reset state change flag
        if (buttonState & 0x01)//Down key 
        { 
            Dev_stSel=1;
            lcd_clear();
             screen1();

        }// buttonState |= 0x02;
        
         else if (buttonState & 0x04) 
        {
               Dev_stSel=2;
               lcd_clear();
               screen2();
           // Up_Key(Dev_stSel);
            __delay_ms(DEBOUNCE_DELAY); 
        }
        //buttonStateChanged = true;
        //buttonState |= 0x02;
     }//buttonStateChanged
#if 0   
    if (buttonStateChanged) {
        buttonStateChanged = false; // Reset state change flag
        if (buttonState & 0x02)//Select key 
        { 
                Dev_stSel++;
                 
            	if(Dev_stSel >= 3)
				{
					Dev_stSel = 1;
				}

				lcd_clear();					
				// Detect the presence of direction button and change the vehicle speed based on the up and down button
				switch(Dev_stSel)
				{
					case 1://Low and High
					{
                        screen1();
						break;
					}
                    
                    case 2://Low and Pulse
					{
						screen2();
						break;
					}
                  
					default:
					break;
				}
				
           __delay_ms(DEBOUNCE_DELAY*10); 
        } 

}//BUTTON STATE CHANGED
#endif
    
    }//while loop

}

void InitUART(void)
{
	TRISB2 = 1;   					// TX Pin
	TRISB1 = 1;   					// RX Pin
	
	SPBRG = ((_XTAL_FREQ/16)/BAUDRATE) - 1;
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


void SendByteSerially(unsigned char Byte)  // Writes a character to the serial port
{
	while(!TRMT);
    TXREG = Byte;

}

unsigned char ReceiveByteSerially(void)   // Reads a character from the serial port
{
	if(OERR) // If over run error, then reset the receiver
	{
		CREN = 0;
		CREN = 1;
	}
	
	while(!RCIF);  // Wait for transmission to receive
	
	return RCREG;
}

void SendStringSerially(const unsigned char* st)
{
	while(*st)
		SendByteSerially(*st++);
}
