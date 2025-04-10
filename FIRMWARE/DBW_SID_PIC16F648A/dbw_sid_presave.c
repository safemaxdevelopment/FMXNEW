#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 4000000 // 4 MHz
#define DEBOUNCE_DELAY 50  // 50 ms debounce delay
#define DEBOUNCE_DELAY_SELECT_KEY 20
#define SCREEN_START_DELAY 300  // 50 ms debounce delay
//#define DEBUG
#define TOTAL_BYTES 10
#define NO_OF_MODELS 10
#define NO_OF_MENUS 8
#define MENU_START_INDEX 2
#define MENU_END_INDEX (NO_OF_MENUS+1)

#define SPEED_START_DISP_VALUE 5
#define PPR_START_DISP_VALUE 5

#define BAUDRATE 9600  //bps

#pragma config FOSC = INTOSCIO  // Internal oscillator, I/O on RA6/OSC2/CLKOUT pin
#pragma config WDTE = OFF       // Watchdog Timer disabled
#pragma config PWRTE = ON      // Power-up Timer disabled
#pragma config MCLRE = ON     // MCLR pin function is digital input
#pragma config BOREN = ON      // Brown-out Reset disabled
#pragma config LVP = OFF        // Low-Voltage Programming disabled
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection disabled
#pragma config CP = ON         // Flash Program Memory Code Protection disabled

unsigned char Dev_stSel = 0;
unsigned char Dev_stUpKey = 0;
unsigned char Dev_stReqSent = 0;
unsigned char Rx_complete;
unsigned char Dev_stRespTOut = 0;
unsigned char Dev_stInit = 1;
volatile char rx_char=0;
volatile unsigned char highByte = 0;
volatile unsigned char lowByte = 0;
unsigned char sid_val[12];
unsigned int Rx_elapsed_ticks;
int sid_index=0;
volatile bool buttonStateChanged = false;
volatile uint8_t buttonState = 0; // Bitwise state for buttons (0: released, 1: pressed)
volatile uint8_t lastButtonState = 0; // Previous button state for debouncing
bool custom_transmit=0;
bool presave_key=0;
bool button_enter=0;
bool speed_seq = false;
bool pulse_seq = false;
bool speed_pulse_value = false;
volatile bool rcvd_char=false;

uint16_t DBW_Device_Value[NO_OF_MENUS+2]  = {0, 100, 20, 95, 370, 500, 1,7,4,0};//start value for pulse
const unsigned int min_val[NO_OF_MENUS+2]   =     {0,1,1,1,1,1,1,1,1,0};//min val 1 for pulse
const unsigned int max_val[NO_OF_MENUS+2]   =     {0, 250 ,250, 250,5000,5000,5,250,250,0};//max val 5000 for pulse
const unsigned const char * units[NO_OF_MENUS+2]= {"","KM/H","MTR","PS","mV","mV","TY","mV","mV",""};
//const unsigned const char * models[NO_OF_MODELS+2]= {""," SENTRA","  HIACE","URVA-CAN","  URVAN","   H1"," STERIA", " VICTORY","  ISUZU"," JAC-M4",""};
const unsigned const char * models[NO_OF_MODELS+2]= {""," SENTRA","  HIACE","URVA-CAN","  URVAN","   H1"," STERIA","  ISUZU"," JAC-M4","VIC-SCAN"," VICTORY",""};
//const unsigned const char * models[NO_OF_MODELS+2]= {"","VIC-SCAN"," VICTORY"," SENTRA","  HIACE","URVA-CAN","  URVAN","   H1"," STERIA","  ISUZU"," JAC-M4",""};

typedef struct {
    uint8_t speed;
    uint8_t circ;
    uint8_t ppr;
    uint16_t min_voltage;
    uint16_t limit_volt;
    uint8_t type;
    uint8_t uphill;
    uint8_t downhill;
} DBW_Values;




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
void Lcd_FleetMax();
void lcd_init(void);
void lcd_puts(const char *);
void lcd_putInt(int);
void lcd_clear(void);
void lcd_init(void);
void lcd_write(unsigned char);
void lcd_goto(unsigned char line, unsigned char pos);
void clearLine(char line);
void SendSpeedValue_word(uint16_t value);
void transmit_sid();

void Up_Key(char Sel);
void Down_Key(char Sel);
void Up_Key_Presave(char Sel);
void Lcd_Screen_data_error();
void Lcd_Screen_data_saved();


void Lcd_Screen_Save();
void Lcd_Screen_Sending();

//Function Prototypes
void disp_init(void);

void InitUART(void);
void SendByteSerially(unsigned char);
unsigned char ReceiveByteSerially(void);
void SendStringSerially(const unsigned char*);
int calculateVoltage(int minVoltage, int VoltagePerKm,int inputSpeed); 
bool checkDBW_data();




void main() {
    
    while (1) {
    
        if(Dev_stInit)
		{
                if(Dev_stReqSent == 0)
                    {
                        //Get the data stored in device to the SID
                        TXREG = '@';
                        while(!TRMT);
                        __delay_ms(1);
                        Dev_stReqSent = 1;
                        Dev_stRespTOut = 0;
                        TMR2IF = 0;
                        TMR2ON = 1;
                    }
                    else if((Dev_stReqSent == 1)&&(!Rx_complete)&&(Dev_stRespTOut))
                    {
                        Dev_stReqSent = 0;
                    }

                    if(rcvd_char==1)
                    {
                        rcvd_char=0;
                        Dev_stInit = 0;
                        TMR2ON = 0;
                        TMR2IF = 0;
                        CREN   = 0;
                        Rx_complete =0;
                        Rx_elapsed_ticks = 0;
                        //Speed_Limit
                        DBW_Device_Value[1]=sid_val[0];
                        //Circumference
                        DBW_Device_Value[2]=sid_val[1];
                        //PPR
                        DBW_Device_Value[3]=sid_val[2];
                        //MIN_VOLTAGE
                        DBW_Device_Value[4]=(sid_val[3] << 8) | sid_val[4];
                        //LIMIT_VOLTAGE
                        DBW_Device_Value[5]=(sid_val[5] << 8) | sid_val[6];
                        //VEH_TYPE
                        DBW_Device_Value[6]=sid_val[7];
                        //UPHILL
                        DBW_Device_Value[7]=sid_val[8];
                        //DOWNHILL
                        DBW_Device_Value[8]=sid_val[9];
                    }
        }
        else
        {
        buttonState = 0;
   
    
    // SELECT KEY
    if (SELECT == 0) {
        
        buttonStateChanged = true;
        buttonState |= 0x02;
     }
    // UP KEY
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
    // DOWN Key
    else if (UP == 1 && DOWN == 0) {
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
         
    if (buttonStateChanged) {
        buttonStateChanged = false; // Reset state change flag
       // presave_key=0;
        if (buttonState & 0x02)//Select key 
        { 
                presave_key=0;
            	if(Dev_stSel >= (NO_OF_MENUS+1))
				{
					Dev_stSel = 0;
				}
				else
				{
					Dev_stSel++;
				}
				lcd_clear();					
				// Detect the presence of direction button and change the vehicle speed based on the up and down button
				switch(Dev_stSel)
				{
                    //Speed,Type,Circumference,PPR,LOWLIM
					case 0:
					{	
						lcd_puts("|-SAVE-|");
						break;
					}
                    case 1:
					{
                        button_enter=1;
                        lcd_goto(1,2);
						lcd_puts("SAVE");
                        lcd_goto(2,1);
                        lcd_puts("CUSTOM");
						break;
					}
					case 2:
					{
						lcd_puts("SPD_LIM");
						break;
					}
                    
                    case 3:
					{
						lcd_puts("CIRC");
						break;
					}
                    
                    case 4:
					{
						lcd_puts("PPR");
						break;
					}
                    
                    case 5:
					{
						lcd_puts("MIN_LOW");
						break;
					}
                    
                    case 6:
					{
						lcd_puts("LIM_LOW");
						break;
					}
                    
                    case 7:
					{
						lcd_puts("VH_TYPE");
						break;
					}
                    case 8:
					{
						lcd_puts("UPHILL");
						break;
					}
                    
                    case 9:
					{
						lcd_puts("DOWNHILL");
						break;
					}

					default:
					break;
				}
                
                 if((Dev_stSel <= MENU_END_INDEX)&&(Dev_stSel >= MENU_START_INDEX))
                 {
                        lcd_goto(2,0);
                        lcd_putInt(DBW_Device_Value[Dev_stSel-1]);
                        lcd_puts(" ");
                        lcd_puts(units[Dev_stSel-1]);
                 }	
            
#if 1           
				
#endif               
                
           __delay_ms(DEBOUNCE_DELAY*10); 
        } 
        else if (buttonState & 0x01)
        {
           if (presave_key==1)
           {
               presave_key=0;
               __delay_ms(DEBOUNCE_DELAY*10); 
              // __delay_ms(500);
               //__delay_ms(1000);
               Down_Key(0); 
           }
           else
           {
               
                 if(Dev_stSel !=1)
                {
                    if(Dev_stSel==0)
                    {
                        if(button_enter==1)
                        {    
                          Down_Key(Dev_stSel); 
                        }  
                    }
                    else
                    {    
                       Down_Key(Dev_stSel-1);
                    }
                }   
               
           }
           
            __delay_ms(DEBOUNCE_DELAY); 
        } //down button
        else if (buttonState & 0x04) 
        {
           if(Dev_stSel !=1)
           {
               if(Dev_stSel==0)
               {
                   if(button_enter==1)
                   { 
                      Up_Key(Dev_stSel);
                   } 
               }
               else
               {    
                  Up_Key(Dev_stSel-1);
               }
            }
           else //Up key pressed
            {
                 presave_key=1;
               	if(Dev_stUpKey >= NO_OF_MODELS)
				{
					Dev_stUpKey = 1;
				}
				else
				{
					Dev_stUpKey++;
				}
				lcd_clear();
               Up_Key_Presave(Dev_stUpKey);
                __delay_ms(DEBOUNCE_DELAY*10); 
               //__delay_ms(1000);
               
            }
            __delay_ms(DEBOUNCE_DELAY); 
        }

}//BUTTON STATE CHANGED
        }
    
    }//while loop

}
#if 0
  const DBW_Values DBW_Data[] = {
    {100, 20, 95, 374, 591, 1,7,4},   // Sentra
    {100, 22, 53, 801, 1100, 2,7,4},  // Hiace
    {100, 22, 93, 378, 666, 1,7,4},   // urvan-can
    {100, 22, 95, 374, 591, 1,7,4},   // urvan
    {100, 22, 53, 358, 660, 1,7,4},   // H1
    {100, 22, 52, 388, 664, 1,7,4},   //STERIA
    {100, 19, 92, 510, 795, 1,7,4},    // Victory
    {100, 23, 92, 515, 999, 3,7,4},    // ISUZU
    {100, 22, 73, 368, 800, 1,7,4}     //JAC-M4 
};
  const unsigned const char * models[NO_OF_MODELS+2]= {"","VIC-SCAN"," VICTORY"," SENTRA","  HIACE","URVA-CAN","  URVAN","   H1"," STERIA","  ISUZU"," JAC-M4",""};
#endif  
  //const unsigned const char * models[NO_OF_MODELS+2]= {""," SENTRA","  HIACE","URVA-CAN","  URVAN","   H1"," STERIA","  ISUZU"," JAC-M4","VIC-SCAN"," VICTORY",""};
  
void Up_Key_Presave(char Sel) {
   

    
    lcd_clear();
    lcd_goto(1,2);
    lcd_puts("SAVE");
    lcd_goto(2,0);
    lcd_puts(models[Sel]);
    
    DBW_Values *selected = &DBW_Data[Dev_stUpKey - 1];
        DBW_Device_Value[1] = selected->speed;
       // __delay_ms(10); 
        DBW_Device_Value[2] = selected->circ;
        // __delay_ms(10); 
        DBW_Device_Value[3] = selected->ppr;
        // __delay_ms(10); 
        DBW_Device_Value[4] = selected->min_voltage;
         //__delay_ms(10); 
        DBW_Device_Value[5] = selected->limit_volt;
        // __delay_ms(10); 
        DBW_Device_Value[6] = selected->type;
         //__delay_ms(10); 
        DBW_Device_Value[7] = selected->uphill;
        // __delay_ms(10); 
        DBW_Device_Value[8] = selected->downhill;
         //__delay_ms(10); 

}

void Up_Key(char Sel)
{
	switch(Sel)
	{
		case 1://speed
		case 2://circ
        case 3://ppr
        case 6://vh_type
        case 7://uphill
        case 8://downhill    
                
        {
			if(DBW_Device_Value[Sel] >= max_val[Sel])
			{
				DBW_Device_Value[Sel] = max_val[Sel];
			}
			else
			{	
				DBW_Device_Value[Sel] = DBW_Device_Value[Sel] + 1;
    			clearLine(2);      
      			lcd_goto(2,0);
      			lcd_putInt(DBW_Device_Value[Sel]);      
      			lcd_puts(" ");
      			lcd_puts(units[Sel]);
			}
		}  
        break;
  
		case 4:
        case 5:
		{
			if(DBW_Device_Value[Sel] >= max_val[Sel])
			{
				DBW_Device_Value[Sel] = max_val[Sel];
			}
			else
			{	
				DBW_Device_Value[Sel] = DBW_Device_Value[Sel] + 10;
    			clearLine(2);      
      			lcd_goto(2,0);
      			lcd_putInt(DBW_Device_Value[Sel]);      
      			lcd_puts(" ");
      			lcd_puts(units[Sel]);
			}
		}
		break;
		
		case 0:
		{
            Lcd_Screen_Sending();
            transmit_sid();
 		}
		break;
		default:
		break;
	}
}

void Down_Key(char Sel)
{
	switch(Sel)
	{
		case 1://speed
		case 2://circ
        case 3://ppr
        case 6://vh_type
        case 7://uphill
        case 8://downhill  
            {
			if(DBW_Device_Value[Sel] <= min_val[Sel])
			{
				DBW_Device_Value[Sel] = min_val[Sel];
			}
			else
			{	
				DBW_Device_Value[Sel] = DBW_Device_Value[Sel] - 1;
    			clearLine(2);      
      			lcd_goto(2,0);
      			lcd_putInt(DBW_Device_Value[Sel]);      
      			lcd_puts(" ");
      			lcd_puts(units[Sel]);
			}
		}
        break;
		case 4:
        case 5:
		{
			if(DBW_Device_Value[Sel] <= min_val[Sel])
			{
				DBW_Device_Value[Sel] = min_val[Sel];
			}
			else
			{	
				DBW_Device_Value[Sel] = DBW_Device_Value[Sel] - 10;
    			clearLine(2);      
      			lcd_goto(2,0);
      			lcd_putInt(DBW_Device_Value[Sel]);      
      			lcd_puts(" ");
      			lcd_puts(units[Sel]);
			}
		}
		break;
				
        case 0:
		{
                        Lcd_Screen_Sending();
                        transmit_sid();
		}
		break;
		default:
		break;
	}
}

void Lcd_SAFEMAX() {
    lcd_clear();
    lcd_goto(1,0);
    lcd_puts("SAFEMAX");
    lcd_goto(2,0);
    lcd_puts("CAL-TOOL");
 }

void Lcd_FleetMax() {
    lcd_clear();
    lcd_goto(1,0);
    lcd_puts("PAL-AUTO");
    lcd_goto(2,1);
    lcd_puts("SID_V2");
}

void Lcd_Screen_Sending() {
    lcd_clear();
	lcd_goto(1,0);
	lcd_puts("SENDING");
	lcd_goto(2,2);
	lcd_puts("DATA..");
}

void Lcd_Screen_data_saved() {
    lcd_clear();
	lcd_goto(1,2);
	lcd_puts("DATA");
	lcd_goto(2,2);
	lcd_puts("SAVED");
}

void Lcd_Screen_data_error() {
    lcd_clear();
	lcd_goto(1,2);
	lcd_puts("DATA");
	lcd_goto(2,2);
	lcd_puts("ERROR");
}



void InitHardware(void) {  
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
 
   
}


void __interrupt() ISR(void)
{
	
	GIE = 0;
    
    if((TMR2IE)&&(TMR2IF))
    {
        TMR2IF=0;

        if(Rx_elapsed_ticks > 1000)
        {
			Dev_stRespTOut = 1;
   	        Rx_complete = 0;
		}
        else
        {
            Rx_elapsed_ticks++;
        }
    }

    if((RCIE) && (RCIF))
    {
        
         if(OERR) // If over run error, then reset the receiver
        {
            CREN = 0;
            CREN = 1;
        }
        
         rx_char=RCREG;
         
              // starting character is "*" and ending char is "#" //
         if (rx_char == '*')
         {
             sid_index=0;//make ready to write next character
         }
         
         if (rx_char == '#')
         {
              rcvd_char=true; 
              sid_index=0;
         }
         
         //dont write to eeprom
         if ( (rx_char == '*') || (rx_char == '#'))
         {
             
         }  
         else
         {
             sid_val[sid_index]=rx_char;
             sid_index++;
         }    

       RCIF = 0; // Clear the UART receive interrupt flag 
       
    }
	GIE = 1; 
	return;
}


void SendSpeedValue_word(uint16_t value) 
{
    unsigned char highByte = (value >> 8) & 0xFF; // Get the higher byte
    unsigned char lowByte = value & 0xFF;         // Get the lower byte
    
    SendByteSerially(highByte);  // Send the higher byte first
    __delay_ms(100);
    SendByteSerially(lowByte);   // Send the lower byte
    __delay_ms(100);
}




void transmit_sid() {
    bool verify_ok = 1;
    unsigned int DBW_Device_verify_Value[NO_OF_MENUS+2];

    for (int i = 1; i <= NO_OF_MENUS; i++) {
        DBW_Device_verify_Value[i] = DBW_Device_Value[i];
    }

    SendByteSerially('*');
    __delay_ms(10);
    
    SendByteSerially(DBW_Device_Value[1]&0xFF);
    __delay_ms(10);

    SendByteSerially(DBW_Device_Value[2]&0xFF);//Circ
    __delay_ms(10);

    SendByteSerially(DBW_Device_Value[3]&0xFF);//ppr
    __delay_ms(10);

    SendSpeedValue_word(DBW_Device_Value[4]);//min voltage
    __delay_ms(10);

    SendSpeedValue_word(DBW_Device_Value[5]);//limiting voltage
    __delay_ms(10);

    SendByteSerially(DBW_Device_Value[6]&0xFF);//vehicle type
    __delay_ms(10);
    
     SendByteSerially(DBW_Device_Value[7]&0xFF);//uphill
    __delay_ms(10);

    SendByteSerially(DBW_Device_Value[8]&0xFF);//downhill
    __delay_ms(10);

    for (int i = (MENU_START_INDEX-1); i <= NO_OF_MENUS; i++) {
        DBW_Device_Value[i] = 0;
    }

    for (int i = 0; i <= (TOTAL_BYTES-1); i++) {
        sid_val[i] = 0;
    }
    sid_index=0;

    SendByteSerially('#');
    __delay_ms(10);

    TMR2IF = 0;
    TMR2ON = 1;
    CREN = 1;

    while (rcvd_char == 0) {
        // Do nothing
    }

    TMR2IF = 0;
    TMR2ON = 0;
    CREN = 0;

    rcvd_char = 0;
    
     DBW_Device_Value[1] = sid_val[0];
     DBW_Device_Value[2] = sid_val[1];
     DBW_Device_Value[3] = sid_val[2];
     DBW_Device_Value[4] = (sid_val[3] << 8) | sid_val[4];
     DBW_Device_Value[5] = (sid_val[5] << 8) | sid_val[6];
     DBW_Device_Value[6] = sid_val[7];
     DBW_Device_Value[7] = sid_val[8];
     DBW_Device_Value[8] = sid_val[9];
   

    for (int i = (MENU_START_INDEX-1); i <= NO_OF_MENUS; i++) {
        if (DBW_Device_verify_Value[i] != DBW_Device_Value[i]) {
            verify_ok = 0;
            break;
        }
    }

    if (verify_ok == 1) {
        Lcd_Screen_data_saved();
    } else {
        Dev_stUpKey--;
#ifdef DEBUG  // For Debugging
        for (int i = 1; i <= 8; i += 2) {
           // if (SELECT == 0) {
                lcd_clear();
                lcd_goto(1, 0);
                lcd_putInt(DBW_Device_verify_Value[i]);
                lcd_goto(2, 0);
                lcd_putInt(DBW_Device_verify_Value[i + 1]);
                __delay_ms(5000);
            //}
        }

        for (int i = 1; i <= 8; i += 2) {
           // if (SELECT == 0) {
                lcd_clear();
                lcd_goto(1, 0);
                lcd_putInt(DBW_Device_Value[i]);
                lcd_goto(2, 0);
                lcd_putInt(DBW_Device_Value[i + 1]);
                __delay_ms(5000);
            //}
        }
#endif
        Lcd_Screen_data_error();
    }
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

void lcd_write(unsigned char c)
{
   __delay_us(40);   
   PORTA = c >> 4;
   LCD_STROBE;
   PORTA = c;
   LCD_STROBE;
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





