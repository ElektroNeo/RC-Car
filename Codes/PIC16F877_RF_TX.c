/*
 * Author: ElektroNEO
 *
 * Created on 16.12.2017, 23:38
 */

// PIC16F877 Configuration Bit Settings
// CONFIG - Konfigürasyon ayarları.
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config CP = OFF         // FLASH Program Memory Code Protection bits
                                // (Code protection off)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low Voltage In-Circuit Serial Programming
                                // Enable bit (RB3 is digital I/O, HV on MCLR
                                // must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection (Code
                                // Protection off)
#pragma config WRT = ON         // FLASH Program Memory Write Enable
                                // (Unprotected program memory may be written
                                // to by EECON control)

// 8Mhz external crystal used
#define _XTAL_FREQ 8000000

#include <xc.h>

//=======================================================================
//   SEND_RF_BYTE: Send 1 byte data function.
//=======================================================================
void send_rf_byte(unsigned char txdat)
{
  //-------------------------------------------------------
  // This function sneds every bits in data to receiver module.
  // Timing;
  //   HIGH pulse width; always 100uS
  //   0 bit, LOW pulse width; 50uS (total 0 bit pulse width is 150uS)
  //   1 bit, LOW pulse width; 150uS (total 1 bit pulse width is 250uS)
  //   delay between bytes, LOW pulse width; 100uS , HI pulse width
  //   250uS (total delay period is 350uS)
  //-------------------------------------------------------
  unsigned char tbit;

  // Send 300us starting bit.
  // Receiver starts get data when reads this bit.
  __delay_us(100);      // 100uS LOW
  PORTCbits.RC5 = 1;
  __delay_us(250);     // 250uS HIGH
  PORTCbits.RC5 = 0;

  // Sending 8bit data
  for(tbit=0; tbit<8; tbit++)
  {
    __delay_us(50);         // Default 0 bit LOW pulse width: 50uS
    if((txdat >> 7) & (0b1))
        __delay_us(100);    // If bit is 1 then, increase LOW pulse width 100uS

    PORTCbits.RC5 = 1;
    __delay_us(100);        // HIGH pulse width: 100uS
    PORTCbits.RC5 = 0;
    txdat = txdat << 1;     // Data shifts to left
  }
}

void main(void) {
    // I/Os
    // Joystick inputs.
    TRISAbits.TRISA0 = 1; // A0 input.
    TRISAbits.TRISA1 = 1; // A1 input.
    ADCON1bits.PCFG = 0;  // PORTA analog input.
    ADCON1bits.ADFM = 0;  // Left justified. 6 Least Significant bits of
                          // ADRESL are read as ?0?.

    // Buzzer, light and obstacle switch input.
    TRISBbits.TRISB1 = 1; // Buzzer
    TRISBbits.TRISB2 = 1; // Light
    TRISBbits.TRISB4 = 1; // Obstacle

    // Receiver output.
    TRISCbits.TRISC5 = 0; // RF-RX
    PORTCbits.RC5 = 0;

    unsigned char data = 128;
    while(1) {

        // Joystick left-right configurations.
        ADCON0bits.CHS = 0b001; // Channel 1
        ADCON0bits.ADCS = 0b01; // Fosc/8
        ADCON0bits.ADON = 1;    // Turn on ADC module.
        __delay_us(20);
        ADCON0bits.GO_nDONE = 1;
        while(ADCON0bits.GO_nDONE);
        ADCON0bits.ADON = 0;    // Turn off ADC module.

        if(ADRESH > 250) {
            asm("BANKSEL main@data");
            asm("bsf main@data,3");
            asm("bsf main@data,2");
        }
        else if(ADRESH < 5 ) {
            asm("BANKSEL main@data");
            asm("bcf main@data,3");
            asm("bcf main@data,2");
        }
        else {
            asm("BANKSEL main@data");
            asm("bsf main@data,3");
            asm("bcf main@data,2");
        }

        // Joystick forward-backward configurations.
        ADCON0bits.CHS = 0b000; // Channel 0
        ADCON0bits.ADCS = 0b01; // Fosc/8
        ADCON0bits.ADON = 1;    // Turn on ADC module.
        __delay_us(20);
        ADCON0bits.GO_nDONE = 1;
        while(ADCON0bits.GO_nDONE);
        ADCON0bits.ADON = 0;    // Turn off ADC module.

        if(ADRESH > 200) {
            asm("BANKSEL main@data");
            asm("bcf main@data,1");
            asm("bcf main@data,0");
        }
        else if(ADRESH < 50) {
            asm("BANKSEL main@data");
            asm("bsf main@data,1");
            asm("bsf main@data,0");
        }
        else {
            asm("BANKSEL main@data");
            asm("bcf main@data,1");
            asm("bsf main@data,0");
        }

        // Buzzer
        if(PORTBbits.RB1 == 1) {
            asm("BANKSEL main@data");
            asm("bcf main@data,6");
        }
        else if(PORTBbits.RB1 == 0){
            asm("BANKSEL main@data");
            asm("bsf main@data,6");
        }
        // Front lights
        if(PORTBbits.RB2 == 1) {
            asm("BANKSEL main@data");
            asm("bsf main@data,5");
        }
        else if(PORTBbits.RB2 == 0){
            asm("BANKSEL main@data");
            asm("bcf main@data,5");
        }

        // Obstacle
        if(PORTBbits.RB4 == 1) {
            asm("BANKSEL main@data");
            asm("bsf main@data,7");
        }
        else if(PORTBbits.RB4 == 0){
            asm("BANKSEL main@data");
            asm("bcf main@data,7");
        }

        // Sends 1 byte data.
        send_rf_byte(data);
    }
}
