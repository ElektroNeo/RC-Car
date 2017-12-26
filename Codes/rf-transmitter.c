/*
 * File:   rf-transmitter.c
 * Author: ElektroNEO
 *
 * Created on 16 Kasim 2017 Persembe, 23:38
 */

// PIC16F877 Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config CP = OFF         // FLASH Program Memory Code Protection bits (Code protection off)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low Voltage In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection (Code Protection off)
#pragma config WRT = ON         // FLASH Program Memory Write Enable (Unprotected program memory may be written to by EECON control)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// 8 Mhz kristal kullaniliyor.
#define _XTAL_FREQ 8000000

#include <xc.h>

//=============================================================================
//   SEND_RF_BYTE: 1 bytelik veri gonderme fonksiyonu.
//=============================================================================
void send_rf_byte(unsigned char txdat)
{
  //-------------------------------------------------------
  // Bu fonksiyon ile bir verinin herbir bitini teker teker verici
  // module gonderir. Bits are sent MSB first. Each byte sends 9 pulses (makes 8 periods).
  // Zamanlama;
  //   HIGH pulse uzunlugu; her zaman 80uS
  //   0 bit, LOW pulse uzunlugu; 20uS (toplam 0 bit pulse periyodu 100uS)
  //   1 bit, LOW pulse uzunlugu; 70uS (toplam 1 bit pulse periyodu 150uS)
  //   byteler arasi bosluk, LOW pulse uzunlugu; 170uS (toplam bosluk periyodu 250uS)
  //-------------------------------------------------------
  unsigned char tbit;

  // 350 us lik baslangic biti gonderiliyor. 
  // Alici bu biti aldiginda veriyi almaya baslar.
  __delay_us(100);      // 100uS LOW
  PORTCbits.RC5 = 1;
  __delay_us(250);     // 150uS HIGH
  PORTCbits.RC5 = 0;
  
  // 8 bitlik veri gonderiliyor.
  for(tbit=0; tbit<8; tbit++)
  {
    __delay_us(19);         // default 0 bit LO period is 20uS
    if((txdat >> 7) & (0b1)) 
        __delay_us(50);     // increase the LO period if is a 1 bit!
    PORTCbits.RC5 = 1;
    __delay_us(79);         // 80uS HI pulse
    PORTCbits.RC5 = 0;
    txdat = txdat << 1;     // Veri 1 sola kaydiriliyor.
  }
}
//-----------------------------------------------------------------------------

void main(void) {
    // Giris cikislar.
    // Joystik icin girisler.
    TRISAbits.TRISA0 = 1; // A0 giris.
    TRISAbits.TRISA1 = 1; // A2 giris.
    ADCON1bits.PCFG = 0; // PORTA analog giris.
    ADCON1bits.ADFM = 0; // Left justified. 6 Least Significant bits of ADRESL are read as ?0?.
    
    // Korna, far ve engel girisi.
    TRISBbits.TRISB1 = 1; // Korna
    TRISBbits.TRISB2 = 1; // Far
    TRISBbits.TRISB4 = 1; // Engel
    
    // Verici cikisi.
    TRISCbits.TRISC5 = 0; // RF-RX
    PORTCbits.RC5 = 0;
    
    unsigned char data = 128;
    while(1) {
        
        // Sag - Sol ayari.
        ADCON0bits.CHS = 0b001; // Channel 1
        ADCON0bits.ADCS = 0b01; // Fosc/8
        ADCON0bits.ADON = 1; //Turn on the conversation
        __delay_us(20);
        ADCON0bits.GO_nDONE = 1;
        while(ADCON0bits.GO_nDONE);
        ADCON0bits.ADON = 0; //Turn off the conversation
        
        if(ADRESH > 245) {
            asm("BANKSEL main@data");
            asm("bsf main@data,3");
            asm("bsf main@data,2");
        }
        else if(ADRESH < 15 ) {
            asm("BANKSEL main@data");
            asm("bcf main@data,3");
            asm("bcf main@data,2");
        }
        else {
            asm("BANKSEL main@data");
            asm("bsf main@data,3");
            asm("bcf main@data,2");
        }
        
        // Ileri - Geri ayari.
        ADCON0bits.CHS = 0b000; // Channel 0
        ADCON0bits.ADCS = 0b01; // Fosc/8
        ADCON0bits.ADON = 1; //Turn on the conversation
        __delay_us(20);
        ADCON0bits.GO_nDONE = 1;
        while(ADCON0bits.GO_nDONE);
        ADCON0bits.ADON = 0; //Turn off the conversation
        
        if(ADRESH > 180) {
            asm("BANKSEL main@data");
            asm("bcf main@data,1");
            asm("bcf main@data,0");
        }
        else if(ADRESH < 70) {
            asm("BANKSEL main@data");
            asm("bsf main@data,1");
            asm("bsf main@data,0");
        }
        else {
            asm("BANKSEL main@data");
            asm("bcf main@data,1");
            asm("bsf main@data,0");
        }
        
        // Korna
        if(PORTBbits.RB1 == 1) {
            asm("BANKSEL main@data");
            asm("bcf main@data,6");
        }
        else if(PORTBbits.RB1 == 0){
            asm("BANKSEL main@data");
            asm("bsf main@data,6");
        }
        // On farlar
        if(PORTBbits.RB2 == 1) {
            asm("BANKSEL main@data");
            asm("bsf main@data,5");
        }
        else if(PORTBbits.RB2 == 0){
            asm("BANKSEL main@data");
            asm("bcf main@data,5");
        }
        
        // Engel
        if(PORTBbits.RB4 == 1) {
            asm("BANKSEL main@data");
            asm("bsf main@data,7");
        }
        else if(PORTBbits.RB4 == 0){
            asm("BANKSEL main@data");
            asm("bcf main@data,7");
        }
        
        // 1 bytelik veri gonderilir.
        send_rf_byte(data);
        
    }
}