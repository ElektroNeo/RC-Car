/*
 * Author: ElektroNEO
 *
 * Created on 16 Kasım 2017 Perşembe, 23:38
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

// 8 Mhz kristal kullanılıyor.
#define _XTAL_FREQ 8000000

#include <xc.h>

//=======================================================================
//   SEND_RF_BYTE: 1 bytelik veri gonderme fonksiyonu.
//=======================================================================
void send_rf_byte(unsigned char txdat)
{
  //-------------------------------------------------------
  // Bu fonksiyon ile bir verinin herbir bitini teker teker verici
  // module gonderir.
  // Zamanlama;
  //   HIGH pulse uzunlugu; her zaman 100uS
  //   0 bit, LOW pulse uzunlugu; 50uS (toplam 0 bit pulse periyodu 150uS)
  //   1 bit, LOW pulse uzunlugu; 150uS (toplam 1 bit pulse periyodu 250uS)
  //   byteler arası boşluk, LOW pulse uzunluğu; 100uS , HI pulse uzunluğu
  //   250uS (toplam boşluk periyodu 350uS)
  //-------------------------------------------------------
  unsigned char tbit;

  // 350 us lik başlangıç biti gönderiliyor.
  // Alıcı bu biti aldığında veriyi almaya başlar.
  __delay_us(100);      // 100uS LOW
  PORTCbits.RC5 = 1;
  __delay_us(250);     // 250uS HIGH
  PORTCbits.RC5 = 0;

  // 8 bitlik veri gönderiliyor.
  for(tbit=0; tbit<8; tbit++)
  {
    __delay_us(50);         // varsayılan 0 bit LOW pulse periyodu: 50uS
    if((txdat >> 7) & (0b1))
        __delay_us(100);    // bit değeri 1 ise LOW pulse periyodunu
                            // 100uS arttır.
    PORTCbits.RC5 = 1;
    __delay_us(100);        // HIGH pulse değeri: 100uS
    PORTCbits.RC5 = 0;
    txdat = txdat << 1;     // Veri 1 sola kaydırılıyor.
  }
}

void main(void) {
    // Giriş - çıkışlar.
    // Joystik için girişler.
    TRISAbits.TRISA0 = 1; // A0 giriş.
    TRISAbits.TRISA1 = 1; // A2 giriş.
    ADCON1bits.PCFG = 0;  // PORTA analog giriş.
    ADCON1bits.ADFM = 0;  // Left justified. 6 Least Significant bits of
                          // ADRESL are read as ?0?.

    // Korna, far ve engel girişi.
    TRISBbits.TRISB1 = 1; // Korna
    TRISBbits.TRISB2 = 1; // Far
    TRISBbits.TRISB4 = 1; // Engel

    // Verici çıkışı.
    TRISCbits.TRISC5 = 0; // RF-RX
    PORTCbits.RC5 = 0;

    unsigned char data = 128;
    while(1) {

        // Sağ - Sol ayarı.
        ADCON0bits.CHS = 0b001; // Channel 1
        ADCON0bits.ADCS = 0b01; // Fosc/8
        ADCON0bits.ADON = 1;    // ADC modülünü aç.
        __delay_us(20);
        ADCON0bits.GO_nDONE = 1;
        while(ADCON0bits.GO_nDONE);
        ADCON0bits.ADON = 0;    // ADC modülünü kapat.

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

        // Ileri - Geri ayarı.
        ADCON0bits.CHS = 0b000; // Channel 0
        ADCON0bits.ADCS = 0b01; // Fosc/8
        ADCON0bits.ADON = 1;    // ADC modülünü aç.
        __delay_us(20);
        ADCON0bits.GO_nDONE = 1;
        while(ADCON0bits.GO_nDONE);
        ADCON0bits.ADON = 0;    // ADC modülünü kapat.

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

        // Korna
        if(PORTBbits.RB1 == 1) {
            asm("BANKSEL main@data");
            asm("bcf main@data,6");
        }
        else if(PORTBbits.RB1 == 0){
            asm("BANKSEL main@data");
            asm("bsf main@data,6");
        }
        // Ön farlar
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

        // 1 bytelik veri gönderilir.
        send_rf_byte(data);
    }
}
