/*
 * File:   rf-receiver-rx.c
 * Author: ElektroNEO
 *
 * Created on 07 Aralik 2017 Persembe, 22:44
 */
// PIC18F45K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config CCP2MX = PORTB3  // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = INTMCLR  // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       
#pragma config WRT1 = OFF       
#pragma config WRT2 = OFF       
#pragma config WRT3 = OFF       

// CONFIG6H
#pragma config WRTC = OFF       
#pragma config WRTB = OFF       
#pragma config WRTD = OFF       

// CONFIG7L
#pragma config EBTR0 = OFF      
#pragma config EBTR1 = OFF      
#pragma config EBTR2 = OFF      
#pragma config EBTR3 = OFF      

// CONFIG7H
#pragma config EBTRB = OFF      

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// 8 Mhz dahili osilator kullaniliyor.
#define _XTAL_FREQ 8000000
#include <xc.h>


unsigned char rxdat[3];  // (global var) Gelen RF verisini tutar.

//=============================================================================
//   RECEIVE_RF_PACKET // 1 bytelik veri alma fonksiyonu.
//=============================================================================
void receive_rf_packet(void)
{
  //-------------------------------------------------------
  // Bu fonksiyon gelen veriyi cozer ve rxdat[] dizisine kaydeder.
  // Ard arda 3 tane geçerli 1 bytelik bilgi alinmak zorundadir.
  // Fonksiyon bu 3 bilgiyi alincaya kadar tekrar eder.
  // global degisken; unsigned char rxdat[10] 3 bylelik sonucu tutar.
  // Not: TMR0 500kHz frekansta calisiyor, bu yüzden 200uS = 100 TMR0 sayaci
  //-------------------------------------------------------
  unsigned char rrp_data;
  unsigned char rrp_period;
  unsigned char rrp_bits;
  unsigned char rrp_bytes;

  rrp_bytes = 0;
  while(rrp_bytes < 3) // Gecerli 3 byte veri alincaya kadar dongude kal.
  {
    //-----------------------------------------
    // Baslangic biti icin bekle. Bu bitin periyodu > 330uS
    while(1)
    {
      TMR0L = 0;                         // Zamanlayiciyi sifirla.
      while(!PORTCbits.RC4) continue;    // Yukselen kenar (/) icin bekle.
      while(PORTCbits.RC4) continue;     // Dusen kenar (\) icin bekle.
      rrp_period = TMR0L;                // Gecen sureyi kaydet.

      if(rrp_period < 165) rrp_bytes = 0;   // Gecerli veri degerini sifirla eger
                                            // hala gurultu varsa.
      else break;                           // Eger sure >330uS ise donguden cik.
    }

    //-----------------------------------------
    // Simdi 8 bitlik veriyi alabiliriz.
    rrp_bits = 8;
    // rrp_bits 0 oluncaya kadar dongude kal.
    while(rrp_bits)
    {
      TMR0L = 0; // Zamanlayiciyi sifirla.
      while(!PORTCbits.RC4) continue;    // Yukselen kenar (/) icin bekle.
      while(PORTCbits.RC4) continue;     // Dusen kenar (\) icin bekle.
      rrp_period = TMR0L;                // Gecen sureyi kaydet.
      
      if(rrp_period >= 100) break;       // Eger sure >=200uS ise donguden cik.
                                         // Bu beklenmedik bir sinyaldir.
      if(rrp_period < 75)                // Eger sure <75uS ise gelen veri 0'dir.
          rrp_data &= (unsigned char)254;// 75 = 150uS
      else                
          rrp_data |= (unsigned char)1;  // Eger sure >75uS ise gelen veri 1'dir.
      // En son biti aldiktan sonra kaydirma islemini yapmasin.
      if (rrp_bits == 1) {
          rrp_bits--;
          break;
      }
      rrp_data = (rrp_data << 1);   // Veriyi 1 bit sola kaydir.
      rrp_bits--;                   // Bir sonraki bit icin yer ac.
    }

    //-----------------------------------------
    // gets to here after 8 good bits OR after an error (unexpected start pulse)
    if(rrp_bits)      // Hata varsa...
    {
      rrp_bytes = 0;  // Gecerli byteleri sifirla ve tekrar 
                      // veriyi almak icin basa don
    }
    else              // Hata yoksa 8 bitlik veriyi diziye kaydet.
    {
      rxdat[rrp_bytes] = rrp_data;
      rrp_bytes++;                  // Bir sonraki veriyi almaya hazirla.
    }
  }
}
//-----------------------------------------------------------------------------

void main(void) {
    // Osilator ayari.
    OSCCONbits.IRCF = 6; // 110 - 8 Mhz
    // RF al?c? girisi.
    TRISCbits.TRISC4 = 1;
    ANSELCbits.ANSC4 = 0;
    PORTCbits.RC4 = 0;
    
    // Korna cikisi.
    TRISAbits.TRISA1 = 0;
    ANSELAbits.ANSA1 = 0;
    
    // Engel algilayici girisi.
    TRISAbits.TRISA5 = 1;
    ANSELAbits.ANSA5 = 0;
    PORTAbits.RA5 = 0;
    
    // On far cikislari.
    // 1. far
    TRISAbits.TRISA7 = 0;
    PORTAbits.RA7 = 0;
    // 2. far
    TRISCbits.TRISC0 = 0;
    PORTCbits.RC0 = 0;
    
    // Arka far cikislari
    // 1. far
    TRISBbits.TRISB3 = 0;
    ANSELBbits.ANSB3 = 0;
    PORTBbits.RB3 = 0;
    // 2. far
    TRISBbits.TRISB5 = 0;
    ANSELBbits.ANSB5 = 0;
    PORTBbits.RB5 = 0;
    
    // Motor kontrol.
    // PWM1
    TRISCbits.RC2 = 0;
    ANSELCbits.ANSC2 = 0;
    PORTCbits.RC2 = 0;
    // PWM2
    TRISCbits.RC1 = 0;
    PORTCbits.RC1 = 0;
    // PWM3
    TRISEbits.RE0 = 0;
    ANSELEbits.ANSE0 = 0;
    PORTEbits.RE0 = 0;
    // PWM4
    TRISDbits.RD1 = 0;
    ANSELDbits.ANSD1 = 0;
    PORTDbits.RD1 = 0;
    
    
    T0CONbits.T08BIT = 1; // TMR0'i 8 bit olarak ayarla.
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 0;
    T0CONbits.T0PS = 0b001; // Prescaler = 1:4 = 500Khz
    T0CONbits.TMR0ON = 1;
    
    unsigned char data;     // Verinin kaydedilecegi degisken.
   
    while(1) {
        // Veriyi al.
        receive_rf_packet();
        data = rxdat[1]; // alinan veriyi data degiskenine ata.
        
        // Korna
        if (data & (0b01000000)) {
            LATAbits.LA1 = 1;
        }
        else {
            LATAbits.LA1 = 0;
        }
        
        // Engel
        if((data & (0b10000000)) && !(data & (0b01000000))) {
            if(PORTAbits.RA5 == 1) {
                LATAbits.LA1 = 1;
            }
            else {
                LATAbits.LA1 = 0;
            }
        }
        // On farlar
        if (data & (0b00100000)) {
            LATAbits.LA7 = 1;
            LATCbits.LC0 = 1;
        }
        else {
            LATAbits.LA7 = 0;
            LATCbits.LC0 = 0;
        }
        // Motor kntrolu
        if ((data & (0b00001111)) == 0b0000) {
            // ?leri-Sol
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 1;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001111)) == 0b0001) {
            // ?leri
            LATCbits.LATC2 = 1;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 1;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001111)) == 0b0011) {
            // Ileri-Sag
            LATCbits.LATC2 = 1;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001111)) == 0b1100) {
            // Geri-Sol
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 1;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001111)) == 0b1101) {
            // Geri
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 1;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 1;
            LATBbits.LATB3 = 1;
            LATBbits.LATB5 = 1;
        }
        else if ((data & (0b00001111)) == 0b1111) {
            // Geri-Sag
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 1;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001100)) == 0b1000) {
            // Dur
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
    }
    return;
}