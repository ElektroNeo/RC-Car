/*
 * Author: ElektroNEO
 *
 * Created on 07.12.2017, 22:44
 */

// PIC18F45K22 Configuration Bit Settings
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal
                                // oscillator block)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock
                                // is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit
                                // (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover
                                // bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up
                                // timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out
                                // Reset enabled in hardware only
                                // (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits
                                // (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog
                                // timer is always disabled. SWDTEN has
                                // no effect.)
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config CCP2MX = PORTB3  // CCP2 MUX bit (CCP2 input/output is
                                // multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are
                                // configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output
                                // is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output
                                // and ready status are not delayed by the
                                // oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = INTMCLR  // MCLR Pin Enable bit (RE3 input pin enabled;
                                // MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit
                                // (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply
                                // ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit
                                // (Instruction set extension and Indexed
                                // Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0
                                // (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1
                                // (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2
                                // (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3
                                // (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block
                                // (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data
                                // EEPROM not code-protected)

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

// 8MHz internal oscillator used.
#define _XTAL_FREQ 8000000
#include <xc.h>


unsigned char rxdat[5];  // (global variable) Stores received data.

//=======================================================================
//   RECEIVE_RF_PACKET // Read 1 byte data function.
//=======================================================================
void receive_rf_packet(void)
{
  //-------------------------------------------------------
  // This function decodes received data and stores it to rxdat[] array.
  // 3 valid 1-byte information must be obtained consecutively.
  // The function repeats until you get these 3 pieces of information.
  // global variable; unsigned char rxdat[10] stores 3 bytes of data.
  // Note: TMR0 runs at 500kHz frequency, That is why 200uS = 100 TMR0 counter
  //-------------------------------------------------------
  unsigned char rrp_data;
  unsigned char rrp_period;
  unsigned char rrp_bits;
  unsigned char rrp_bytes;

  rrp_bytes = 0;
  while(rrp_bytes < 3) // Loop until you get valid 3 bytes of data.
  {
    //-----------------------------------------
    // Wait for starting bit.
    while(1)
    {
      TMR0L = 0;                          // Reset timer.
      while(!PORTCbits.RC4) continue;     // Wait for rising edge.
      while(PORTCbits.RC4) continue;      // Wait for falling edge.
      rrp_period = TMR0L;                 // Save time.

      if(rrp_period < 150) rrp_bytes = 0; // Reset current data value
                                          // if there is still noise.
      else break;                         // If time is greater than 300uS then, exit the loop.
    }

    //-----------------------------------------
    // Now we can get 8-bit data
    rrp_bits = 8;
    // Wait until rrp_bits will be 0.
    while(rrp_bits)
    {
      TMR0L = 0; // Reset timer.
      while(!PORTCbits.RC4) continue;    // Wait for rising edge.
      while(PORTCbits.RC4) continue;     // Wait for falling edge.
      rrp_period = TMR0L;                // Save time.

      if(rrp_period >= 150) break;       // If time is greater than 300uS then, exit the loop.
                                         // This is an unexpected signal.
      if(rrp_period < 100)               // If time is less than 200uS then, data is 0.
          rrp_data &= (unsigned char)254;// 100 = 200uS
      else if(rrp_period < 150)
          rrp_data |= (unsigned char)1;  // If time is less than 75uS then, data is 0.
      else break;
      // Do not shift after receiving the last bit.
      if (rrp_bits == 1) {
          rrp_bits--;
          break;
      }
      rrp_data = (rrp_data << 1);   // Shift data by 1.
      rrp_bits--;                   // Make room for the next bit.
    }

    //-----------------------------------------
    if(rrp_bits)      // If an error...
    {
      rrp_bytes = 0;  // Reset the current bytes and return to 
                      // top to get the data again.
    }
    else              // If there are no errors, save the 8-bit data to the array.
    {
      rxdat[rrp_bytes] = rrp_data;
      rrp_bytes++;    // Prepare to receive the next data.
    }
  }
}
//-----------------------------------------------------------------------------

void main(void) {
    // Oscillator configuration.
    OSCCONbits.IRCF = 6; // 110 - 8 Mhz
    // RF receiver input.
    TRISCbits.TRISC4 = 1;
    ANSELCbits.ANSC4 = 0;
    PORTCbits.RC4 = 0;

    // Buzzer output.
    TRISAbits.TRISA1 = 0;
    ANSELAbits.ANSA1 = 0;

    // Obstacle detector input.
    TRISAbits.TRISA5 = 1;
    ANSELAbits.ANSA5 = 0;
    PORTAbits.RA5 = 0;

    // Front light outputs.
    // 1. light
    TRISAbits.TRISA7 = 0;
    PORTAbits.RA7 = 0;
    // 2. light
    TRISCbits.TRISC0 = 0;
    PORTCbits.RC0 = 0;

    // Back light outputs.
    // 1. light
    TRISBbits.TRISB3 = 0;
    ANSELBbits.ANSB3 = 0;
    PORTBbits.RB3 = 0;
    // 2. light
    TRISBbits.TRISB5 = 0;
    ANSELBbits.ANSB5 = 0;
    PORTBbits.RB5 = 0;

    // Motor control.
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


    T0CONbits.T08BIT = 1;   // Set TMR0 to 8 bit.
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 0;
    T0CONbits.T0PS = 0b001; // Prescaler = 1:4 = 500Khz
    T0CONbits.TMR0ON = 1;

    unsigned char data;     // Data variable.

    while(1) {
        // Get data.
        receive_rf_packet();
        data = rxdat[1]; // Copy receivet data to data variable.

        // Buzzer
        if (data & (0b01000000)) {
            LATAbits.LA1 = 1;
        }
        else {
            LATAbits.LA1 = 0;
        }

        // Obstacle
        if((data & (0b10000000)) && !(data & (0b01000000))) {
            if(PORTAbits.RA5 == 1) {
                LATAbits.LA1 = 1;
                // Backward
                LATCbits.LATC2 = 0;
                LATCbits.LATC1 = 1;
                LATEbits.LATE0 = 0;
                LATDbits.LATD1 = 1;
                LATBbits.LATB3 = 1;
                LATBbits.LATB5 = 1;
                __delay_ms(100);
            }
            else {
                LATAbits.LA1 = 0;
            }
        }
        // Front lights
        if (data & (0b00100000)) {
            LATAbits.LA7 = 1;
            LATCbits.LC0 = 1;
        }
        else {
            LATAbits.LA7 = 0;
            LATCbits.LC0 = 0;
        }
        // Motor controle
        if ((data & (0b00001111)) == 0b0000) {
            // Forward-Left
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 1;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001111)) == 0b0001) {
            // Forward
            LATCbits.LATC2 = 1;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 1;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001111)) == 0b0011) {
            // Forward-Right
            LATCbits.LATC2 = 1;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001111)) == 0b1100) {
            // Backward-Left
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 0;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 1;
            LATBbits.LATB3 = 1;
            LATBbits.LATB5 = 0;
        }
        else if ((data & (0b00001111)) == 0b1101) {
            // Backward
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 1;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 1;
            LATBbits.LATB3 = 1;
            LATBbits.LATB5 = 1;
        }
        else if ((data & (0b00001111)) == 0b1111) {
            // Backward-Right
            LATCbits.LATC2 = 0;
            LATCbits.LATC1 = 1;
            LATEbits.LATE0 = 0;
            LATDbits.LATD1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 1;
        }
        else if ((data & (0b00001100)) == 0b1000) {
            // Stop
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
