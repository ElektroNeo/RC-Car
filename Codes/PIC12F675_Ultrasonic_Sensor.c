/*
 * Author: ElektroNEO
 *
 * Created on 20.12.2017, 14:27
 */

// PIC12F675 Configuration Bit Settings
#pragma config FOSC = INTRCIO // Oscillator Selection bits (INTOSC           \
                              // oscillator: I/O function on GP4/OSC2/CLKOUT \
                              // pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF     // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF    // Power-Up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF    // GP3/MCLR pin function select (GP3/MCLR pin \
                              // function is digital I/O, MCLR internally   \
                              // tied to VDD)
#pragma config BOREN = ON     // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF       // Code Protection bit (Program Memory code \
                              // protection is disabled)
#pragma config CPD = OFF      // Data Code Protection bit (Data memory code \
                              // protection is disabled)

// 4Mhz internal oscillator
#define _XTAL_FREQ 4000000
#include <xc.h>

void main(void)
{

    OSCCALbits.CAL = 0b111111; // 4Mhz
    TRISIObits.TRISIO5 = 0;    // Trigger
    TRISIObits.TRISIO4 = 1;    // Echo
    TRISIObits.TRISIO1 = 0;    // Output

    ANSELbits.ANS = 0b0000;

    T1CONbits.T1CKPS = 0b00; // 1:1 Prescale Value
    T1CONbits.TMR1CS = 0;    // Internal clock (Fosc/4) = 1Mz

    // Variable decleration
    int distance;

    while (1)
    {
        // Reset TMR1
        TMR1H = 0;
        TMR1L = 0;

        // Send 10uS signal to the Trigger pin
        GPIObits.GP5 = 1;
        __delay_us(10);
        GPIObits.GP5 = 0;

        // Wait for sensor response
        while (!GPIObits.GP4);
        T1CONbits.TMR1ON = 1;
        while (GPIObits.GP4);
        T1CONbits.TMR1ON = 0;

        // Save TMR1 value to distance variable
        distance = (TMR1L | (TMR1H << 8));
        // Divide distance to 88 for get value in cm
        // The 88 value is get by tests
        distance = ((double)distance / 88);

        // If distance is between 2 and 20 then make output high
        if ((distance > 2) && (distance < 20))
        {
            GPIObits.GP1 = 1;
        }
        // If not, make output low
        else
        {
            GPIObits.GP1 = 0;
        }
        __delay_ms(10); // Wait 10ms
    }

    return;
}
