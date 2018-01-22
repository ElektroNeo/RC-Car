/*
 * Author: ElektroNEO
 *
 * Created on 20 Aralik 2017 Çarsamba, 14:27
 */

// PIC12F675 Configuration Bit Settings
// CONFIG- Konfigürasyon ayarları
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC
                                // oscillator: I/O function on GP4/OSC2/CLKOUT
                                // pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-Up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // GP3/MCLR pin function select (GP3/MCLR pin
                                // function is digital I/O, MCLR internally
                                // tied to VDD)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code
                                // protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code
                                // protection is disabled)

// 4 Mhz dahili osilatör kullanılıyor.
#define _XTAL_FREQ 4000000
#include <xc.h>

void main(void) {

    OSCCALbits.CAL = 0b111111; // 4Mhz
    TRISIObits.TRISIO5 = 0; // Trigger
    TRISIObits.TRISIO4 = 1; // Echo
    TRISIObits.TRISIO1 = 0; // Çıkış

    ANSELbits.ANS = 0b0000;

    T1CONbits.T1CKPS = 0b00; // 1:1 Prescale Değeri
    T1CONbits.TMR1CS = 0; // Internal clock (Fosc/4) = 1Mz

    // Uzaklığı kaydetmek için değişken.
    int distance;

    while(1) {
        // TMR1'i sıfırla.
        TMR1H = 0;
        TMR1L = 0;

        // Trigger pinine 10uS lik sinyal gönder.
        GPIObits.GP5 = 1;
        __delay_us(10);
        GPIObits.GP5 = 0;

        // Sesin gidip gelme süresini TMR1'e kaydet.
        while(!GPIObits.GP4);
        T1CONbits.TMR1ON = 1;
        while(GPIObits.GP4);
        T1CONbits.TMR1ON = 0;

        // Ölçülen süreyi distance değişkenine ata.
        distance = (TMR1L | (TMR1H<<8));
        // Bu süreyi 88 değerine bölerek mesafeyi cm cinsinden hesapla.
        // 88 değeri hesaplamalar ve denemeler sonucu bulunmuştur.
        distance = ((double)distance / 88);

        // Mesafe 2cm ile 15cm arasında ise çıkışa 1 sinyalini gönder.
        if((distance > 2) && (distance < 20)) {
            GPIObits.GP1 = 1;
        }
        // Değilse çıkışa 0 sinyalini gönder.
        else {
            GPIObits.GP1 = 0;
        }
        __delay_ms(10); // 10ms bekle.
    }

    return;
}
