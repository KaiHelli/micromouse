
/*! \file   adc.c
 * Author: Alexander Lenz
 *
 * Created on 10 Oct 2018, 16:53
 */

#include "adc.h"
#include "interrupts.h"
#include <xc.h>




void startADC1(void)
{
        AD1CON1bits.ADON=1; //set on-bit
        AD1CON1bits.ASAM=1;
}



void setupADC1()
{
    AD1CON1bits.ADON = 0;   // disable ADC1 module

    AD1CON1bits.ADSIDL=1;   // no sampling in idle mode
    AD1CON1bits.ADDMABM=1;  // DMA channels are written in order of conversion
    AD1CON1bits.AD12B=1;    // 12-bit operation
    AD1CON1bits.FORM=0b00;  // format is unsigned integer
    AD1CON1bits.SSRC=0b111; // see comment below for options
            
             //   SSRC (Sample Clock Source Select bits) options:
             //   111 = Internal counter ends sampling and starts conversion (auto-convert)
             //   110 = Reserved
             //   101 = Motor Control PWM2 interval ends sampling and starts conversion
             //   100 = GP timer (Timer5 for ADC1) compare ends sampling and starts conversion
             //   011 = Motor Control PWM1 interval ends sampling and starts conversion
             //   010 = GP timer (Timer3 for ADC1) compare ends sampling and starts conversion
             //   001 = Active transition on INT0 pin ends sampling and starts conversion
             //   000 = Clearing sample bit ends sampling and starts conversion

    AD1CON1bits.SIMSAM=1;   // simultaneous sampling in 10 bit mode unimplemented in 12bit mode
    AD1CON1bits.ASAM=0;     // sampling starts not immediately after last conversion but when SAMP bit is set
    AD1CON1bits.SAMP=0;     // bit is automatically set if ASAM=1 (auto-sampling)
    //AD1CON1bits.DONE=x;   // status bit set by hardware when conversion done

    //ADCON2

    AD1CON2bits.VCFG=0b000; // select Avdd and Avss as internal reference voltage
    AD1CON2bits.CSCNA=1;    // enable analog input SCAN on channel 0
    AD1CON2bits.CHPS=0b11;  // important for 10 bit mode but unimplemented in 12-bit mode
    //AD1CON2bits.BUFS=x;   // status bit indicates which buffer is currently written (only if BUFM=1)
    AD1CON2bits.SMPI=2;     // !!!CHANGE HERE!!! Selects Increment Rate for DMA Addresses bits or number of sample/conversion operations per interrupt
                            // set to 2 since we scan 3 channels
    AD1CON2bits.BUFM=0;     // always fill buffer starting at address 0x00
    AD1CON2bits.ALTS=0;     // always use channel A and do not alternate

    //ADCON3
    AD1CON3bits.ADRC=0;     // use internal clock source
    AD1CON3bits.SAMC=0x06;  // auto sample time bits, number of Tad
                            // only valid when SSRC=0b111
                            // with tad = 250ns -> 6 * tad = 6 * 250ns = 1.5us
    AD1CON3bits.ADCS=0x09 ; // 8-bits to derive ADC clock when ADRC = 0
                            // tad = tcy * (ADCS<7:0> + 1) only bits ADCS<5:0> are usable
                            // fcy = 40MHz -> tcy = 25ns -> tad = (9 + 1) * 25ns = 250ns

    //ADCON4
    AD1CON4bits.DMABL=0b000;    // <2:0>: Selects Number of DMA Buffer Locations per Analog Input bits
                                // configured for 1 word of buffer to each analog input

    // set all pins to digital IO
    AD1PCFGL = 0xFFFF;
    
    // set distance sensor pins to be analog
    AD1PCFGLbits.PCFG4 = 0;
    AD1PCFGLbits.PCFG5 = 0;
    AD1PCFGLbits.PCFG6 = 0;
    
    // disable all channels for input scan selection
    AD1CSSL = 0x0000;
    
    // enable channel scan for distance sensors
    AD1CSSLbits.CSS4 = 1;
    AD1CSSLbits.CSS5 = 1;
    AD1CSSLbits.CSS6 = 1;

    AD1CHS123bits.CH123NA = 0b00;   // negative input for S/H 123 is Vref- (only in 10bit mode)
    AD1CHS123bits.CH123SA = 1 ;     // sample channel 4 and 5  on S/h 2 and 3 (only in 10bit mode)
    
    //interrupt configuration
    IFS0bits.AD1IF = 0;
    IEC0bits.AD1IE = 0;
    IPC3bits.AD1IP = IP_ADC;
}

