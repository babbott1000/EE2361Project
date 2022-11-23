/*
 * File:   main.c
 * Author: Ben Abbott
 *
 * Created on November 22, 2022, 12:06 PM
 */

#include <p24FJ64GA002.h>

#include "xc.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "main.h"




// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL      // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))



volatile unsigned char buffer[32];
volatile unsigned char front = 0;
volatile unsigned char back = 0;

// This code is for interfacing with the MCP4821 DAC unit, which has an SPI interface.
// /LDAC => GROUND
// /SHDN => VDD
// SCK and SDI => RP2 and RP3
// /CS => RB6


#define SIGMASK 0x0FFF //0000 1111 1111 1111
#define CONBITS 0x3000 //0011 0000 0000 0000

// Create a variable to store the output signal
volatile uint16_t signal = 0;

// Create a flag to store if SPI is transmitting
volatile uint8_t SPI_TRMT = 0;
volatile uint8_t parity = 0;

int main()
{
    
    setup();
    setupSPI();
    
    // Timer causes issues
//    setupTMR();
    
    
    uint16_t t = 0;
    while(1) {
        signal = 2047+2047*sin((float)t*6.28318531/256.0);
        
        t++;
        
        if((t % 256) == 0) {
            t = 0;
        }
        
        LATBbits.LATB6 = 0;
      
        asm("nop");
        asm("nop");
        
        signal &= SIGMASK;
        signal |= CONBITS;

        SPI_TRMT = 1;
        
        parity = !parity;
        
        SPI1BUF = signal;
        
        LATBbits.LATB15 = 1;

        // Wait until SPI is done transmitting
        while(SPI_TRMT) continue;
        LATBbits.LATB15 = 0;

        
        // Pull chip select high
        LATBbits.LATB6 = 1;
        
        // Wait a bit
        delay_us(5);
        
        
//        delay_ms(2000);
    }
}


void setup(void) {
    // Set up the clock prescalar
    CLKDIVbits.RCDIV = 0;
    // Set all pins to digital
    AD1PCFG = 0x9fff;
    //  Set all pins as inputs
    TRISB = 0xffff;
    
    // Set pin RB6 as an output
    TRISBbits.TRISB6 = 0;
    // Set chip select high
    LATBbits.LATB6 = 1;
    
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;
}

void setupTMR(void){
    // Set up the timer with no prescalar
    T3CON = 0;
    // Set the period register to 4000
    PR3 = 4000;
    // Set the timer count register to 0
    TMR3 = 0;
  
    // CLaer the interrupt flags and enable interrupts
    IFS0bits.T3IF = 0;
    IPC2bits.T3IP = 4;
    IEC0bits.T3IE = 1;
    
    // Turn the timer on
    T3CONbits.TON = 1;
}



void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void)
{
    
//    PORTBbits.RB7 = ~PORTBbits.RB7;
//    
//    signal = 0xAAAA;
//
//    
//    signal &= SIGMASK;
//    signal |= CONBITS;
//
//    IFS0bits.T2IF = 0;
//    SPI1BUF = signal;
//
//    while(_SPI1IF == 0);
//    _SPI1IF = 0;
//    
//    PORTBbits.RB7 = ~PORTBbits.RB7;
//  
//    signal += 41;       // the next "step" value.
}



void __attribute__((__interrupt__,__auto_psv__)) _SPI1Interrupt(void)
{
    SPI_TRMT = 0;
    
    parity = !parity;
    
    
    SPI1STATbits.SPIROV = 0;
    
    IFS0bits.SPI1IF = 0;
}


void setupSPI(void) {
    
    IFS0bits.SPI1IF = 0;
    IPC2bits.SPI1IP = 1;
    IEC0bits.SPI1IE = 1;
    
    // Peripheral Pin Select
    __builtin_write_OSCCONL(OSCCON & 0xbf);
    RPOR4bits.RP8R = 8; // SPI Clock
    RPOR4bits.RP9R = 7; // SPI Data
    __builtin_write_OSCCONL(OSCCON | 0x40);
    

    SPI1CON1 = 0x0000; // Clear the SPI Control Register
    SPI1CON1bits.MSTEN = 1;  // Master Mode
    SPI1CON1bits.MODE16 = 1; // 16-bit
    SPI1CON1bits.CKE = 1;
    SPI1CON1bits.CKP = 0;
    SPI1CON1bits.SPRE = 0b000; // Secondary Prescalar = 8;
    SPI1CON1bits.PPRE = 0b10;  // Primary Prescalar = 64;
    
    
    SPI1CON2 = 0x0000;
    
//    // For enhanced mode
//    SPI1CON2bits.SPIBEN = 1;
//    SPI1STAT = 0x0000;
//    SPI1STATbits.SISEL = 0b101; // IF set when last bit is shifted out
//                                // That means the SPI xfer is complete.
    
    SPI1STATbits.SPIROV = 0;
    
    SPI1STATbits.SPIEN = 1;
}

void delay_ms(uint32_t ms) {
    while(ms-- > 0){
        asm("repeat #15998");
        asm("nop");
    }
    return;
}

void delay_us(uint32_t us) {
    while(us-- > 0){
        asm("repeat #12");
        asm("nop");
    }
    return;
}