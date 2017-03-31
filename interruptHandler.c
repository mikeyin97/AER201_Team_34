#include <xc.h>
#include <stdio.h>
#include "configBits.h"
#include "constants.h"
#include "main.h"
#include "macros.h"
#include "I2C.h"
#include "lcd.h"

int timerCounter = 0;
int motorCounter = 0;

// Interrupt handler
void interrupt handler(void) {   
    di(); //disable interrupts
    
    if(TMR0IE && TMR0IF){ 
        if (timerCounter == 38){ //38 Overflows amount to 1 second
            updateCurrentTime();
            printTime();
            timerCounter = 0;
        }else{   
            timerCounter++;
        }
        TMR0IF = 0; //Clear Flag
    }
    
    if(TMR1IE && TMR1IF){
        if (motorCounter == getMotorCounterBasedOnBottlesLeft()){
            PORTCbits.RC1 = 1;
            motorCounter = 0;
        }else{
            PORTCbits.RC1 = 0;
            motorCounter += 1;
        }
        TMR1IF = 0; //Clear Flag
        
        float time = 10;
        unsigned int set_time = 65535-(int)((float)time*2000/8); //oscillation 8MHz, prescalar 1:8
        TMR1H = set_time >> 8;
        TMR1L = set_time & 0b11111111;
        TMR1ON = 1;
    }
    
    ei(); //enable interrupts
}