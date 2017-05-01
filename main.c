/*
 * File:   main.c
 * Author: True Administrator
 *
 * Created on July 18, 2016, 12:11 PM
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "configBits.h"
#include "constants.h"
#include "lcd.h"
#include "I2C.h"
#include "macros.h"
#include "main.h"
#include "interruptHandler.h"

#define __lcd_newline() lcdInst(0b11000000);

const char keys[] = "123A456B789C*0#D"; 
const int LEFT = 1, RIGHT = 2, ENTER = 3, BACK = 0;
int runInfo[6] = {40,10,2,3,3,2};
unsigned char time[7];
unsigned char lastRunTime[7];
int timerDisabled = 0;
int currentMainProgramScreen = 0;
int currentBottleBin = 0;
int currentBottle = 0;
int breakCounter = 0;
int BREAK_CONSTANT = 300;

void main(void) {
    initializePicSettings(); //Set up the PIC
    
    unsigned char userInput;
    while(1){
        userInput = getPressedKeypadKey(); //Wait for user to press a button
        if (userInput == keys[ENTER]){ //Enter selected subprogram
            enterSelectedSubProgramFromMainProgram();
        }else if (userInput == keys[LEFT]){ //Cycle the screen to the left
            cycleProgramScreenLeft(&currentMainProgramScreen,0,2);
            displayMainProgramLcdScreen();
        }else if (userInput == keys[RIGHT]){ //Cycle the screen to the right
            cycleProgramScreenRight(&currentMainProgramScreen,0,2);
            displayMainProgramLcdScreen();
        }
}
     
void initializePicSettings(){
    initializePicPins(); //Set up input/output pins and timers
    initializeI2C(10000); //RTC initialized with 100kHz clock
    di(); //Disable all interrupts
    initializeLcdScreen(); //Set up the LCD screen
}

void initializePicPins(){
    // Force internal oscillator operation at 8 MHz (pg. 7-39)
    OSCCON = 0b11110000; 
    // Set  OSCTUNE<6> to enable PLL (4x  phase-lock-loop) thereby setting internal oscillator frequency to 32 MHz
    OSCTUNEbits.PLLEN = 1; 
    
    //Set Input Pins
    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISC = 0x18; //Turn on  SDA and SCL
    TRISD = 0x01; 
    
    //Set Output Pins
    TRISE = 0x00; 
    
    //Clear LAT Settings
    LATA = 0x00;
    LATB = 0x00; 
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
    
    //Disable ADC
    ADCON0 = 0x00;  
    //Set PORTB Pins as Digital
    ADCON1 = 0xFF;    
    nRBPU = 0;
    
    CVRCON = 0x00; // Disable comparator voltage reference (pg. 239 of datasheet)
    CMCONbits.CIS = 0; // Connect C1 Vin and C2 Vin to RA0/AN0 and RA1/AN1 (pg. 233 datasheet)
    ADFM = 1; // Right justify A/D result
    
    //Set Timer0 Settings
    T0PS0=1; //Prescaler is divide by 256
    T0PS1=1;
    T0PS2=1;
    PSA=0;      //Timer Clock Source is from Prescaler
    T0CS=0;     //Prescaler gets clock from FCPU (5MHz)
    T08BIT=1;   //8 BIT MODE    
    
    //Set Timer1 Settings
    T1CON = 0b10000000; //16 bit operation
    T1CKPS1 = 1; //Prescalar 1:8
    T1CKPS0 = 1;
    T1OSCEN = 0; //timer1 oscillation off
    T1SYNC = 1; //Sync on
    TMR1CS = 0; //Timer mode
    
    float time = 10; //Set overflow time duration
    unsigned int set_time = 65535-(int)((float)time*2000/8); //oscillation 8MHz, prescalar 1:8
    TMR1H = set_time >> 8;
    TMR1L = set_time & 0b11111111;
    
     //Enable Timer0 and Timer1
    TMR0IE = 1; //Enable TIMER0 Interrupt
    TMR0IF = 1; //Clear TIMER0 Interrupt
    TMR0ON = 1; //Enable TIMER0
    TMR1IE = 0; //Disable TIMER1 Interrupt
    TMR1IF = 1; //Clear TIMER1 Interrupt
    TMR1ON = 1; //Enable TIMER1
    PEIE = 1; //Enable Peripheral Interrupts
}

//Set up LCD Screen
void initializeLcdScreen(){
    initializeLcdSettings();
    displayMainProgramLcdScreen(0); //Main program starting screen
}

//Get keypad input from user
unsigned char getPressedKeypadKey(){ 
    while(PORTBbits.RB1 == 0){}
    unsigned char keypress = (PORTB & 0xF0)>>4; // Read the 4 bit character code
    while(PORTBbits.RB1 == 1){ //Wait for user to release key
    }
    Nop();  //Apply breakpoint here because of compiler optimizations
    Nop();
    return keys[keypress]; //Return the pressed key
}

//Print out timer array in readable form
void printTime(){
    lcdInst(0b10000000);
    printf("%02x/%02x   ", time[5],time[4]);    //Print date in YY/MM/DD
    printf("%02x:%02x:%02x", time[2]+1,time[1],time[0]);    //HH:MM:SS
}

//Enter the correct operation when start button is pressed
void enterSelectedSubProgramFromMainProgram(){
    switch(currentMainProgramScreen){
                case 0: operation();
                        break;
                case 2: EEPROM();
                        break;
    }    
    displayMainProgramLcdScreen();
}

//Shift program screen left
void cycleProgramScreenLeft(int *programScreen, int min, int max){
    if (*programScreen == min){
        *programScreen = max;
    }else{
        *programScreen -= 1;
    }
}

//Shift program screen right
void cycleProgramScreenRight(int *programScreen, int min, int max){
    if (*programScreen == max){
        *programScreen = min;
    }else{
        *programScreen += 1;
    }
}

//On the RTC clock screen, turn on interrupts
void toggleRTCBasedOnCurrentMainProgramScreen(){
    if (currentMainProgramScreen == 1){
        ei();
    }else{
        di();
    }
}

//Update the time array with values of current time
void updateCurrentTime(){
    //Reset RTC memory pointer 
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    I2C_Master_Stop(); //Stop condition

    //Read Current Time
    I2C_Master_Start();
    I2C_Master_Write(0b11010001); //7 bit RTC address + Read
    for(unsigned char i=0;i<0x06;i++){
        time[i] = I2C_Master_Read(1);
    }
    time[6] = I2C_Master_Read(0);       //Final Read without ack
    I2C_Master_Stop();
}

//Convert timer array values into comparable integer
int getCurrentTimeInSeconds(){
    updateCurrentTime();
    int currentTime = 0;
    char str[2];
    sprintf(str, "%02x",time[0]);
    currentTime += atoi(str);
    sprintf(str, "%02x",time[1]);
    currentTime += atoi(str)*60;
    sprintf(str, "%02x",time[2]);
    currentTime += atoi(str)*3600;
    return currentTime;
}

//Detect if cap is on the bottle
int capIsOnBottle(){
    unsigned char rgbc[4];
    int v1, v2;
    
    //Clear RGBC values
    for (int i = 0; i < 4; i++){
        rgbc[i] = 0;
    }
    
    //Query each sensor for detection values
    setMultiplexerToSensor(2);
    v1 = storeRgbSensorInputIntegerValuesInto(&rgbc, 2);
    
    setMultiplexerToSensor(1);
    v2 = storeRgbSensorInputIntegerValuesInto(&rgbc, 1);
    
    __lcd_newline();
    printf("%i/%i",v2,v1);
    return v1 | v2;   
}

//Set SDA/SCL pins to certain sensor
void setMultiplexerToSensor(int num){
    if (num == 1){
        //Write to Multiplexer
        I2C_Master_Start();
        I2C_Master_Write(0b11100000);   //7bit address 0x70 + Write
        I2C_Master_Write(0b10000000);   //Write to cmdreg
        I2C_Master_Write(0b10000000);   //Enable control register 7 (Sensor 1)
        I2C_Master_Stop();
    }else if (num == 2){
        //Write to Multiplexer
        I2C_Master_Start();
        I2C_Master_Write(0b11100000);   //7bit address 0x70 + Write
        I2C_Master_Write(0b10000000);   //Write to cmdreg
        I2C_Master_Write(0b00000100);   //Enable control register 2 (Sensor 2)
        I2C_Master_Stop();
    }
}

//Get RGBC values from the sensors and determine if cap is on bottle
int storeRgbSensorInputIntegerValuesInto(unsigned char *colorValues[4], int sensorNum){
   //Write Start Condition
    I2C_Master_Start();
    I2C_Master_Write(0b01010010);   //7bit address 0x29 + Write
    I2C_Master_Write(0b10000000);   //Write to cmdreg + access enable reg
    I2C_Master_Write(0b00000011);   //Start RGBC and POWER 
    I2C_Master_Stop();
    
    //Colour data
    I2C_Master_Start();
    I2C_Master_Write(0b01010010);   //7bit address 0x29 + Write
    I2C_Master_Write(0b10110100);   //Write to cmdreg + access&increment clear low reg
    I2C_Master_Start();
    I2C_Master_Write(0b01010011);   //7bit address 0x29 + Read
    
    unsigned char red[2];
    unsigned char green[2];
    unsigned char blue[2];
    unsigned char clear[2];
    
    clear[1] = I2C_Master_Read(1);  //Read clear with acknowledge (low bit)
    clear[0] = I2C_Master_Read(1);  //High bit
    
    red[1] = I2C_Master_Read(1);    //Read red with acknowledge (low bit)
    red[0] = I2C_Master_Read(1);    //High bit
    
    green[1] = I2C_Master_Read(1);  //Read green red with acknowledge (low bit)
    green[0] = I2C_Master_Read(1);  //High bit
    
    blue[1] = I2C_Master_Read(1);   //Read blue red with acknowledge (low bit)
    blue[0] = I2C_Master_Read(0);   //High bit
 
    
    unsigned char colorValue[4];
    I2C_Master_Stop();              //Stop condition
    colorValue[0] = (red[0]<<8) | red[1];               //Concatenate the high and low bits
    colorValue[1] = (green[0]<<8) | green[1];
    colorValue[2] = (blue[0]<<8) | blue[1];
    colorValue[3] = (clear[0]<<8) | clear[1];
    
    int r1 = colorValue[0], g1 = colorValue[1], b1 = colorValue[2], c1 =colorValue[3];
    
    return analyzeRgbcForSensor(sensorNum, colorValue);
}

//Use formula to determine if cap is on the bottle
int analyzeRgbcForSensor(int sensorNum, unsigned char rgbc[4]){
    //Store in RGBC values
    int r1 = rgbc[0], g1 = rgbc[1], b1 = rgbc[2], c1 =rgbc[3];
    
    lcdClear();
    printf("%i/%i/%i/%i", r1,g1,b1,c1);   
    
    //Apply formulas to determine if cap is on bottle
    if (sensorNum == 2){
        if (r1 > 1.2*g1 && currentBottle == 0){
            return 1;
        }else{
            return 0;
        }
    }else if (sensorNum == 1){
        if (b1 > 1.2*g1 && (r1 + 2*g1) < 350 && currentBottle == 1){
            return 1;        
        }else if (r1 > 1.2*g1 && currentBottle == 0){
            return 1;
        }else{
            return 0;
        }
    }
}

//activate autonomous sensor testing
void operation(){
    activateDCmotors();
    
    //Clear variables
    int bottle_type = 0;
    int bottle_cap = 0;
    int i;
    for (i=0;i<6;i++){  //Clear run information
        runInfo[i] = 0;
    }
    lcdClear();
    printf("...Operating...");
    int startingTime = getCurrentTimeInSeconds();
    
    while (runInfo[1] < 10){    //Run operation until 10 bottles are reached
        __lcd_newline();
        while(PORTDbits.RD0 == 0){ //Terminate operation at 170 seconds
            if (getCurrentTimeInSeconds() - startingTime > 170){
                endOperation(startingTime);
                return;
            }
        }
        if (PORTDbits.RD0){ //Microswitch Sensor
            __lcd_newline();
            printf("??? INSIDE   ");
            __delay_ms(500);
            retractSensorAreaArm();
            __delay_ms(2000);
            if (PORTAbits.RA1 == 0){ //Photovoltaic Sensor
                __lcd_newline();
                printf("YOP");
                currentBottle = 0;
                bottle_type = 0;
            }else{
                __lcd_newline();
                currentBottle = 1;
                printf("ESKA");
                bottle_type = 1;
            }
            
            if (capIsOnBottle()){ //RGB Sensors
                printf(" w/ Cap    ");
                bottle_cap = 0;
            }else{
                printf(" w/o Cap    ");
                bottle_cap = 1;
            }
            
            //Calculate run info labels
            runInfo[1] += 1;
            int bottle = bottle_type*2 + bottle_cap + 2;
            runInfo[bottle] += 1;
            
            //Rotate correct bin under sorting area
            int requiredBottleBin = bottle - 2;
            int numberRotationsNeeded;
            if (requiredBottleBin >= currentBottleBin){
                numberRotationsNeeded = requiredBottleBin - currentBottleBin;
            }else if (requiredBottleBin < currentBottleBin){
                numberRotationsNeeded = requiredBottleBin - currentBottleBin + 4;
            }
            moveBinServo(numberRotationsNeeded); //move correct bin under drop area
            currentBottleBin = requiredBottleBin;
            
            di();
            extendSensorAreaArm(); //push bottle into bin using servo
            extendSensorAreaArm();
            ei();
        }          
    }
    //End operation
    endOperation(startingTime);
}

void endOperation(int startingTime){
    disableDCmotors(); //Stop main sorting motors
    //Display complete message
    lcdClear();
    printf("COMPLETED!!!     ");
    __lcd_newline();
    printf("A to back       ");  
    int endingTime = getCurrentTimeInSeconds(); //Obtain time taken during run
    runInfo[0] = endingTime - startingTime;
    currentBottleBin = 0;
    for (int i = 0; i<7;i++){ //Save date and time of run
        lastRunTime[i] = time[i];
    }
    saveEEPROM(); //Save data into EEPROM
    while(1){
        unsigned char temp = getPressedKeypadKey();    //Wait for user to return to main menu
        if (temp == keys[3]){
            break;
        }
    }
}

//Activate DC motors
void activateDCmotors(){
    TMR0IE = 0; //Disable TIMER0
    TMR1IE = 1; //Enable TIMER1
    ei(); //Allow general interrupts
}

//Disable DC motors
void disableDCmotors(){
    di(); //Disable general interrupts
    TMR0IE = 1; //Enable TIMER0
    TMR1IE = 0; //Disable TIMER1
    PORTCbits.RC1 = 0;
}

//Check if 170s have passed since run has started
int timeIsStillLeft(int startingTime){
    int currentTime = getCurrentTimeInSeconds();
    if ((currentTime - startingTime) > 150){
        return 0;
    }else{
        return 1;
    }
}

//Activate servo to move to specific bin
void moveBinServo(int num90Rotations){
    while (num90Rotations != 0){
        PORTDbits.RD1 = 1;
        __delay_us(2200);
        PORTDbits.RD1 = 0;
        __delay_us(17800);
        if (PORTAbits.RA0 == 0){
            while (PORTAbits.RA0 == 0){
                PORTDbits.RD1 = 1;
                __delay_us(2200);
                PORTDbits.RD1 = 0;
                __delay_us(17800);
            }
            num90Rotations -= 1;
            __delay_ms(100);
        }
    }
    __delay_ms(200);
}

//Retract piston arm
void retractSensorAreaArm(){
     unsigned int i;
    for(i=0;i<20;i++){
        PORTCbits.RC2 = 1;
        __delay_us(2900);
        PORTCbits.RC2 = 0;
        __delay_us(17100);
    }
    __delay_ms(500);
}

//Extend piston arm
void extendSensorAreaArm(){
    unsigned int i;
    for(i=0;i<20;i++){
        PORTCbits.RC2 = 1;
        __delay_us(1800);
        PORTCbits.RC2 = 0;
        __delay_us(18200);
    } 
    __delay_ms(500);
}

//Activate servo to push bottle off the sensor area
void pushBottle(){
    retractSensorAreaArm();
    __delay_ms(1000);
    extendSensorAreaArm();
}   

//Determine DC motor speed
int getMotorCounterBasedOnBottlesLeft(){
    breakCounter += 1;
    if (breakCounter < BREAK_CONSTANT){
        return 4 + (runInfo[1]/2);
    }else{
        if (breakCounter > 350){
            breakCounter = 0;
        }
        return 20000;
    } 
}

//Activate EEPROM operations
void EEPROM(){
    int screen_state = 0;
    update_eeprom_screen_state(screen_state);
    
    while(1){
        unsigned char temp = getPressedKeypadKey(); //Cycle screens/exit/enter run info depending on user input
        if (temp == keys[ENTER]){ //"A"
            int storage[6], timeStorage[7], i;
            for (i = 0; i <6; i++){ //Save current runData to enter EEPROM saved run
                storage[i] = runInfo[i];
            }   
            for (i = 0; i<7; i++){
                timeStorage[i] = lastRunTime[i];
            }
            getEEPROM(screen_state);
            past_run_access();  //Enter EEPROM Display
            update_eeprom_screen_state(screen_state);
             for (i = 0; i <6; i++){    //Return runData to what it is
                runInfo[i] = storage[i];
            }   
              for (i = 0; i<7; i++){
                lastRunTime[i] = timeStorage[i];
            }
        }else if (temp == keys[BACK]){ //"D" Go Back
            break;    
        }else if (temp == keys[LEFT]){ //"4" Cycle Left
            cycleProgramScreenLeft(&screen_state,0,3);
            update_eeprom_screen_state(screen_state);
        }else if (temp == keys[RIGHT]){ //"6" Cycle Right
            cycleProgramScreenRight(&screen_state,0,3);
            update_eeprom_screen_state(screen_state);
        } 
    }
}

//Access past run information
void past_run_access(){
    int screen_state = 6;
    update_past_run_screen_state(screen_state);
    while(1){
        unsigned char temp = getPressedKeypadKey(); //Cycle/exits screens based on user input
        if (temp == keys[BACK]){ //"D"
            break;    
        }else if (temp == keys[LEFT]){ //"4"
            cycleProgramScreenLeft(&screen_state,0,6);
            update_past_run_screen_state(screen_state);
        }else if (temp == keys[RIGHT]){ //"6"
            cycleProgramScreenRight(&screen_state,0,6);
            update_past_run_screen_state(screen_state);
        } 
    }
}

//Display main program screens 
void displayMainProgramLcdScreen(){
    lcdClear();
    switch(currentMainProgramScreen){   
        case 0:
            lcdClear();
            printf("Begin Sorting");
            break;
        case 1:
            updateCurrentTime();
            printTime();
            break;
        case 2:
            lcdClear();
            printf("Access Past Runs");
            break;
    }
    __lcd_newline();
    printf("<2  Start: A  3>");
    toggleRTCBasedOnCurrentMainProgramScreen();
}

//Updates EEPROM screen display depending on input
void update_eeprom_screen_state(int num){
    lcdClear();
    switch(num){
        case 0:
            lcdClear();
            printf("Recent Run");
            break;
        case 1:
            lcdClear();
            printf("   Past Run 2  ");
            break;
        case 2:
            lcdClear();
            printf("   Past Run 3  ");
            break;
        case 3:
            lcdClear();
            printf("   Past Run 4  ");
            break;    
    }
    __lcd_newline();
    printf("<2  Nav: A/1  3>");
}

//Updates past run screen display depending on input
void update_past_run_screen_state(int num){
    lcdClear();
    switch(num){
        case 0:
            lcdClear();
            printf("Time taken: %is", runInfo[0]);
            break;
        case 1:
            lcdClear();
            printf("# Bottles: %i", runInfo[1]);
            break;
        case 2:
            lcdClear();
            printf("Yop w/Cap: %i", runInfo[2]);
            break;
        case 3:
            lcdClear();
            printf("Yop w/o Cap: %i", runInfo[3]);
            break; 
        case 4:
            lcdClear();
            printf("Eska w/ Cap: %i", runInfo[4]);
            break;
        case 5:
            lcdClear();
            printf("Eska w/o Cap: %i", runInfo[5]);
            break;  
        case 6:
            lcdClear();
            printf("%02x/%02x   ", lastRunTime[5], lastRunTime[4]);    //Print date in YY/MM/DD
            printf("%02x:%02x:%02x", lastRunTime[2] + 1, lastRunTime[1], lastRunTime[0]);    //HH:MM:SS
            break;
    }
    __lcd_newline();
    printf("<2  Back:  1  3> ");
}

//Save the EEPROM data for the run
void saveEEPROM(){
    int i;
    int temp;
    for (i = 0; i<13; i++){
        temp = Eeprom_ReadByte(12*i+2*200);
        Eeprom_WriteByte(12*i+3*200,temp);
    } 
    for (i = 0; i<13; i++){
        temp = Eeprom_ReadByte(12*i+1*200);
        Eeprom_WriteByte(12*i+2*200,temp);
    } 
    for (i = 0; i<13; i++){
        temp = Eeprom_ReadByte(12*i+0*200);
        Eeprom_WriteByte(12*i+1*200,temp);
    }
    for (i = 0; i<6; i++){
        Eeprom_WriteByte(12*i,runInfo[i]);
    }
    for (i = 0; i<7; i++){
        Eeprom_WriteByte(12*(6+i),lastRunTime[i]);
    }
}

//Obtain specific EEPROM data
void getEEPROM(int num){
    int i;
    for (i = 0; i<6; i++){
        runInfo[i] = Eeprom_ReadByte(12*i+num*200);
    }
    for (i = 0; i<7; i++){
        lastRunTime[i] = Eeprom_ReadByte(12*(6+i)+num*200);
    }
}

//! @brief      Reads a single byte of data from the EEPROM.
//! @param      address     The EEPROM address to write the data to (note that not all
//!                         16-bits of this variable may be supported).
//! @returns    The byte of data read from EEPROM.
//! @warning    This function does not return until read operation is complete.
uint8_t Eeprom_ReadByte(uint16_t address)
{

    // Set address registers
    EEADRH = (uint8_t)(address >> 8);
    EEADR = (uint8_t)address;

    EECON1bits.EEPGD = 0;       // Select EEPROM Data Memory
    EECON1bits.CFGS = 0;        // Access flash/EEPROM NOT config. registers
    EECON1bits.RD = 1;          // Start a read cycle

    // A read should only take one cycle, and then the hardware will clear
    // the RD bit
    while(EECON1bits.RD == 1);

    return EEDATA;              // Return data

}

//! @brief      Writes a single byte of data to the EEPROM.
//! @param      address     The EEPROM address to write the data to (note that not all
//!                         16-bits of this variable may be supported).
//! @param      data        The data to write to EEPROM.
//! @warning    This function does not return until write operation is complete.
void Eeprom_WriteByte(uint16_t address, uint8_t data)
{    
    // Set address registers
    EEADRH = (uint8_t)(address >> 8);
    EEADR = (uint8_t)address;

    EEDATA = data;          // Write data we want to write to SFR
    EECON1bits.EEPGD = 0;   // Select EEPROM data memory
    EECON1bits.CFGS = 0;    // Access flash/EEPROM NOT config. registers
    EECON1bits.WREN = 1;    // Enable writing of EEPROM (this is disabled again after the write completes)

    // The next three lines of code perform the required operations to
    // initiate a EEPROM write
    EECON2 = 0x55;          // Part of required sequence for write to internal EEPROM
    EECON2 = 0xAA;          // Part of required sequence for write to internal EEPROM
    EECON1bits.WR = 1;      // Part of required sequence for write to internal EEPROM

    // Loop until write operation is complete
    while(PIR2bits.EEIF == 0)
    {
        continue;   // Do nothing, are just waiting
    }

    PIR2bits.EEIF = 0;      //Clearing EEIF bit (this MUST be cleared in software after each write)
    EECON1bits.WREN = 0;    // Disable write (for safety, it is re-enabled next time a EEPROM write is performed)
}
