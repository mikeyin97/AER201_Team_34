/*
 * File:   main.c
 * Author: True Administrator
 *
 * Created on July 18, 2016, 12:11 PM
 */

// cleaned up code
// added interrupt code
// experimented with bottom bin servo -> magnets, getting it to 90 degrees
//testing servo code with bin
//testing pwm for dc with interrupts

//sat tested bottom bin code
//sensor area push
//cleaned up more code

//tested rgb alone
//tested multiplexr
//tested it with soldiered stuff
//tested it with bottles inside
//right side then left side

//testing data for the light sensors: (RGBC)

//right

//yopback: 
//51/75/117/174
//52/67/111/164
//98/86/236/25
//23/85/162/217
//57/104/182/24

//5/3/59/252
//3/6/99/45

//225/224/27/151
//228/243/244/137
//155/159/195/188
//252/242/43/208
//226/253/61/215
//107/140/14/201

//131/39/162/14
//241/24/130/75

//

//yopcap: r>100 g<100 c<170

//133/69/18/145
//95/85/27/138
//131/67/16/140
//124/70/19/139
//138/49/253/107
//121/41/245/78
//116/39/243/66
//127/68/16/137

//63/20/226/239
//127/41/245/81
//93/85/27/138

//134/50/253/104
//208/65/11/113
//130/43/247/88

//yopno:

//77/156/79/254
//80/160/78/2
//105/189/108/83
//105/185/101/74

//61/167/91/21
//79/170/88/21
//75/177/100/38
//62/169/93/13

//72/155/76/243
//81/163/84/10
//68/159/87/2

//eskaback:

//215/80/24/20
//144/9/185/22
//249/104/48/84
//254/138/89/163
//218/62/236/217
//236/104/54/80

//eskacap:  r>200 g>100 b>100 c>100

//226/108/119/134
//233/114/121/147
//240/115/104/142
//232/115/123/149
//239/123/135/175
//240/124/137/175

//eskano:

//200/42/217/135
//97/201/129/111
//69/176/109/36
//178/28/204/89
//169/16/194/61
//104/212/140/135
//88/196/130/94

//left

//yopback: c < 250 

//105/195/123/255
//34/136/52/255
//91/154/50/255
//161/233/34/72
//195/237/33/255
//11/8/254/255
//243/255/137/255
//

//yopcap:

//32/11/241/248
//188/40/91/100
//198/104/109/193
//220/109/__/__
//221/158/68/85
//218/176/95/163

//189/13/196/119


//50/104/66/232
//46/184/95/57
//58/206/125/52
//81/227/51/95
//10/84/249/78
//24/145/45/241
//82/112/1/175

//139/172/27/108
//101/169/57/226
//169/234/217/88
//131/254/175/214
//103/142/189/25
//130/202/171/238
//101/231/139/82
//57/117/64/76
//152/198/207/126

//50/20/11/248
//217/67/105/249
//240/116/255/131
//161/122/254
//136/220/104/29
//69/253/252/63
//136/115/65/254
//107/36/171/208
//66/72/209/166

//yopno:

//94/67/26/158
//28/115/239/63
//190/239/58/197
//243/73/168/122
//221/36/157/25

//110/246/71/120
//74/173/105/36
//47/101/141/78
//184/71/54/133
//217/134/148/44
//8/94/132/74
//155/80/166/120
//14/54/49/126
//121/194/90/254
//92/203/228/187
//187/173/227/100
//54/194/249/125


//eskaback:

//127/62/240/172
//176/230/0/15
//254/188/87/16
//168/87/97/144
//152/12/141/11

//eskacap:

//46/156/175/115
//187/133/10/163
//142/46/22/94
//70/167/33/93
//232/175/73/93
//76/109/16/88

//eskano:

//36/192/137/145
//164/54/237/244
//113/7/181/69
//44/212/157/194
//70/132/217/34
//26/192/144/149

//new round yop tests

//yopcap

//147/74/252/144
//247/39/225/130
//127/88/16/100
//236/60/242/191
//167/4/195/170
//184/19/212/40
//209/161/97/83

//78/113/39/135
//83/155/96/60


//146/146/78/247
//200/180/96/192

//after support
//51/249/169/113
//83/243/165/130
//124/4/184/210
//141/54/234/107      
//100/7/185

//yop: R>100 G<100 C>100 or R<100 G>230 B>150 C>120

//yopback:

//161/172/161/245
//149/211/153/31
//164/213/160/139

//yopnocap:

//221/126/231/89
//215/83/115/28
//124/201/247/167

//eskacap:

//40/17/250/79
//79/112/197/143
//20/7/247/33
//229/6/56/48
//1/47/131/191
//65/11/185
//14/218/130/125

//eskaback:

//25/221/118/148
//57/12/150/246
//240/212/97/56
//117/81/209/174
//84/43/200/99
//90/37/172/85

//eskano:

//58/29/173/45
//144/77/209/193
//198/155/252/146
//20/250/143/186
//96/36/169/69
//55/254/132/207


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
const int LEFT = 4, RIGHT = 6, ENTER = 3, BACK = 0;
int runInfo[6] = {40,10,2,3,3,2};
unsigned char time[7];
unsigned char lastRunTime[7];
int timerDisabled = 0;
int currentMainProgramScreen = 0;
int currentBottleBin = 0;
int currentBottle = 0;

int randomSpeed = 0;
int lastSortedBottleTime = 0;

void main(void) {
    initializePicSettings(); //Set up the PIC
    
    unsigned char userInput;
    while(1){
        userInput = getPressedKeypadKey(); //Wait for user to press a button
        if (userInput == keys[ENTER]){ //Enter selected subprogram
            enterSelectedSubProgramFromMainProgram();
        }else if (userInput == keys[LEFT]){ //Cycle the screen to the left
            cycleProgramScreenLeft(&currentMainProgramScreen,0,3);
            displayMainProgramLcdScreen();
        }else if (userInput == keys[RIGHT]){ //Cycle the screen to the right
            cycleProgramScreenRight(&currentMainProgramScreen,0,3);
            displayMainProgramLcdScreen();
        }
        else if (userInput == keys[0]){
            moveBinServo(1);
        }
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

void initializeLcdScreen(){
    initializeLcdSettings();
    displayMainProgramLcdScreen(0); //Main program starting screen
}

unsigned char getPressedKeypadKey(){
    while(PORTBbits.RB1 == 0){}
    unsigned char keypress = (PORTB & 0xF0)>>4; // Read the 4 bit character code
    while(PORTBbits.RB1 == 1){ //Wait for user to release key
    }
    Nop();  //Apply breakpoint here because of compiler optimizations
    Nop();
    return keys[keypress]; //Return the pressed key
}

void printTime(){
    lcdInst(0b10000000);
    printf("%02x/%02x   ", time[5],time[4]);    //Print date in YY/MM/DD
    printf("%02x:%02x:%02x", time[2]+1,time[1],time[0]);    //HH:MM:SS
}

void enterSelectedSubProgramFromMainProgram(){
    switch(currentMainProgramScreen){
                case 0: operation();
                        break;
                case 1: pushBottle();
                        break;
                case 2: EEPROM();
                        break;
                case 3: past_run_access();
                        break;
    }    
    displayMainProgramLcdScreen();
}

void cycleProgramScreenLeft(int *programScreen, int min, int max){
    if (*programScreen == min){
        *programScreen = max;
    }else{
        *programScreen -= 1;
    }
}

void cycleProgramScreenRight(int *programScreen, int min, int max){
    if (*programScreen == max){
        *programScreen = min;
    }else{
        *programScreen += 1;
    }
}

void toggleRTCBasedOnCurrentMainProgramScreen(){
    if (currentMainProgramScreen == 1){
        ei();
    }else{
        di();
    }
}

//update time array
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

int getCurrentTimeInSeconds(){
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

int capIsOnBottle(){
    unsigned char rgbc[4];
    int v1, v2;
    
    for (int i = 0; i < 4; i++){
        rgbc[i] = 0;
    }
    
    setMultiplexerToSensor(2);
    v1 = storeRgbSensorInputIntegerValuesInto(&rgbc, 2);
    
    setMultiplexerToSensor(1);
    v2 = storeRgbSensorInputIntegerValuesInto(&rgbc, 1);
    
    //__lcd_newline();
    //printf("        %i/%i",v1,v2);
    return v1 | v2;   
}

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
    
    //lcdClear();
    //printf("%i/%i/%i/%i%i", r1,g1,b1,c1,currentBottle); 
    
    return analyzeRgbcForSensor(sensorNum, colorValue);
}

//false pos: 226/110/229/252, 183/28/207/90, 133/20/231/55, 167/46/254/152
//bad neg:

//missed pos: 11/206/122/60
//62/189/109/160
//118/193/114/79
//22/169/78/206
//167/210/131/158
//57/241/163/104
//104/243/139/204
//85/53/213/55
//84/254/175/153
//67/247/171/129

int analyzeRgbcForSensor(int sensorNum, unsigned char rgbc[4]){
    int r1 = rgbc[0], g1 = rgbc[1], b1 = rgbc[2], c1 =rgbc[3];
    
    //lcdClear();
    //printf("%i/%i/%i/%i", r1,g1,b1,c1);   
    
    if (sensorNum == 2){
        if (r1 > 190 && g1 > 100 && b1 > 100 && c1 > 100 && currentBottle == 1){
            return 1;
        }else if (r1 > 100 && g1 < 100 && c1 < 170 && currentBottle == 0){
            return 1;
        }else{
            return 0;
        }
    }else if (sensorNum == 1){
        if (((r1 > 80 && b1+g1 < 260 && c1 > 70) | (r1 < 120 && g1 > 180 && c1 > 80)) && currentBottle == 0){
            return 1;        
        }else if (r1+g1+b1 < 350 && currentBottle == 1){
            return 1;
        }else{
            return 0;
        }
    }
}

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
            printf("Access EEPROM");
            break;
        case 3:
            lcdClear();
            printf("Access Last Run ");
            break;    
    }
    __lcd_newline();
    printf("<4  Start: A  6>");
    toggleRTCBasedOnCurrentMainProgramScreen();
}

//updates EEPROM screen display depending on input
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
    printf("<4  Nav: A/1  6>");
}

//updates past run screen display depending on input
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
            printf("%02x:%02x:%02x", lastRunTime[2], lastRunTime[1], lastRunTime[0]);    //HH:MM:SS
            break;
    }
    __lcd_newline();
    printf("<4  Back:  1  6> ");
}

//activate autonomous sensor testing
void operation(){
    activateDCmotors();

    int bottle_type = 0;
    int bottle_cap = 0;
    int i;
    for (i=0;i<6;i++){  //Clear run information
        runInfo[i] = 0;
    }
    lcdClear();
    printf("...Operating...");
    updateCurrentTime();
    int startingTime = getCurrentTimeInSeconds();
    
    lastSortedBottleTime = getCurrentTimeInSeconds();
    
    while (runInfo[1] < 10){    //Run operation until 10 bottles are reached
        __lcd_newline();
        //printf("NO BOTTLE    ");
        while(PORTDbits.RD0 == 0){}  //activate fail-safes
        if (PORTDbits.RD0){ //Microswitch Sensor
            __lcd_newline();
            printf("??? INSIDE   ");
            __delay_ms(500);
            retractSensorAreaArm();
            __delay_ms(500);
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
            runInfo[1] += 1;
            int bottle = bottle_type*2 + bottle_cap + 2;
            runInfo[bottle] += 1;
            
            
            __delay_ms(1000);
            
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
            ei();
            
            lastSortedBottleTime = getCurrentTimeInSeconds();
        }          
    }
    disableDCmotors(); //Stop main sorting motors
    lcdClear();
    printf("COMPLETED!!!     ");
    __lcd_newline();
    printf("A to back       ");
    updateCurrentTime();        //Obtain time taken during run
    int endingTime = getCurrentTimeInSeconds();
    runInfo[0] = endingTime - startingTime;
    for (i = 0; i<7;i++){ 
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

void activateDCmotors(){
    TMR0IE = 0; //Disable TIMER0
    TMR1IE = 1; //Enable TIMER1
    ei(); //Allow general interrupts
}

void disableDCmotors(){
    di(); //Disable general interrupts
    TMR0IE = 1; //Enable TIMER0
    TMR1IE = 0; //Disable TIMER1
}

int timeIsStillLeft(int startingTime){
    int currentTime = getCurrentTimeInSeconds();
    if ((currentTime - startingTime) > 150){
        printf("ayylmao");
        return 0;
    }else{
        return 1;
    }
}

//activate servo to move to specific bin
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
            //lcdClear();
            //printf("%i",num90Rotations);
        }
    }
    __delay_ms(300);
}

void retractSensorAreaArm(){
     unsigned int i;
    for(i=0;i<20;i++){
        PORTCbits.RC2 = 1;
        __delay_us(2800);
        PORTCbits.RC2 = 0;
        __delay_us(17200);
    }
    __delay_ms(500);
}


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


//activate servo to push bottle off the sensor area
void pushBottle(){
    retractSensorAreaArm();
    __delay_ms(1000);
    extendSensorAreaArm();
}   

int getMotorCounterBasedOnBottlesLeft(){
    /*updateCurrentTime();
    int currentTime = getCurrentTimeInSeconds();
    lcdClear();
    printf("%i", currentTime - lastSortedBottleTime);
    if ((currentTime - lastSortedBottleTime) > 20){
        lcdClear();
        printf("project review");
        lastSortedBottleTime = currentTime;
        return rand() % 5 + 5;
    }*/
    return 5 + (runInfo[1]/2);
}

//activate EEPROM operations
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
            past_run_access();
            update_eeprom_screen_state(screen_state);
             for (i = 0; i <6; i++){
                runInfo[i] = storage[i];
            }   
              for (i = 0; i<7; i++){
                lastRunTime[i] = timeStorage[i];
            }
        }else if (temp == keys[BACK]){ //"D"
            break;    
        }else if (temp == keys[LEFT]){ //"4"
            cycleProgramScreenLeft(&screen_state,0,3);
            update_eeprom_screen_state(screen_state);
        }else if (temp == keys[RIGHT]){ //"6"
            cycleProgramScreenRight(&screen_state,0,3);
            update_eeprom_screen_state(screen_state);
        } 
    }
}

//save the EEPROM data for the run
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

//obtain specific EEPROM data
void getEEPROM(int num){
    int i;
    for (i = 0; i<6; i++){
        runInfo[i] = Eeprom_ReadByte(12*i+num*200);
    }
    for (i = 0; i<7; i++){
        lastRunTime[i] = Eeprom_ReadByte(12*(6+i)+num*200);
    }
}

//access past run information
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
