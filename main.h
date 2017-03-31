/*
 * File:   main.h
 * Author: True Administrator
 *
 * Created on Feb 28, 2017, 2:31 PM
 */

#include <stdint.h>

void initializePicSettings();
void initializePicPins();
void initializeLcdScreen();

void enterSelectedSubProgramFromMainProgram();
void cycleProgramScreenLeft(int *programScreen, int min, int max);
void cycleProgramScreenRight(int *programScreen, int min, int max);
void toggleRTCBasedOnCurrentMainProgramScreen();

unsigned char getPressedKeypadKey();
void printTime();

int storeRgbSensorInputIntegerValuesInto(unsigned char *colorValues[4], int sensorNum);
int analyzeRgbcForSensor(int sensorNum, unsigned char rgbc[4]);
int capIsOnBottle();
void setMultiplexerToSensor(int num);
void updateCurrentTime();
int getCurrentTimeInSeconds();
void displayMainProgramLcdScreen();
void update_eeprom_screen_state(int num);
void update_past_run_screen_state(int num);

void operation();
void activateDCmotors();
void disableDCmotors();
int timeIsStillLeft(int startingTime);
void moveBinServo(int num90Rotations); 
void retractSensorAreaArm();
void extendSensorAreaArm();
void pushBottle();
int getMotorCounterBasedOnBottlesLeft();

void EEPROM();
void saveEEPROM();
void getEEPROM(int num);
void past_run_access();
uint8_t Eeprom_ReadByte(uint16_t address);
void Eeprom_WriteByte(uint16_t address, uint8_t data);

