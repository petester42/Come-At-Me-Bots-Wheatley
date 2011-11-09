//
//  Wheatly_V1.c
//  
//
//  Created by Pete on 11-11-08.
//  Copyright 2011 Pierre-Marc Airoldi, Fawzi Kabbara. All rights reserved.
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>

#define TRUE 1
#define FALSE 0
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

#define flLineSensorPort PORTD0
#define flLineSensorDir DDRD0
#define flLineSensorPin PIND0

#define flIRSensor 0
#define fcIRSensor 1
#define frIRSensor 2

#define scan 1
#define position 2
#define flank 3
#define push 4
#define escape 5
#define avoidLine 6
#define returnToRing 7
#define losingOutput 8
#define winningOutput 9

#define forward 1
#define backward 2
#define left 3
#define right 4
#define brake 0

int state = 0;
int frIRSensorValue = 0;
int frLineSensorValue = 0;
int fContactSensorValue = 0;

//values to store the adc values and the pin values

void setup(){
    //set up the DDR
    //adc
    //interrupts
    
}

int readADC(int sensor){
    //sensor is the adc to read
    //read a value 
    
    return 0;
}

void initialize(){
    //fawzi
}

int scan(){
    //fawzi
    while (TRUE) {
        
        checkLineSensors();
        
        if (frLineSensorValue == 1) {
            return avoidLine;
        }
        
        checkContactSwitches();
        
    }
    
    return 0;
}

int position(){
    //fawzi
    return 0;
}

int flank(){
    //pierre-marc
    return 0;
}

int push(){
    //fawzi
    return 0;
}

int escape(){
    //fawzi
    return 0;
}

int avoidLine() {
    //fawzi
    return 0;
}

int returnToRing() {
    //pierre-marc
    return 0;
}

int losingOutput() {
    //pierre marc
    return 0;
}

int winningOutput() {
    //pierre marc
    return 0;
}

void move (int direction, int speed, int duration){
    
    
}

void checkLineSensors(){
    
    frLineSensorValue = flLineSensorPin;
    
}

void checkContactSwitches(){

    fContactSensorValue = 0/*flLineSensorPin*/;

}

void checkIRSensors(){
    
    frIRSensorValue = readADC(frIRSensorValue);
    
    
}

int main(){
    
    setup();
    _delay_ms(5000);
    initialize();
    
    while (TRUE) {
        
        switch (state) {
            case 1:
                state = scan();
                break;
            case 2:
                state = position();
                break;
                
            default:
                state = scan();
                break;
        }
    }
    
    return 0;
}