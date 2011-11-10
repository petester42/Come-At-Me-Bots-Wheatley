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

//front left line sensor
#define flLineSensorPort PORTD0
#define flLineSensorDir DDD0
#define flLineSensorPin PIND0

//front right line sensor
#define frLineSensorPort PORTD1
#define frLineSensorDir DDD1
#define frLineSensorPin PIND1

//back left line sensor
#define blLineSensorPort PORTD2
#define blLineSensorDir DDD2
#define blLineSensorPin PIND2

//back right line sensor
#define brLineSensorPort PORTD3
#define brLineSensorDir DDD3
#define brLineSensorPin PIND3

//front contact sensor
#define fContactSensorPort PORTB6
#define fContactSensorDir DDB6
#define fContactSensorPin PINB6

//back contact sensor
#define bContactSensorPort PORTB7
#define bContactSensorDir DDB7
#define bContactSensorPin PINB7

//right contact sensor
#define rContactSensorPort PORTD4
#define rContactSensorDir DDD4
#define rContactSensorPin PIND4

//left contact sensor
#define lContactSensorPort PORTB5
#define lContactSensorDir DDB5
#define lContactSensorPin PINB5

//front IR sensors
#define flIRSensor 0 //ADC0 PC0
#define fcIRSensor 1 //ADC1 PC1
#define frIRSensor 2 //ADC2 PC2

//back IR sensors
#define blIRSensor 3 //ADC3 PC3
#define bcIRSensor 4 //ADC4 PC4
#define brIRSensor 5 //ADC5 PC5

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

//use uint8_t instead of int since it takes less space on the memory

//current state value
uint8_t state = 0;

//value that IR sensor reads
uint8_t frIRSensorValue = 0;
uint8_t fcIRSensorValue = 0;
uint8_t flIRSensorValue = 0;

//value that IR sensor reads
uint8_t brIRSensorValue = 0;
uint8_t bcIRSensorValue = 0;
uint8_t blIRSensorValue = 0;

//value that line sensor reads
uint8_t frLineSensorValue = 0;
uint8_t flLineSensorValue = 0;
uint8_t brLineSensorValue = 0;
uint8_t blLineSensorValue = 0;

//value that contact sensor reads
uint8_t fContactSensorValue = 0;
uint8_t bContactSensorValue = 0;
uint8_t rContactSensorValue = 0;
uint8_t lContactSensorValue = 0;

//values to store the adc values and the pin values

void setup(){
    //set up the DDR
    //adc
    //interrupts
    
    flLineSensorDir = 0; //make port input
    flLineSensorPort = 0; //dont enable pull-up resistor
    
    frLineSensorDir = 0; //make port input
    frLineSensorPort = 0; //dont enable pull-up resistor
    
    blLineSensorDir = 0; //make port input
    blLineSensorPort = 0; //dont enable pull-up resistor
    
    brLineSensorDir = 0; //make port input
    brLineSensorPort = 0; //dont enable pull-up resistor
    
    fContactSensorDir = 0; //make port input
    fContactSensorPort = 1; //enable pull-up resistor
    
    bContactSensorDir = 0; //make port input
    bContactSensorPort = 1; //enable pull-up resistor
    
    rContactSensorDir = 0; //make port input
    rContactSensorPort = 1; //enable pull-up resistor
    
    lContactSensorDir = 0; //make port input
    lContactSensorPort = 1; //enable pull-up resistor
    
    //??should do the line before for all adc ports
    for (uint8_t i = 0; i<6; ++i) {
    
    ADMUX = (1 << REFS0) | (1 <<REFS1) | (1<<ADLAR) | i; 
    
	ADCSRA |= (0<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //set prescaler to 8
	ADCSRA |= (1<<ADEN); //enable ADC
    
	ADCSRA |= ( 1 << ADSC);
	while (ADCSRA & (1 << ADSC));  // Discard this reading
    
    }
//    sei(); //interrupts needed??
}


uint8_t readADC(uint8_t channel) { 
    
    ADMUX = (1 << REFS0) | (1 <<REFS1) | (1<<ADLAR) | channel; 
    ADCSRA |= (1 << ADSC);
    
    while (ADCSRA & (1<<ADSC)); 
    return ADCH; 
} 

void initialize(){
    //fawzi
}

uint8_t scan(){
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

uint8_t position(){
    //fawzi
    return 0;
}

uint8_t flank(){
    //pierre-marc
    return 0;
}

uint8_t push(){
    //fawzi
    return 0;
}

uint8_t escape(){
    //fawzi
    return 0;
}

uint8_t avoidLine() {
    //fawzi
    return 0;
}

uint8_t returnToRing() {
    //pierre-marc
    return 0;
}

uint8_t losingOutput() {
    //pierre marc
    return 0;
}

uint8_t winningOutput() {
    //pierre-marc
    return 0;
}

void move (uint8_t direction, uint8_t speed, uint8_t duration){
    //pierre-marc
    
}

void readLineSensors(){
    
    frLineSensorValue = frLineSensorPin;
    flLineSensorValue = flLineSensorPin;
    brLineSensorValue = brLineSensorPin;
    blLineSensorValue = blLineSensorPin;
}

void readContactSwitches(){

    fContactSensorValue = fContactSensorPin;
    bContactSensorValue = bContactSensorPin;
    lContactSensorValue = lContactSensorPin;
    rContactSensorValue = rContactSensorPin;

}

void readIRSensors(){
    
    frIRSensorValue = readADC(frIRSensor);
    fcIRSensorValue = readADC(fcIRSensor);
    flIRSensorValue = readADC(flIRSensor);
    brIRSensorValue = readADC(brIRSensor);
    bcIRSensorValue = readADC(bcIRSensor);
    blIRSensorValue = readADC(blIRSensor);
    
}

int main(){
    
    setup(); //setting up the ports
    _delay_ms(5000); //wait state
    initialize(); //initialize state
    
    while (TRUE) {

        switch (state) {
            case 1:
                state = scan();
                break;
            case 2:
                state = position();
                break;
            case 3:
                state = flank();
                break;
            case 4:
                state = push();
                break;
            case 5:
                state = escape();
                break;
            case 6:
                state = avoidLine();
                break;
            case 7:
                state = returnToRing();
                break;
            case 8:
                state = losingOutput();
                break;
            case 9:
                state = winningOutput();
                break;
            default:
                state = scan();
                break;
        }
    }
    
    return 0;
}