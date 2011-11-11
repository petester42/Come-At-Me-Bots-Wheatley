//
//  Wheatly_V1.c
//  
//
//  Created by Pete on 11-11-08.
//  Copyright 2011 Pierre-Marc Airoldi, Fawzi Kabbara. All rights reserved.
//
#define F_CPU 1000000UL

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

#define scanState 1
#define positionState 2
#define flankState 3
#define pushState 4
#define escapeState 5
#define avoidLineState 6
#define returnToRingState 7
#define losingOutputState 8
#define winningOutputState 9

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
    
    DDRD |= (0 << flLineSensorDir) | (0 << frLineSensorDir) | (0 << blLineSensorDir) | (0 << brLineSensorDir);//make line sensor ports input
    PORTD |= (0 << flLineSensorPort) | (0 << frLineSensorPort) | (0 << blLineSensorPort) | (0 << brLineSensorPort);//dont enable pull up resistors
    
    DDRD |= (0 << rContactSensorDir); //make contact switch ports input
    PORTD |= (1 << rContactSensorPort); //enable pull-up resisitors
    
    DDRB |= (0 << fContactSensorDir) | (0 << bContactSensorDir) | (0 << lContactSensorDir); //make contact switch ports input
    PORTB |= (1 << fContactSensorPort) | (1 << bContactSensorPort) | (1 << lContactSensorPort); //enable pull-up resistors
    
    //??should do the line before for all adc ports
    uint8_t i;
    
    for (i = 0; i < 6; ++i) {
    
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

void initialize(){
	//fawzi
	readLineSensors();
	readContactSwitches();
	readIRSensors();
        
	while(frLineSensorValue == 0 || flLineSensorValue == 0 || brLineSensorValue == 0 || blLineSensorValue = 0){
		//move backwards
	if (frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue = 1){
		//move away from line
		_delay_ms(2000);
	}

				
		
    
}

uint8_t scan(){
    //fawzi
    while (TRUE) {
        
        readLineSensors();
		readContactSwitches();
		readIRSensors();
		if(/*no sensors activated*/){
			//turn 60 degrees right
		}
		else (/*check all sensors and return state accordingly*/){
			
		}
		
		readLineSensors();
	    readContactSwitches();
	    readIRSensors();
		if(/*no sensors activated*/){
			//move forward for 2 seconds
			//_delay_ms(2000) ???
		}
		else (/*check all sensors and return state accordingly*/){
			
		}
		
		readLineSensors();
	    readContactSwitches();
	    readIRSensors();
		if(/*no sensors activated*/){
			//turn 120 degrees left
		}
		else (/*check all sensors and return state accordingly*/){
			
		}
		
		readLineSensors();
	    readContactSwitches();
	    readIRSensors();
		if(/*no sensors activated*/){
			//move forward for 2 seconds
			//_delay_ms(2000) ???
		}
		else (/*check all sensors and return state accordingly*/){
		}	
		
		//loop back to beginning of function		
           
    }
    
    return 0;
}

uint8_t position(){
    //fawzi
	readLineSensors();
	readContactSwitches();
	readIRSensors();
	/*if (line sensors activated && IR sensors deactivated){
		return avoidLineState;
	} */	
	/*if (front sensor activated){
		do something;
	} */	
	
	if (flIRSensorValue == 1){
		//turn left
	}
	
	else if (frIRSensorValue == 1){
		//turn right
	}				
    return 0;
}

uint8_t flank(){
    //pierre-marc
    return 0;
}

uint8_t push(){
    //fawzi
	readLineSensors();
	readContactSwitches();
	readIRSensors();
	/*if (line sensors activated && contact sensors deactivated){
		return avoidLineState;
	} */
	
	if(fContactSensorValue == 1 || bContactSensorValue == 1 || rContactSensorValue == 1 || lContactSensorValue == 1){
	//push opponent
	}		
    return 0;
}

uint8_t escape(){
    //fawzi
	readLineSensors();
	readContactSwitches();
	readIRSensors();
	/*if (line sensors activated &&  rContactSensorValue == 0 && lContactSensorValue == 0){
		return avoidLineState;
	} */
	
	if(rContactSensorValue == 1){
		//move forward and left
	}
	else if(lContactSensorValue == 1){
		//move forward and right
	}
	else if(rContactSensorValue == 1 && lContactSensorValue == 1){
		//move forward full power
	}						
    return 0;
}

uint8_t avoidLine() {
    //fawzi
	readLineSensors();
	readContactSwitches();
	readIRSensors();
	/*if (line sensors deactivated){
		return scanState;
	} */
	while (frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue = 1){
		//go in opposite direction of line
		//if pushed over line
		//return returnToRingState
		//OR return losingOutputState ???
	}
	
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

int main(){
    
    setup(); //setting up the ports
    _delay_ms(5000); //wait state
    initialize(); //initialize state

    while (TRUE) {

        switch (state) {
            case scanState:
                state = scan();
                break;
            case positionState:
                state = position();
                break;
            case flankState:
                state = flank();
                break;
            case pushState:
                state = push();
                break;
            case escapeState:
                state = escape();
                break;
            case avoidLineState:
                state = avoidLine();
                break;
            case returnToRingState:
                state = returnToRing();
                break;
            case losingOutputState:
                state = losingOutput();
                break;
            case winningOutputState:
                state = winningOutput();
                break;
            default:
                state = scan();
                break;
        }
    }
    
    return 0;
}