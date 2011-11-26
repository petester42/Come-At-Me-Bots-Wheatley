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
#define lContactSensorPort PORTB2
#define lContactSensorDir DDB2
#define lContactSensorPin PINB2

//front IR sensors
#define flIRSensor 0 //ADC0 PC0
#define fcIRSensor 1 //ADC1 PC1
#define frIRSensor 2 //ADC2 PC2

//back IR sensors
#define blIRSensor 3 //ADC3 PC3
#define bcIRSensor 4 //ADC4 PC4
#define brIRSensor 5 //ADC5 PC5

//PWM
#define PWMLeftMotorDir DDB1
#define PWMRightMotorDir DDB1
#define PWMSpeakerDir DDB1

//Left Motor 
#define HighLeftMotorPort PORTD7 //1A
#define HighLeftMotorDir DDD7
#define LowLeftMotorPort PORTB0 //2A
#define LowLeftMotorDir DDB0

//Right Motor
#define HighRightMotorPort PORTD6 //4A
#define HighRightMotorDir DDD6
#define LowRightMotorPort PORTD5 //3A
#define LowRightMotorDir DDD5

#define resetPort PORTC6
#define resetDir DDC6

//states
#define scanState 1
#define positionState 2
#define flankState 3
#define pushState 4
#define escapeState 5
#define avoidLineState 6
#define returnToRingState 7
#define losingOutputState 8
#define winningOutputState 9

//direction of motor
#define brake 0
#define forward 1
#define backward 2
#define frontleft 3
#define frontright 4
#define backleft 5
#define backright 6

//return to ring array
int returnToRingArray[] = {0,0,0,0};

//#define DEFAULT_VOLUME 100
const int distance[] = {1,2,3,4,5,6,7}; //look up table for distances
const int triangle[] = {5,10,15,20,25,30,35}; //look up table for trangle sides

//#include "notes.h"

/// Default volume, value assigned to OCR1B
#define DEFAULT_VOLUME 100

//current state value
uint8_t state = 0;
uint8_t resetScan = 0;

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
uint8_t fContactSensorValue = 1;
uint8_t bContactSensorValue = 1;
uint8_t rContactSensorValue = 1;
uint8_t lContactSensorValue = 1;

//values of move and scan
uint8_t scanMoveState = 0;
uint8_t scanMove = 0;
uint8_t flankMoveState = 0;
uint8_t flankMove = 0;
int numberOfMS = 0;
uint8_t isFront = 0;
int distanceInMS = 0;
//values to store the adc values and the pin values

void setup(){
    //set up the DDR
    //adc
    //interrupts
    
    DDRD &= ~(1 << flLineSensorDir) & ~(1 << frLineSensorDir) & ~(1 << blLineSensorDir) & ~(1 << brLineSensorDir);//make line sensor ports input
    PORTD &= ~(1 << flLineSensorPort) & ~(1 << frLineSensorPort) & ~(1 << blLineSensorPort) & ~(1 << brLineSensorPort);//dont enable pull up resistors
    
    DDRD &= ~(1 << rContactSensorDir); //make contact switch ports input
    PORTD |= (1 << rContactSensorPort); //enable pull-up resistors
    
    DDRC &= ~(1 << resetDir); //make reset input
    PORTC |= (1<< resetPort); //enable pull-up resistor
    
    DDRB &= ~(1 << fContactSensorDir) & ~(1 << bContactSensorDir) & ~(1 << lContactSensorDir); //make contact switch ports input
    PORTB |= (1 << fContactSensorPort) | (1 << bContactSensorPort) | (1 << lContactSensorPort); //enable pull-up resistors
    
    DDRB |= (1 << PWMSpeakerDir) | (1 << PWMLeftMotorDir) | (1 << PWMRightMotorDir) | (1 << LowLeftMotorDir); //output
    DDRD |= (1 << HighLeftMotorDir) | (1 << LowRightMotorDir) | (1 << HighRightMotorDir);
    
    PORTD &= ~(1 << HighLeftMotorPort);
    PORTB &= ~(1 << LowLeftMotorPort);
    PORTD &= ~(1 << HighRightMotorPort);
    PORTD &= ~(1 << LowRightMotorPort);
    
	//set up motor PWM
    
    TCCR2 = 0; // Turn all PWM whilst setting up
    TCCR2 |= (1<<WGM20); // Select 8 bit phase correct with TOP=255
    TCCR2 |= (1<<CS20); // Select prescaler = 1
    TCCR2 |= (1<<COM21); // Turn on the PWM channel using non-inverting mode
    
    //set up sound PWM
    
	TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
	TCCR1B = (1 << WGM13); // Phase/Freq-correct PWM, top value = ICR1, Prescaler: Off   
    
    uint8_t i;
    
    for (i = 0; i < 6; ++i) {
        
        ADMUX = (1 << REFS0) | (1 <<REFS1) | (1<<ADLAR) | i; 
        
        ADCSRA |= (0<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //set prescaler to 8
        ADCSRA |= (1<<ADEN); //enable ADC
        
        ADCSRA |= ( 1 << ADSC);
		
        while (ADCSRA & (1 << ADSC));  // Discard this reading
        
    }
    
    
    sei(); //interrupts needed??
}


uint8_t readADC(uint8_t channel) { 
    
    ADMUX = (1 << REFS0) | (1 <<REFS1) | (1<<ADLAR) | channel; 
    ADCSRA |= (1 << ADSC);
    
    while (ADCSRA & (1<<ADSC)); 
    return ADCH; 
} 

void move (uint8_t direction, uint8_t speed){
    //pierre-marc
    //speed are values from 0-255
    //OCR2 is for speed
    
    // HighLeftMotorPort PORTD7 //1A
    // LowLeftMotorPort PORTB0 //2A
    // HighRightMotorPort PORTD6 //4A
    // LowRightMotorPort PORTD5 //3A
    
    OCR2 = speed;
    
    switch (direction) {
            
        case backward:
            PORTD &= ~(1 << HighLeftMotorPort);
            PORTB |= (1 << LowLeftMotorPort);
            PORTD &= ~(1 << HighRightMotorPort);
            PORTD |= (1 << LowRightMotorPort);
            
            break;
            
        case forward:
            PORTD |= (1 << HighLeftMotorPort);
            PORTB &= ~(1 << LowLeftMotorPort);
            PORTD |= (1 << HighRightMotorPort);
            PORTD &= ~(1 << LowRightMotorPort);
            
            break;
            
        case frontleft:
            PORTD |= (1 << HighLeftMotorPort);
            PORTB &= ~(1 << LowLeftMotorPort);
            PORTD &= ~(1 << HighRightMotorPort);
            PORTD &= ~(1 << LowRightMotorPort);
            
            break;
            
        case frontright:
            PORTD &= ~(1 << HighLeftMotorPort);
            PORTB &= ~(1 << LowLeftMotorPort);
            PORTD |= (1 << HighRightMotorPort);
            PORTD &= ~(1 << LowRightMotorPort);
            
            break;
            
        case backleft:
            PORTD &= ~(1 << HighLeftMotorPort);
            PORTB |= (1 << LowLeftMotorPort);
            PORTD &= ~(1 << HighRightMotorPort);
            PORTD &= ~(1 << LowRightMotorPort);
            
            break;
            
        case backright:
            PORTD &= ~(1 << HighLeftMotorPort);
            PORTB &= ~(1 << LowLeftMotorPort);
            PORTD &= ~(1 << HighRightMotorPort);
            PORTD |= (1 << LowRightMotorPort);
            
            break;
            
        case brake:
            PORTD &= ~(1 << HighLeftMotorPort);
            PORTB &= ~(1 << LowLeftMotorPort);
            PORTD &= ~(1 << HighRightMotorPort);
            PORTD &= ~(1 << LowRightMotorPort);
            
            break;
            
        default:
            PORTD &= ~(1 << HighLeftMotorPort);
            PORTB &= ~(1 << LowLeftMotorPort);
            PORTD &= ~(1 << HighRightMotorPort);
            PORTD &= ~(1 << LowRightMotorPort);
            
            break;
    }
    //here
}

void readLineSensors(){
    
    if (CHECKBIT(PIND,frLineSensorPin)) {
        frLineSensorValue = 1;
    }
    
    else {
        frLineSensorValue = 0; 
    }
    
    if (CHECKBIT(PIND,flLineSensorPin)) {
        flLineSensorValue = 1;
    }
    
    else {
        flLineSensorValue = 0; 
    }
    
    if (CHECKBIT(PIND,brLineSensorPin)) {
        brLineSensorValue = 1;
    }
    
    else {
        brLineSensorValue = 0; 
    }
    
    if (CHECKBIT(PIND,blLineSensorPin)) {
        blLineSensorValue = 1;
    }
    
    else {
        blLineSensorValue = 0; 
    }
    
}

void readContactSwitches(){
    //active low, switch on is 0, off is 1
    
    if (CHECKBIT(PINB,fContactSensorPin)) {
        fContactSensorValue = 0;
    }
    
    else {
        fContactSensorValue = 1;
    }
    
    if (CHECKBIT(PINB,bContactSensorPin)) {
        bContactSensorValue = 0;
    }
    
    else {
        bContactSensorValue = 1; 
    }
    
    if (CHECKBIT(PINB,lContactSensorPin)) {
        lContactSensorValue = 0;
    }
    
    else {
        lContactSensorValue = 1; 
    }
    
    if (CHECKBIT(PIND,rContactSensorPin)) {
        rContactSensorValue = 0;
    }
    
    else {
        rContactSensorValue = 1; 
    }
    
    
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
    //done
    
    move(backward,255);
	while(TRUE){
		readLineSensors();
        
        if (frLineSensorValue == 1 || flLineSensorValue == 1){
            //move away from line
            move(backward,255);
			isFront = 0;
            _delay_ms(1000);
            move(brake,0);
            
            break;
        }
        
		else if (brLineSensorValue == 1 || blLineSensorValue == 1){
			move(forward,255);
			isFront = 1;
            _delay_ms(1000);
            move(brake,0);
			break;
		}
        
    }
}

void startTimer(){
    
    TIMSK = (1<<TOV0);
    TCNT0 = 125;
    TCCR0 |= (1<<CS01);
    
}

void stopTimer(){
    
    TIMSK = (0<<TOV0);
    
}

ISR(TIMER0_OVF_vect){
    
	numberOfMS += 1;
    
	if (scanMove == 1 && numberOfMS == 300){
		scanMove = 2;
		scanMoveState = 0;
		numberOfMS = 0;
		stopTimer();
	}
    
	else if (scanMove == 2 && numberOfMS == 800){
		scanMove = 3;
		scanMoveState = 0;
		numberOfMS = 0;
		stopTimer();
	}
    
	else if (scanMove == 3 && numberOfMS == 600){
		scanMove = 4;
		scanMoveState = 0;
		numberOfMS = 0;
		stopTimer();
	}
    
	else if (scanMove == 4 && numberOfMS == 800){
		scanMove = 5;
		scanMoveState = 0;
		numberOfMS = 0;
		stopTimer();
	}
    
	else if (scanMove == 5 && numberOfMS == 600){
		scanMove = 2;
		scanMoveState = 0;
		numberOfMS = 0;
		stopTimer();
	}
    
	if (flankMoveState == 1 && numberOfMS == distanceInMS){
		flankMoveState = 2;
		flankMove = 0;
		distanceInMS = 0;
		numberOfMS = 0;
		stopTimer();
	}
    
}

uint8_t scan(){
    //done
	scanMoveState = 0;
	numberOfMS = 0;
	scanMove = 1;	
    
    while (TRUE) {
        
        readLineSensors();
        
        if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue == 1){
            
            return avoidLineState;
        }
        
		readContactSwitches();
        
        if(fContactSensorValue == 0 || bContactSensorValue == 0 || lContactSensorValue == 0 || rContactSensorValue == 0){
			
            if(lContactSensorValue == 0 && rContactSensorValue == 1 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else if(lContactSensorValue == 1 && rContactSensorValue == 0 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else if(lContactSensorValue == 0 && rContactSensorValue == 0 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else {
                return pushState;
            }
			
        }	
        
        readIRSensors();
        
		if (fcIRSensorValue > 85 || bcIRSensorValue > 85){
            
			return flankState;
		}
        
        if(frIRSensorValue > 85 || flIRSensorValue > 85 || brIRSensorValue > 85 || blIRSensorValue > 85){
            return positionState;
        }
        
		if (scanMove == 1 && scanMoveState == 0){
			if (isFront == 1){
				move(frontleft,255);
			}
            
			else {
				move(backright,255);
			}
            
			scanMoveState = 1;
			startTimer();
		}
        
        else if (scanMove == 2 && scanMoveState == 0){
			if (isFront == 1){
				move(forward,255);
			}
            
			else {
				move(backward,255);
			}
            
            scanMoveState = 1;
            startTimer();
        }
        
        else if (scanMove == 3 && scanMoveState == 0){
            
            if (isFront == 1){
				move(frontright,255);
			}
            
			else {
				move(backleft,255);
			}
            scanMoveState = 1;
            startTimer();
        }
        
        else if (scanMove == 4 && scanMoveState == 0){
            
            if (isFront == 1){
				move(forward,255);
			}
            
			else {
				move(backward,255);
			}
            scanMoveState = 1;
            startTimer();
        }
        
        else if (scanMove == 5 && scanMoveState == 0){
            if (isFront == 1){
				move(frontleft,255);
			}
            
			else {
				move(backright,255);
			}
            scanMoveState = 1;
            startTimer();
        }
        
        
        
    }	
    
    return 0;
    
}

uint8_t position(){
    //done
    
	while (TRUE){
        readLineSensors();
        
        if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue == 1){
            return avoidLineState;
        }
        
        readContactSwitches();
        
        if(fContactSensorValue == 0 || bContactSensorValue == 0 || lContactSensorValue == 0 || rContactSensorValue == 0){
            
            if(lContactSensorValue == 0 && rContactSensorValue == 1 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else if(lContactSensorValue == 1 && rContactSensorValue == 0 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else if(lContactSensorValue == 0 && rContactSensorValue == 0 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else {
                return pushState;
            }
			
        }	
        
        readIRSensors();
        
        if(frIRSensorValue > 85 || flIRSensorValue > 85 || brIRSensorValue > 85 || blIRSensorValue > 85 || fcIRSensorValue > 85 || bcIRSensorValue > 85){
            
            if (fcIRSensorValue > 85 || bcIRSensorValue >85){
                return flankState;
            }
            
            else {
                
                if (frIRSensorValue > 85 && flIRSensorValue <= 85 && brIRSensorValue <= 85 && blIRSensorValue <= 85){
                    move(frontright,255); //fr only
                }
                
                else if (frIRSensorValue <= 85 && flIRSensorValue > 85 && brIRSensorValue <= 85 && blIRSensorValue <= 85){
                    move(frontleft,255); //fl only
                }	
                
                /*else if (frIRSensorValue > 85 && flIRSensorValue > 85 && brIRSensorValue <= 85 && blIRSensorValue <= 85){
                 move(frontright,255); //f both 
                 }*/	
                
                else if (frIRSensorValue <= 85 && flIRSensorValue <= 85 && brIRSensorValue <= 85 && blIRSensorValue > 85){
                    move(backleft,255); //bl only
                }
                
                else if (frIRSensorValue <= 85 && flIRSensorValue <= 85 && brIRSensorValue > 85 && blIRSensorValue <= 85){
                    move(backright,255); //br only
                }	
                
                /*else if (frIRSensorValue <= 85 && flIRSensorValue <= 85 && brIRSensorValue > 85 && blIRSensorValue > 85){
                 move(backright,255); //b both 
                 }*/	
                
                else if (frIRSensorValue > 85 && flIRSensorValue <= 85 && brIRSensorValue > 85 && blIRSensorValue <= 85){
                    move(frontright,255); //fr only with back
                }
                
                else if (frIRSensorValue > 85 && flIRSensorValue <= 85 && brIRSensorValue <= 85 && blIRSensorValue > 85){
                    move(frontright,255); //fr only with back
                }
                
                else if (frIRSensorValue > 85 && flIRSensorValue <= 85 && brIRSensorValue > 85 && blIRSensorValue > 85){
                    move(frontright,255); //fr only with back
                }
                
                else if (frIRSensorValue <= 85 && flIRSensorValue > 85 && brIRSensorValue > 85 && blIRSensorValue <= 85){
                    move(frontleft,255); //fl only with back
                }	
                
                else if (frIRSensorValue <= 85 && flIRSensorValue > 85 && brIRSensorValue <= 85 && blIRSensorValue > 85){
                    move(frontleft,255); //fl only with back
                }
                
                else if (frIRSensorValue <= 85 && flIRSensorValue > 85 && brIRSensorValue > 85 && blIRSensorValue > 85){
                    move(frontleft,255); //fl only with back
                }
                
                /*else if (frIRSensorValue > 85 && flIRSensorValue > 85 && brIRSensorValue <= 85 && blIRSensorValue <= 85){
                 move(frontright,255); //f both with back
                 }*/
                
                /* else if (frIRSensorValue > 85 && flIRSensorValue > 85 && brIRSensorValue > 85 && blIRSensorValue <= 85){
                 move(frontright,255); //f both with back
                 }*/
                
                /*else if (frIRSensorValue > 85 && flIRSensorValue > 85 && brIRSensorValue <= 85 && blIRSensorValue > 85){
                 move(frontright,255); //f both with back
                 }*/
                
                /*else if (frIRSensorValue > 85 && flIRSensorValue > 85 && brIRSensorValue > 85 && blIRSensorValue > 85){
                 move(frontright,255); //f both with back
                 }
                 */
                else{
                    
                }
            }
            
        }
        
        else {
            scanMoveState = 0;
            scanMove = 1;
            
            return scanState;
        }
        
    }
    return 0;
}

uint16_t getDistanceTime(uint16_t sensorValue){
	
	uint16_t time;
    
	if (sensorValue < 270 && sensorValue >= 240){
		time = 565;
	}
	
	else if (sensorValue < 240 && sensorValue >= 225){
		time = 636;
	}
    
	else if (sensorValue < 225 && sensorValue >= 212){
		time = 707;
	}
    
	else if (sensorValue < 212 && sensorValue >= 196){
		time = 777;
	}
    
	else if (sensorValue < 196 && sensorValue >= 181){
		time = 849;
	}
	
	else if (sensorValue < 181 && sensorValue >= 168){
		time = 919;
	}
    
	else if (sensorValue < 168 && sensorValue >= 158){
		time = 989;
	}
    
	else if (sensorValue < 158 && sensorValue >= 153){
		time = 1060;
	}
    
	else if (sensorValue < 153 && sensorValue >= 147){
		time = 1130;
	}
	
	else if (sensorValue < 147 && sensorValue >= 137){
		time = 1202;
	}
    
	else if (sensorValue < 137 && sensorValue >= 132){
		time = 1272;
	}
    
	else if (sensorValue < 132 && sensorValue >= 128){
		time = 1344;
	}
	
	else if (sensorValue < 128 && sensorValue >= 122){
		time = 1414;
	}
    
	else if (sensorValue < 122 && sensorValue >= 116){
		time = 1484;
	}
    
	else if (sensorValue < 116 && sensorValue >= 113){
		time = 1555;
	}
    
	else if (sensorValue < 113 && sensorValue >= 107){
		time = 1626;
	}
	
	else if (sensorValue < 107 && sensorValue >= 105){
		time = 1697;
	}
    
	else if (sensorValue < 105 && sensorValue >= 100){
		time = 1767;
	}
    
	else if (sensorValue < 100 && sensorValue >= 96){
		time = 1838;
	}
    
	else if (sensorValue < 96 && sensorValue >= 92){
		time = 1909;
	}
    
	else if (sensorValue < 92 && sensorValue >= 90){
		time = 1979;
	}
    
	else if (sensorValue < 90 && sensorValue >= 88){
		time = 2050;
	}
    
	else if (sensorValue < 88 && sensorValue >= 86){
		time = 2121;
	}
    
	else {
		time = 2192;
	}
    
	return time;
}

uint8_t flank(){
    //pierre-marc
    
	uint8_t chooseFront = 0;
    
	while (TRUE){
		
        readLineSensors();
        
        if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue == 1){
            return avoidLineState;
        }
        
        readContactSwitches();
        
        if(fContactSensorValue == 0 || bContactSensorValue == 0 || lContactSensorValue == 0 || rContactSensorValue == 0){
            
            if(lContactSensorValue == 0 && rContactSensorValue == 1 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else if(lContactSensorValue == 1 && rContactSensorValue == 0 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else if(lContactSensorValue == 0 && rContactSensorValue == 0 && fContactSensorValue == 1 && bContactSensorValue == 1){
                return escapeState;
            }
            
            else {
                return pushState;
            }
			
        }	
		
		readIRSensors();
		if (fcIRSensorValue > 85 || bcIRSensorValue > 85) { //values changed after testing
            //front sensor not in range
            
            if (fcIRSensorValue > 85 && bcIRSensorValue <= 85){
                chooseFront = 1;
            }
            
            else if (fcIRSensorValue <= 85 && bcIRSensorValue > 85){
                chooseFront = 0;
            }
            
            else if (fcIRSensorValue > 85 && bcIRSensorValue > 85){
                chooseFront = 1;
            }
            
            if (chooseFront == 1){
                move(forward,255);
            }
            
            else {
                move(backward,255);
            }
        } 
        
        else {
            move(brake,255);
            
            if (flIRSensorValue <= 85 && frIRSensorValue <= 85) { //values changed after testing
                scanMoveState = 0;
                scanMove = 1;
                
                return scanState;
            }  
            
            else {
                return positionState;
            }
            
        }   
        
	}
    
    /* uint8_t isFront = 0;
     uint8_t checkIR = 1;
     flankMoveState = 0;
     flankMove = 0;
     distanceInMS = 0;
     numberOfMS = 0;
     
     while (TRUE) {
     
     readLineSensors();
     
     if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue == 1){
     return avoidLineState;
     }
     
     readContactSwitches();
     
     if(fContactSensorValue == 0 || bContactSensorValue == 0 || lContactSensorValue == 0 || rContactSensorValue == 0){
     
     if(lContactSensorValue == 0 && rContactSensorValue == 1 && fContactSensorValue == 1 && bContactSensorValue == 1){
     return escapeState;
     }
     
     else if(lContactSensorValue == 1 && rContactSensorValue == 0 && fContactSensorValue == 1 && bContactSensorValue == 1){
     return escapeState;
     }
     
     else if(lContactSensorValue == 0 && rContactSensorValue == 0 && fContactSensorValue == 1 && bContactSensorValue == 1){
     return escapeState;
     }
     
     else {
     return pushState;
     }
     
     }	
     
     if (checkIR == 1) {
     
     readIRSensors();
     
     if (fcIRSensorValue <= 85) { //values changed after testing
     //front sensor not in range
     
     if (flIRSensorValue <= 85 && frIRSensorValue <= 85) { //values changed after testing
     
     return scanState; //not visible anymore
     }  
     
     return positionState;
     }    
     
     else {
     
     isFront = 1;
     }
     
     if (isFront == 0){
     if (bcIRSensorValue <= 85) { //values changed after testing
     //front sensor not in range
     
     if (blIRSensorValue <= 85 && brIRSensorValue <= 85) { //values changed after testing
     
     return scanState; //not visible anymore
     }
     
     return positionState;
     }
     
     
     else {
     isFront = 0;
     
     }
     }
     
     }
     
     if (isFront == 1) {
     
     if (flankMove == 0) { //not moving
     
     if (flankMoveState == 0) {
     checkIR = 0;
     move(frontright,255); //turn to 45 deg.
     _delay_ms(500);
     move(brake,0);
     flankMoveState = 1;
     }
     
     else if (flankMoveState == 1){
     distanceInMS = getDistanceTime(fcIRSensorValue);
     flankMove = 1;
     move(forward,255); //go forward
     
     startTimer();
     }
     
     else if (flankMoveState == 2){
     move(frontleft,255); 
     _delay_ms(1000); //turn 90 deg.
     move(brake,0);
     flankMoveState = 3;
     checkIR = 1;
     }
     
     else if (flankMoveState == 3){
     flankMove = 1;
     move(forward,255); //go forward
     }
     }
     
     }
     
     else {
     
     if (flankMove == 0) { //not moving
     
     if (flankMoveState == 0) {
     
     checkIR = 0;
     move(backright,255); //turn to 45 deg.
     _delay_ms(500);
     move(brake,0);
     flankMoveState = 1;
     }
     
     else if (flankMoveState == 1){
     distanceInMS = getDistanceTime(bcIRSensorValue);
     flankMove = 1;
     move(backward,255); //go forward
     startTimer();
     }
     
     else if (flankMoveState == 2){
     move(backleft,255); 
     _delay_ms(1000); //turn 90 deg.
     move(brake,0);
     flankMoveState = 3;
     checkIR = 1;
     }
     
     else if (flankMoveState == 3){
     flankMove = 1;
     move(backward,255); //go forward
     }
     }
     
     }
     
     }    */
    
    return 0;
    
}

uint8_t push(){
    //fawzi
    
    while (TRUE){
        
        readLineSensors();
        
        if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue == 1){
            return avoidLineState;
        }
        
        readContactSwitches();
        readIRSensors();
        
        if(fContactSensorValue == 0 || bContactSensorValue == 0 || lContactSensorValue == 0 || rContactSensorValue == 0){
            
            if(fContactSensorValue == 1 && bContactSensorValue == 0 && lContactSensorValue == 1 && rContactSensorValue == 1){
                
                /*if (brIRSensorValue > 85 || blIRSensorValue > 85 || bcIRSensorValue > 85){
                 move(forward,255);
                 _delay_ms(350);
                 isFront = 0;
                 return scanState;
                 }
                 
                 else {
                 isFront = 1;
                 move(backward,255);
                 }*/
                
                
                isFront = 0;
                move(backward,255);
            }
            
            else if(fContactSensorValue == 1 && bContactSensorValue == 0 && lContactSensorValue == 1 && rContactSensorValue == 0){
                
                /*if (brIRSensorValue < 85 || blIRSensorValue < 85 || bcIRSensorValue < 85){
                 move(forward,255);
                 _delay_ms(350);
                 isFront = 0;
                 return scanState;
                 }
                 
                 else {
                 isFront = 1;
                 move(backward,255);
                 }*/
                
                isFront = 0;
                move(backward,255);
            }
            
            else if(fContactSensorValue == 1 && bContactSensorValue == 0 && lContactSensorValue == 0 && rContactSensorValue == 1){
                
                /*if (brIRSensorValue < 85 || blIRSensorValue < 85 || bcIRSensorValue < 85){
                 move(forward,255);
                 _delay_ms(350);
                 isFront = 0;
                 return scanState;
                 }
                 
                 else {
                 isFront = 1;
                 move(backward,255);
                 }*/
                
                isFront = 0;
                move(backward,255);
            }
            
            else if(fContactSensorValue == 1 && bContactSensorValue == 0 && lContactSensorValue == 0 && rContactSensorValue == 0){
                
                /*if (brIRSensorValue < 85 || blIRSensorValue < 85 || bcIRSensorValue < 85){
                 move(forward,255);
                 _delay_ms(350);
                 isFront = 0;
                 return scanState;
                 }
                 
                 else {
                 isFront = 1;
                 move(backward,255);
                 }*/
                
                isFront = 0;
                move(backward,255);
                
            }
            
            else if(fContactSensorValue == 0 && bContactSensorValue == 1 && lContactSensorValue == 1 && rContactSensorValue == 1){
                
                /*if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85){
                 move(backward,255);
                 _delay_ms(350);
                 isFront = 1;
                 return scanState;
                 }
                 
                 else {
                 isFront = 0;
                 move(forward,255);
                 }*/
                
                isFront = 1;
                move(forward,255);
                
            }
            
            else if(fContactSensorValue == 0 && bContactSensorValue == 1 && lContactSensorValue == 1 && rContactSensorValue == 0){
                /* if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85){
                 move(backward,255);
                 _delay_ms(350);
                 isFront = 1;
                 return scanState;
                 }
                 
                 else {
                 isFront = 0;
                 move(forward,255);
                 }*/
                
                isFront = 1;
                move(forward,255);
            }
            
            else if(fContactSensorValue == 0 && bContactSensorValue == 1 && lContactSensorValue == 0 && rContactSensorValue == 1){
                /*if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85){
                 move(backward,255);
                 _delay_ms(350);
                 isFront = 1;
                 return scanState;
                 }
                 
                 else {
                 isFront = 0;
                 move(forward,255);
                 }*/
                
                isFront = 1;
                move(forward,255);
                
            }
            
            else if(fContactSensorValue == 0 && bContactSensorValue == 1 && lContactSensorValue == 0 && rContactSensorValue == 0){
                /*if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85){
                 move(backward,255);
                 _delay_ms(350);
                 isFront = 1;
                 return scanState;
                 }
                 
                 else {
                 isFront = 0;
                 move(forward,255);
                 }*/
                
                isFront = 1;
                move(forward,255);
            }
            
            else if(fContactSensorValue == 0 && bContactSensorValue == 0 && lContactSensorValue == 1 && rContactSensorValue == 1){
                /*if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85 || brIRSensorValue < 85 || blIRSensorValue < 85 || bcIRSensorValue < 85){
                 
                 if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85){
                 move(backward,255);
                 _delay_ms(350);
                 isFront = 1;
                 
                 }
                 
                 else {
                 move(forward,255);
                 _delay_ms(350);
                 isFront = 0;
                 }
                 
                 return scanState;
                 }
                 
                 else {
                 isFront = 0;
                 move(forward,255);
                 }
                 */
                
                isFront = 1;
                move(forward,255);
            }
            
            else if(fContactSensorValue == 0 && bContactSensorValue == 0 && lContactSensorValue == 1 && rContactSensorValue == 0){
                /*if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85 || brIRSensorValue < 85 || blIRSensorValue < 85 || bcIRSensorValue < 85){
                 
                 if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85){
                 move(backward,255);
                 _delay_ms(350);
                 isFront = 1;
                 
                 }
                 
                 else {
                 move(forward,255);
                 _delay_ms(350);
                 isFront = 0;
                 }
                 
                 return scanState;
                 }
                 
                 else {
                 isFront = 0;
                 move(forward,255);
                 }*/
                
                isFront = 1;
                move(forward,255);
            }
            
            else if(fContactSensorValue == 0 && bContactSensorValue == 0 && lContactSensorValue == 0 && rContactSensorValue == 1){
                /*if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85 || brIRSensorValue < 85 || blIRSensorValue < 85 || bcIRSensorValue < 85){
                 
                 if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85){
                 move(backward,255);
                 _delay_ms(350);
                 isFront = 1;
                 
                 }
                 
                 else {
                 move(forward,255);
                 _delay_ms(350);
                 isFront = 0;
                 }
                 
                 return scanState;
                 }
                 
                 else {
                 isFront = 0;
                 move(forward,255);
                 }
                 */
                
                isFront = 1;
                move(forward,255);
                
            }
            
            else if(fContactSensorValue == 0 && bContactSensorValue == 0 && lContactSensorValue == 0 && rContactSensorValue == 0){
                
                /*if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85 || brIRSensorValue < 85 || blIRSensorValue < 85 || bcIRSensorValue < 85){
                 
                 if (frIRSensorValue < 85 || flIRSensorValue < 85 || fcIRSensorValue < 85){
                 move(backward,255);
                 _delay_ms(350);
                 isFront = 1;
                 
                 }
                 
                 else {
                 move(forward,255);
                 _delay_ms(350);
                 isFront = 0;
                 }
                 
                 return scanState;
                 }
                 
                 else {
                 isFront = 0;
                 move(forward,255);
                 }*/
                
                isFront = 1;
                move(forward,255);
            }
            
            else if (fContactSensorValue == 1 && bContactSensorValue == 1 && lContactSensorValue == 0 && rContactSensorValue == 0){
                
                //return escapeState;
                
            }
            
            else {
                
            }
            
        }
        else {
            scanMoveState = 0;
            scanMove = 1;
            
            return scanState;
        }
        
    }
    return 0;
}

uint8_t escape(){
    //fawzi
    
    while (TRUE){
        readLineSensors();
        
        if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue == 1){
            return avoidLineState;
        }
        
        readContactSwitches();
        
        if(rContactSensorValue == 0){
            move(backward,255);
        }
        else if(lContactSensorValue == 0){
            move(backward,255);
        }
        else if(rContactSensorValue == 0 && lContactSensorValue == 0){
            move(backward,255);
        }
        
        else {
            scanMoveState = 0;
            scanMove = 1;
            
            return scanState;
        }
        
    }
    
    return 0;
}

uint8_t avoidLine() {
    //fawzi
    //uint8_t tempflLine = 0;
    //uint8_t tempfrLine = 0;
    //uint8_t tempblLine = 0;
    //uint8_t tempbrLine = 0;
    
    while (TRUE){
        
        readLineSensors();
        
        if (flLineSensorValue == 1 || frLineSensorValue == 1 || blLineSensorValue == 1 || brLineSensorValue == 1){
            
            if (flLineSensorValue == 1 && frLineSensorValue == 1 && blLineSensorValue == 0 && brLineSensorValue == 0){
                move(backward,255);
                _delay_ms(350);
                
                isFront = 0;
            }
            
            else if (flLineSensorValue == 0 && frLineSensorValue == 0 && blLineSensorValue == 1 && brLineSensorValue == 1){
                move(forward,255);
                _delay_ms(350);
                isFront = 1;
            }
            
            else if (flLineSensorValue == 0 && frLineSensorValue == 0 && blLineSensorValue == 0 && brLineSensorValue == 1){
                //go fl
                
                move(frontright,255);
                _delay_ms(350);
                move(forward,255);
                _delay_ms(350);
                isFront = 1;
            }
            
            else if (flLineSensorValue == 0 && frLineSensorValue == 0 && blLineSensorValue == 1 && brLineSensorValue == 0){
                //go fr
                move(frontleft,255);
                _delay_ms(350);
                move(forward,255);
                _delay_ms(350);
                
                isFront = 1;
                
            }
            
            else if (flLineSensorValue == 0 && frLineSensorValue == 1 && blLineSensorValue == 0 && brLineSensorValue == 0){
                //go bl
                move(backleft,255);
                _delay_ms(350);
                move(backward,255);
                _delay_ms(350);
                
                isFront = 0;
                
            }
            
            else if (flLineSensorValue == 1 && frLineSensorValue == 0 && blLineSensorValue == 0 && brLineSensorValue == 0){
                
                move(backright,255);
                _delay_ms(350);
                move(backward,255);
                _delay_ms(350);
                
                isFront = 0;
                
                
            }
            
            else if (flLineSensorValue == 1 && frLineSensorValue == 0 && blLineSensorValue == 1 && brLineSensorValue == 0){
                //left wheel outside go right
                move(frontright,255);
                _delay_ms(350);
                move(forward,255);
                _delay_ms(500);
                isFront = 1;
                
            }
            
            else if (flLineSensorValue == 0 && frLineSensorValue == 1 && blLineSensorValue == 0 && brLineSensorValue == 1){
                //right wheel outside go left
                move(frontleft,255);
                _delay_ms(350);
                move(forward,255);
                _delay_ms(500);
                isFront = 1;
            }
            
            else if (flLineSensorValue == 0 && frLineSensorValue == 1 && blLineSensorValue == 1 && brLineSensorValue == 0){
                //wheel outside turn 90 right & go forward
                //return returnToRing;
            }
            
            else if (flLineSensorValue == 0 && frLineSensorValue == 0 && blLineSensorValue == 0 && brLineSensorValue == 1){
                //wheel outside turn 90  and go 
                
            }
            
            else {
                
            }
        }
        
        else {
            scanMoveState = 0;
            scanMove = 1;
            
            return scanState;
        }
    }
    return 0;
}

uint8_t returnToRing() {
    //pierre-marc
    
    /* uint8_t firstValue = *returnToRingArray;
     uint8_t secondValue = *(returnToRingArray + 1);
     uint8_t thirdValue = *(returnToRingArray + 2);
     uint8_t fourthValue = *(returnToRingArray + 3);
     */
    uint16_t counter2;
    
    for (counter2 = 0; counter2 < 50000; counter2++) {
        readLineSensors();
        
        
    }
    
    return losingOutputState;
    
    return 0;
}

#define C4  262
#define Cc4 277
#define D4  294
#define Dc4 311
#define E4  330
#define F4  349
#define Fc4 370
#define G4  392
#define Gc4 415
#define A4  440
#define Ac4 466
#define B4  494

#define C5  523
#define Cc5 554
#define D5  587
#define Dc5 622
#define E5  659
#define F5  698
#define Fc5 740
#define G5  783
#define Gc5 831
#define A5  880
#define Ac5 932
#define B5  988
#define C6  1047

#define D3 147
#define Gc3 208
#define Ac3 233

void PlayNotes(unsigned int note_frequency,unsigned int duration)
{
    unsigned int top_value,duty_cycle;    // Calculate the Top Value
    // TOP = Board Clock Frequency / (2 x N x Notes Frequency)
    // Where N is Prescler: 8
    top_value=(F_CPU / (16 * note_frequency));    // Reset the TIMER1 16 bit Counter
    TCNT1H = 0;
    TCNT1L = 0;
    
    // Set the TIMER1 Counter TOP value on ICR1H and ICR1L
    ICR1H = (top_value >> 8 ) & 0x00FF;
    ICR1L = top_value;   
    
    // Set the TIMER1 PWM Duty Cycle on OCR1AH and OCR1AL
    // Always use half of the TOP value (PWM Ducty Cycle ~ 50%)
    duty_cycle=top_value / 2;   
    OCR1AH=(duty_cycle >> 8 ) & 0x00FF;
    OCR1AL=duty_cycle;    // Turn ON the TIMER1 Prescaler of 8
    TCCR1B |= (1<<CS11); 
    
    // Notes Delay Duration
    
    if (duration == 500)
    {
        _delay_ms(500); //4th
    }
    
    else if (duration == 250){
	    _delay_ms(250);//8th
    }
    
    else if (duration == 182){
	    _delay_ms(182);
    }
    
	else if (duration == 1000){
		_delay_ms(1000);	
	}
	
	else if (duration == 1300){
		_delay_ms(1300);	
	}				
	
	else if (duration == 325){
		_delay_ms(325);	
	}		
	
	else if (duration == 118){
		_delay_ms(118);	
	}
    
    // Turn OFF the TIMER1 Prescaler of 8
    TCCR1B &= ~(1<<CS11);    // Delay Between Each Notes 1/5 duration
    
    if (duration == 500)
    {
        _delay_ms(100); //4th
    }
    
    else if (duration == 250){
	    _delay_ms(50);//8th
    }
    
    else if (duration == 182){
        _delay_ms(36); //3plet
    }
    
    else if (duration == 1000){
        _delay_ms(200); //1/2
    }
    
   	else if (duration == 1300){
		_delay_ms(260);	
	}				
	
	else if (duration == 325){
		_delay_ms(65);	
	}		
	
	else if (duration == 118){
		_delay_ms(24);	
	}
    
	
}

uint8_t losingOutput() {
    
    unsigned int notes[10]={C5,G4,E4,A4,B4,A4,Gc4,Ac4,Gc4,G4};    	         
    
    PlayNotes(notes[0],250);
    PlayNotes(notes[1],250);
    PlayNotes(notes[2],500);
    PlayNotes(notes[3],182);
    PlayNotes(notes[4],182);
    PlayNotes(notes[5],182);
    PlayNotes(notes[6],182);
    PlayNotes(notes[7],182);
    PlayNotes(notes[8],182);
    PlayNotes(notes[9],1000);
    
    return 0;
    
}

uint8_t winningOutput(){
	
	unsigned int notes[27]={D3,C4,E4,G4,C5,E5,G5,E5,Gc3,Cc4,F4,Gc4,Cc5,F5,Gc5,F5,Ac3,Dc4,G4,Ac4,Dc5,G5,Ac5,Ac5,Ac5,Ac5,C6}; 	         
    
    PlayNotes(notes[0],118);
    PlayNotes(notes[1],118);
    PlayNotes(notes[2],118);
    PlayNotes(notes[3],118);
    PlayNotes(notes[4],118);
    PlayNotes(notes[5],118);
    PlayNotes(notes[6],325);
    PlayNotes(notes[7],325);
    PlayNotes(notes[8],118);
    PlayNotes(notes[9],118);
    PlayNotes(notes[10],118);
    PlayNotes(notes[11],118);
    PlayNotes(notes[12],118);
    PlayNotes(notes[13],118);
    PlayNotes(notes[14],325);
    PlayNotes(notes[15],325);
    PlayNotes(notes[16],118);
    PlayNotes(notes[17],118);
    PlayNotes(notes[18],118);
    PlayNotes(notes[19],118);
    PlayNotes(notes[20],118);
    PlayNotes(notes[21],118);
    PlayNotes(notes[22],325);
    PlayNotes(notes[23],118);
    PlayNotes(notes[24],118);
    PlayNotes(notes[25],118);
    PlayNotes(notes[26],1300);
    
}


int main(){
    
    setup(); //setting up the ports
    _delay_ms(5000); //wait state
    initialize(); //initialize states
	
	while (TRUE){
		state = winningOutput();
		_delay_ms(1000);
		state = losingOutput();
		_delay_ms(1000);	
	}
    
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





