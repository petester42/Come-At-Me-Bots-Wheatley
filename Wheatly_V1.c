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

//PWM
#define PWMLeftMotorDir DDB1
#define PWMRightMotorDir DDB2
#define PWMSpeakerDir DDB3

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
    
    DDRB |= (1 << PWMSpeakerDir) | (1 << PWMLeftMotorDir) | (1 << PWMRightMotorDir) | (1 << HighLeftMotorDir) | (1 << LowLeftMotorDir) | (1 << HighRightMotorDir) | (1 << LowRightMotorDir); 
    
    //set up motor PWM
    
    TCCR1A = 0; 
    
    TCCR1A |= (1<<WGM13) | (1<<WGM10); // Phase & Frequency Correct PWM
    
    TCCR1B = 0;
    
    TCCR1B = (1 << WGM13) | (1 << CS10); // Phase and Frequency Correct PWM mode, with no prescaling
    
    TCCR1A = (1 << COM1A1) | (1 << COM1B1); // enable channel A & B in non-inverting mode
    
    
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

void move (uint8_t direction, uint8_t speed){
    //pierre-marc
    //speed are values from 0-255
    //OC1A left motor
    //OC1B right motor
    
    // HighLeftMotorPort PORTD7 //1A
    // LowLeftMotorPort PORTB0 //2A
    // HighRightMotorPort PORTD6 //4A
    // LowRightMotorPort PORTD5 //3A
    
    switch (direction) {
            
        case forward:
            PORTD | = (0 << HighLeftMotorPort);
            PORTB | = (1 << LowLeftMotorPort);
            PORTD | = (0 << HighRightMotorPort);
            PORTD | = (1 << LowRightMotorPort);
            
            OCR1A = speed;
            OCR1B = speed;
            
            break;
            
        case backward:
            PORTD | = (1 << HighLeftMotorPort);
            PORTB | = (0 << LowLeftMotorPort);
            PORTD | = (1 << HighRightMotorPort);
            PORTD | = (0 << LowRightMotorPort);
            
            OCR1A = speed;
            OCR1B = speed;
            
            break;
            
        case frontleft:
            PORTD | = (0 << HighLeftMotorPort);
            PORTB | = (0 << LowLeftMotorPort);
            PORTD | = (0 << HighRightMotorPort);
            PORTD | = (1 << LowRightMotorPort);
            
            OCR1A = speed;
            OCR1B = 0;
            
            break;
            
        case frontright:
            PORTD | = (0 << HighLeftMotorPort);
            PORTB | = (1 << LowLeftMotorPort);
            PORTD | = (0 << HighRightMotorPort);
            PORTD | = (0 << LowRightMotorPort);
            
            OCR1A = 0;
            OCR1B = speed;
            
            break;
            
        case backleft:
            PORTD | = (0 << HighLeftMotorPort);
            PORTB | = (0 << LowLeftMotorPort);
            PORTD | = (1 << HighRightMotorPort);
            PORTD | = (0 << LowRightMotorPort);
            
            OCR1A = speed;
            OCR1B = 0;
            
            break;
            
        case backright:
            PORTD | = (1 << HighLeftMotorPort);
            PORTB | = (0 << LowLeftMotorPort);
            PORTD | = (0 << HighRightMotorPort);
            PORTD | = (0 << LowRightMotorPort);
            
            OCR1A = 0;
            OCR1B = speed;
            
            break;
            
        case brake:
            PORTD | = (0 << HighLeftMotorPort);
            PORTB | = (0 << LowLeftMotorPort);
            PORTD | = (0 << HighRightMotorPort);
            PORTD | = (0 << LowRightMotorPort);
            
            OCR1A = 0;
            OCR1B = 0;
            
            break;
            
        default:
            PORTD | = (0 << HighLeftMotorPort);
            PORTB | = (0 << LowLeftMotorPort);
            PORTD | = (0 << HighRightMotorPort);
            PORTD | = (0 << LowRightMotorPort);
            
            OCR1A = 0;
            OCR1B = 0;
            
            break;
    }
    
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
        
	while(TRUE){
    if (frLineSensorValue == 0 || flLineSensorValue == 0 || brLineSensorValue == 0 || blLineSensorValue == 0){
		//move backwards
		return backward;
	}
			
	else if (frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue == 1){
		//move away from line
		return avoidLineState;
		_delay_ms(2000);
	}
	return 0;
}

uint8_t scan(){
    //fawzi
    while (TRUE) {
        
        readLineSensors();
		readContactSwitches();
		readIRSensors();
		if(frLineSensorValue == 0 && flLineSensorValue == 0 && brLineSensorValue == 0 && blLineSensorValue == 0 
		   && fContactSensorValue == 0 && bContactSensorValue == 0 && rContactSensorValue == 0 && lContactSensorValue == 0
		   && frIRSensorValue > 200 && fcIRSensorValue > 200 && flIRSensorValue > 200 
		   && brIRSensorValue > 200 && bcIRSensorValue > 200 && blIRSensorValue > 200){
			//turn 60 degrees right
			/*move forward
			_delay_ms(2000);
			//turn 120 degrees left
			//move forward
			_delay_ms(2000); */
		}
		
		/*check all sensors and return position, push, avoidLine or escape state accordingly*/
		
		else if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue = 1){
			return avoidLineState;
		}
		
		else if(fContactSensorValue == 1 || bContactSensorValue == 1 || rContactSensorValue == 1 || lContactSensorValue == 1){
			return pushState;
		}	
		
        else if(frIRSensorValue < 200 || fcIRSensorValue < 200 || flIRSensorValue < 200 
		   || brIRSensorValue < 200 || bcIRSensorValue < 200 || blIRSensorValue < 200){
			   return positionState;
		   }
		   
	    else if(lContactSensorValue == 1 && rContactSensorValue ==1){
			return escapeState;
		}			


		readLineSensors();
	    readContactSwitches();
	    readIRSensors();
		if(frLineSensorValue == 0 && flLineSensorValue == 0 && brLineSensorValue == 0 && blLineSensorValue == 0 
		   && fContactSensorValue == 0 && bContactSensorValue == 0 && rContactSensorValue == 0 && lContactSensorValue == 0
		   && frIRSensorValue > 200 && fcIRSensorValue > 200 && flIRSensorValue > 200 
		   && brIRSensorValue > 200 && bcIRSensorValue > 200 && blIRSensorValue > 200){
			return forward;
			_delay_ms(2000);
		}

		/*check all sensors and return position, push, avoidLine or escape state accordingly*/
		
		else if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue == 1){
			return avoidLineState;
		}
		
		else if(fContactSensorValue == 1 || bContactSensorValue == 1 || rContactSensorValue == 1 || lContactSensorValue == 1){
			return pushState;
		}	
		
        else if(frIRSensorValue < 200 || fcIRSensorValue < 200 || flIRSensorValue < 200 
		   || brIRSensorValue < 200 || bcIRSensorValue < 200 || blIRSensorValue < 200){
			   return positionState;
		   }
		   
	    else if(lContactSensorValue == 1 && rContactSensorValue ==1){
			return escapeState;
		}

		readLineSensors();
	    readContactSwitches();
	    readIRSensors();
		if(frLineSensorValue == 0 && flLineSensorValue == 0 && brLineSensorValue == 0 && blLineSensorValue == 0 
		   && fContactSensorValue == 0 && bContactSensorValue == 0 && rContactSensorValue == 0 && lContactSensorValue == 0
		   && frIRSensorValue > 200 && fcIRSensorValue > 200 && flIRSensorValue > 200 
		   && brIRSensorValue > 200 && bcIRSensorValue > 200 && blIRSensorValue > 200){
			//turn 120 degrees left
		}
		
		/*check all sensors and return position, push, avoidLine or escape state accordingly*/
		
		else if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue = 1){
			return avoidLineState;
		}
		
		else if(fContactSensorValue == 1 || bContactSensorValue == 1 || rContactSensorValue == 1 || lContactSensorValue == 1){
			return pushState;
		}	
		
        else if(frIRSensorValue < 200 || fcIRSensorValue < 200 || flIRSensorValue < 200 
		   || brIRSensorValue < 200 || bcIRSensorValue < 200 || blIRSensorValue < 200){
			   return positionState;
		   }
		   
	    else if(lContactSensorValue == 1 && rContactSensorValue ==1){
			return escapeState;
		}

		readLineSensors();
	    readContactSwitches();
	    readIRSensors();
		if(frLineSensorValue == 0 && flLineSensorValue == 0 && brLineSensorValue == 0 && blLineSensorValue == 0 
		   && fContactSensorValue == 0 && bContactSensorValue == 0 && rContactSensorValue == 0 && lContactSensorValue == 0
		   && frIRSensorValue > 200 && fcIRSensorValue > 200 && flIRSensorValue > 200 
		   && brIRSensorValue > 200 && bcIRSensorValue > 200 && blIRSensorValue > 200){
			return forward; 
			_delay_ms(2000);
		}
		
		/*check all sensors and return position, push, avoidLine or escape state accordingly*/
		
		else if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue = 1){
			return avoidLineState;
		}
		
		else if(fContactSensorValue == 1 || bContactSensorValue == 1 || rContactSensorValue == 1 || lContactSensorValue == 1){
			return pushState;
		}	
		
        else if(frIRSensorValue < 200 || fcIRSensorValue < 200 || flIRSensorValue < 200 
		   || brIRSensorValue < 200 || bcIRSensorValue < 200 || blIRSensorValue < 200){
			   return positionState;
		   }
		   
	    else if(lContactSensorValue == 1 && rContactSensorValue ==1){
			return escapeState;
		}	

		//loop back to beginning of function
		//too much sensor checking
		//there must be an easier way to check sensors		
               
    }
    
    return 0;
}

uint8_t position(){
    //fawzi
	readLineSensors();
	readContactSwitches();
	readIRSensors();
	
	if (frIRSensorValue < 200){
		return frontright;
	}
	
	else if (flIRSensorValue < 200){
		return frontleft;
	}	

	else if (frIRSensorValue < 200 && flIRSensorValue < 200){
		return pushState;
	
	 else if (brIRSensorValue < 200){
		return backright;
	}
	
	else if (brIRSensorValue < 200){
		return backleft;
	}	

	else if (brIRSensorValue < 200 && blIRSensorValue < 200){
		return pushState;
	}
	
	else if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue = 1){
		return avoidLineState;
	}
		
	else if(rContactSensorValue == 1 || lContactSensorValue == 1){
		return escapeState;
	}
	
	else if(frLineSensorValue == 0 && flLineSensorValue == 0 && brLineSensorValue == 0 && blLineSensorValue == 0 
		   && fContactSensorValue == 0 && bContactSensorValue == 0 && rContactSensorValue == 0 && lContactSensorValue == 0
		   && frIRSensorValue > 200 && fcIRSensorValue > 200 && flIRSensorValue > 200 
		   && brIRSensorValue > 200 && bcIRSensorValue > 200 && blIRSensorValue > 200){
		return scanState;
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

	if(fContactSensorValue == 0 && bContactSensorValue == 1 && lContactSensorValue == 0 && rContactSensorValue == 0){
	  return backward;
	}
	
	else if(fContactSensorValue == 0 && bContactSensorValue == 1 && lContactSensorValue == 0 && rContactSensorValue == 1){
	  return backright;
	}
	
	else if(fContactSensorValue == 0 && bContactSensorValue == 1 && lContactSensorValue == 1 && rContactSensorValue == 0){
	  return backleft;
	}
	
	else if(fContactSensorValue == 0 && bContactSensorValue == 1 && lContactSensorValue == 1 && rContactSensorValue == 1){
	  return backward;
	}
	
	else if(fContactSensorValue == 1 && bContactSensorValue == 0 && lContactSensorValue == 0 && rContactSensorValue == 0){
	  return forward;
	}
	
	else if(fContactSensorValue == 1 && bContactSensorValue == 0 && lContactSensorValue == 0 && rContactSensorValue == 1){
	  return frontright;
	}
	
	else if(fContactSensorValue == 1 && bContactSensorValue == 0 && lContactSensorValue == 1 && rContactSensorValue == 0){
	  return frontleft;
	}
	
	else if(fContactSensorValue == 1 && bContactSensorValue == 0 && lContactSensorValue == 1 && rContactSensorValue == 1){
	 return forward;
	}
	
	else if(fContactSensorValue == 1 && bContactSensorValue == 1 && lContactSensorValue == 0 && rContactSensorValue == 0){
	  return forward;
	}
	
	else if(fContactSensorValue == 1 && bContactSensorValue == 1 && lContactSensorValue == 0 && rContactSensorValue == 1){
	  return frontright;
	}
	
	else if(fContactSensorValue == 1 && bContactSensorValue == 1 && lContactSensorValue == 1 && rContactSensorValue == 0){
	  return frontleft;
	}
	
	else if(fContactSensorValue == 1 && bContactSensorValue == 1 && lContactSensorValue == 1 && rContactSensorValue == 1){
	  return forward;
	}
	
	else if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue = 1){
		return avoidLineState;
	}
		
	else if(rContactSensorValue == 1 || lContactSensorValue == 1){
		return escapeState;
	}
	
	else if(frLineSensorValue == 0 && flLineSensorValue == 0 && brLineSensorValue == 0 && blLineSensorValue == 0 
		   && fContactSensorValue == 0 && bContactSensorValue == 0 && rContactSensorValue == 0 && lContactSensorValue == 0
		   && frIRSensorValue > 200 && fcIRSensorValue > 200 && flIRSensorValue > 200 
		   && brIRSensorValue > 200 && bcIRSensorValue > 200 && blIRSensorValue > 200){
		return scanState;
	}
	
    return 0;
}

uint8_t escape(){
    //fawzi
	readLineSensors();
	readContactSwitches();
	readIRSensors();

	if(rContactSensorValue == 1){
		return frontleft;
	}
	else if(lContactSensorValue == 1){
		return frontright;
	}
	else if(rContactSensorValue == 1 && lContactSensorValue == 1){
		return forward;
	}
	
	else if(frLineSensorValue == 1 || flLineSensorValue == 1 || brLineSensorValue == 1 || blLineSensorValue = 1){
		return avoidLineState;
	}
	
	else if(frLineSensorValue == 0 && flLineSensorValue == 0 && brLineSensorValue == 0 && blLineSensorValue == 0 
		   && fContactSensorValue == 0 && bContactSensorValue == 0 && rContactSensorValue == 0 && lContactSensorValue == 0
		   && frIRSensorValue > 200 && fcIRSensorValue > 200 && flIRSensorValue > 200 
		   && brIRSensorValue > 200 && bcIRSensorValue > 200 && blIRSensorValue > 200){
		return scanState;
	}
    return 0;
}

uint8_t avoidLine() {
    //fawzi
	readLineSensors();
	readContactSwitches();
	readIRSensors();
	
	if (frLineSensorValue == 0 && flLineSensorValue == 0 && brLineSensorValue == 0 && blLineSensorValue == 0 
		   && fContactSensorValue == 0 && bContactSensorValue == 0 && rContactSensorValue == 0 && lContactSensorValue == 0
		   && frIRSensorValue > 200 && fcIRSensorValue > 200 && flIRSensorValue > 200 
		   && brIRSensorValue > 200 && bcIRSensorValue > 200 && blIRSensorValue > 200 ){
		return scanState;
	}
	
	else if (flLineSensorValue == 0 && frLineSensorValue == 0 && blLineSensorValue == 0 && brLineSensorValue = 1 ){
		return frontleft;
	}
	
	else if (flLineSensorValue == 0 && frLineSensorValue == 0 && blLineSensorValue == 1 && brLineSensorValue = 0 ){
		return frontright;
	}
	
	else if (flLineSensorValue == 0 && frLineSensorValue == 0 && blLineSensorValue == 1 && brLineSensorValue = 1 ){
		return forward;
	}
	
	else if (flLineSensorValue == 0 && frLineSensorValue == 1 && blLineSensorValue == 0 && brLineSensorValue = 0 ){
		return backleft;
	}
	
	else if (flLineSensorValue == 0 && frLineSensorValue == 1 && blLineSensorValue == 0 && brLineSensorValue = 1 ){
		return frontleft;
	}
	
	else if (flLineSensorValue == 1 && frLineSensorValue == 0 && blLineSensorValue == 0 && brLineSensorValue = 0 ){
		return backright;
	}
	
	else if (flLineSensorValue == 1 && frLineSensorValue == 0 && blLineSensorValue == 1 && brLineSensorValue = 0 ){
		return frontright;
	}
	
	else if (flLineSensorValue == 1 && frLineSensorValue == 1 && blLineSensorValue == 0 && brLineSensorValue = 0 ){
		return backward;
	}
	
	
	if (/* pushed over line */){
		return returnToRingState;
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