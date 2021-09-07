/*
 * Line_Following_Sensor.c
 *
 * Created: 7/9/2019 10:00:32 AM
 * Edited: 6/23/2021
 * Final Edits: 7/1/2021
 * Author : Nick Carter, Quinn Glancy, AdriAnne Welton, Arshpreet Singh
 *
 * Operation:
 *  Code is used to move autonomous vehicle along black taped path. Whilst the
 *  the car is off the line it will spin in a circle until it finds the path.
 *  Code works with conjunction with.
 *	
 *	IMPORTANT NOTE: Make sure to calibrate the line sensors before make any edits to the code. The sensors show a red light when over white and are otherwise considered to be over black.
 *
 * Hardware:
 *  Inputs:
 *    line following sensors (VCC,GND,SIG) x4  -> PA.0 - PA.3 -> Pins 22-25
 *  Outputs:
 *    DC motors(VCC,GND,sig1-3) x2  PWM -> PE.3 & PH.3 -> Pins 5 & 6
 *                    Dir - > PL.0-> Pins 49
 */ 

#define F_CPU 16000000UL							// set speed to 16MHz

#include <avr/io.h>									// include standard io and syntax
#include <util/delay.h>								// include delay functions.
#include <avr/interrupt.h>

#define Left_sensor 1								//Left Sensor is number 1
#define Left_middle_sensor 2						//Left Middle Sensor is number 2
#define Right_middle_sensor 4						//Right Middle Sensor is number 4
#define Right_sensor 8								//Right Sensor is number 8

#define Line_following_sensor (PINA & 0x0F)			//use in case statement

#define forward 1									//define forward for direction control
#define reverse 0									//define reverse for direction control


#define one_meter 1									//define one_meter as number 1 for distance sensing



// function prototypes
void init_io (void);

void init_timer3_phase_pwm (void);
void PWM_phase_left (uint8_t duty_cycle);

void init_timer4_phase_pwm (void);
void PWM_phase_right (uint8_t duty_cycle);


// global variables
uint8_t left_speed = 0;
uint8_t right_speed = 0;


int main(void)
{
  // initialize direction variables
  uint8_t left_direction = 0; 
  uint8_t right_direction = 0;
  // run initialization functions
  init_io();
  init_timer3_phase_pwm();
  init_timer4_phase_pwm();

    while (1) 
    {

    switch (Line_following_sensor)								//Case Statement to monitor Line Following Sensors
    {
      case Left_sensor:											//The left Sensor is on the black line
       {
        left_direction =forward;								// set left direction as going reverse
        right_direction = reverse;								// set right direction as going forward
        right_direction = (right_direction << 1);				// shift right direction over one bit
        PORTL = left_direction | right_direction;				// set PORTL as direction port
        left_speed = 75;										// set default left speed to 28
        right_speed = 28;										// set default right speed to 75
		
		//FOR PING SENSORS
// 		        if(PIND & one_meter)							// if there is something within 1 meter
// 		        {
// 		          left_speed = 0;								// set left speed to 0
// 		          right_speed = 0;								// set right speed to 0
// 		        }
				
          PWM_phase_left(left_speed);							// Have left wheels move at 28% Duty cycle
          PWM_phase_right(right_speed);							// Have right wheels move at 75% Duty cycle
	   }
      break;
      
      case Left_sensor | Left_middle_sensor:				//The left and Left Middle Sensors are on the black line
       {    
        left_direction = forward;							// set left direction as going reverse
        right_direction = reverse;							// set right direction as going forward
        right_direction = (right_direction << 1);			// shift right direction over one bit
        PORTL = left_direction | right_direction;			// set PORTL as direction port
        left_speed = 71;									// set default left speed to 26
        right_speed = 26;									// set default right speed to 71
		
// 		if(PIND & one_meter)								// if there is something within 1 meter
// 		{
// 			left_speed = 0;									// set left speed to 0
// 			right_speed = 0;								// set right speed to 0
// 		}	
		
          PWM_phase_left(left_speed);						// Have left wheels move at 26% Duty cycle
          PWM_phase_right(right_speed);						// Have right wheels move at 71% Duty cycle
       
	   }
      break;
      
      case Left_middle_sensor:								//The left Middle Sensor is on the black line
       {
        left_direction = forward;							// set left direction as going reverse
        right_direction = reverse;							// set right direction as going forward
        right_direction = (right_direction << 1);			// shift right direction over one bit
        PORTL = left_direction | right_direction;			// set PORTL as direction port
        left_speed = 65;									// set default speed to 24
        right_speed = 24;									// set default speed to 65

//         if(PIND & one_meter)								// if there is something within 1 meter
//         {
// 	        left_speed = 0;									// set left speed to 0
// 	        right_speed = 0;								// set right speed to 0
//         }
          
		  PWM_phase_left(left_speed);						// Have left wheels move at 55% Duty cycle
          PWM_phase_right(right_speed);						// Have right wheels move at 55% Duty cycle
       }
      break;
      
      case Left_middle_sensor | Right_middle_sensor:		//The left Middle and Right Middle Sensors are on the black line
       {
        left_direction = forward;							// set left direction as going forward
        right_direction = forward;							// set right direction as going forward
        right_direction = (right_direction << 1);			// shift right direction over one bit
        PORTL = left_direction | right_direction;			// set PORTL as direction port
        left_speed = 45;									// set default speed to 45
        right_speed = 45;									// set default speed to 45
		
//         if(PIND & one_meter)								// if there is something within 1 meter
//         {
// 	        left_speed = 0;									// set left speed to 0
// 	        right_speed = 0;								// set right speed to 0
//         }
		
          PWM_phase_left(left_speed);						// Have left wheels move at left_speed
          PWM_phase_right(right_speed);						// Have right wheels move at right_speed
       }
      break;
      
      case Right_middle_sensor:								//The Right Middle Sensor is on the black line
       {    
        left_direction = reverse;							// set left direction as going forward
        right_direction = forward;							// set right direction as going reverse
        right_direction = (right_direction << 1);			// shift right direction over one bit
        PORTL = left_direction | right_direction;			// set PORTL as direction port
        left_speed = 24;									// set default speed to 65
        right_speed = 65;									// set default speed to 24
		
//         if(PIND & one_meter)								// if there is something within 1 meter
//         {
// 	        left_speed = 0;									// set left speed to 0
// 	        right_speed = 0;								// set right speed to 0
//         }
		PWM_phase_left(left_speed);							// Have left wheels move at left_speed
        PWM_phase_right(right_speed);						// Have right wheels move at right_speed
       }
      break;
      
      case Right_middle_sensor | Right_sensor:				//The Right Middle and Right Sensors are on the black line
       {
        left_direction = reverse;							// set left direction as going forward
        right_direction = forward;							// set right direction as going reverse
        right_direction = (right_direction << 1);			// shift right direction over one bit
        PORTL = left_direction | right_direction;			// set PORTL as direction port
        left_speed = 26;									// set default speed to 71
        right_speed = 71;									// set default speed to 26
		
// 	if(PIND & one_meter)									// if there is something within 1 meter
// 	{
// 		left_speed = 0;										// set left speed to 0
// 		right_speed = 0;									// set right speed to 0
// 	}
        
        PWM_phase_left(left_speed);							// Have left wheels move at left_speed
        PWM_phase_right(right_speed);						// Have right wheels move at right_speed
       }
      break;
 
      case Right_sensor:									//The Right Sensor is on the black line
       {                
        left_direction = reverse;							// set left direction as going forward
        right_direction = forward;							// set right direction as going reverse
        right_direction = (right_direction << 1);			// shift right direction over one bit
        PORTL = left_direction | right_direction;			// set PORTL as direction port
        left_speed = 28;									// set default left speed to 75
        right_speed = 75;									// set default right speed to 28
		
//         if(PIND & one_meter)								// if there is something within 1 meter
//         {
// 	        left_speed = 0;									// set left speed to 0
// 	        right_speed = 0;								// set right speed to 0
//         }
      
          PWM_phase_left(left_speed);						// Have left wheels move at left_speed
			PWM_phase_right(right_speed);					// Have right wheels move at right_speed
       }
      break;
	    
	  //U-TURN Code
	  case Left_sensor|Left_middle_sensor|Right_middle_sensor|Right_sensor:		//When all Sensors are off
	  {	
				//Moving Forward for 750 ms
				left_direction=forward;
				right_direction=forward;
				right_direction=(right_direction<<1);
				PORTL=left_direction|right_direction;
				left_speed=45;
				right_speed=45;
				PWM_phase_left(left_speed);
				PWM_phase_right(right_speed);
				_delay_ms(750);
		 
				//U-TURN, MIGHT NEED TO EDIT DELAY AND SPEEDS TO MAKE SURE EEVRYTHING IS PROPER
				left_direction = forward;					// set left direction as going forward
				right_direction = reverse;					// set right direction as going forward
				right_direction = (right_direction << 1);   // shift right direction over one bit
				PORTL = left_direction | right_direction;   // set PORTL as direction port
				left_speed =62;
				right_speed =62;
				PWM_phase_left(left_speed);					// Have left wheels move at left_speed
				PWM_phase_right(right_speed);				// Have right wheels move at right_speed
				_delay_ms(2125);
			  
				//Moving Back 750 ms
				left_direction=forward;
				right_direction=forward;
				right_direction=(right_direction<<1);
				PORTL=left_direction|right_direction;
				left_speed=45;
				right_speed=45;
				PWM_phase_left(left_speed);
				PWM_phase_right(right_speed);
				_delay_ms(750);
				continue;
		  
	  }
	  break;

  
  
      
	  default:											// If the car isn't over the line
      {
        left_direction = forward;						// set left direction as going forward
        right_direction = reverse;						// set right direction as going forward
        right_direction = (right_direction << 1);		// shift right direction over one bit
        PORTL = left_direction | right_direction;		// set PORTL as direction port
        left_speed =0;
        right_speed =0;
        PWM_phase_left(left_speed);						// Have left wheels move at left_speed
        PWM_phase_right(right_speed);					// Have right wheels move at right_speed
      }
      break;
    }
    _delay_ms(25);                      // 25 millisecond delay to allow smooth transition between states
    }
}



void init_io (void)     //Initialize DC Motor IO
{
  //timer 3 for PWM for left motors
  DDRE = 0xFF;        //Set PORT E as outputs
  PORTE = 0x00;       //Turn off PORT E pull up Resistors
  
  // timer 3 for PWM for right motors
  DDRH = 0xFF;        //Set PORT H as outputs
  PORTH = 0x00;       //Turn off PORT H pull up Resistors
  
  // init PORTL as output for directions
  DDRL = 0xFF;        //Set PORT L as outputs
  PORTL = 0x00;       //initialize PORTL as all off
  
  // init PORTA as input for light sensors
  DDRA = 0x00;        //Initialize Port A as an input
  PORTA = 0xFF;       // enable all pull up resistors

  //init PORTD as input for ping sensor
  DDRD = 0x00;        //Initialize Port D as input
  PORTD = 0xFF;       // enable all pull up resistors
  
  // init PORTC as output for testing
  DDRC = 0xFF;        //Set PORTC as outputs
  DDRC = 0x00;        //Initialize PORTC as all off

}

void init_timer3_phase_pwm (void) // initialization function for PWM 2
{
  /*
  PWM, phase correct, 8 bit
  Clear OC3A on compare match when up-counting, set OC3A on compare match when down-counting
  Pre-scaler of 64
  */
  TCCR3A = (1<<COM3A1) | (1<<WGM30); 
  TCCR3B = (1<<CS31) | (1<<CS30); 
  OCR3A = 0;
  
}
void PWM_phase_left (uint8_t duty_cycle) // Duty cycle function for left motors
{
  OCR3A = (((uint32_t)duty_cycle * 255)/100); // input duty cycle to set compare match value
}

void init_timer4_phase_pwm (void) // initialization function for PWM 1
{
  /*
  PWM, phase correct, 8 bit
  Clear OC4A on compare match when up-counting, set OC4A on compare match when down-counting
  Pre-scaler of 64
  */
  TCCR4A = (1<<COM4A1) | (1<<WGM40);
  TCCR4B = (1<<CS41) | (1<<CS40);
  OCR4A = 0;
}

void PWM_phase_right (uint8_t duty_cycle) // Duty cycle function for right motors
{
  OCR4A = (((uint32_t)duty_cycle * 255)/100); // input duty cycle to set compare match value
}


