/* ------------------------------------------
   Motor Demo
     Show green     
     When button pressed, run motor slowly clockwise; show red
     Stop on button press
     Run motor quickly backwards to starting position 
   Stepping is done by counting 10ms cycles
     Fast: every cycle - 1 Rev in 480ms - 100pps, 125 RPM
     Slow: every 5 cycles - 20 pps, 25 RPM
     
     The following GPIO pins are used for the motor
        Motor Cnnctn   Port E Pin
     -----------------------------
         IN1           pin 30       (phase A+)
         IN2           pin 29       (phase A-)
         IN3           pin 23       (phase B+)
         IN4           pin 22       (phase B-)
     -------------------------------------------- */

#include <MKL25Z4.H>
#include "../include/gpio_defs.h"
#include "../include/stepperMotor.h"
#include "../include/SysTick.h"
#include "../include/pit.h"
#include <stdbool.h>
#include <stdlib.h>

#define START (0)
#define MAKINGMOVE (1)
#define STOPPEDMOVE (2)
#define STOPPEDRET (3)
#define RETURNING (4) 
#define COMPLETED (5)

#define BUTTONOPEN (0)
#define BUTTONCLOSED (1)
#define BUTTONBOUNCE (2)

void calculateReturn();

/*----------------------------------------------------------------------------
  GPIO Configuration
  Configure the port B pin for the on-board red & green leds as an output
 *----------------------------------------------------------------------------*/
void configureGPIOoutput() {
        // Configuration steps
    //   1. Enable clock to GPIO ports
    //   2. Enable GPIO ports
    //   3. Set GPIO direction to output
    //   4. Ensure LEDs are off

    // Enable clock to ports B 
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK ;
    
    // Make the pin GPIO
    PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
    PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);          
    
    // Set ports to outputs
    PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS) ;

    // Turn off the LED
    PTB->PSOR = MASK(RED_LED_POS) | MASK(GREEN_LED_POS) ;
}

/*----------------------------------------------------------------------------
  GPIO Input Configuration
  Initialse a Port D pin as an input, with no interrupt
  Bit number given by BUTTON_POS
 *----------------------------------------------------------------------------*/ 
void configureGPIOinput(void) {
    SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; /* enable clock for port D */

    /* Select GPIO and enable pull-up resistors and no interrupts */
    PORTD->PCR[BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
    PORTD->PCR[BUTTON2_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);

    /* Set port D switch bit to inputs */
    PTD->PDDR &= ~MASK(BUTTON_POS);
	  PTD->PDDR &= ~MASK(BUTTON2_POS);
}

/*----------------------------------------------------------------------------
  Motor Configuration
 *----------------------------------------------------------------------------*/
motorType mcb ;   // motor control block
void configureMotor() {
    m1 = & mcb ;
    m1->port = PTE ;
    m1->bitAp = MOTOR_IN1 ;
    m1->bitAm = MOTOR_IN2 ;
    m1->bitBp = MOTOR_IN3 ;
    m1->bitBm = MOTOR_IN4 ;

    // Enable clock to port E
    SIM->SCGC5 |=  SIM_SCGC5_PORTE_MASK; /* enable clock for port E */
    
    // Initialise motor data and set to state 1
    initMotor(m1) ; // motor initially stopped, with step 1 powered
}

/*----------------------------------------------------------------------------
  ledOn: Set led LED on, assumes port B
  Note use of the clear register (c.f. the data output)
 *----------------------------------------------------------------------------*/    
void ledOn(int pos)
{
       // set led on without changing anything else
       // LED is actve low
         PTB->PCOR |= MASK(pos) ;
}

/*----------------------------------------------------------------------------
  ledOff: Set LED off, assumes port B
  Note use of the clear register (c.f. the data output)
 *----------------------------------------------------------------------------*/
void ledOff(int pos)
{
       // set led off with changing anything else
       // LED is actve low
         PTB->PSOR |= MASK(pos) ;
}

/*----------------------------------------------------------------------------
  isPressed: test the switch
  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.
 *----------------------------------------------------------------------------*/
bool isPressed(void) {
    if (PTD->PDIR & MASK(BUTTON_POS)) {
            return false ;
    }
    return true ;
}

bool isPressed2(void) {
    if (PTD->PDIR & MASK(BUTTON2_POS)) {
            return false ;
    }
    return true ;
}

/*----------------------------------------------------------------------------
  Poll the input
 Detect changes in the switch state.
    isPressed and not closed --> new press; 
     ~isPressed and closed -> not closed
*----------------------------------------------------------------------------*/
int b_state1 = BUTTONOPEN ;
int b_state2 = BUTTONOPEN ;
int pressed1 = 0 ;
int pressed2 = 0 ;
int bounceCounter1 = 0 ;
int bounceCounter2 = 0 ;

void task1PollInput1(void)
{
    if (bounceCounter1 > 0) bounceCounter1 -- ;
    
    switch (b_state1) {
        case BUTTONOPEN :
            if (isPressed()) {
                pressed1 = 1 ;  // create a 'pressed' event
                b_state1 = BUTTONCLOSED ;
            }
            break ;
        case BUTTONCLOSED :
            if (!isPressed()) {
                b_state1 = BUTTONBOUNCE ;
                bounceCounter1 = 50 ;
            }
            break ;
        case BUTTONBOUNCE :
            if (isPressed()) {
                b_state1 = BUTTONCLOSED ;
            }
            if (bounceCounter1 == 0) {
                b_state1 = BUTTONOPEN ;
            }
            break ;
    }
}

void task2PollInput2(void)
{
    if (bounceCounter2 > 0) bounceCounter2 -- ;
    
    switch (b_state2) {
        case BUTTONOPEN :
            if (isPressed2()) {
                pressed2 = 1 ;  // create a 'pressed' event
                b_state2 = BUTTONCLOSED ;
            }
            break ;
        case BUTTONCLOSED :
            if (!isPressed2()) {
                b_state2 = BUTTONBOUNCE ;
                bounceCounter2 = 50 ;
            }
            break ;
        case BUTTONBOUNCE :
            if (isPressed2()) {
                b_state2 = BUTTONCLOSED ;
            }
            if (bounceCounter2 == 0) {
                b_state2 = BUTTONOPEN ;
            }
            break ;
    }
}

/*----------------------------------------------------------------------------
   task3 Motor Control
       initially stopped
*----------------------------------------------------------------------------*/
#define CW (0) // false
#define CCW (1) // true

int i = 0;
bool dir = CW;
int counter = 0;
int ret_steps = 0;
int delay = 0;
int sys_state = START;
int turn_steps[6] = {64, 272, 736, 512, 960, 1472};
int pit_val_steps[6] = {3285012, 771920, 285002, 211475, 109227, 71250};
bool direction[6] = {CW, CW, CCW, CCW, CCW, CW};

void task3ControlMotor(void) {
    int steps ; 
    switch (sys_state) {
        case START:	  
            if (pressed1) { //runPress
                pressed1 = 0; // acknowledge
                moveSteps(m1, turn_steps[i], direction[i]); 
                setTimer(0,pit_val_steps[i]);
				startTimer(0);
                sys_state =  MAKINGMOVE;
            }
            break ;
						
        case MAKINGMOVE:
			if (!isMoving(m1)) {
				sys_state = COMPLETED;
			}
            if(pressed2) {//Stop Press
				pressed2 = 0; 
				stopTimer(0);
				stopMotor(0);
				sys_state = STOPPEDMOVE;
			}
            break;
						
        case STOPPEDMOVE:
            if (pressed1) {//Run Press
			    pressed1 = 0 ; 
			    startTimer(0);
                sys_state =  MAKINGMOVE;
            }
            break;
						
		case RETURNING:
			if (!isMoving(m1)) {
				sys_state = START;
			}
			if(pressed2) {//Stop Press
				pressed2 = 0 ; 
				stopTimer(0);
				stopMotor(0);
				sys_state = STOPPEDRET;
			}
			break;	

        case STOPPEDRET:
				if (pressed1) {//Run Press
					pressed1 = 0 ; 
					startTimer(0); 
                sys_state =  RETURNING;
            }
            break ;
				
		case COMPLETED:
			if (pressed1) {//Run Press
				pressed1 = 0 ; 
					
				calculateReturn();
				
				if (i < 5) {
					i++; // Go to next move
				}
				else 
				{
					i = 0;	// Reset moves
				}
				
				sys_state =  RETURNING;
			}
			break;
    }
}

void calculateReturn(){

	ret_steps = getSteps(m1); // Get the number of steps that have already been done
	
	if((ret_steps % 48) != 0) //The motor did not already go back to origin
	{					
		ret_steps = (abs(ret_steps)%48);
		
		if(ret_steps < 24) 
		// The motor passed less than half of the circle from either CW or CCW direction
		// The motor should reverse from whence it came
		{
			dir = !direction[i]; // ! complements boolean value rather than ~ bitwise complement
		}
		else 
		// The motor passed more than half of the circle
		// The motor should continue in the same direction
		{
			ret_steps = (48-ret_steps); // Find the remaining steps to complete the circle
			dir = direction[i];

		} 
		moveSteps(m1, ret_steps, dir) ; 
	}

}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/

int main (void) {
    configureGPIOoutput() ;
    configureGPIOinput() ;
    configureMotor() ;
		configurePIT(0) ;
    Init_SysTick(1000) ; // SysTick every ms
    waitSysTickCounter(10) ; // initialise counter
    
    while (1) {        

        task1PollInput1() ;
			  task2PollInput2();
        task3ControlMotor() ;
        waitSysTickCounter(10) ; // cycle every 10ms
    }
}