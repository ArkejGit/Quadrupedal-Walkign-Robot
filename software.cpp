#include <avr/io.h>
#include <util/delay.h>
#include <AVR/interrupt.h>

# define F_CPU 16000000UL

// variables declarations

volatile uint16_t counter=0, result, distans, distans_front, distans_left, distans_right;


ISR(INT0_vect)
{
distans = counter;
}

ISR(INT1_vect)
{
distans = counter;
}

ISR(INT2_vect)
{
distans = counter;
}

#define reload_value 49536;

ISR (TIMER1_OVF_vect)
	{
	 TCNT1 = reload_value;
	 counter++;
	}

// ********************************* FUNCTIONS ***************************************

#define BAUD 9600
#define MY_UBRR F_CPU/16/BAUD-1

//--------------------------------- UsartInit ---------------------------------------

void UsartInit(void)        
{
    UBRRH = (MY_UBRR >> 8);        //write high byte
    UBRRL = MY_UBRR;             //write low byte
 
    UCSRB = (1<<RXEN)|(1<<TXEN); //receiving and transmitting enable
    UCSRC = (1 << URSEL) | (3<<UCSZ0);     //asynchronous, no parity control, 1 stop bit, 8 data bits
 
} 
// ---------------------------------------------------------------------------------

//--------------------------------- TimerInit ---------------------------------------

void TimerInit()
{
TCCR1B |= (1<<CS10);	// prescaler
TCCR1B &=~ (1<<CS11);
TCCR1B &=~ (1<<CS12);

TIMSK|=(1<<TOIE1);	// overflow interrupt enabled

TCNT1 = reload_value;
}

// ---------------------------------------------------------------------------------

//------------------------------- InterruptsInit -----------------------------------

void InterruptInit()
{
MCUCR &=~ (1<<ISC00);	// set trigger INT0 interrupt
MCUCR |= (1<<ISC01);
MCUCR &=~ (1<<ISC10);	// set trigger INT1 interrupt
MCUCR |= (1<<ISC11);
MCUCSR &=~ (1<<ISC2);	// set trigger INT2 interrupt
GICR |= (1<<INT0);		// enable interrupt INT0
GICR |= (1<<INT1);		// enable interrupt INT1
GICR |= (1<<INT2);		// enable interrupt INT2
}

// ---------------------------------------------------------------------------------

//--------------------------------- DiodesInit --------------------------------------

void DiodesInit()
{
#define diode_front (1<<PB1);
DDRB |= diode_front; // set as output 
PORTB &=~ diode_front;	// set off by default

#define diode_right (1<<PC1);
DDRC |= diode_right; // set as output 
PORTC &=~ diode_right;	// set off by default

#define diode_left (1<<PB3);
DDRB |= diode_left; // set as output 
PORTB &= ~diode_left;	// set off by default
}

// ---------------------------------------------------------------------------------

//--------------------------------- SensorsInit -----------------------------------

SensorsInit()
{
// front sensor --------------------------------------------
#define front_sensor_echo (1<<PB2)
DDRB &= ~front_sensor_echo;              //set as input
PORTB &= ~front_sensor_echo;             //low logic level

#define front_sensor_trig (1<<PA1)
DDRA |= front_sensor_trig;               //set as output
PORTA &= ~front_sensor_trig;             //low logic level 

// right sensor---------------------------------------------
#define right_sensor_echo (1<<PD3)
DDRD &= ~right_sensor_echo;              //set as input
PORTD &= ~right_sensor_echo;             //low logic level

#define right_sensor_trig (1<<PC0)
DDRC |= right_sensor_trig;               //set as output
PORTC &= ~right_sensor_trig;             //low logic level 

// left sensor---------------------------------------------
#define left_sensor_echo (1<<PD2)
DDRD &= ~left_sensor_echo;              //set as input
PORTD &= ~left_sensor_echo;             //low logic level

#define left_sensor_trig (1<<PA0)
DDRA |= left_sensor_trig;               //set as output
PORTA &= ~left_sensor_trig;             //low logic level 
}

// ---------------------------------------------------------------------------------

//--------------------------------- ServoMove -----------------------------------

ServoMove(int nr, int a, int b)
{
while ( !( UCSRA & (1<<UDRE)) );
        UDR = 132;
while ( !( UCSRA & (1<<UDRE)) );
        UDR = nr;
while ( !( UCSRA & (1<<UDRE)) );
        UDR = a;
while ( !( UCSRA & (1<<UDRE)) ); // 2110 us  62   16
        UDR = b;
}

// ---------------------------------------------------------------------------------

//--------------------------------- ObstaclesDetection -----------------------------------

char flag;

char ObstaclesDetection ()

{
flag = 0;

PORTA |= front_sensor_trig;
_delay_us(10);   
PORTA &= ~front_sensor_trig;
counter = 0;
_delay_ms(38);
distans_front = distans * 1000 / 58; 

if (distans_front <40 )
	{
	 PORTB |= diode_front;
	 flag = 1;
	}
else PORTB &=~ diode_front;

//--------------------------

PORTA |= left_sensor_trig;
_delay_us(10);   
PORTA &= ~left_sensor_trig;
counter = 0;
_delay_ms(38);
distans_left = distans * 1000 / 58; 

if (distans_left <40 )
	{
	 PORTB |= diode_left;
	 if (flag != 0) flag = 2;
	}
else PORTB &=~ diode_left;

//--------------------------

PORTC |= right_sensor_trig;
_delay_us(10);   
PORTC &= ~right_sensor_trig;
counter = 0;
_delay_ms(38);
distans_right = distans * 1000 / 58; 

if (distans_right < 40 )
	{
	 PORTC |= diode_right;
	 if (flag == 2) flag = 3;
	 if (flag == 1) flag = 4;
	}
else PORTC &=~ diode_right;

return flag;
}

// ---------------------------------------------------------------------------------

//--------------------------------- TurnRight ------------------------------------

TurnRight()
{

ServoMove (6, 90, 26);		
ServoMove (7, 124, 53);		
_delay_ms(3000);			
ServoMove (7, 108,50);			

_delay_ms(1000);

ServoMove (0, 60, 32);
ServoMove (1, 116, 50);
ServoMove (2, 8, 47);
ServoMove (3, 40, 49);
ServoMove (4, 80, 29);
ServoMove (5, 80, 45);
ServoMove (6, 26, 47);
ServoMove (7, 40, 53);
ServoMove (8, 78, 44);
ServoMove (9, 0, 31);
ServoMove (10, 32, 37);
ServoMove (11, 112, 48);

_delay_ms(1000);

ServoMove (0, 0, 48);		
ServoMove (1, 40, 56);		  
_delay_ms(3000);			
ServoMove (1, 24, 53);			

_delay_ms(1000);

ServoMove (0, 40, 50);
ServoMove (1, 84, 32);
ServoMove (2, 108, 37);
ServoMove (3, 23, 57);
ServoMove (4, 64,52);
ServoMove (5, 16, 46);
ServoMove (6, 84, 34);
ServoMove (7, 64, 50);
ServoMove (8, 104, 47);
ServoMove (9, 8, 52);
ServoMove (10, 0, 43);
ServoMove (11, 118, 50);

_delay_ms(1000);

ServoMove (9, 118, 52);		
ServoMove (10, 68, 53);		  
_delay_ms(3000);			
ServoMove (10, 52, 50);			

_delay_ms(1000);

ServoMove (0, 112, 32);
ServoMove (1, 64, 56);
ServoMove (2, 0, 50);
ServoMove (3, 14, 26);
ServoMove (4, 112, 33);
ServoMove (5, 0, 41);
ServoMove (6, 112, 53);
ServoMove (7, 40, 40);
ServoMove (8, 34, 18);
ServoMove (9, 64, 37);
ServoMove (10, 80, 47);
ServoMove (11, 8, 36);

_delay_ms(1000);

ServoMove (3, 112, 32);		
ServoMove (4, 120, 55);		  
_delay_ms(3000);			
ServoMove (4, 4, 53);			

_delay_ms(1000);

ServoMove (0, 56, 46);
ServoMove (1, 116, 50);
ServoMove (2, 80, 39);
ServoMove (3, 84, 46);
ServoMove (4, 52,40);
ServoMove (5, 36, 37);
ServoMove (6, 84, 52);
ServoMove (7, 64, 38);
ServoMove (8, 108,31);
ServoMove (9, 112, 49);
ServoMove (10, 112, 31);
ServoMove (11, 8, 38);

_delay_ms(1000);

}

// ---------------------------------------------------------------------------------

//--------------------------------- TurnLeft --------------------------------------

TurnLeft()
{

ServoMove (6, 112, 51);		
ServoMove (7, 124, 53);		
_delay_ms(3000);			
ServoMove (7, 108,50);			

_delay_ms(1000);

ServoMove (0, 4, 38);
ServoMove (1, 84, 52);
ServoMove (2, 80,50);
ServoMove (3, 116, 40);
ServoMove (4, 64, 52);
ServoMove (5, 88, 42);
ServoMove (6, 36, 62);
ServoMove (7, 108, 50);
ServoMove (8, 112,44);
ServoMove (9, 40, 50);
ServoMove (10, 112, 49);
ServoMove (11, 56, 46);

_delay_ms(1000);

ServoMove (0, 2, 36);		
ServoMove (1, 40, 56);		  
_delay_ms(3000);			
ServoMove (1, 24, 53);			

_delay_ms(1000);

ServoMove (0, 0, 37);
ServoMove (1, 118, 38);
ServoMove (2, 23, 43);
ServoMove (3, 8, 51);
ServoMove (4, 104,38);
ServoMove (5, 32, 37);
ServoMove (6, 64, 57);
ServoMove (7, 104, 52);
ServoMove (8, 8, 32);
ServoMove (9, 84, 42);
ServoMove (10, 118, 39);
ServoMove (11, 112, 50);

_delay_ms(1000);

ServoMove (9, 112, 46);		
ServoMove (10, 68, 53);		  
_delay_ms(3000);			
ServoMove (10, 52, 50);			

_delay_ms(1000);

ServoMove (0, 80, 39);
ServoMove (1, 116, 50);
ServoMove (2, 84, 47);
ServoMove (3, 8, 53);
ServoMove (4, 112, 42);
ServoMove (5, 80, 53);
ServoMove (6, 8, 38);
ServoMove (7, 24, 18);
ServoMove (8, 64, 39);
ServoMove (9, 80, 36);
ServoMove (10, 80, 52);
ServoMove (11, 56, 38);

_delay_ms(1000);

ServoMove (3, 118, 41);		
ServoMove (4, 120, 55);		  
_delay_ms(3000);			
ServoMove (4, 4, 53);			

_delay_ms(1000);

ServoMove (0, 64, 38);
ServoMove (1, 80, 31);
ServoMove (2, 112, 38);
ServoMove (3, 112, 40);
ServoMove (4, 52,31);
ServoMove (5, 36, 38);
ServoMove (6, 0, 52);
ServoMove (7, 32, 49);
ServoMove (8, 112,41);
ServoMove (9, 64, 32);
ServoMove (10, 68, 38);
ServoMove (11, 108, 50);

_delay_ms(1000);

}

// ---------------------------------------------------------------------------------

//--------------------------------- Turn180 ---------------------------------------

Turn180()
{

TurnRight();
TurnRight();

}


// ********************** MAIN FUNCTION *****************************

void main ()

{
PORTD = (1<<PD1);  // pull-up
UsartInit();

TimerInit();
DiodesInit();
SensorsInit();
InterruptInit();

sei();

_delay_ms(3000);

ServoMove (0, 4, 38);
ServoMove (1, 84, 52);
ServoMove (2, 80, 50);
ServoMove (3, 116, 40);
ServoMove (4, 64,52);
ServoMove (5, 32, 46);
ServoMove (6, 36, 62);
ServoMove (7, 108, 50);
ServoMove (8, 112,44);
ServoMove (9, 40, 50);
ServoMove (10, 112, 49);
ServoMove (11, 56, 46);

_delay_ms(4000);

while (1)
{

if (ObstaclesDetection() == 1 || ObstaclesDetection() == 2) TurnRight();

if (ObstaclesDetection() == 3) Turn180();

if (ObstaclesDetection() == 4) TurnLeft();


ServoMove (6, 100, 26);		//
ServoMove (7, 124, 53);		//  
_delay_ms(3000);			//  left front leg step
ServoMove (7, 108,50);		//	

if ((bit_is_clear(PIND,4)))
	{

	ServoMove (6, 36, 62);		
	ServoMove (7, 124, 53);		 
	_delay_ms(3000);		
	ServoMove (7, 108,50);			

	Turn180();
	}


_delay_ms(1000);

ServoMove (0, 80, 29);
ServoMove (1, 24, 53);
ServoMove (2, 88, 47);
ServoMove (3, 40, 49);
ServoMove (4, 64,52);
ServoMove (5, 80, 45);
ServoMove (6, 24, 35);
ServoMove (7, 40, 50);
ServoMove (8, 72, 42);
ServoMove (9, 116, 41);
ServoMove (10, 112, 49);
ServoMove (11, 8, 47);

_delay_ms(1000);

ServoMove (0, 32, 63);		//
ServoMove (1, 40, 56);		//  
_delay_ms(3000);			//  right rear leg step
ServoMove (1, 24, 53);		//	

_delay_ms(1000);

ServoMove (0, 108, 54);
ServoMove (1, 84, 52);
ServoMove (2, 80, 50);
ServoMove (3, 92, 57);
ServoMove (4, 64,52);
ServoMove (5, 32, 46);
ServoMove (6, 60, 45);
ServoMove (7, 40, 50);
ServoMove (8, 104, 47);
ServoMove (9, 0, 35);
ServoMove (10, 52, 50);
ServoMove (11, 64, 43);

_delay_ms(1000);

ServoMove (9, 16, 67);		//
ServoMove (10, 68, 53);		//  
_delay_ms(3000);			//  right front leg step
ServoMove (10, 52, 50);		//	

if ((bit_is_clear(PIND,7)))
	{

	ServoMove (9, 0,35);		
	ServoMove (10, 68, 53);		
	_delay_ms(3000);			
	ServoMove (10, 52, 50);		

	Turn180();
	}

_delay_ms(1000);

ServoMove (0, 56, 46);
ServoMove (1, 84, 52);
ServoMove (2, 0, 50);
ServoMove (3, 16, 66);
ServoMove (4, 4, 53);
ServoMove (5, 88, 42);
ServoMove (6, 112, 53);
ServoMove (7, 40, 50);
ServoMove (8, 56, 48);
ServoMove (9, 92, 58);
ServoMove (10, 112, 49);
ServoMove (11, 124, 46);

_delay_ms(1000);

ServoMove (3, 64, 32);		//
ServoMove (4, 120, 55);		//  
_delay_ms(3000);			//  left rear leg step
ServoMove (4, 4, 53);		//	

_delay_ms(1000);

ServoMove (0, 4, 38);
ServoMove (1, 84, 52);
ServoMove (2, 80,50);
ServoMove (3, 116, 40);
ServoMove (4, 64, 52);
ServoMove (5, 88, 42);
ServoMove (6, 36, 62);
ServoMove (7, 108, 50);
ServoMove (8, 112,44);
ServoMove (9, 40, 50);
ServoMove (10, 112, 49);
ServoMove (11, 56, 46);

_delay_ms(1000);

}

}