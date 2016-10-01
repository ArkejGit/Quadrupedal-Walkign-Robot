#include <avr/io.h>
#include <util/delay.h>
#include <AVR/interrupt.h>

# define F_CPU 16000000UL

// deklaracje zmiennych

volatile uint16_t licznik=0, wynik, dystans, dystans_prz, dystans_l, dystans_p;


ISR(INT0_vect)
{
dystans = licznik;
}

ISR(INT1_vect)
{
dystans = licznik;
}

ISR(INT2_vect)
{
dystans = licznik;
}

#define wartosc_przeladowania 49536;

ISR (TIMER1_OVF_vect)
	{
	 TCNT1 = wartosc_przeladowania;
	 licznik++;
	}

// ********************************* FUNKCJE ***************************************

#define BAUD 9600
#define MY_UBRR F_CPU/16/BAUD-1

//--------------------------------- UsartInit ---------------------------------------

void UsartInit(void)        
{
    UBRRH = (MY_UBRR >> 8);        //wpisanie starszego bajtu
    UBRRL = MY_UBRR;             //wpisanie mlodszego bajtu
 
    UCSRB = (1<<RXEN)|(1<<TXEN); //wlaczenie nadawania i odbierania USART
    UCSRC = (1 << URSEL) | (3<<UCSZ0);     //asynchroniczny, brak kontroli parzystoœci, 1 bit stopu, 8 bitów danych
 
} 
// ---------------------------------------------------------------------------------

//--------------------------------- TimerInit ---------------------------------------

void TimerInit()
{
TCCR1B |= (1<<CS10);	// ustawienie preskalera
TCCR1B &=~ (1<<CS11);
TCCR1B &=~ (1<<CS12);

TIMSK|=(1<<TOIE1);	// odblokowanie przerwania od przepe³nienia licznika

TCNT1 = wartosc_przeladowania;
}

// ---------------------------------------------------------------------------------

//------------------------------- PrzerwaniaInit -----------------------------------

void PrzerwaniaInit()
{
MCUCR &=~ (1<<ISC00);	// ustawienie zdarzenia wyzwalaj¹cego przerwanie INT0
MCUCR |= (1<<ISC01);
MCUCR &=~ (1<<ISC10);	// ustawienie zdarzenia wyzwalaj¹cego przerwanie INT1
MCUCR |= (1<<ISC11);
MCUCSR &=~ (1<<ISC2);	// ustawienie zdarzenia wyzwalaj¹cego przerwanie INT2
GICR |= (1<<INT0);		// odblokowanie przerwania INT0
GICR |= (1<<INT1);		// odblokowanie przerwania INT1
GICR |= (1<<INT2);		// odblokowanie przerwania INT2
}

// ---------------------------------------------------------------------------------

//--------------------------------- DiodyInit --------------------------------------

void DiodyInit()
{
#define dioda_prz (1<<PB1);	// dioda przednia
DDRB |= dioda_prz; // ustawienie jako wyjœcie 
PORTB &=~ dioda_prz;	// domyœlnie zgaszona

#define dioda_p (1<<PC1);	// dioda prawa
DDRC |= dioda_p; // ustawienie jako wyjœcie 
PORTC &=~ dioda_p;	// domyœlnie zgaszona

#define dioda_l (1<<PB3);	// dioda lewa
DDRB |= dioda_l; // ustawienie jako wyjœcie 
PORTB &= ~dioda_l;	// domyœlnie zgaszona
}

// ---------------------------------------------------------------------------------

//--------------------------------- CzujnikiInit -----------------------------------

CzujnikiInit()
{
// czujnik przedni --------------------------------------------
#define cz_prz_echo (1<<PB2)		// czujnik przedni echo
DDRB &= ~cz_prz_echo;              //ustawienie jako wejscie
PORTB &= ~cz_prz_echo;             //stan niski

#define cz_prz_trig (1<<PA1)		// czujnik przedni trigger
DDRA |= cz_prz_trig;               // ustawienie jako wyjscie
PORTA &= ~cz_prz_trig;             //stan niski 

// czujnik prawy ----------------------------------------------
#define cz_p_echo (1<<PD3)		// czujnik prawy echo
DDRD &= ~cz_p_echo;              //ustawienie jako wejscie
PORTD &= ~cz_p_echo;             //stan niski

#define cz_p_trig (1<<PC0)		// czujnik prawy trigger
DDRC |= cz_p_trig;               // ustawienie jako wyjscie
PORTC &= ~cz_p_trig;             //stan niski 

// czujnik lewy -----------------------------------------------
#define cz_l_echo (1<<PD2)		// czujnik lewy echo
DDRD &= ~cz_l_echo;              //ustawienie jako wejscie
PORTD &= ~cz_l_echo;             //stan niski

#define cz_l_trig (1<<PA0)		// czujnik lewy trigger
DDRA |= cz_l_trig;               // ustawienie jako wyjscie
PORTA &= ~cz_l_trig;             //stan niski 
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

//--------------------------------- DetekcjaPrzeszkod -----------------------------------

char flag;

char DetekcjaPrzeszkod ()

{
flag = 0;

PORTA |= cz_prz_trig;
_delay_us(10);   
PORTA &= ~cz_prz_trig;
licznik = 0;
_delay_ms(38);
dystans_prz = dystans * 1000 / 58; 

if (dystans_prz <40 )
	{
	 PORTB |= dioda_prz;
	 flag = 1;
	}
else PORTB &=~ dioda_prz;

//--------------------------

PORTA |= cz_l_trig;
_delay_us(10);   
PORTA &= ~cz_l_trig;
licznik = 0;
_delay_ms(38);
dystans_l = dystans * 1000 / 58; 

if (dystans_l <40 )
	{
	 PORTB |= dioda_l;
	 if (flag != 0) flag = 2;
	}
else PORTB &=~ dioda_l;

//--------------------------

PORTC |= cz_p_trig;
_delay_us(10);   
PORTC &= ~cz_p_trig;
licznik = 0;
_delay_ms(38);
dystans_p = dystans * 1000 / 58; 

if (dystans_p < 40 )
	{
	 PORTC |= dioda_p;
	 if (flag == 2) flag = 3;
	 if (flag == 1) flag = 4;
	}
else PORTC &=~ dioda_p;

return flag;
}

// ---------------------------------------------------------------------------------

//--------------------------------- ObrotPrawo ------------------------------------

ObrotPrawo()
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

//--------------------------------- ObrotLewo --------------------------------------

ObrotLewo()
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

//--------------------------------- Obrot180 ---------------------------------------

Obrot180()
{

ObrotPrawo();
ObrotPrawo();

}


// ********************** G£ÓWNA FUNKCJA *****************************

void main ()

{
PORTD = (1<<PD1);  // podciagniece pull-up
UsartInit();

TimerInit();
DiodyInit();
CzujnikiInit();
PrzerwaniaInit();

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

if (DetekcjaPrzeszkod() == 1 || DetekcjaPrzeszkod() == 2) ObrotPrawo();

if (DetekcjaPrzeszkod() == 3) Obrot180();

if (DetekcjaPrzeszkod() == 4) ObrotLewo();


ServoMove (6, 100, 26);		//
ServoMove (7, 124, 53);		//  
_delay_ms(3000);			//  lewa przednia - krok
ServoMove (7, 108,50);		//	

if ((bit_is_clear(PIND,4)))
	{

	ServoMove (6, 36, 62);		
	ServoMove (7, 124, 53);		 
	_delay_ms(3000);		
	ServoMove (7, 108,50);			

	Obrot180();
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
_delay_ms(3000);			//  prawa tylna - krok
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
_delay_ms(3000);			//  prawa przednia - krok
ServoMove (10, 52, 50);		//	

if ((bit_is_clear(PIND,7)))
	{

	ServoMove (9, 0,35);		
	ServoMove (10, 68, 53);		
	_delay_ms(3000);			
	ServoMove (10, 52, 50);		

	Obrot180();
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
_delay_ms(3000);			//  lewa tylna - krok
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
