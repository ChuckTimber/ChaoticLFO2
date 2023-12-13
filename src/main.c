/*

	Title:		Chaotic LFO - PWM version
	Author:		Satoshi "Chuck" Takai
	Date:		Dec., 13, 2005
	Software:	WinAvr
	Target:		ATmega88

*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define TIMESET(N) (65536-(N))
#define TIMER1_PERIOD (65536-(8000))

#define MAXX 19.492022
#define MINX -19.412596
#define MAXY 26.908178
#define MINY -26.754900
#define MAXZ 47.866940
#define MINZ 0.000000


typedef uint8_t bool;
#define true 1
#define false 0

volatile bool gfOut0 = false;
volatile bool gfOut1 = false;

volatile uint16_t gSet0, gSet1;


ISR (TIMER0_OVF_vect)
{
	static uint8_t Tmcnt1m;

	// 256 cnt in 8MHz system
	// 256/8M * 32 = 1msec
	if (Tmcnt1m)
		Tmcnt1m--;
	else { 
		Tmcnt1m = 31;
		PORTD ^= (1<<PD2);
	}
}

ISR (INT0_vect)
{
	static uint16_t cnt0, cnt1;

	if (!gfOut0)
		if (cnt0++ > gSet0) {
			gfOut0 = true;
			cnt0 = 0;
		}

	if (!gfOut1)
		if (cnt1++ > gSet1) {
			gfOut1 = true;
			cnt1 = 0;
		}

	ADMUX ^= 0x01;		// toggle AD channel 0 <=> 1
	if(!(ADCSRA & (1<<ADSC)))
		ADCSRA |= (1<<ADSC);
}


ISR (ADC_vect)
{
	if ((ADMUX & 0x0F)==0)
		gSet0 = ADC;
	if ((ADMUX & 0x0F)==1)
		gSet1 = ADC;
}

void init_io(void)
{
	DDRD = 0xff;		// Port D output
	PORTD = 0;

	DDRB = 0xFF;		// Port B output
	PORTB = 0;

	// PWM
	TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(1<<WGM00);
	TCCR0B = (1<<CS00);
	TCCR1A = (1<<COM1A1)|(1<<COM1A0)|(1<<COM1B1)|(1<<COM1B0)|(1<<WGM10);
	TCCR1B = (1<<WGM12)|(1<<CS10);
						//						(1<<CS10);			// timer1 prescaled by 1/1024
	TCCR2A = (1<<COM0A1)|(1<<COM0A0)|(1<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(1<<WGM00);
	TCCR2B = (1<<CS20);

	/* timer1 initialization */			// timer1 for Tempo Control
	TIMSK0 = (1<<TOIE0);				// timer1 interrupt enabled

	/* ADC initial state*/
	ADMUX = (1<<ADLAR);
	ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADCSRB = 0x00;	// Timer1 OVF auto triggered ADC
	DIDR0 = (1<<ADC0D)|(1<<ADC1D);

	/* External interrupt */
	EICRA = (1<<ISC00);
	EIMSK = (1<<INT0);

}

void logistic( float *x, float *y, float *z, float *dt )
{
	float dx, dy, dz;

	dx = (-10.0 * (*x) + 10.0 * (*y)) * (*dt);
	dy = (28.0 * (*x) - (*y) - (*x) * (*z)) * (*dt);
	dz = (-8.0 / 3.0 * (*z) + (*x) * (*y)) * (*dt);
	
	*x += dx;
	*y += dy;
	*z += dz;
}

void satulate( float x, float y, float z, uint8_t *ix, uint8_t *iy, uint8_t *iz)
{
	float tx, ty, tz;

	tx = (x - MINX) / (MAXX - MINX) * 255.;
	ty = (y - MINY) / (MAXY - MINY) * 255.;
	tz = (z - MINZ) / (MAXZ - MINZ) * 255.;
	if (tx < 0)		tx = 0;
	if (tx > 255)	tx = 255;
	if (ty < 0)		ty = 0;
	if (ty > 255)	ty = 255;
	if (tz < 0)		tz = 0;
	if (tz > 255)	tz = 255;
	*ix = (uint8_t)(tx);
	*iy = (uint8_t)(ty);
	*iz = (uint8_t)(tz);
}


int main(void)
{
	float x0 = 0.0, y0 = 0.01, z0 = 0.0, dt0 = 1.0e-2;
	float x1 = 0.0, y1 = 0.02, z1 = 0.0, dt1 = 1.1e-2;

	uint8_t ix0, iy0, iz0;
	uint8_t ix1, iy1, iz1;

	init_io();
	sei();				/* enable interrupts     */

	for (;;) {
		if (gfOut0) {
			logistic( &x0, &y0, &z0, &dt0 );
			satulate( x0, y0, z0, &ix0, &iy0, &iz0);

			OCR0A = ix0;
			OCR1A = iy0;
			OCR2A = iz0;
						
			gfOut0 = false;
		}

		if (gfOut1) {
			logistic( &x1, &y1, &z1, &dt1 );
			satulate( x1, y1, z1, &ix1, &iy1, &iz1);

			OCR0B = ix1;
			OCR1B = iy1;
			OCR2B = iz1;

			gfOut1 = false;
		}

	}

}
