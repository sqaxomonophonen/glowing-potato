////////////////////////////
// SETUP
////////////////////////////

#define I2C_ADDRESS (0x10) // XXX edit me :-)

/* if defined it will use 300ns hold time if not defined it will use 100ns hold
 * time docs say that 300ns may help on buses with large capacitance */
//#define USE_300NS_HOLD_TIME

/* if defined it enables slew rate control for i2c; docs recommend this for
 * 400kbit */
#define ENABLE_SLEW_RATE_CONTROL


////////////////////////////
// NCO stuff
//   >>> import math
//   >>> clk = 32e3  # 32khz clock source
//   >>> clk = 500e3 # 500khz clock source
//   >>> i2f = lambda inc: (clk*inc) / (2**20)
//   >>> f2i = lambda freq: freq * (2**20) / clk
//   >>> cents = lambda at_freq: math.log(i2f(f2i(at_freq)+1) / at_freq, 2) * 1200



// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = OFF      // Master Clear Enable bit (MCLR pin function is port defined function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = OFF    // Peripheral Pin Select one-way control (The PPSLOCK bit can be set and cleared repeatedly by software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/Vpp must be used for programming)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

#define _XTAL_FREQ 32000000

#include <xc.h> // /opt/microchip/xc8/v2.00/pic/include/pic16f15313.h

#define MFINTOSC_32khz (4u)
#define MFINTOSC_500khz (3u)
#define PWMxCON_enable 0x80

// NOTE
// writing NCO1INCL triggers the actual write(-through) of the
// entire 24-bit NCO1INC value. it's buffered, apparently!


void main(void)
{
	/* GIE (Global Interrupt Enable) off; disable all interrupts for now */
	INTCONbits.GIE = 0;


	// ======================================
	// ========== I/O SETUP =================
	// ======================================

	/* Port-A Analog Select: set all pins to digital */
	ANSELA = 0;

	/* Port-A Output Latch; clear all */
	LATA = 0;

	/* Port-A Tri-State Register: temporarily set all pins to input (to
	 * disable output); this is changed later */
	TRISA = 0xff;

	/* RA1 and RA2 on pin 6 and 5 are used for SCLK and SDAT, respectively;
	 * they must be in input mode for I2C to work.
	 * The remaining outputs, RA0, RA4 and RA5 are used for PWM output*/

	/* Configure I/O PPS (peripheral pin select */

	/* set i2c I/O */
	SSP1CLKPPS = 0x01; // RA1 (default)
	SSP1DATPPS = 0x02; // RA2 (default)
	RA1PPS = 0x15; // I2C SCLK
	RA2PPS = 0x16; // I2C SDAT

	// ------------------------------------~-

	/* set outputs */
	RA0PPS = 0x0B; // PWM3OUT
	RA4PPS = 0x0C; // PWM4OUT
	RA5PPS = 0x1A; // NCO1OUT


	// ======================================
	// ========== I2C SETUP =================
	// ======================================

	/* set i2c address */
	SSP1ADD = I2C_ADDRESS << 1;
	SSP1MSK = 0xff;

	SSP1STAT = 0;
	#ifndef ENABLE_SLEW_RATE_CONTROL
	SSP1STATbits.SMP = 1;
	#else
	SSP1STATbits.SMP = 0;
	#endif

	SSP1CON1 = 0x26; // 0x06 (simple i2c slave mode) + 0x20 (enable)
	SSP1CON2 = 0;
	SSP1CON3 = 0;
	#ifdef USE_300NS_HOLD_TIME
	SSP1CON3bits.SDAHT = 1;
	#else
	SSP1CON3bits.SDAHT = 0;
	#endif

	// ------------------------------------~-



	// ======================================
	// ========== NCO SETUP =================
	// ======================================

	NCO1INCH = 0;
	NCO1INCU = 0;
	NCO1INCL = 0; // write! see NOTE above

	NCO1CONbits.N1EN = 1; // enable=on
	NCO1CONbits.N1PFM = 0; // pulse-frequency mode? no thanks, just wonderful square waves! :-)~
	NCO1CONbits.N1POL = 0; // polarity; don't care
	NCO1CLKbits.N1CKS = MFINTOSC_32khz; // clock source; changable from i2c
	PIE7bits.NCO1IE = 0; // no interrupt!
	PIR7bits.NCO1IF = 0; // no interrupt!

	// ------------------------------------~-



	// ======================================
	// ========== PWM SETUP =================
	// ======================================

	T2CLKCON = 0x2; // make timer2 slave to Fosc (32mhz); all PWM uses timer2
	PR2 = 0xff; // set counter period to 1024 ((PR2+1)*4) for timer2

	/* clear all duty cycles */
	PWM3DCL = 0;
	PWM3DCH = 0;
	PWM4DCL = 0;
	PWM4DCH = 0;
	//PWM5DCL = 0;
	//PWM5DCH = 0;

	/* enable all PWM */
	PWM3CON = PWMxCON_enable;
	PWM4CON = PWMxCON_enable;
	//PWM5CON = PWMxCON_enable;

	// ------------------------------------~-




	/* clear timer2 interrupt flag */
	PIR4bits.TMR2IF = 0;

	/* set timer2 prescale and enable */
	T2CON = 0xa0;

	/* setup i2c pins (RA1=2/RA2=4) as inputs (required), and the rest as outputs */
	TRISA = 6;

	/* enable interrupts */
	PIR3bits.SSP1IF = 0; // clear MSSP interrupt (for i2c)
	PIE3bits.SSP1IE = 1; // enable MSSP interrupts (for i2c)
	INTCONbits.PEIE = 1; // required for PIE1-7 interrupts to work
	INTCONbits.GIE = 1; // global interrupt enable


	#if 0 // initial test values
	PWM3DCH = 10;
	PWM4DCH = 1;
	NCO1INCL = 33; // freq=1(.007)hz
	NCO1CLKbits.N1CKS = MFINTOSC_32khz;
	#endif

	for (;;) {
		// loop forever
	}
}

unsigned char buf;

#define MSG_LENGTH (5)
unsigned char msg_index;
unsigned char msg[MSG_LENGTH];

void __interrupt () isr0() {
	/* handle MSSP (i2c) interrupt */
	if (!PIR3bits.SSP1IF) return;

	PIR3bits.SSP1IF = 0; // clear interrupt
	buf = SSP1BUF; // read byte
	if (SSP1CON2bits.SEN) SSP1CON1bits.CKP = 1;

	if (SSP1STATbits.D_nA == 0) {
		// buf is address; reset state
		msg_index = 0;
		return;
	}

	if (msg_index >= MSG_LENGTH) return;
	msg[msg_index++] = buf;

	if (msg_index == 3) {
		NCO1INCH = msg[1] & 0x1fu;
		NCO1INCL = msg[0]; // write! see NOTE above

		NCO1CLKbits.N1CKS = (msg[1] & 0x20u) ? MFINTOSC_500khz : MFINTOSC_32khz;
		PWM3DCL = msg[1] /*& 0xc0u*/; // only PWM3DCL<6:7> are read; same orientation as in our format :)
		PWM3DCH = msg[2];
	} else if (msg_index == 5) {
		PWM4DCL = msg[3];
		PWM4DCH = msg[4];
		// 6 unused bits...
	}
}

// This is more fun than TIS-100!
