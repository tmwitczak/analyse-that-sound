/////////////////////////////////////////////////////////////////////// Includes
#include <LPC17xx.h>
#include <lpc17xx_clkpwr.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_dac.h>
#include <oled.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_ssp.h>
#include <lpc17xx_i2c.h>


#include "arm_math.h"
#include "arm_const_structs.h"


//#include <stdint.h>
//#include <cr_section_macros.h>
//#include <rgb.h>

void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    // the buffer must not be null and at least have a length of 2 to handle one
    // digit and null-terminator
    if (pBuf == NULL || len < 2)
    {
        return;
    }

    // a valid base cannot be less than 2 or larger than 36
    // a base value of 2 means binary representation. A value of 1 would mean only zeros
    // a base larger than 36 can only be used if a larger alphabet were used.
    if (base < 2 || base > 36)
    {
        return;
    }

    // negative value
    if (value < 0)
    {
        tmpValue = -tmpValue;
        value    = -value;
        pBuf[pos++] = '-';
    }

    // calculate the required length of the buffer
    do {
        pos++;
        tmpValue /= base;
    } while(tmpValue > 0);


    if (pos > len)
    {
        // the len parameter is invalid.
        return;
    }

    pBuf[len - 1] = '\0';

    for(int p = pos; p < len; p++) {
    	pBuf[p] = ' ';
    }

    do {
        pBuf[--pos] = pAscii[value % base];
        value /= base;
    } while(value > 0);

    return;

}

///////////////////////////////////////////////////////////////////////// Macros
#define BIT(number) (1 << (number))

/////////////////////////////////////////////////////////////// Global variables
volatile int iterator = 0;
volatile int value = 0;

volatile int frequency = 500;


// BUFFER
#define FFT_POINTS_NUMBER 512
volatile int16_t sampleBuffer[FFT_POINTS_NUMBER];
volatile int16_t* currentSample = sampleBuffer;

volatile float32_t fft_buffer[FFT_POINTS_NUMBER * 2];
volatile float32_t amplitude[FFT_POINTS_NUMBER];

#define SAMPLE_RATE 8000


void fillFftBuffer() {
	uint16_t *sample = currentSample;
	float32_t* fft_buffer_ptr = fft_buffer;
	memset(fft_buffer, 0, sizeof(float32_t) * FFT_POINTS_NUMBER * 2);
	for(int i = 0; i < FFT_POINTS_NUMBER; i++) {
		*fft_buffer_ptr = (*sample);
		sample += 1;
		fft_buffer_ptr += 2;
		if (sample == sampleBuffer + FFT_POINTS_NUMBER) {
			sample = sampleBuffer;
		}
	}
}


//volatile uint16_t loudness;

///////////////////////////////////////////////////////////// Interrupt handlers
void TIMER1_IRQHandler(void)
{
	// Clear interrupt status!!!!!
	LPC_TIM1->IR = 0xffffffff;
}



volatile int x = 0;
volatile int adcIterator = 0;

void ADC_IRQHandler()
{
	uint32_t data = ADC_ChannelGetData(LPC_ADC, 0);

	// writing samples to buffer
	(*currentSample) = (uint16_t)data;
	currentSample++;
	if (currentSample == sampleBuffer + FFT_POINTS_NUMBER) {
		currentSample = sampleBuffer;
	}

	//------------------------------------------ Speaker frequency
	int step = SAMPLE_RATE / (2 * frequency);
	adcIterator = (adcIterator + 1) % step;
	if(adcIterator == 0){
		DAC_UpdateValue(LPC_DAC, x * 1023 / 2);
		x ^= 1;
	}
	//------------------------------------------

	uint32_t unused = LPC_ADC->ADGDR;	//clear interrupt flag!!!!!
}

////////////////////////////////////////////////////// Peripheral configurations
void configureAndStartTimer0() {
	// Turn on power
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0, ENABLE);

	// Select clock divisor
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0, CLKPWR_PCLKSEL_CCLK_DIV_4);

	//Procesor domyslnie jest taktowany z wewnetrznego oscylatora RC 4 MHz
	//PLL0 (pozwalajacy zwiekszyc/zmniejszyc czestotliwosc) domyslnie nie jest wykorzystywany
	//CCLKCFG (clock divider) domyslnie na 0 (nie ma dzielenia)

	// Reset counters
	LPC_TIM0->TCR |= BIT(1);
	LPC_TIM0->TCR &= ~BIT(1);

	// Set prescaler
	LPC_TIM0->PR = 0;

	// Configure external match for MAT0.1
	LPC_TIM0->EMR |= BIT(7);
	LPC_TIM0->EMR |= BIT(6);

	// Configure interrupts
	LPC_TIM0->MR1 = 1000000 / SAMPLE_RATE / 2;		// When counter reaches match register value (8000 Hz)
	LPC_TIM0->MCR |= BIT(4);						// then reset
	//LPC_TIM0->MCR |= BIT(3);						// then interrupt

	// Start
	LPC_TIM0->TCR |= BIT(0);
}

void configureAndStartTimer1() {
	// Turn on power
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM1, ENABLE);

	// Select clock divisor
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER1, CLKPWR_PCLKSEL_CCLK_DIV_4);

	//Procesor domyslnie jest taktowany z wewnetrznego oscylatora RC 4 MHz
	//PLL0 (pozwalajacy zwiekszyc/zmniejszyc czestotliwosc) domyslnie nie jest wykorzystywany
	//CCLKCFG (clock divider) domyslnie na 0 (nie ma dzielenia)

	// Reset counters
	LPC_TIM1->TCR |= BIT(1);
	LPC_TIM1->TCR &= ~BIT(1);

	// Set prescaler
	LPC_TIM1->PR = 0;

	// Configure interrupts
	LPC_TIM1->MR1 = 1000000 / 10;				// When counter reaches match register value (10 Hz)
	LPC_TIM1->MCR |= BIT(4);				// then reset
	LPC_TIM1->MCR |= BIT(3);				// then interrupt

	// Start
	LPC_TIM1->TCR |= BIT(0);
}

void configureAndStartADC() {
	// Turn on power
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCAD, ENABLE);

	// Select clock divisor
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_ADC, CLKPWR_PCLKSEL_CCLK_DIV_1);

	// Confiugre pin
	LPC_PINCON->PINSEL1 &= ~(BIT(14) | BIT(15));
	LPC_PINCON->PINSEL1 |= BIT(14);

	// Reset ADCR
	LPC_ADC->ADCR = 0;
	LPC_ADC->ADCR |= BIT(21);

	// Select channel
	LPC_ADC->ADCR |= BIT(0);

	// Select additional clock divisor
	LPC_ADC->ADCR &= ~BIT(8); 		// Other bits are already 0 since reset

	// Enable interrupts on channel 0
	LPC_ADC->ADINTEN |= BIT(0);

	// Disable burst mode
	LPC_ADC->ADCR &= ~BIT(16);

	// Start conversion when edge occurs on MAT0.1
	LPC_ADC->ADCR |= BIT(26);
	LPC_ADC->ADCR &= ~BIT(25);
	LPC_ADC->ADCR &= ~BIT(24);
}

void configureAndStartDAC() {
	// Turn on power and configure pin
	LPC_PINCON->PINSEL1 &= ~(BIT(21) | BIT(20));
	LPC_PINCON->PINSEL1 |= BIT(21);

	// Select clock divisor
	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_DAC, CLKPWR_PCLKSEL_CCLK_DIV_4);
}

void configureAndStartSpeakerAmplifier() {
	GPIO_SetDir(0, 1<<27, 1);
	GPIO_SetDir(0, 1<<28, 1);
	GPIO_SetDir(2, 1<<13, 1);

	GPIO_ClearValue(0, 1<<27);	//LM4811-clk
	GPIO_ClearValue(0, 1<<28);	//LM4811-up/dn
	GPIO_ClearValue(2, 1<<13);	//LM4811-shutdn
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	// Initialize I2C2 pin connect
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	I2C_Cmd(LPC_I2C2, ENABLE);
}

/////////////////////////////////////////////////////////////////////////// Main
int main(void) {

	// Configure peripherals
	configureAndStartTimer0();
	configureAndStartTimer1();
	configureAndStartADC();
	configureAndStartDAC();
	configureAndStartSpeakerAmplifier();

	// Enable NVIC interrupts
	NVIC->ICER[0] |= BIT(1);		// Disable timer0 interrupts
	NVIC->ISER[0] |= BIT(2);		// Enable timer1 interrupts
	NVIC->ISER[0] |= BIT(22);		// Enable ADC interrupts

	//configure oled and used interfaces
	init_i2c();
	init_ssp();
	oled_init();

	static uint8_t buf[10];

	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(1,1, "F: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	//oled_putString(1,9, "Max: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);


	uint16_t loudness = 0;


	//////////////////////////////////// | FFT Configuration

	////////////////////////////////////////////////////////

    // Main loop
    while(1) {

    	fillFftBuffer();

    	arm_cfft_f32(&arm_cfft_sR_f32_len512, fft_buffer, 0, 1);
    	arm_cmplx_mag_f32(fft_buffer, amplitude, FFT_POINTS_NUMBER);

    	//fix_fftr(fft_buffer, FFT_POINTS_NUMBER, 0);
    	//for(int i = 0; i < FFT_POINTS_NUMBER / 2; i++)
    	//	amplitude[i] = fft_buffer[i];

		uint16_t maxIndex = 1;
		for(int i = 2; i < FFT_POINTS_NUMBER / 2; i++) {
			if(amplitude[i] > amplitude[maxIndex]) {
				maxIndex = i;
			}
		}


		frequency = SAMPLE_RATE * maxIndex / FFT_POINTS_NUMBER;

    	//intToString(frequency, buf, 10, 10);
		//oled_putString(45,1, buf, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

// CODE GRAVEYARD
//configure led
/*LPC_GPIO2->FIODIR |= 1;
LPC_GPIO0->FIODIR |= (1 << 26);
LPC_GPIO2->FIODIR |= (1 << 1);
LPC_GPIO2->FIOCLR |= 1;
LPC_GPIO0->FIOCLR |= (1 << 26);
LPC_GPIO2->FIOCLR |= (1 << 1);*/

/*
	//red
	if(iterator > red){
		LPC_GPIO2->FIOCLR |= 1;
	}else{
		LPC_GPIO2->FIOSET |= 1;
	}

	//green
	if(iterator > green){
		LPC_GPIO2->FIOCLR |= (1 << 1);
	}else{
		LPC_GPIO2->FIOSET |= (1 << 1);
	}

	//blue
	if(iterator > blue){
		LPC_GPIO0->FIOCLR |= (1 << 26);
	}else{
		LPC_GPIO0->FIOSET |= (1 << 26);
	}
	iterator = (iterator + 1) % 100;

	if(iterator == 0){
		red = (red + 1) % 100;
		green = (green + 2) % 100;
		blue = (blue + 3) % 100;
	}
*/

//volatile int red = 0;
//volatile int green = -1;
//volatile int blue = -1;
