/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include <LPC17xx.h>
#include <lpc17xx_clkpwr.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_dac.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_ssp.h>
#include <lpc17xx_i2c.h>
#include <oled.h>

#include "arm_math.h"
#include "arm_const_structs.h"


#include "analyse-that-sound/typedefs.h"
#include "analyse-that-sound/macros.h"


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Constants */
/* -------------------------------------------------------- Boolean values -- */
#define TRUE                    1
#define FALSE                   0

/* -------------------------------------------------------- FFT parameters -- */
#define SAMPLE_RATE 	  		8000
#define FFT_POINTS_NUMBER 		1024

/* ---------------------------------------------- Sine function parameters -- */
#define SINE_LOOKUP_TABLE_SIZE	128


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Global variables */
/* --------------------------------------------------------- Sine function -- */
uint16_t sineLookupTable[SINE_LOOKUP_TABLE_SIZE]
        = {  512,  537,  562,  587,  612,  637,  661,  685,
             709,  732,  755,  777,  798,  819,  838,  858,
             876,  893,  910,  925,  939,  953,  965,  976,
             986,  995, 1003, 1009, 1015, 1019, 1022, 1023,
            1023, 1023, 1020, 1017, 1012, 1006,  999,  991,
             981,  971,  959,  946,  932,  917,  901,  885,
             867,  848,  829,  808,  787,  766,  743,  721,
             697,  673,  649,  625,  600,  575,  549,  524,
             499,  474,  448,  423,  398,  374,  350,  326,
             302,  280,  257,  236,  215,  194,  175,  156,
             138,  122,  106,   91,   77,   64,   52,   42,
              32,   24,   17,   11,    6,    3,    0,    0,
               0,    1,    4,    8,   14,   20,   28,   37,
              47,   58,   70,   84,   98,  113,  130,  147,
             165,  185,  204,  225,  246,  268,  291,  314,
             338,  362,  386,  411,  436,  461,  486,  511 };


/* ------------------------------------------------------------------- FFT -- */
volatile int      value = 0;
volatile int      frequency = 500;

volatile int16_t  sampleBuffer[FFT_POINTS_NUMBER];
volatile int16_t  *currentSample = sampleBuffer;

volatile float    fft_buffer[FFT_POINTS_NUMBER * 2];
volatile float    amplitude[FFT_POINTS_NUMBER];


volatile uint8_t  normalizedAmplitude[OLED_DISPLAY_WIDTH];
volatile float    globalAmplitudeMax;
volatile uint16_t localAmplitudeMaxIndex = 1;

volatile int      dacAmplitudeValue = 0;
volatile int      dacIterator = 0;

/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Functions */
/* ------------------------------------------------------------------- FFT -- */
void normalizeAmplitudes()
{
    float sum = 0;

    for (int i = 1;
         i < (FFT_POINTS_NUMBER / 2);
         ++i)
    {
        sum += amplitude[i];

        if ((i != 0)
            &&
            ((i % (FFT_POINTS_NUMBER / 2 / OLED_DISPLAY_WIDTH)) == 0))
        {
            sum /= (FFT_POINTS_NUMBER / 2 / OLED_DISPLAY_WIDTH);

            normalizedAmplitude[i / (FFT_POINTS_NUMBER / 2 / OLED_DISPLAY_WIDTH)]
                    = sum / globalAmplitudeMax * OLED_DISPLAY_HEIGHT;

            sum = 0;
        }
    }
}

void findLocalAndGlobalMaxAmplitudes()
{
    /* Find local max amplitude for one FFT computation */
    for(int i = 1;
        i < (FFT_POINTS_NUMBER / 2);
        ++i)
    {
        if(amplitude[i] > amplitude[localAmplitudeMaxIndex])
        {
            localAmplitudeMaxIndex = i;
        }
    }

    /* Save global maximum value for further normalization */
    if(amplitude[localAmplitudeMaxIndex] > globalAmplitudeMax)
    {
        globalAmplitudeMax = amplitude[localAmplitudeMaxIndex];
    }
}

void fillFftBuffer()
{
    uint16_t *sample = currentSample;
    float *fft_buffer_ptr = fft_buffer;

    memset(fft_buffer, 0, (int)sizeof(float) * FFT_POINTS_NUMBER * 2);

    for(int i = 0;
        i < FFT_POINTS_NUMBER;
        ++i)
    {
        *fft_buffer_ptr = (*sample);

        sample += 1;
        fft_buffer_ptr += 2;

        if (sample == (sampleBuffer + FFT_POINTS_NUMBER))
        {
            sample = sampleBuffer;
        }
    }
}


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Interrupt handlers */
void TIMER1_IRQHandler()
{
    //generate pure tone using dac
    int step = 100000 / frequency;	//na przestrzeni tylu przerwan musimy zrobic caly okres
    int sinPhase = dacIterator * SINE_LOOKUP_TABLE_SIZE / step;
    dacIterator = (dacIterator + 1) % step;

    if(sinPhase >= SINE_LOOKUP_TABLE_SIZE)
    {
        sinPhase = SINE_LOOKUP_TABLE_SIZE - 1;
    }

    DAC_UpdateValue(LPC_DAC, sineLookupTable[sinPhase]);

    /* Clear interrupt status */
    LPC_TIM1->IR = 0xffffffff;
}

void ADC_IRQHandler()
{
    /* Read ADC sample */
    uint32_t adcSampledValue = ADC_ChannelGetData(LPC_ADC, 0);

    /* Write samples to buffer */
    (*currentSample) = (uint16_t)adcSampledValue;
    currentSample++;

    if (currentSample == (sampleBuffer + FFT_POINTS_NUMBER))
    {
        currentSample = sampleBuffer;
    }


    /* Clear interrupt status */
    uint32_t unused = LPC_ADC->ADGDR;
}


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Peripheral configurations */
void configureAndStartTimer0()
{
    /* Turn on power */
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0, ENABLE);

    /* Select clock divisor */
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0, CLKPWR_PCLKSEL_CCLK_DIV_4);

    /* Reset counters */
    LPC_TIM0->TCR |= BIT(1);
    LPC_TIM0->TCR &= ~BIT(1);

    /* Set prescaler */
    LPC_TIM0->PR = 0;

    /* Configure external match for MAT0.1 */
    LPC_TIM0->EMR |= BIT(7);
    LPC_TIM0->EMR |= BIT(6);

    /* Configure interrupts */
    LPC_TIM0->MR1 = 25000000 / SAMPLE_RATE / 2;		// When counter reaches match register value (8000 Hz)
    LPC_TIM0->MCR |= BIT(4);						// then reset
    /* LPC_TIM0->MCR |= BIT(3); */					// then interrupt

    /* Start */
    LPC_TIM0->TCR |= BIT(0);
}

void configureAndStartTimer1()
{
    /* Turn on power */
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM1,
                      ENABLE);

    /* Select clock divisor */
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER1,
                      CLKPWR_PCLKSEL_CCLK_DIV_4);

    /* Reset counters */
    LPC_TIM1->TCR |= BIT(1);
    LPC_TIM1->TCR &= ~BIT(1);

    /* Set prescaler */
    LPC_TIM1->PR = 0;

    /* Configure interrupts */
    LPC_TIM1->MR1 = 25000000 / 100000;			// When counter reaches match register value (100 000 Hz)
    LPC_TIM1->MCR |= BIT(4);				// then reset
    LPC_TIM1->MCR |= BIT(3);				// then interrupt

    /* Start */
    LPC_TIM1->TCR |= BIT(0);
}

void configureAndStartADC()
{
    /* Turn on power */
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCAD,
                      ENABLE);

    /* Select clock divisor */
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_ADC,
                      CLKPWR_PCLKSEL_CCLK_DIV_4);

    /* Confiugre pin */
    LPC_PINCON->PINSEL1 &= ~(BIT(14) | BIT(15));
    LPC_PINCON->PINSEL1 |= BIT(14);

    /* Reset ADCR */
    LPC_ADC->ADCR = 0;
    LPC_ADC->ADCR |= BIT(21);

    /* Select channel */
    LPC_ADC->ADCR |= BIT(0);

    /* Select additional clock divisor */
    LPC_ADC->ADCR |= BIT(8); 		// Other bits are already 0 since reset, clock divisor: 2

    /* Enable interrupts on channel 0 */
    LPC_ADC->ADINTEN |= BIT(0);

    /* Disable burst mode */
    LPC_ADC->ADCR &= ~BIT(16);

    /* Start conversion when edge occurs on MAT0.1 */
    LPC_ADC->ADCR |= BIT(26);
    LPC_ADC->ADCR &= ~BIT(25);
    LPC_ADC->ADCR &= ~BIT(24);
}

void configureAndStartDAC()
{
    /* Turn on power and configureBoard pin */
    LPC_PINCON->PINSEL1 &= ~(BIT(21) | BIT(20));
    LPC_PINCON->PINSEL1 |= BIT(21);

    /* Select clock divisor */
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_DAC,
                      CLKPWR_PCLKSEL_CCLK_DIV_4);
}

void configureAndStartSpeakerAmplifier() {
    GPIO_SetDir(0, BIT(27), 1);
    GPIO_SetDir(0, BIT(28), 1);
    GPIO_SetDir(2, BIT(13), 1);

    GPIO_ClearValue(0, BIT(27));	//LM4811-clk
    GPIO_ClearValue(0, BIT(28));	//LM4811-up/dn
    GPIO_ClearValue(2, BIT(13));	//LM4811-shutdn
}

static void init_ssp()
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

static void init_i2c()
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

void writeByteToSPI(uint8_t byte) {
    LPC_SPI->SPDR = byte;
    while(!(LPC_SPI->SPSR & BIT(7))) {}
}

void configureBoard()
{
    configureSystemClock100Mhz();
    configurePeripherials();
    configureInterrupts();
}

void configurePeripherials()
{
    configureAndStartOLED();
    configureAndStartTimer0();
    configureAndStartTimer1();
    configureAndStartADC();
    configureAndStartDAC();
    configureAndStartSpeakerAmplifier();
}

void configureInterrupts()
{
    /* Enable/Disable NVIC interrupts */
    NVIC->ICER[0] |= BIT(1);		// Disable timer0 interrupts
    NVIC->ISER[0] |= BIT(2);		// Enable timer1 interrupts
    NVIC->ISER[0] |= BIT(22);		// Enable ADC interrupts
}

void configureSystemClock100Mhz(){
    LPC_SC->SCS       = BIT(5);				// Enable main oscillator (30)
    while ((LPC_SC->SCS & (BIT(6))) == 0) {}	// Wait for Oscillator to be ready
    LPC_SC->CCLKCFG   =	2;      			// Setup Clock Divider - 3 (57)
    LPC_SC->PCLKSEL0  = 0;       			// Peripheral Clock Selection - 4 for every peripheral
    LPC_SC->PCLKSEL1  = 0;
    LPC_SC->CLKSRCSEL = 1;    				// Select Clock Source for PLL0 - Main oscillator (36)

    //target frequency - 100MHz
    //pll source - main oscillator, FIN=12MHz
    //pll output frquency, FCCO=300MHz (must be in range 275 to 550 MHz so we are using CLKCFG = 3)
    //using M = (FCCO * N) / (2 * FIN) we have:
    //M=25
    //N=2

    //configure PLL0
    uint16_t M = 25u;
    uint16_t N = 2u;
    LPC_SC->PLL0CFG = ((N - (uint16_t)1) << (uint16_t)16) | (M - (uint16_t)1);
    LPC_SC->PLL0FEED  = 0xAA;				//A correct feed sequence must be written to the PLL0FEED register
    LPC_SC->PLL0FEED  = 0x55;				//in order for changes to the PLL0CON and PLL0CFG registers to take effect

    //enable PLL0							(39)
    LPC_SC->PLL0CON   = BIT(0);
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;
    while (!(LPC_SC->PLL0STAT & (BIT(26)))) {} // Wait for PLOCK0

    //PLL0 Enable & Connect					(39)
    LPC_SC->PLL0CON   = BIT(1) | BIT(0);
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;
    while (!(LPC_SC->PLL0STAT & ((BIT(25)) | (BIT(24))))) {}  //Wait for PLLC0_STAT & PLLE0_STAT
}


void configureAndStartOLED()
{
    init_ssp();
    init_i2c();
    oled_init();
}

void runMainProgramLoop()
{
    while(TRUE)
    {
        fillFftBuffer();

        arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_buffer, 0, 1);
        arm_cmplx_mag_f32(fft_buffer, amplitude, FFT_POINTS_NUMBER);

        findLocalAndGlobalMaxAmplitudes();
        normalizeAmplitudes();

        frequency = (uint16_t)SAMPLE_RATE * localAmplitudeMaxIndex / (uint16_t)FFT_POINTS_NUMBER;


        //------------------------------- OLED

        for (int page = 0; page < 8; ++page) {
            writeCommand(0xb0 + page); //page number
            // start column = 18
            writeCommand(0x02); //start column low 2
            writeCommand(0x11); //start column high 1

            for (int column = OLED_DISPLAY_WIDTH - 1; column >= 0; --column) {
                if (normalizedAmplitude[column] >= (uint8_t)8) {
                    writeData(0xff);
                    normalizedAmplitude[column] -= (uint8_t)8;
                }
                else {
                    writeData(((uint8_t)0xff) >> ((uint8_t)8 - normalizedAmplitude[column]));
                    normalizedAmplitude[column] = 0;
                }
            }
        }


        //-------------------------------



    }
}

/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ AnalyseThatSound */
int main()
{
    configureBoard();
    runMainProgramLoop();

    return 0;
}


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */




/* void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
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
*/