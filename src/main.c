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
#include "analyse-that-sound/constants.h"
#include "analyse-that-sound/global-variables.h"


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Functions */
/* ------------------------------------------------------------------- FFT -- */
void averageAmplitudes(void)
{
    float sumOfAmplitudes = 0;

    for (int amplitudeIndex = 0,
             averagedAmplitudeIndex = 0;
         amplitudeIndex < FFT_AMPLITUDE_USABLE_RANGE;
         ++amplitudeIndex)
    {
        sumOfAmplitudes += amplitude[amplitudeIndex];

        if ((amplitudeIndex != 0)
            &&
            ((amplitudeIndex % NUMBER_OF_AVERAGED_AMPLITUDES) == 0))
        {
            averagedAmplitude[averagedAmplitudeIndex]
                    = sumOfAmplitudes / NUMBER_OF_AVERAGED_AMPLITUDES;

            averagedAmplitudeIndex++;
            sumOfAmplitudes = 0;
        }
    }
}

void normalizeAmplitudes(void)
{
    for (int i = 0;
         i < OLED_DISPLAY_WIDTH;
         ++i)
    {
        normalizedAmplitude[i]
                = averagedAmplitude[i]
                  / averagedAmplitude[averagedAmplitudeMaxIndex]
                    * OLED_DISPLAY_HEIGHT;
    }
}

void findMaxAmplitudes(void)
{
    for(int i = 0;
        i < FFT_AMPLITUDE_USABLE_RANGE;
        ++i)
    {
        if(amplitude[i] > amplitude[amplitudeMaxIndex])
        {
            amplitudeMaxIndex = i;
        }
    }

    for(int i = 0;
        i < OLED_DISPLAY_WIDTH;
        ++i)
    {
        if(averagedAmplitude[i] > averagedAmplitude[averagedAmplitudeMaxIndex])
        {
            averagedAmplitudeMaxIndex = i;
        }
    }
}

void fillFftBuffer(void)
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
void TIMER1_IRQHandler(void)
{
    // choose interval
    uint32_t intervalFrequency = frequency;
    for (int i = 0; i < interval; i++)
    {
        intervalFrequency = intervalFrequency * SEMITONE_RATIO_NUMERATOR / SEMITONE_RATIO_DENOMINATOR;
    }


    //generate pure tone using dac
    int step = 100000 / intervalFrequency;	//na przestrzeni tylu przerwan musimy zrobic caly okres
    int sinPhase = dacIterator * SINE_LOOKUP_TABLE_SIZE / step;
    dacIterator = (dacIterator + 1) % step;

    if(sinPhase >= SINE_LOOKUP_TABLE_SIZE)
    {
        sinPhase = SINE_LOOKUP_TABLE_SIZE - 1;
    }

    DAC_UpdateValue(LPC_DAC, sineLookupTable[sinPhase] * volume / 100);

    /* Clear interrupt status */
    LPC_TIM1->IR = 0xffffffff;
}

void ADC_IRQHandler(void)
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
void configureAndStartTimer0(void)
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

void configureAndStartTimer1(void)
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

void configureAndStartADC(void)
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

void configureAndStartDAC(void)
{
    /* Turn on power and configureBoard pin */
    LPC_PINCON->PINSEL1 &= ~(BIT(21) | BIT(20));
    LPC_PINCON->PINSEL1 |= BIT(21);

    /* Select clock divisor */
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_DAC,
                      CLKPWR_PCLKSEL_CCLK_DIV_4);
}

void configureAndStartSpeakerAmplifier(void) {
    GPIO_SetDir(0, BIT(27), 1);
    GPIO_SetDir(0, BIT(28), 1);
    GPIO_SetDir(2, BIT(13), 1);

    GPIO_ClearValue(0, BIT(27));	//LM4811-clk
    GPIO_ClearValue(0, BIT(28));	//LM4811-up/dn
    GPIO_ClearValue(2, BIT(13));	//LM4811-shutdn
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

void configureBoard(void)
{
    configureSystemClock();
    configurePeripherials();
    configureInterrupts();
}

void configurePeripherials(void)
{
    configureAndStartOLED();
    configureAndStartTimer0();
    configureAndStartTimer1();
    configureAndStartADC();
    configureAndStartDAC();
    configureAndStartSpeakerAmplifier();
    configureJoystick();
}

void configureInterrupts(void)
{
    /* Enable/Disable NVIC interrupts */
    NVIC->ICER[0] |= BIT(1);		// Disable timer0 interrupts
    NVIC->ISER[0] |= BIT(2);		// Enable timer1 interrupts
    NVIC->ISER[0] |= BIT(22);		// Enable ADC interrupts
}

void configureSystemClock(void){
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

void processJoystick(void){
    //top
    int lastTopStatus = joystickStatus.top;
    int currentTopStatus = !(LPC_GPIO0->FIOPIN & BIT(15));
    joystickStatus.top = currentTopStatus;
    if(currentTopStatus == 1 && lastTopStatus == 0){
        if(volume > 0)
            volume -= 10;
    }

    //bottom
    int lastBottomStatus = joystickStatus.bottom;
    int currentBottomStatus = !(LPC_GPIO2->FIOPIN & BIT(3));
    joystickStatus.bottom = currentBottomStatus;
    if(currentBottomStatus == 1 && lastBottomStatus == 0){
        if(volume < 100)
                    volume += 10;
    }

    //right
    int lastRightStatus = joystickStatus.right;
    int currentRightStatus = !(LPC_GPIO2->FIOPIN & BIT(4));
    joystickStatus.right = currentRightStatus;
    if(currentRightStatus == 1 && lastRightStatus == 0){
        if(interval > 0)
            interval -= 1;
    }

    //left
    int lastLeftStatus = joystickStatus.left;
    int currentLeftStatus = !(LPC_GPIO0->FIOPIN & BIT(16));
    joystickStatus.left = currentLeftStatus;
    if(currentLeftStatus == 1 && lastLeftStatus == 0){
        if(interval < 11)
            interval += 1;
    }

    //center
    int lastCenterStatus = joystickStatus.center;
    int currentCenterStatus = !(LPC_GPIO0->FIOPIN & BIT(17));
    joystickStatus.center = currentCenterStatus;
    if(currentCenterStatus == 1 && lastCenterStatus == 0){
        graphStatus = (graphStatus + 1) % 2;
    }

    //set current status
}

void configureJoystick(void){
    //buttons address
    //GPIO 0_15, 0_16, 0_17, 2_3, 2_4

    //set direction - input
    LPC_GPIO0->FIODIR &= ~BIT(15);	//top
    LPC_GPIO0->FIODIR &= ~BIT(16);	//left
    LPC_GPIO0->FIODIR &= ~BIT(17);	//center
    LPC_GPIO2->FIODIR &= ~BIT(3);	//bottom
    LPC_GPIO2->FIODIR &= ~BIT(4);	//right
}


void configureAndStartOLED(void)
{
    init_ssp();
    init_i2c();
    oled_init();
}

void runMainProgramLoop(void)
{
    while(TRUE)
    {
        fillFftBuffer();

        arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_buffer, 0, 1);
        arm_cmplx_mag_f32(fft_buffer, amplitude, FFT_POINTS_NUMBER);

        //bad value :(
        amplitude[0] = 0;

        averageAmplitudes();
        findMaxAmplitudes();
        normalizeAmplitudes();

        frequency = (uint16_t)SAMPLE_RATE * amplitudeMaxIndex / (uint16_t)FFT_POINTS_NUMBER;


        //------------------------------- OLED
        if (graphStatus == OLED_DISPLAY_MODE_FFT_GRAPH)
        {
            for (int page = 7;
                 page >= 0;
                 --page)
            {
                writeCommand(0xb0 + page); //page number
                // start column = 18 = 0x12
                writeCommand(0x02); //start column low 2
                writeCommand(0x11); //start column high 1

                for (int column = 0;
                     column < OLED_DISPLAY_WIDTH;
                     ++column)
                {
                    if (normalizedAmplitude[column] >= (uint8_t)8)
                    {
                        writeData(0xff);
                        normalizedAmplitude[column] -= (uint8_t)8;
                    }
                    else if (normalizedAmplitude[column] > (uint8_t)0)
                    {
                        writeData(((uint8_t)0xff)
                                  << ((uint8_t)8
                                      - normalizedAmplitude[column]));
                        normalizedAmplitude[column] = (uint8_t)0;
                    }
                    else
                    {
                        writeData(((uint8_t)00));
                    }
                }
            }
        }
        else if (graphStatus == OLED_DISPLAY_MODE_FFT_STATISTICS)
        {
            char stringFrequencyBuff[10];
            char stringVolumeBuff[10];
            char stringIntervalBuff[10];

            intToString(frequency, stringFrequencyBuff, 10, 10);
            intToString(volume, stringVolumeBuff, 10, 10);
            intToString(interval, stringIntervalBuff, 10, 10);

            //oled_clearScreen(OLED_COLOR_BLACK);
            oled_putString(0, 0, "frequency:", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(0, 10, "interval:", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(0, 20, "volume:", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(60, 0, stringFrequencyBuff, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(60, 10, stringIntervalBuff, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(60, 20, stringVolumeBuff, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        }

        //------------------------------- JOYSTICK
        processJoystick();
    }
}


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ AnalyseThatSound */
int main(void)
{
    configureBoard();
    runMainProgramLoop();

    return 0;
}


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */




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
