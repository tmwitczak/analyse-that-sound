/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "analyse-that-sound/constants.h"
#include "analyse-that-sound/global-variables.h"
#include "analyse-that-sound/macros.h"
#include "analyse-that-sound/typedefs.h"

#include <arm_math.h>
#include <arm_const_structs.h>

#include <LPC17xx.h>

#include <lpc17xx_adc.h>
#include <lpc17xx_clkpwr.h>
#include <lpc17xx_dac.h>
#include <lpc17xx_i2c.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_ssp.h>

#include <oled.h>


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Interrupt handlers */
void TIMER1_IRQHandler(void)
{
    /* Choose interval */
    uint32_t intervalFrequency = frequency;
    for (int i = 0;
         i < interval;
         ++i)
    {
        intervalFrequency = intervalFrequency * SEMITONE_RATIO_NUMERATOR
                            / SEMITONE_RATIO_DENOMINATOR;
    }


    /* Generate pure tone with DAC */
    int step = SINE_DAC_FREQUENCY       /* over that many interrupts we
               / intervalFrequency;        must do the whole iteration  */

    int sinPhase = dacIterator * SINE_LOOKUP_TABLE_SIZE
                   / step;

    dacIterator = (dacIterator + 1) % step;

    if(sinPhase >= SINE_LOOKUP_TABLE_SIZE)
    {
        sinPhase = SINE_LOOKUP_TABLE_SIZE - 1;
    }

    DAC_UpdateValue(LPC_DAC, sineLookupTable[sinPhase] * volume
                             / 100);

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


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */
