/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "analyse-that-sound/functions/fft-operations.h"

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

void findFundamentalFrequency(void)
{
    frequency = (uint16_t)SAMPLE_RATE * amplitudeMaxIndex
                / (uint16_t)FFT_POINTS_NUMBER;
}

void zeroUnusedAmplitudes(void)
{
    amplitude[0] = 0;
}

void computeFft(void)
{
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_buffer, 0, 1);
    arm_cmplx_mag_f32(fft_buffer, amplitude, FFT_POINTS_NUMBER);
}

void doFftComputations(void)
{
    fillFftBuffer();
    computeFft();

    zeroUnusedAmplitudes();
    averageAmplitudes();
    findMaxAmplitudes();
    normalizeAmplitudes();

    findFundamentalFrequency();
}