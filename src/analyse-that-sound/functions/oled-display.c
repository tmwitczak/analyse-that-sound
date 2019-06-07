/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "analyse-that-sound/functions/oled-display.h"

#include "analyse-that-sound/functions/helpers.h"

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


void drawFftGraph(void)
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
                          << ((uint8_t)8 - normalizedAmplitude[column]));
                normalizedAmplitude[column] = (uint8_t)0;
            }
            else
            {
                writeData(((uint8_t)00));
            }
        }
    }
}
void drawFftStatistics(void)
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
