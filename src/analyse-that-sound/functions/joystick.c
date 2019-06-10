/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "analyse-that-sound/functions/joystick.h"

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


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Joystick */
void updateJoystickDirectionStatus(JoystickDirection *joystickDirection,
                                   LPC_GPIO_TypeDef *gpio,
                                   int bit)
{
    joystickDirection->previous = joystickDirection->current;
    joystickDirection->current = !(gpio->FIOPIN & BIT(bit));
}

BOOL isJoystickDirectionToggledOn(JoystickDirection *joystickDirection)
{
    return ((joystickDirection->current == JOYSTICK_DIRECTION_STATUS_ON)
            &&
            (joystickDirection->previous == JOYSTICK_DIRECTION_STATUS_OFF));
}


void updateJoystickStatus(void)
{
    updateJoystickDirectionStatus(&joystick.top,    LPC_GPIO0, 15);
    updateJoystickDirectionStatus(&joystick.bottom, LPC_GPIO2,  3);
    updateJoystickDirectionStatus(&joystick.left,   LPC_GPIO0, 16);
    updateJoystickDirectionStatus(&joystick.right,  LPC_GPIO2,  4);
    updateJoystickDirectionStatus(&joystick.center, LPC_GPIO0, 17);
}

void handleJoystickEvents(void)
{
    // Volume adjustment | Top/Bottom
    if (isJoystickDirectionToggledOn(&joystick.top) == TRUE)
    {
        if (volume > 0)
        {
            volume -= 10;
        }
    }

    if (isJoystickDirectionToggledOn(&joystick.bottom) == TRUE)
    {
        if (volume < 100)
        {
            volume += 10;
        }
    }

    // Interval adjustment | Left/Right
    if (isJoystickDirectionToggledOn(&joystick.right) == TRUE)
    {
        if (interval > 0)
        {
            interval -= 1;
        }
    }

    if (isJoystickDirectionToggledOn(&joystick.left) == TRUE)
    {
        if (interval < 11)
        {
            interval += 1;
        }
    }

    // OLED display mode adjustment | Center
    if (isJoystickDirectionToggledOn(&joystick.center) == TRUE)
    {
        graphStatus = (graphStatus + 1) % 2;
    }
}


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */
