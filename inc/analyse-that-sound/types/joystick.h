#ifndef ANALYSE_THAT_SOUND_TYPES_JOYSTICK_H
#define ANALYSE_THAT_SOUND_TYPES_JOYSTICK_H


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "joystick-direction.h"


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Struct: Joystick */
typedef struct Joystick
{
    JoystickDirection left,
                      right,
                      bottom,
                      top,
                      center;
}
Joystick;


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */


#endif /* ANALYSE_THAT_SOUND_TYPES_JOYSTICK_H */
