#ifndef ANALYSE_THAT_SOUND_TYPES_JOYSTICK_DIRECTION_H
#define ANALYSE_THAT_SOUND_TYPES_JOYSTICK_DIRECTION_H


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "joystick-direction-status.h"


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Struct: JoystickDirection */
typedef struct JoystickDirection
{
    JoystickDirectionStatus previous,
                            current;
}
JoystickDirection;


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */


#endif /* ANALYSE_THAT_SOUND_TYPES_JOYSTICK_DIRECTION_H */
