/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "analyse-that-sound/functions/main-program-loop.h"

#include "analyse-that-sound/functions/fft-operations.h"
#include "analyse-that-sound/functions/joystick.h"
#include "analyse-that-sound/functions/oled-display.h"

#include "analyse-that-sound/constants.h"
#include "analyse-that-sound/global-variables.h"


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Main program loop */
void runMainProgramLoop(void)
{
    while(TRUE == TRUE)
    {
        doFftComputations();

        switch(graphStatus)
        {
            case OLED_DISPLAY_MODE_FFT_GRAPH:      drawFftGraph();      break;
            case OLED_DISPLAY_MODE_FFT_STATISTICS: drawFftStatistics(); break;
            default:                                                    break;
        }

        updateJoystickStatus();
        handleJoystickEvents();
    }
}


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */
