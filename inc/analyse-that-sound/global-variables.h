/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "analyse-that-sound/constants.h"

#include <stdint.h>
#include <oled.h>


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Global variables */
/* --------------------------------------------------------- Sine function -- */
extern          uint16_t sineLookupTable[SINE_LOOKUP_TABLE_SIZE]


/* ------------------------------------------------------------------- FFT -- */
extern volatile int      frequency;
extern volatile int	     interval;
extern volatile int	     volume;
extern volatile int      graphStatus;

extern volatile int16_t  sampleBuffer[FFT_POINTS_NUMBER];
extern volatile int16_t  *currentSample;

extern volatile float    fft_buffer[FFT_BUFFER_SIZE];
extern volatile float    amplitude[FFT_POINTS_NUMBER];
extern volatile float	 averagedAmplitude[OLED_DISPLAY_WIDTH];
extern volatile uint8_t  normalizedAmplitude[OLED_DISPLAY_WIDTH];
extern volatile uint16_t averagedAmplitudeMaxIndex;
extern volatile uint16_t amplitudeMaxIndex;

extern volatile int      dacIterator;

extern struct
{
	int left;
	int right;
	int bottom;
	int top;
	int center;
}
joystickStatus;


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */
