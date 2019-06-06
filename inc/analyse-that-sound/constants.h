#ifndef ANALYSE_THAT_SOUND_CONSTANTS_H
#define ANALYSE_THAT_SOUND_CONSTANTS_H

/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Constants */
/* -------------------------------------------------------- Boolean values -- */
#define TRUE  1
#define FALSE 0

/* -------------------------------------------------------- FFT parameters -- */
#define SAMPLE_RATE                8000
#define FFT_POINTS_NUMBER          1024
#define FFT_BUFFER_SIZE            (2 * FFT_POINTS_NUMBER)
#define FFT_AMPLITUDE_USABLE_RANGE (FFT_POINTS_NUMBER / 2)

/* --------------------------------------------------------- FFT computations */
#define NUMBER_OF_AVERAGED_AMPLITUDES                                          \
                (FFT_AMPLITUDE_USABLE_RANGE / OLED_DISPLAY_WIDTH               \
                 + (((FFT_AMPLITUDE_USABLE_RANGE % OLED_DISPLAY_WIDTH) != 0)   \
                    ? 1                                                        \
                    : 0))

/* ---------------------------------------------- Sine function parameters -- */
#define SINE_LOOKUP_TABLE_SIZE 128

/* ----------------------------------------------------- OLED display mode -- */
#define NUMBER_OF_OLED_DISPLAY_MODES 2

enum
{
    OLED_DISPLAY_MODE_FFT_GRAPH,
    OLED_DISPLAY_MODE_FFT_STATISTICS
};

/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */


#endif /* ANALYSE_THAT_SOUND_CONSTANTS_H */
