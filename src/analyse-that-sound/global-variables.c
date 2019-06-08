/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Includes */
#include "analyse-that-sound/global-variables.h"


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ Global variables */
/* --------------------------------------------------------- Sine function -- */
uint16_t sineLookupTable[SINE_LOOKUP_TABLE_SIZE]
        = {  512,  537,  562,  587,  612,  637,  661,  685,
             709,  732,  755,  777,  798,  819,  838,  858,
             876,  893,  910,  925,  939,  953,  965,  976,
             986,  995, 1003, 1009, 1015, 1019, 1022, 1023,
            1023, 1023, 1020, 1017, 1012, 1006,  999,  991,
             981,  971,  959,  946,  932,  917,  901,  885,
             867,  848,  829,  808,  787,  766,  743,  721,
             697,  673,  649,  625,  600,  575,  549,  524,
             499,  474,  448,  423,  398,  374,  350,  326,
             302,  280,  257,  236,  215,  194,  175,  156,
             138,  122,  106,   91,   77,   64,   52,   42,
              32,   24,   17,   11,    6,    3,    0,    0,
               0,    1,    4,    8,   14,   20,   28,   37,
              47,   58,   70,   84,   98,  113,  130,  147,
             165,  185,  204,  225,  246,  268,  291,  314,
             338,  362,  386,  411,  436,  461,  486,  511 };


/* ------------------------------------------------------------------- FFT -- */
volatile int      frequency   = 500;
volatile int      interval    = 0;
volatile int      volume      = 100;
volatile int      graphStatus = OLED_DISPLAY_MODE_FFT_GRAPH;

volatile int16_t  sampleBuffer[FFT_POINTS_NUMBER];
volatile int16_t  *currentSample = sampleBuffer;

volatile float    fft_buffer[FFT_BUFFER_SIZE];
volatile float    amplitude[FFT_POINTS_NUMBER];
volatile float    averagedAmplitude[OLED_DISPLAY_WIDTH];
volatile uint8_t  normalizedAmplitude[OLED_DISPLAY_WIDTH];
volatile uint16_t averagedAmplitudeMaxIndex = 0;
volatile uint16_t amplitudeMaxIndex = 0;

volatile int      dacIterator = 0;

volatile Joystick joystick;


/* \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */
