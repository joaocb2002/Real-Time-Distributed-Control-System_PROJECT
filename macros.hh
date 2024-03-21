#ifndef MACROS_HH
#define MACROS_HH

// Macros for board configuration
#define LED_PIN 15
#define PWM_FREQ 60000
#define DAC_RANGE 4095
#define SERIAL_BAUD 115200

//Macros for LUX function (for each luminaire A, B and C)
#define LUM_A_ID "E66118604B807E27"
#define bA 6.127
#define mA -0.80
#define GA 0.75614
#define LUM_B_ID "E66118604B2C3A21"
#define bB 6.480
#define mB -0.81
#define GB 0.76952
#define LUM_C_ID "E660C0D1C7680F2F"
#define bC 5.820
#define mC -0.77
#define GC 0.73082

//Sampling time
#define TIME_SAMPLING 0.01 //0.01s -> 100Hz

//LED characteristics
#define PMAX 0.01915 //W

//Number of samples to be computed on low pass average filter
#define AVGR_FILTER_SAMPLE_NUM 10

#endif // MACROS_HH