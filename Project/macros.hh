#ifndef MACROS_HH
#define MACROS_HH

// Macros for consensus algorithm
#define MAX_ITER_CONSENSUS 30

//Time between streamed value
#define STREAM_TIME 10000 //microseconds

//The size of the minute buffer
#define BUFFER_SIZE 600 //10 items/s

// Macros for user input
#define USER_INPUT_MAX_SIZE 8 // Maximum size of user input: 7 characters + '\0'

// Macros for CAN communication
#define CAN_MSG_SIZE 8
#define CAN_ID_BROADCAST 255

//Macro for maximum number of picos possible
#define MAX_LUMINAIRES 8

// Macros for board configuration
#define LED_PIN 15
#define LDR_PIN A2
#define PWM_FREQ 60000
#define DAC_RANGE 4095
#define SERIAL_BAUD 115200

// Macros for Luminaire ID
#define LUM_A_ID "E66118604B807E27"
#define LUM_B_ID "E66118604B2C3A21"
#define LUM_C_ID "E660C0D1C7680F2F"
#define LUM_ID_SIZE 17

// Macros for Luminaire ID - Mike
#define LUM_A_ID_mike "E66118604B6E5021"
#define LUM_B_ID_mike "E66118604B555C21"
#define LUM_C_ID_mike "E66118604B714221"

//Macros for LUX function (for each luminaire A, B and C)
#define bA 6.127
#define mA -0.80
#define bB 6.480
#define mB -0.81
#define bC 5.820
#define mC -0.77

//Macros for LUX function (for each luminaire A, B and C) (do Mike)
#define bA_mike 6.06
#define mA_mike -0.90
#define bB_mike 5.85
#define mB_mike -0.90
#define bC_mike 5.85
#define mC_mike -0.90

//Macros for luminaire gains
#define GA 0.75614
#define GB 0.76952
#define GC 0.73082

//Sampling time
#define TIME_SAMPLING 0.01 //0.01s -> 100Hz

//LED characteristics
#define PMAX 0.01915 //W

//Number of samples to be computed on low pass average filter
#define AVGR_FILTER_SAMPLE_NUM 10

#endif // MACROS_HH
