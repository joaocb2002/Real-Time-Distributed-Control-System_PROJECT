//*************Libraries**************
#include "macros.hh"
#include "luminaire.hh"
#include "PID.hh"
#include "interface.hh"
#include "mcp2515.h"
#include "can.h"
#include <hardware/flash.h> 
#include <hardware/gpio.h>


//*************Global Variables**************
float x_ref = 50.0; //Reference value in LUX (default = 50) 
float r; // Reference value in voltage - obtained from x_ref 
float y; // Measured value in voltage (0-4095) - obvained from converting x to voltage
float u; // Control signal in PWM (0-4095) - applied in the actuator
unsigned long int delta_control; // Timer variables
volatile bool timer_fired {false};
struct repeating_timer timer;
time_t initial_time = millis();


//*************CAN BUS**************
uint8_t this_pico_flash_id[8], node_address;
struct can_frame canMsgTx, canMsgRx;
unsigned long counterTx {0}, counterRx {0};
MCP2515::ERROR err;
unsigned long time_to_write = millis();
unsigned long write_delay {1000};
const byte interruptPin {20};
volatile byte data_available {false};

MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000}; // CS, INT, SCK, MISO, MOSI



//*************Functions**************
bool my_repeating_timer_callback(struct repeating_timer *t)
{
  if(!timer_fired)
    timer_fired = true;
  return true;
}

float analog_low_pass_filter() {
  // Declare an array of floats
  static float arr[AVGR_FILTER_SAMPLE_NUM] = {0.0};

  // Analog read
  for (int i = 0; i < AVGR_FILTER_SAMPLE_NUM; i++) {
    arr[i] = analogRead(A1);
  }

  // Compute the median - start by ordering the array
  for (int i = 0; i < AVGR_FILTER_SAMPLE_NUM; i++) {
    for (int j = i + 1; j < AVGR_FILTER_SAMPLE_NUM; j++) {
      if (arr[i] > arr[j]) {
        float temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }

  // Return the median
  int i = floor(AVGR_FILTER_SAMPLE_NUM / 2);
  return arr[i];
}

void read_interrupt(uint gpio, uint32_t events) {
  data_available = true;
}


//*************Objects**************
CLuminaire lum; //Luminaire object
CPID my_pid(TIME_SAMPLING, 1, 1, 1, 0, 1, 10); //Sampling time = 10ms, K = 1, b = 1, Ti = 1, Td = 0, Tt = 1, N = 10

//*****************Setup****************
void setup() {
  //Get unique flash and node address
  flash_get_unique_id(this_pico_flash_id);
  node_address = this_pico_flash_id[7];

  //Initialize CAN bus
  can0.reset();
  can0.setBitrate(CAN_1000KBPS); 
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt); //Enable interrupt
  time_to_write = millis() + write_delay;

  //Setup serial communication and IO pins
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(ADC_RESOLUTION); //2^12 bits = 4095
  analogWriteFreq(PWM_FREQ); //PWM frequency = 60kHz
  analogWriteRange(DAC_RANGE); //PWM range = 4095 

  //Setup timer for sampling time
  add_repeating_timer_ms(-10, my_repeating_timer_callback, NULL, &timer); //10ms period

  //Get unique board ID
  int len = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
  char id[len];
  pico_get_unique_board_id_string(id, len);

  //Initialize luminaire object, according to board type
  lum.init_lum(id, len);
   
  //Calibrate open loop gain G
  lum.G = lum.calibrate_open_loop_gain_G();

  //Compute H and tau, according to the new LUX reference (x_ref)
  lum.H = lum.compute_open_loop_gain_H(x_ref);
  lum.tau = lum.compute_compute_tau_time_const(x_ref);

  // Update control system parameters, according to the new parameters values
  my_pid.update_parameters(lum.G, lum.H, lum.tau);
  
  //Compute coefficients
  my_pid.compute_coefficients();

  //Compute inicial reference value in voltage
  r = lum.voltage_func(x_ref);
} 


//*************Main Loop*********************
void loop() {

  //Check for user input
  handle_serial(lum, my_pid, x_ref, r, y, u, initial_time);

  //Check for CAN bus data - send
  if( millis() - time_to_write > 5000) { //Send message every 5 seconds
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = 8;
    unsigned long div = counterTx*10;
    for( int i = 0; i < 8; i++ )
      canMsgTx.data[7-i]='0'+((div/=10)%10);

    //Send message
    err = can0.sendMessage(&canMsgTx);
    Serial.print("Error: ");
    Serial.println(err);
    Serial.print("Sending message ");
    Serial.print( counterTx );
    Serial.print(" from node ");
    Serial.print( node_address, HEX );
    Serial.print(" : ");
    for (int i=0 ; i < canMsgTx.can_dlc ; i++)
      Serial.print((char) canMsgTx.data[ i ]);

    Serial.println(" ");
    counterTx++;
    time_to_write = millis();

  }

  //Check for CAN bus data - receive
  if (data_available) {

    Serial.println("Data available!");

    //Read message
    err=can0.readMessage(&canMsgRx);    
    Serial.print("Error: ");
    Serial.println(err);

    //Print message
    Serial.print("Received message number "); Serial.print(counterRx++);
    Serial.print(" from node "); Serial.print(canMsgRx.can_id, HEX);
    Serial.print(" : ");
    for (int i=0 ; i < canMsgRx.can_dlc ; i++)
      Serial.print((char) canMsgRx.data[ i ]);
    Serial.println(" ");

    //Set data_available flag to false
    data_available = false;
  }
  
  //Check for new sampling time
  if(timer_fired) {

    //Check time
    delta_control = -micros();

    //Read analog input value
    y = analog_low_pass_filter(); 

    //Compute H and tau, according to the new LUX reference (x_ref)
    lum.H = lum.compute_open_loop_gain_H(x_ref);
    lum.tau = lum.compute_compute_tau_time_const(x_ref);

    // Update control system parameters, according to the new parameters values
    my_pid.update_parameters(lum.G, lum.H, lum.tau);

    //Compute coefficients
    my_pid.compute_coefficients();

    //Compute reference value in voltage
    r = lum.voltage_func(x_ref);
    
    //Compute control signal (in PWM from 0 to 4095)
    u = my_pid.compute_control(r, y);

    //Write analog output
    analogWrite(LED_PIN, int(u)); 

    //Housekeeping
    my_pid.housekeeping(r, y);

    // Store last minute data (PWM stored in percentage, r in LUX and y in volts)
    lum.store_luminaire_data(u*100/4095, lum.lux_func(y), x_ref);

    //Reset timer
    timer_fired = false;

    //Check time elapsed for control - if it is greater than the sampling time, print a warning
    if ((delta_control + micros()) > TIME_SAMPLING*1000000) { 
      Serial.println("Sampling time exceeded! Check control loop!");
    }
  }  
}


//*************End of code***************
