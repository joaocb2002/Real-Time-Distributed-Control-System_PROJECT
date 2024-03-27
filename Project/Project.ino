//*************Libraries**************
#include "macros.hh"
#include "luminaire.hh"
#include "PID.hh"
#include "interface.hh"
#include "mcp2515.h"
#include "can.h"
#include <hardware/flash.h>
#include <hardware/gpio.h>
#include "utils.h"


//*************Global Variables**************
float x_ref = 50.0;               //Reference value in LUX (default = 50)
float r;                          // Reference value in voltage - obtained from x_ref
float y;                          // Measured value in voltage (0-4095) - obvained from converting x to voltage
float u;                          // Control signal in PWM (0-4095) - applied in the actuator
unsigned long int delta_control;  // Timer variables
volatile bool timer_fired{ false };
struct repeating_timer timer;
time_t initial_time = millis();


//*************CAN BUS**************
uint8_t pico_flash_id[8];
struct can_frame canMsgTx, canMsgRx;
unsigned long counterTx{ 0 }, counterRx{ 0 };
MCP2515::ERROR err;
unsigned long time_to_write = millis();
unsigned long write_delay{ 1000 };
const byte interruptPin{ 20 };
volatile byte data_available{ false };
uint8_t canintf_save{ 0 };
uint8_t eflg_save{ 0 };
bool detected_errors{ false };

MCP2515 can0{ spi0, 17, 19, 16, 18, 10000000 };  // CS, INT, SCK, MISO, MOSI

//*************Calibration stuff**************
//Maxium number of picos possible
const uint8_t N = 8;
//List to hold the ids of the other picos
uint8_t id_list[N] = {};
//NxN matrix to hold the values of the cross coupling gains
uint8_t Gains[N][N] = {};
//o is the external illumination I measure, I do not care about other
uint8_t o = 0; 
//flag to hold status of calibration.
bool calibrated = false;
//flag to hold if there is a message to be sent 
bool message = false;

//*************Functions**************
bool my_repeating_timer_callback(struct repeating_timer *t) {
  if (!timer_fired)
    timer_fired = true;
  return true;
}

float analog_low_pass_filter() {
  // Declare an array of floats
  static float arr[AVGR_FILTER_SAMPLE_NUM] = { 0.0 };

  // Analog read
  for (int i = 0; i < AVGR_FILTER_SAMPLE_NUM; i++) {
    arr[i] = analogRead(LDR_PIN);
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
CLuminaire lum;                                 //Luminaire object
CPID my_pid(TIME_SAMPLING, 1, 1, 1, 0, 1, 10);  //Sampling time = 10ms, K = 1, b = 1, Ti = 1, Td = 0, Tt = 1, N = 10

//*****************Setup****************
// The setup for the main core
void setup() {
  //Get unique flash and node address
  rp2040.idleOtherCore();
  //flash calls are unsafe if two cores are operating
  flash_get_unique_id(pico_flash_id);
  id_list[0] = pico_flash_id[6];
  rp2040.resumeOtherCore();

  //Setup serial communication and IO pins
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(ADC_RESOLUTION);  //2^12 bits = 4095
  analogWriteFreq(PWM_FREQ);             //PWM frequency = 60kHz
  analogWriteRange(DAC_RANGE);           //PWM range = 4095

  //Setup timer for sampling time
  add_repeating_timer_ms(-10, my_repeating_timer_callback, NULL, &timer);  //10ms period

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

// The setup for the core dedicated to the can bus
void setup1() {
  //Initialize CAN bus
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt);  //Enable interrupt
  time_to_write = millis() + write_delay;
}

//*************Main Loop*********************
void loop() {

  can_frame frm;
  uint32_t msg;
  uint8_t b[4];

  //Check for user input
  uint8_t reset = handle_serial(lum, my_pid, x_ref, r, y, u, initial_time);

  if (reset == 2){
    reset_func();
  }

  //If pico is NOT yet calibrated, send the I am alive message
  if(!calibrated){
    message = true;
    //The message: (i)m (a)live
    b[2] = id_list[0];
    b[0] = int('i'); //Convert the character to corresponding ascii value
    b[1] = int('a');
  }

  //If it is time to send message, send message to other core
  //Conditions: there must be a message to be sent AND there is a minimum time between messages to avoid overflow
  if (message && millis() >= time_to_write) {
    message = false;
    b[3] = ICC_WRITE_DATA;  // Identifier
    //b[2] = id_list[0];      // Which id is sending the msg? 0 index corresponds to me
    //b[0] = counterTx;
    //b[1] = (counterTx >> 8);
    rp2040.fifo.push(bytes_to_msg(b));

    Serial.print(">>>>>> Sending ");
    print_message(b[0], b[1], b[2], b[2]);
    //counterTx++;
    time_to_write = millis() + write_delay;
  }

  //check incoming data in fifo queue
  if (rp2040.fifo.pop_nb(&msg)) {
    msg_to_bytes(msg, b);
    if (b[3] == ICC_READ_DATA) {

      //Print the message received
      Serial.print("<<<<<< Received: ");
      print_message(b[0], b[1], id_list[0], b[2]);

      //If I received a I am alive message
      if(char(b[0]) == 'i' && char(b[1]) == 'a')
      {
        //Update the list of known ids
        id_list[find(id_list, b[2])] = b[2];
      }

      //If i receive a reset message
      if(char(b[0]) == 'R')
      {
        if (b[1] == 0){
          //Set calibrated to off, we are in reset mode (PID control is off)
          calibrated = false;
        }
        else if(b[1] == 1){
          calibrated = true;
        }
      }

      //If message is for me 
      if(b[2] == id_list[0]){
        //Someone is ordering me to set my duty cycle
        if (char(b[0]) == 'd'){
          analogWrite(LED_PIN, int(float(b[1]/100*DAC_RANGE)));
        }
      }

      //Temporary print id_list
      Serial.print("\nIds I am aware: ");
      for (int i = 0; i < sizeof(id_list) / sizeof(id_list[0]); i++) {
        Serial.print(id_list[i]);
        Serial.print(" ");
      }
      Serial.println("");
    
    } else if (b[3] == ICC_ERROR_DATA) {
      //print_can_errors(b[1],b[0]);
      if (b[0] & 0b11111000) {  //RX0OV | RX1OV | TXBO | TXEP | RXEP
        eflg_save = b[0];
        canintf_save = b[1];
        can0.clearRXnOVRFlags();
        can0.clearInterrupts();
        detected_errors = true;
      }
    }
    if (detected_errors) {
      Serial.println(" ***** Detected errors in past operations *****");
      //print_can_errors(canintf_save, eflg_save);
    }
  }

  //Check for new sampling time and if the system is calibrated 
  if (timer_fired && calibrated) {

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
    lum.store_luminaire_data(u * 100 / 4095, lum.lux_func(y), x_ref);

    //Reset timer
    timer_fired = false;

    //Check time elapsed for control - if it is greater than the sampling time, print a warning
    if ((delta_control + micros()) > TIME_SAMPLING * 1000000) {
      Serial.println("Sampling time exceeded! Check control loop!");
    }
  }
}

void loop1() {
  can_frame frm;
  uint32_t msg;
  uint8_t b[4];

  //reading the can-bus and writing the fifo
  //if interrupt has happened (message received)
  if (data_available) {
    data_available = false;
    uint8_t irq = can0.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF) {
      can0.readMessage(MCP2515::RXB0, &frm);
      //Push message to other core
      rp2040.fifo.push_nb(can_frame_to_msg(&frm));
    }
    if (irq & MCP2515::CANINTF_RX1IF) {
      can0.readMessage(MCP2515::RXB1, &frm);

      rp2040.fifo.push_nb(can_frame_to_msg(&frm));
    }
    uint8_t err = can0.getErrorFlags();
    rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
  }

  //send message to the can bus
  if (rp2040.fifo.pop_nb(&msg)) {
    msg_to_bytes(msg, b);
    if (b[3] == ICC_WRITE_DATA) {
      frm.can_id = b[2];
      frm.can_dlc = 2;
      frm.data[1] = b[1];
      frm.data[0] = b[0];
      can0.sendMessage(&frm);
    }
    uint8_t irq = can0.getInterrupts();
    uint8_t err = can0.getErrorFlags();
    rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
  }
}

void reset_func(){
  //Tell other picos i am entering reset/calibration mode
  send_message_every(int('R'),0);

  //Turn off my led and all the other LEDs
  analogWrite(LED_PIN, int(0)); //turn off my led
  send_message_every(int('d'),0);

  delay(1000); //Wait some time for steady state
  //Measure my external illuminance
  o = lum.lux_func(analog_low_pass_filter());
  Serial.println(o);
  //Send message to other picos ordering them to measure their external illuminance
  //TO_DO

  //Iterate over each one of the picos
  //At each iteration, turn one led on and measure stuff
  for (int i = 0; i < sizeof(id_list) / sizeof(id_list[0]); i++) {
    if(id_list[i] != 0){
      turn_off_except(id_list[i]);
      //Measure gains
      //Send message to other picos, telling them to measure gains
      //TO_DO
    }
  }

  calibrated = true;
  //Tell other picos calibration is done
  send_message_every(int('R'),1);
}

//Function to turn off the LED of all of the picos except one
void turn_off_except(uint8_t id){
  analogWrite(LED_PIN, int(0)); //turn off my led
  //Loop through each know pico and tell them to turn off their led
  for (int i = 0; i < sizeof(id_list) / sizeof(id_list[0]); i++) {
    if(id_list[i] != 0){

      //If the id to turn on is my id, i turn me on
      if(id_list[i] == id && i == 0){
        analogWrite(LED_PIN, int(DAC_RANGE)); //turn on my led
      }

      //If the id is other than my id
      //Send message to turn that id on
      else if(id_list[i] == id && i != 0){
        //The message: set (d)uty cycle to max
        //Max 8byte int is 255, so send the pwm from 0 to 100, where 100 is converted to DAC_RANGE
        send_message(id_list[i],int('d'),100);
      }

      //Else send message to turn off
      else{ 
         //The message: set (d)uty cycle to zero
        send_message(id_list[i],int('d'),0);
      }
      
      delay(write_delay);//Delay to make sure messages are sent with no errors
    }
  }
}

//Function to format message and send to fifo queue
void send_message(uint8_t id, uint8_t first_byte, uint8_t second_byte){
    uint8_t b[4];
    b[3] = ICC_WRITE_DATA;  // Identifier
    b[2] = id;
    b[0] = first_byte; 
    b[1] = second_byte;
    rp2040.fifo.push(bytes_to_msg(b));
    Serial.print(">>>>>> Sending ");
    print_message(b[0], b[1], b[2], b[2]);
}

//Function to send message to all the picos except me 
void send_message_every(uint8_t first_byte, uint8_t second_byte){
  //Loop through each know pico other than me
  for (int i = 1; i < sizeof(id_list) / sizeof(id_list[0]); i++) {
    if(id_list[i] != 0){
      send_message(id_list[i],first_byte, second_byte);
      delay(write_delay);//Delay to make sure messages are sent with no errors
    }
  }
}
//*************End of code***************
