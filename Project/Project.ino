//*************Libraries**************
#include "macros.hh"
#include "luminaire.hh"
#include "PID.hh"
#include "interface.hh"
#include "mcp2515.h"
#include "can.h"
#include <hardware/flash.h>
#include <hardware/gpio.h>
#include "utils.hh"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <stdio.h>
#include "pico/stdlib.h"



//*************Global Variables**************
float x_ref = 50.0;                   // Reference value in LUX (default = 50)
float r;                              // Reference value in voltage - obtained from x_ref
float y;                              // Measured value in voltage (0-4095) - obvained from converting x to voltage
float u;                              // Control signal in PWM (0-4095) - applied in the actuator
float o;                              // External illuminance (to be measured...)
unsigned long int delta_control;      // Timer variables
volatile bool control_flag{ false };  // Timer flag for control loop
struct repeating_timer timer;         // Timer for control loop
time_t initial_time = millis();       // Initial time measurement

//************* HUB Node **************
bool hub = false;    //Flag to hold if this pico is the hub node
uint8_t hub_id = 0;  //Id of the hub node

//*************CAN BUS**************
uint8_t pico_flash_id[8];
MCP2515::ERROR err;
const byte interruptPin{ 20 };
volatile byte data_available{ false };
uint8_t canintf_save{ 0 };
uint8_t eflg_save{ 0 };
bool detected_errors{ false };
MCP2515 can0{ spi0, 17, 19, 16, 18, 10000000 };  // CS, INT, SCK, MISO, MOSI

//*************Calibration **************
uint8_t num_luminaires = 1;                  //Number of luminaires in the system (including me)
uint8_t pico_id_list[MAX_LUMINAIRES] = {};   //List to hold the ids of the other picos
char luminaire_types[MAX_LUMINAIRES] = {};   //List to hold the types of the other picos
float luminaire_gains[MAX_LUMINAIRES] = {};  //NxN matrix to hold the values of the cross coupling gains
bool calibrated = false;                     //flag to hold status of calibration of this pico
bool message = false;                        //flag to hold if there is a message to be sent

//******** Function prototypes *****
void wait4start();
void calibrate_system();
bool my_repeating_control_callback(struct repeating_timer *t);
float analog_low_pass_filter();
void read_interrupt(uint gpio, uint32_t events);
void calibration_Hub_Node();
void calibration_Other_Node();
void hub_function();


//*************Objects**************
CLuminaire lum;                                 //Luminaire object
CPID my_pid(TIME_SAMPLING, 1, 1, 1, 0, 1, 10);  //Sampling time = 10ms, K = 1, b = 1, Ti = 1, Td = 0, Tt = 1, N = 10

//*************Inter-core communication**************
QueueHandle_t xQueue_01;  //FreeRTOS queue to store messages to be sent from core 0 to core 1
QueueHandle_t xQueue_10;  //FreeRTOS queue to store messages to be sent from core 1 to core 0

//*************FreeRtos Task Running on core 0**************
void Core0_Loop(void *parameter) {
  // Silence warnings about unused parameters.
  (void)parameter;

  // Infinite loop
  for (;;) {
    //Variables initialization
    can_frame frm;
    icc_msg msg;

    //Check if already calibrated - if not, wait for the system start message and calibrate the system
    if (!calibrated) {
      wait4start();
      print_luminaires_ids(pico_id_list, num_luminaires);
      print_luminaires_types(luminaire_types, num_luminaires);
      calibrate_system();
      print_crossover_gains(luminaire_gains, o, num_luminaires);
      lum.set_G(luminaire_gains[0]);
      calibrated = true;
      Serial.println("Calibration done!");
    }

    //If Hub Node, call Hub Function to check for user input
    if (hub) {
      hub_function();
    }

    //Check for incoming data in xQueue10
    while (xQueueReceive(xQueue_10, &msg, 0) == pdTRUE) {
      if (msg.cmd == ICC_READ_DATA) {  //Check the command
        if (msg.frm.data[0] == pico_id_list[0] || msg.frm.data[0] == CAN_ID_BROADCAST) {  //Check if the message is for me (or everyone)
          
          //Extract the message and print it
          uint8_t src_id, dest_id, num;
          char b[CAN_MSG_SIZE];
          can_frame_to_msg(&msg.frm, &src_id, &num, &dest_id, b);
          Serial.printf("Received: %s from id: %d\n", b, int(msg.frm.can_id));

          //Handle message 
          if (strcmp(b, "Reset") == 0) { //Check if the message is to reset the system
            reset_func(); //Reset the system (induce a reset/calibration)
          }
          else { //If not, probably a user input message - handle it and send response to hub node 
            // Initialize response message
            char response[CAN_MSG_SIZE];
            handle_user_command(b, sizeof(b), response, sizeof(response));

            // Send response to hub node
            msg_to_can_frame(&frm, pico_id_list[0], CAN_MSG_SIZE, src_id, response, sizeof(response));
            msg.frm = frm;
            msg.cmd = ICC_WRITE_DATA;
            if (xQueueSendToBack(xQueue_01, &msg, portMAX_DELAY) == pdTRUE) {
              Serial.printf("Sent '%s' message to node %d!\n", response, src_id);
            }
          }
        }
      }
    }

    //Control computation
    if (control_flag) {

      //Check time for control loop
      delta_control = -micros();

      //Read analog input value
      y = analog_low_pass_filter();

      //Compute H and tau, according to the LUX reference (x_ref)
      lum.compute_open_loop_gain_H(x_ref);
      lum.compute_tau_time_const(x_ref);

      // Update control system parameters, according to the new parameters values
      my_pid.update_parameters(lum.get_G(), lum.get_H(), lum.get_tau());

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
      lum.update_luminaire_metrics(u / DAC_RANGE * 100, lum.lux_func(y), x_ref);

      //Reset timer
      control_flag = false;

      //Check time elapsed for control - if it is greater than the sampling time, print a warning
      if ((delta_control + micros()) > TIME_SAMPLING * 1000000) {
        Serial.println("\nWARNING: Sampling time exceeded! Check control loop!");
      }
    }
  }
}

//*************FreeRtos Task Running on core 1**************
void Core1_Loop(void *parameter) {
  // Silence warnings about unused parameters.
  (void)parameter;

  // Infinite loop
  for (;;) {
    //Variables for CAN communication
    can_frame frm;
    icc_msg msg;

    //INCOMING DATA IN CAN BUS
    //Read the message from available buffers and send it to core 0 (with ICC_READ_DATA flag) if there is data available
    uint8_t irq = can0.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF) {
      if (can0.readMessage(MCP2515::RXB0, &frm) == MCP2515::ERROR_OK) {
        msg.frm = frm;
        msg.cmd = ICC_READ_DATA;
        xQueueSendToBack(xQueue_10, &msg, portMAX_DELAY);
      }
    }

    if (irq & MCP2515::CANINTF_RX1IF) {
      if (can0.readMessage(MCP2515::RXB1, &frm) == MCP2515::ERROR_OK) {
        msg.frm = frm;
        msg.cmd = ICC_READ_DATA;
        xQueueSendToBack(xQueue_10, &msg, portMAX_DELAY);
      }
    }

    //OUTGOING DATA IN CAN BUS
    if (xQueueReceive(xQueue_01, (void *)&msg, 0) == pdTRUE) {
      //Check the command
      if (msg.cmd == ICC_WRITE_DATA) {
        //Write the message to the CAN bus
        if (can0.sendMessage(&msg.frm) != MCP2515::ERROR_OK) {
          Serial.println("Errors in sending...");
        }
      }
    }
  }
}

//*****************Setup 0****************
void setup() {

  //Get unique flash and node address - flash calls are unsafe if two cores are operating
  rp2040.idleOtherCore();
  flash_get_unique_id(pico_flash_id);  //Get unique flash id
  pico_id_list[0] = pico_flash_id[6];  // Store my id in the first position of list of known ids
  rp2040.resumeOtherCore();

  //Setup serial communication and IO pins
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(ADC_RESOLUTION);  //2^12 bits = 4095
  analogWriteFreq(PWM_FREQ);             //PWM frequency = 60kHz
  analogWriteRange(DAC_RANGE);           //PWM range = 4095

  //Setup timer for sampling time
  add_repeating_timer_ms(-10, my_repeating_control_callback, NULL, &timer);  //10ms period

  //Get unique board ID - used to identify the luminaire (this is kind of redundant, but it was used in Part 1)
  char pico_string_id[LUM_ID_SIZE];
  pico_get_unique_board_id_string(pico_string_id, LUM_ID_SIZE);

  //Initialize luminaire object, according to board type and store that type in the list of types
  lum.init_lum(pico_string_id);
  luminaire_types[0] = lum.get_type();

  // Control initialization
  lum.set_G(0.50);  //Initial value for G

  //Compute H and tau, according to the new LUX reference (x_ref)
  lum.compute_open_loop_gain_H(x_ref);
  lum.compute_tau_time_const(x_ref);

  // Update control system parameters, according to the new parameters values
  my_pid.update_parameters(lum.get_G(), lum.get_H(), lum.get_tau());

  //Compute coefficients
  my_pid.compute_coefficients();

  //Compute inicial reference value in voltage
  r = lum.voltage_func(x_ref);

  // Create queue for inter-core communication
  xQueue_01 = xQueueCreate(10, sizeof(icc_msg));  //FreeRTOS queue to store messages to be sent from core 0 to core 1
  xQueue_10 = xQueueCreate(10, sizeof(icc_msg));  //FreeRTOS queue to store messages to be sent from core 1 to core 0

  //Launch tasks for each core
  xTaskCreateAffinitySet(Core0_Loop, "Core0_Loop", 1000, nullptr, 1, 0b01, nullptr);  //runs in core 0 only
  xTaskCreateAffinitySet(Core1_Loop, "Core1_Loop", 1000, nullptr, 1, 0b10, nullptr);  //runs in core 1 only
}

//**************Setup 1*******************
void setup1() {
  //Initialize CAN bus
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  can0.clearRXnOVR();

  //Set up interrupt for CAN bus 
  gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt);  //Enable interrupt
}

//*************Main Loop (Core Zero)*********************
void loop() {
  vTaskDelay(100);
}

void loop1() {
  vTaskDelay(100);
}
/////////////////////////////////////////////////////
//          FUNCTIONS DEFINITIONS
/////////////////////////////////////////////////////


//************** Important System Functions **************

void wait4start() {  //Function to wait for the system start message

  /* The luminaire will wait and send a initial message ("Hello!")
  through the CAN BUS periodically for other picos to detect it and acknowledge it. 
  This process will only stop if a start message is received through serial port
  (which will mean that this luminaire is the Hub Luminaire) or if other
  luminaire sends that same message (which means another luminaire was set to hub node)*/

  // Initialize variables
  icc_msg msg;
  can_frame frm;
  uint8_t src_id, dest_id, num;
  unsigned long int local_time = millis();

  while (1) {

    //Every 3s, send a message to the CAN BUS saying "Hello!"
    if (millis() - local_time > 3000) {

      // Create the message to be sent ("Hello <type>") and broadcast it
      char b[CAN_MSG_SIZE];
      char type = lum.get_type();
      sprintf(b, "Hello %c", type);
      msg_to_can_frame(&frm, pico_id_list[0], CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));

      // Send the message through the xQueue01
      msg.frm = frm;
      msg.cmd = ICC_WRITE_DATA;
      if (xQueueSendToBack(xQueue_01, (void *)&msg, portMAX_DELAY) == pdTRUE) {
        Serial.printf("Sending -> ");
        print_can_frame_msg(&msg.frm, CAN_MSG_SIZE);
      }

      //Reset timer
      local_time = millis();
    }

    //Check for incoming data in xQueue10
    while (xQueueReceive(xQueue_10, &msg, 0) == pdTRUE) {
      if (msg.cmd == ICC_READ_DATA) { //Check the command                                                
        if (msg.frm.data[0] == CAN_ID_BROADCAST) {  //Check if the message is broadcast 
          
          //Extract the message
          char b[CAN_MSG_SIZE];
          can_frame_to_msg(&msg.frm, &src_id, &num, &dest_id, b);
          Serial.printf("Received: %s from id: %d\n", b, int(msg.frm.can_id));

          //Check if the message received starts with "Hello"
          if (strncmp(b, "Hello", 5) == 0) {
            //Check if the id is already in the list
            if (find_id(pico_id_list, msg.frm.can_id) >= num_luminaires) {
              pico_id_list[find_id(pico_id_list, 0)] = msg.frm.can_id;  //Add the id to the list of known ids
              luminaire_types[find_type(luminaire_types, 0)] = b[6];      //Add the type to the list of known types
              num_luminaires++;                                        //Increment the number of luminaires
              Serial.printf("NEW NODE ID: %d\nNEW NODE TYPE: %c", int(msg.frm.can_id), b[6]);
            }
          } else if (strcmp(b, "Start") == 0) {  //Check if the message is a start message sent by Hub Node
            hub = true;
            hub_id = msg.frm.can_id;
            return;
          }
        }
      }
    }

    //Check for incoming data in serial port: if a "Start" message is received, this luminaire will be the hub node
    if (Serial.available()) {
      if (Serial.readStringUntil('\n') == "Start") {
        hub = true;
        hub_id = pico_id_list[0];
        break;
      }
    }
  }
}

void calibrate_system() {  //Function to calibrate the system cross coupling gains

  /* The calibration process is coordinated by the hub function node.
  So, all the nodes will obey to received commands, until calibration
  process is over*/

  if (hub) {
    // Give orders
    calibration_Hub_Node();
  } else {
    //Wait for orders
    calibration_Other_Node();
  }
}

void calibration_Hub_Node() {  //Function to calibrate the system cross coupling gains (hub node)

  // Indicate the start of calibration
  Serial.println("\nStarting Calibration (I am Hub)...");

  // Initialize variables
  icc_msg msg;
  can_frame frm;

  // Send message to all the picos to start calibration
  char b[CAN_MSG_SIZE];
  strcpy(b, "S Cal");
  msg_to_can_frame(&frm, hub_id, CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));
  msg.frm = frm;
  msg.cmd = ICC_WRITE_DATA;
  if (xQueueSendToBack(xQueue_01, (void *)&msg, portMAX_DELAY) == pdTRUE) {
    //Print sent message 'b'
    Serial.printf("Calibration message sent: %s\n", b);
  }

  //Wait for 3 seconds for the other picos to receive their messages
  vTaskDelay(3000);

  //Turn off every led (the other who receives the message will also turn off their led)
  analogWrite(LED_PIN, int(0));  //turn off my led

  //Wait for 3 seconds for the other picos to turn off their leds
  vTaskDelay(3000);

  // Register the external illuminance
  Serial.println("Measuring external illuminance...");
  o = lum.lux_func(analog_low_pass_filter());

  //Tell other picos to measure their external illuminance
  strcpy(b, "Exter");
  msg_to_can_frame(&frm, hub_id, CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));
  msg.frm = frm;
  msg.cmd = ICC_WRITE_DATA;
  if (xQueueSendToBack(xQueue_01, (void *)&msg, portMAX_DELAY) == pdTRUE) {
    //Print sent message 'b'
    Serial.printf("Calibration message sent: %s\n", b);
  }

  //Wait for 3 seconds for the other picos to measure their external illuminance
  vTaskDelay(3000);

  //Iterate over each one of the picos, turning one led on at a time and measuring gains
  for (int i = 0; i < num_luminaires; i++) {
    Serial.printf("\nCalibrating node %d of type %c", pico_id_list[i], luminaire_types[i]);
    if (i == 0) {
      analogWrite(LED_PIN, int(DAC_RANGE));  //turn on my led
    }

    //Create message: "Calibrate <id>"
    char b[CAN_MSG_SIZE];
    strcpy(b, "Cal _");
    b[4] = pico_id_list[i];  // Store the node id to turn on

    // Send message to warn other picos to measure gains (and the corresponding one to turn on its own led)
    msg_to_can_frame(&frm, hub_id, CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));
    msg.frm = frm;
    msg.cmd = ICC_WRITE_DATA;
    if (xQueueSendToBack(xQueue_01, &msg, portMAX_DELAY) == pdTRUE) {
      Serial.printf("Sent 'Calibrate %d' message!\n", pico_id_list[i]);
    }

    //Wait some time for steady state (5s)
    vTaskDelay(5000);  

    //Measure gains
    Serial.println("\nMeasuring gains...");
    luminaire_gains[i] = lum.lux_func(analog_low_pass_filter()) / 100;

    //Wait some time for sync (3s)
    vTaskDelay(3000);  

    //Turn off my led
    analogWrite(LED_PIN, int(0));  //turn off my led
  }

  //Tell other picos calibration is done
  char b[CAN_MSG_SIZE] = "E Cal";
  msg_to_can_frame(&frm, pico_id_list[0], CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));
  msg.frm = frm;
  msg.cmd = ICC_WRITE_DATA;
  if (xQueueSendToBack(xQueue_01, &msg, portMAX_DELAY) == pdTRUE) {
    Serial.println("\n\nSent 'End Calibration' message!");
  }
}

void calibration_Other_Node() {  //Function to calibrate the system cross coupling gains (other nodes)

  // Indicate the start of calibration
  Serial.println("Starting Calibration (I am NOT Hub)...");

  //Turn off my led
  analogWrite(LED_PIN, int(0));  //turn off my led

  //Wait for orders of the hub node
  while (1) {
    //Check for incoming data in xQueue10
    icc_msg msg;
    while (xQueueReceive(xQueue_10, &msg, 0) == pdTRUE) {
      if (msg.cmd == ICC_READ_DATA) {               //Check the command
        if (msg.frm.data[0] == CAN_ID_BROADCAST) {  //Check if the message is for me (or everyone)
          //Extract the message
          uint8_t src_id, dest_id, num, node_id;
          char b[CAN_MSG_SIZE];
          can_frame_to_msg(&msg.frm, &src_id, &num, &dest_id, b);

          // Print the message
          Serial.printf("\nReceived: %s from id: %d\n", b, int(msg.frm.can_id));

          //Check if the message is to end calibration
          if (strcmp(b, "E Cal") == 0) {
            return;
          }

          //Check if the message is to measure external illuminance
          if (strcmp(b, "Exter") == 0) {
            analogWrite(LED_PIN, int(0));  //turn off my led (Redundant but just to make sure)
            o = lum.lux_func(analog_low_pass_filter());
          }

          //Check if the message is to do calibration (starts with Cal)
          if (b[0] == 'C' && b[1] == 'a' && b[2] == 'l') {

            // Extract the node id
            node_id = (uint8_t)b[4];  // Check what is the id to turn on
            Serial.printf("Calibrating node %d\n", node_id);

            // Check the position of the id in the list
            int pos = find_id(pico_id_list, node_id);
            Serial.printf("Position: %d\n", pos);

            // Turn on my led, if it is my turn
            if (pos == 0) {
              analogWrite(LED_PIN, int(DAC_RANGE - 1));  //turn on my led
            } else {
              analogWrite(LED_PIN, 0);  //Redundant but just to make sure
            }

            // Introduce some delay and measure gains
            vTaskDelay(5000);
            luminaire_gains[pos] = lum.lux_func(analog_low_pass_filter()) / 100;
            vTaskDelay(3000);  

            // Turn off my led
            analogWrite(LED_PIN, int(0));  
          }
        }
      }
    }
  }
}

void reset_func() {  //Function to reset the system parameters: will induce a reset 

  // Reset the calibration/initialization flag
  calibrated = false;

  // Reset the control flag
  control_flag = false;

  // Reset the timer
  add_repeating_timer_ms(-10, my_repeating_control_callback, NULL, &timer);

  // Reset the luminaire object
  lum.reset_lum();

  // Reset the PID object
  my_pid.reset_pid();

  // Clear the queues
  xQueueReset(xQueue_01);
  xQueueReset(xQueue_10);

  // Reset the number of luminaires
  num_luminaires = 1;

  // Reset the list of known ids
  for (int i = 1; i < MAX_LUMINAIRES; i++) {
    pico_id_list[i] = 0;
  }

  // Reset the list of known types
  for (int i = 1; i < MAX_LUMINAIRES; i++) {
    luminaire_types[i] = 0;
  }

  // Reset the list of gains
  for (int i = 1; i < MAX_LUMINAIRES; i++) {
    luminaire_gains[i] = 0;
  }

  // Reset the hub flag
  hub = false;
}


//************** Hub Function **************
void hub_function() {  //Function to handle user input in the hub node

  // Initialize variables
  char user_input[USER_INPUT_MAX_SIZE];
  icc_msg msg;
  can_frame frm;

  // Check for user input
  if (Serial.available()) {

    // Read the command, if there is any, and store in user_input
    String command = Serial.readStringUntil('\n');
    command.toCharArray(user_input, USER_INPUT_MAX_SIZE);

    // Check if the command is to reset the system (only command that does not specify a node)
    if (strcmp(user_input, "r") == 0) {

      //Tell other picos to reset their subsystem
      char b[CAN_MSG_SIZE] = "Reset";
      msg_to_can_frame(&frm, pico_id_list[0], CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));
      msg.frm = frm;
      msg.cmd = ICC_WRITE_DATA;
      if (xQueueSendToBack(xQueue_01, &msg, portMAX_DELAY) == pdTRUE) {
        Serial.println("Sent 'Reset' message!");
      }

      //Reset the my subsystem
      reset_func();
    }
    
    // Check for which node the command is intended: each node has a unique type (A, B, C, etc. - in capital letters)
    char node_type = find_user_command_destination(user_input, USER_INPUT_MAX_SIZE);
    uint8_t node_id = pico_id_list[find_type(luminaire_types, node_type)];

    // Check if the command is to a valid node
    if (node_type == 'X') {
      Serial.println("Invalid command! Please, try again.");
      return;
    }

    // Initialize response message
    char response[CAN_MSG_SIZE];

    // If command is intended to Hub node (me), then process it right away
    if (node_id == pico_id_list[0])
    {
      handle_user_command(user_input, sizeof(user_input), response, sizeof(response));
      Serial.printf("Input: %s\nResponse: %s\n", user_input, response);
    }
    
    // If command is intended to other nodes, then send it to the other nodes
    msg_to_can_frame(&frm, pico_id_list[0], CAN_MSG_SIZE, node_id, user_input, sizeof(user_input));
    msg.frm = frm;
    msg.cmd = ICC_WRITE_DATA;
    if (xQueueSendToBack(xQueue_01, &msg, portMAX_DELAY) == pdTRUE) {
      Serial.printf("Sent '%s' message to node %d (type %c)!\n", user_input, node_id, node_type);
    }
  }
}


//************** Timers, interrupts and callbacks functions **************

bool my_repeating_control_callback(struct repeating_timer *t) {  //Function to set control flag to true
  // Silence warnings about unused parameters.
  (void)t;
  if (!control_flag)
    control_flag = true;
  return true;
}

float analog_low_pass_filter() {  //Function to read the analog input value and apply a low pass filter
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

void read_interrupt(uint gpio, uint32_t events) {  //Function to detect interrupt for CAN bus (data available)
  // Silence warnings about unused parameters.
  (void)gpio;
  (void)events;
  data_available = true;
}


//********* User interface functions *********

// Function to detect destination node from a user command
char find_user_command_destination(char *command, int size) {
  
  // Iterate over the command to find the destination node (unique) type: A, B or C
  for (size_t i = 0; i < size; i++)
  {
    if (command[i] == 'A' || command[i] == 'B' || command[i] == 'C') {
      return command[i];
    }
    else if (command[i] == '\0') { // End of string
      break;
    }
  }
  // Return invalid type
  return 'X';
}

// Function to handle user interface commands
void handle_user_command(char* user_input, int size, char* response, int response_size) {
  
  // Convert the user input to a string
  String command = String(user_input);

  // Initialize variables
  char i;
  int occupancy, anti_windup, feedback;
	float duty_cycle, lux;

  // Process the command
	if (command.startsWith("d")) { // Set duty cycle directly (in percentage)
		sscanf(command.c_str(), "d %c %f", &i, &duty_cycle);
		if (i == lum.get_type()) {
      x_ref = lum.get_G() * duty_cycle;
		}
		strcpy(response, "ack");
	} else if (command.startsWith("g d")) { // Get duty cycle
		sscanf(command.c_str(), "g d %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "d %c %f", i, u*100/4095);
		}
	} else if (command.startsWith("r")) { // Set reference value
		sscanf(command.c_str(), "r %c %f", &i, &lux);
		if (i == lum.get_type()) {
      x_ref = lux;
    }
    strcpy(response, "ack");
	} else if (command.startsWith("g r")) { // Get reference value
		sscanf(command.c_str(), "g r %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "r %c %f", i, x_ref);
    }
	} else if (command.startsWith("g l")) { // Measure luminance
		sscanf(command.c_str(), "g l %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "l %c %f", i, lum.lux_func(analog_low_pass_filter()));
    }
	} else if (command.startsWith("o")) { // Set occupancy status
		sscanf(command.c_str(), "o %c %d", &i, &occupancy);
		if (i == lum.get_type()) {
      lum.set_occupancy(occupancy);
      strcpy(response, "ack");
    }
	} else if (command.startsWith("g o")) { // Get occupancy status
		sscanf(command.c_str(), "g o %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "o %c %d", i, lum.get_occupancy());
    }
	} else if (command.startsWith("a")) { // Set anti-windup flag
		sscanf(command.c_str(), "a %c %d", &i, &anti_windup);
		if (i == lum.get_type()) {
      my_pid.set_anti_windup(anti_windup);
      strcpy(response, "ack");
    }
	} else if (command.startsWith("g a")) { // Get anti-windup flag
		sscanf(command.c_str(), "g a %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "a %c %d", i, my_pid.get_anti_windup());
    }
	} else if (command.startsWith("k")) { // Set feedback flag
		sscanf(command.c_str(), "k %c %d", &i, &feedback);
		if (i == lum.get_type()) {
      my_pid.set_feedback(feedback);
      strcpy(response, "ack");
    }
	} else if (command.startsWith("g k")) { // Get feedback flag
		sscanf(command.c_str(), "g k %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "k %c %d", i, my_pid.get_feedback());
    }
	} else if (command.startsWith("g x")) { // Get external illuminance
		sscanf(command.c_str(), "g x %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "x %c %f", i, o);
    }
	} else if (command.startsWith("g p")) { // Get instantaneous power
		sscanf(command.c_str(), "g p %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "p %c %f", i, 1000*u*1/4095*PMAX);
    }
	} else if (command.startsWith("g t")) { //Get time since last restart
		sscanf(command.c_str(), "g t %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "t %c %d", i, (millis() - initial_time) / 1000);
    }
	} else if (command.startsWith("g e")) { // Get energy
		sscanf(command.c_str(), "g e %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "e %c %f", i, lum.get_total_energy_consumption());
    }
	} else if (command.startsWith("g v")) { // Get visibility error
		sscanf(command.c_str(), "g v %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "v %c %f", i, lum.get_total_visibility_error());
    }
	} else if (command.startsWith("g f")) { // Get flicker
		sscanf(command.c_str(), "g f %c", &i);
		if (i == lum.get_type()) {
      sprintf(response, "f %c %f", i, lum.get_total_flicker());
	  }
  } else {
    strcpy(response, "err");
  }
}

//*************End of code***************
