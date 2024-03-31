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
  for (;;) {
    can_frame frm;
    icc_msg msg;

    //Check if already calibrated - if not, wait for the system start message and calibrate the system
    if (!calibrated) {
      wait4start();
      print_luminaires_ids(pico_id_list, num_luminaires);
      calibrate_system();
      print_crossover_gains(luminaire_gains, o, num_luminaires);
      lum.set_G(luminaire_gains[0]);
      calibrated = true;
      Serial.println("Calibration done!");
    }

    //Check for user input (TO DO: MAKE THIS SHIT BETTER)
    uint8_t reset = handle_serial(lum, my_pid, x_ref, y, u, initial_time);

    if (reset == 2) {
      calibrated = false;
      // Create the message to be sent: "Reset"
      //Tell other picos to reset system
      char b[CAN_MSG_SIZE] = "Reset";
      msg_to_can_frame(&frm, pico_id_list[0], CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));
      msg.frm = frm;
      msg.cmd = ICC_WRITE_DATA;
      if (xQueueSendToBack(xQueue_01, &msg, portMAX_DELAY) == pdTRUE) {
        Serial.println("Sent 'Reset' message!");
      }
    }

    //Check for incoming data in xQueue10
    while (xQueueReceive(xQueue_10, &msg, 0) == pdTRUE) {
      if (msg.cmd == ICC_READ_DATA) {                                                     //Check the command
        if (msg.frm.data[0] == pico_id_list[0] || msg.frm.data[0] == CAN_ID_BROADCAST) {  //Check if the message is for me
          //Extract the message
          uint8_t src_id, dest_id, num;
          char b[CAN_MSG_SIZE];
          can_frame_to_msg(&msg.frm, &src_id, &num, &dest_id, b);
          Serial.printf("Received: %s from id: %d\n", b, int(msg.frm.can_id));

          //Check if the message is a Reset message
          if (strcmp(b, "Reset") == 0) {
            calibrated = false;
          }
        }
      }
    }

    //Control computation
    if (control_flag && calibrated) {

      //Check time
      delta_control = -micros();

      //Read analog input value
      y = analog_low_pass_filter();

      //Compute H and tau, according to the new LUX reference (x_ref)
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
        Serial.println("Sampling time exceeded! Check control loop!");
      }
    }
  }
}

//*************FreeRtos Task Running on core 1**************
void Core1_Loop(void *parameter) {
  // Silence warnings about unused parameters.
  (void)parameter;

  for (;;) {
    //Variables for CAN communication
    can_frame frm;
    icc_msg msg;

    //INCOMING DATA IN CAN BUS
    //Read the message from available buffers and send it to core 0 (with ICC_READ_DATA flag)
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
  //while(!Serial.available()){
  //wait for serial comms
  //}
  analogReadResolution(ADC_RESOLUTION);  //2^12 bits = 4095
  analogWriteFreq(PWM_FREQ);             //PWM frequency = 60kHz
  analogWriteRange(DAC_RANGE);           //PWM range = 4095

  //Setup timer for sampling time
  add_repeating_timer_ms(-10, my_repeating_control_callback, NULL, &timer);  //10ms period

  //Get unique board ID - used to identify the luminaire (this is kind of redundant, but it was used in Part 1)
  char pico_string_id[LUM_ID_SIZE];
  pico_get_unique_board_id_string(pico_string_id, LUM_ID_SIZE);

  //Initialize luminaire object, according to board type
  lum.init_lum(pico_string_id);

  /*****************************************/
  //Calibrate open loop gain G
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

  xQueue_01 = xQueueCreate(10, sizeof(icc_msg));  //FreeRTOS queue to store messages to be sent from core 0 to core 1
  xQueue_10 = xQueueCreate(10, sizeof(icc_msg));  //FreeRTOS queue to store messages to be sent from core 1 to core 0

  xTaskCreateAffinitySet(Core0_Loop, "Core0_Loop", 1000, nullptr, 1, 0b01, nullptr);  //runs in core 0 only
  xTaskCreateAffinitySet(Core1_Loop, "Core1_Loop", 1000, nullptr, 1, 0b10, nullptr);  //runs in core 1 only
}

//**************Setup 1*******************
void setup1() {
  //Initialize CAN bus
  delay(3000);
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  can0.clearRXnOVR();
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
  luminaire sends that same message.*/

  // Initialize variables
  icc_msg msg;
  can_frame frm;
  uint8_t src_id, dest_id, num;
  unsigned long int local_time = millis();

  while (1) {

    //Every 3s, send a message to the CAN BUS saying "Hello!"
    if (millis() - local_time > 3000) {

      // Create the message to be sent: "Hello!"
      char b[CAN_MSG_SIZE];
      strcpy(b, "Hello!");
      msg_to_can_frame(&frm, pico_id_list[0], CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));

      // Send the message through the xQueue01
      msg.frm = frm;
      msg.cmd = ICC_WRITE_DATA;
      if (xQueueSendToBack(xQueue_01, (void *)&msg, portMAX_DELAY) == pdTRUE) {
        Serial.printf("Sending ->");
        print_can_frame_msg(&msg.frm, CAN_MSG_SIZE);
      }

      //Reset timer
      local_time = millis();
    }

    //Check for incoming data in xQueue10
    while (xQueueReceive(xQueue_10, &msg, 0) == pdTRUE) {
      if (msg.cmd == ICC_READ_DATA) {                                                     //Check the command
        if (msg.frm.data[0] == pico_id_list[0] || msg.frm.data[0] == CAN_ID_BROADCAST) {  //Check if the message is for me
          //Extract the message
          char b[CAN_MSG_SIZE];
          can_frame_to_msg(&msg.frm, &src_id, &num, &dest_id, b);
          Serial.printf("Received: %s from id: %d\n", b, int(msg.frm.can_id));

          //Check if the message is a Hello message
          if (strcmp(b, "Hello!") == 0) {                                //b+1 beacuse first byte is dest_id
            if (find(pico_id_list, msg.frm.can_id) >= num_luminaires) {  //Check if the id is already in the list
              pico_id_list[find(pico_id_list, 0)] = msg.frm.can_id;      //Add the id to the list of known ids
              num_luminaires++;                                          //Increment the number of luminaires
              Serial.printf("NEW NODE: %d\n", int(msg.frm.can_id));
            }                                    //Increment the number of luminaires
          } else if (strcmp(b, "S Cal") == 0) {  //Check if the message is a start calibration message
            Serial.printf("End of wait4start\n");
            return;
          }
        }
      }
    }

    //Check for incoming data in serial port
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

  Serial.println("Starting Calibration (I am Hub)...");

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

  vTaskDelay(3000);

  //Iterate over each one of the picos, turning one led on at a time and measuring gains
  for (int i = 0; i < num_luminaires; i++) {
    Serial.printf("Calibrating node %d\n", pico_id_list[i]);
    if (i == 0) {
      analogWrite(LED_PIN, int(DAC_RANGE));  //turn on my led
    }

    //Create message: "Calibrate <id>"
    char b[CAN_MSG_SIZE];
    strcpy(b, "Cal _");
    b[4] = pico_id_list[i];  // Check what is the id to turn on

    // Send message to warn other picos to measure gains (and the corresponding one to turn on its own led)
    msg_to_can_frame(&frm, hub_id, CAN_MSG_SIZE, CAN_ID_BROADCAST, b, sizeof(b));

    //Send the message through the xQueue01
    msg.frm = frm;
    msg.cmd = ICC_WRITE_DATA;
    if (xQueueSendToBack(xQueue_01, &msg, portMAX_DELAY) == pdTRUE) {
      Serial.printf("Sent 'Calibrate %d' message!\n", pico_id_list[i]);
    }

    //Measure gains
    vTaskDelay(5000);  //Wait some time for steady state (5s)

    Serial.println("Measuring gains...");
    luminaire_gains[i] = lum.lux_func(analog_low_pass_filter()) / 100;

    vTaskDelay(1000);  //Wait some time for sync (1s)

    //Turn off my led
    analogWrite(LED_PIN, int(0));  //turn off my led
  }

  //Tell other picos calibration is done
  char b2[CAN_MSG_SIZE] = "E Cal";
  msg_to_can_frame(&frm, pico_id_list[0], CAN_MSG_SIZE, CAN_ID_BROADCAST, b2, sizeof(b2));
  msg.frm = frm;
  msg.cmd = ICC_WRITE_DATA;
  if (xQueueSendToBack(xQueue_01, &msg, portMAX_DELAY) == pdTRUE) {
    Serial.println("Sent 'End Calibration' message!");
  }
}

void calibration_Other_Node() {  //Function to calibrate the system cross coupling gains (other nodes)
  Serial.println("Starting Calibration (I am NOT Hub)...");

  //Turn off my led
  analogWrite(LED_PIN, int(0));  //turn off my led

  //Wait for orders
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
          Serial.printf("Received: %s from id: %d\n", b, int(msg.frm.can_id));

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
            int pos = find(pico_id_list, node_id);
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

            vTaskDelay(3000);  //Wait some time for sync (1s)

            // Turn off my led
            analogWrite(LED_PIN, int(0));  //turn off my led
          }
        }
      }
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


//*************End of code***************
