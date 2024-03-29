#ifndef UTILS_H
#define UTILS_H


/////////////////////////////////////////////////////
// Helper function to manage and interact with other picos
/////////////////////////////////////////////////////

// Function that returns index of element in array, else returns the index of available slot
uint8_t find(uint8_t arr[], uint8_t id) {
  int availableSlot = -1;
  int size = sizeof(arr) / sizeof(arr[0]);
  for (int i = 0; i < size; ++i) {
    if (arr[i] == id) {
      return i;  // If id is found, return its index
    } else if (arr[i] == 0 && availableSlot == -1) {
      availableSlot = i;  // Update available slot index if found
    }
  }
  return availableSlot;  // Return available slot index if id not found
}


/////////////////////////////////////////////////////
// CAN FRAME: HELPER FUNCTIONS AND DEFINITIONS
/////////////////////////////////////////////////////

// Reminder of the definition of a CAN frame
/*typedef struct can_frame {
  uint32_t can_id;  // Will be the sending node ID
  uint8_t can_dlc;  // Data length code (number of bytes: 8)
  uint8_t data[CAN_MSG_SIZE];  // Byte 0: Destination node ID, Byte 1-7: Char data
} can_frame; */

// Function to initialize a CAN frame message with a given id and number of bytes
void init_can_frame(can_frame *frm, uint8_t id, uint8_t num) {
  frm->can_id = id;
  frm->can_dlc = num;

  for (int i = 0; i < num; i++) {
    frm->data[i] = '_';
  }
}

// Function to print the contents of a CAN frame
void print_can_frame_msg(can_frame *frm, uint8_t num) {
  Serial.print("\nID: ");
  Serial.println(frm->can_id, HEX);
  Serial.print(" DLC: ");
  Serial.println(frm->can_dlc, HEX);
  Serial.print(" Data: ");
  for (int i = 0; i < num; i++) {
    Serial.print(frm->data[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

// Function to convert a message into a CAN frame
void msg_to_can_frame(can_frame *frm, uint8_t src_id, uint8_t num, uint8_t dest_id, char *msg, int len) {
  // Set can_id to the source node ID and can_dlc to the number of bytes
  frm->can_id = id;
  frm->can_dlc = num;

  // Set the first byte of data to the destination node ID
  frm->data[0] = dest_id;

  // Copy the message into the data array, filling the rest with '_'
  for (int i = 0; i < num; i++) {
    if (i < len) {
      frm->data[i] = msg[i];
    } else {
      frm->data[i] = '_';
    }
  }
}

// Function to convert a CAN frame into a message
void can_frame_to_msg(can_frame *frm, uint8_t *src_id, uint8_t *num, uint8_t *dest_id, char *msg, int len) {
  // Set the source node ID, number of bytes, and destination node ID
  *src_id = frm->can_id;
  *num = frm->can_dlc;
  *dest_id = frm->data[0];

  // Copy the message from the data array (if array has space for null terminator)
  for (int i = 0; i < *num; i++) {
    if (i < len) {
      msg[i] = frm->data[i];
    }
  }
  msg[*num] = '\0';  // Add null terminator to the message
}


/////////////////////////////////////////////////////
// INTER-CORE COMMUNICATION: HELPER FUNCTIONS AND DEFINITIONS
/////////////////////////////////////////////////////

// Command IDs for intercore communication
typedef enum inter_core_cmds {
    ICC_READ_DATA = 1,    // From core1 to core0: contains data read (16 bit)
    ICC_WRITE_DATA = 2,   // From core0 to core1: contains data to write (16 bit)
    ICC_ERROR_DATA = 3    // From core1 to core0: contains regs CANINTF, EFLG
} inter_core_cmds;

// Structure for inter-core communication: CAN frame message and inter-core command
typedef struct icc_msg {
    can_frame frm;         // CAN frame message
    inter_core_cmds cmd;  // Inter-core command
} icc_msg;


/////////////////////////////////////////////////////
//Helper function to print error flags (delete this stuff?)
/////////////////////////////////////////////////////

// Strings to print the error flags
char canintf_str[]{ "| MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | " }; 
char eflg_str[]{ "| RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | " };

// Function to print the error flags
void print_can_errors(uint8_t canintf, uint8_t eflg) {
  Serial.println("-----------------------------------------------------------------");
  Serial.println(canintf_str);
  Serial.print("| ");
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print(" ");
    Serial.write(bitRead(canintf, bit) ? '1' : '0');
    Serial.print(" | ");
  }
  Serial.println("");
  Serial.println("-----------------------------------------------------------------");
  Serial.println(eflg_str);
  Serial.print("| ");
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print(" ");
    Serial.write(bitRead(eflg, bit) ? '1' : '0');
    Serial.print(" | ");
  }
  Serial.println("");
  Serial.println("-----------------------------------------------------------------");
}

#endif