#ifndef UTILS_H
#define UTILS_H
#include "can.h"
#include "macros.hh"

/////////////////////////////////////////////////////
// Helper function to manage and interact with other picos
/////////////////////////////////////////////////////

// Function that returns index of element in array, else returns the index of available slot
uint8_t find_id(uint8_t arr[], uint8_t id) {
  int availableSlot = -1;
  int size = MAX_LUMINAIRES;
  for (int i = 0; i < size; ++i) {
    if (arr[i] == id) {
      return i;  // If id is found, return its index
    } 
    else if (arr[i] == 0 && availableSlot == -1) {
      availableSlot = i;  // Update available slot index if found
    }
  }

  if(availableSlot == -1){
    Serial.println("Wrong ID!");
  }

  return availableSlot;  // Return available slot index if id not found
}

uint8_t find_type(char arr[], char type) {
  int availableSlot = -1;
  int size = MAX_LUMINAIRES;
  for (int i = 0; i < size; ++i) {
    if (arr[i] == type) {
      return i;  // If id is found, return its index
    } else if (arr[i] == 'X' && availableSlot == -1) {
      availableSlot = i;  // Update available slot index if found
    }
  }
  return availableSlot;  // Return available slot index if id not found
}




/////////////////////////////////////////////////////
// Functions to print stuff
/////////////////////////////////////////////////////

// Function to print the IDs of the luminaires
void print_luminaires_ids(uint8_t arr[], int size) {
  Serial.print("Luminaires = [ ");
  for (int i = 0; i < size; ++i) {
    if (arr[i] != 0) {
      Serial.print(arr[i]);
      Serial.print(" ");
    }
  }
  Serial.println("]");
}

//Function to print the crossover gains
void print_crossover_gains(float gains[], float o, int size) {
  Serial.print("Crossover Gains = [ ");
  for (int i = 0; i < size; ++i) {
    Serial.print(gains[i]);
    Serial.print(" ");
  }
  Serial.print("] o(t) = ");
  Serial.println(o);
}

//Function to print int array
void print_array(int arr[], int size) {
  Serial.print("Array = [ ");
  for (int i = 0; i < size; ++i) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println("] ");
}

//Function to print char array
void print_char_array(char arr[], int size) {
  Serial.print("Array = [ ");
  for (int i = 0; i < size; ++i) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println("] ");
}

//Function to print uint8_t array
void print_uint8_t_array(uint8_t arr[], int size) {
  Serial.print("Array = [ ");
  for (int i = 0; i < size; ++i) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println("] ");
}

//Function to print float array
void print_float_array(float arr[], int size) {
  Serial.print("Array = [ ");
  for (int i = 0; i < size; ++i) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println("] ");
}

// Function to print the types of the luminaires
void print_luminaires_types(char arr[], int size) {
  Serial.print("Luminaires types = [ ");
  for (int i = 0; i < size; ++i) {
    if (arr[i] != 'X') {
      Serial.print(arr[i]);
      Serial.print(" ");
    }
  }
  Serial.println("]");
}


/////////////////////////////////////////////////////
// Functions for vector operations
/////////////////////////////////////////////////////

//Function to compute the dot product between two vectors.
float dot(float vec1[], float vec2[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += vec1[i] * vec2[i];
  }
  return sum;
};

// Function to subtract corresponding elements of two vectors (vec1 - vec2)
void subtractArrays(float* vec1, float* vec2, int size, float* result) {
  //vec1 - vec2
  for (int i = 0; i < size; ++i) {
    result[i] = vec1[i] - vec2[i];
  }
}

// Function to sum corresponding elements of two vectors (vec1 + vec2)
void sumArrays(float* vec1, float* vec2, int size, float* result) {
  //vec1 + vec2
  for (int i = 0; i < size; ++i) {
    result[i] = vec1[i] + vec2[i];
  }
}

// Function to compute the product of a vector and a scalar
void scalarProduct(float* vec, int size, float scalar, float* result) {
  for (int i = 0; i < size; i++) {
    result[i] = scalar * vec[i];
  }
}


/////////////////////////////////////////////////////
// Functions for array operations
/////////////////////////////////////////////////////

// Function to copy the contents of one vector to another (float)
void copyFloatArray(const float* src, int size, float* dest) {
  for (int i = 0; i < size; i++) {
    dest[i] = src[i];
  }
}

// Function to copy the contents of one vector to another (uint8_t)
void copyUint8Array(const uint8_t* src, int size, uint8_t* dest) {
  for (int i = 0; i < size; i++) {
    dest[i] = src[i];
  }
}

// Function to copy the contents of one vector to another (char)
void copyCharArray(const char* src, int size, char* dest) {
  for (int i = 0; i < size; i++) {
    dest[i] = src[i];
  }
}

// Function to copy the contents of one vector to another (int)
void copyIntArray(const int* src, int size, int* dest) {
  for (int i = 0; i < size; i++) {
    dest[i] = src[i];
  }
}

// Function to print the contents of a float array
void printArray(float arr[], int size) {
  Serial.print("[");
  for (int i = 0; i < size; i++) {
    Serial.print(arr[i], 4);
    if (i < size - 1) {
      Serial.print(", ");
    }
  }
  Serial.println("]");
}




/////////////////////////////////////////////////////
// Other useful functions
/////////////////////////////////////////////////////
int findNthSmallestIndex(uint8_t arr[], int size, int n) {
  if (n < 0 || n > size) {
    Serial.printf("Invalid value of n\n");
    return -1;
  }

  // Create a copy of the array to preserve the original order
  int* sortedArr = new int[size];
  for (int i = 0; i < size; ++i) {
    sortedArr[i] = arr[i];
  }

  // Sort the copy of the array (using bubble sort)
  for (int i = 0; i < size - 1; ++i) {
    for (int j = 0; j < size - i - 1; ++j) {
      if (sortedArr[j] > sortedArr[j + 1]) {
        int temp = sortedArr[j];
        sortedArr[j] = sortedArr[j + 1];
        sortedArr[j + 1] = temp;
      }
    }
  }

  // Find the index of the nth smallest element in the original array
  int index = -1;
  for (int i = 0; i < size; ++i) {
    if (arr[i] == sortedArr[n]) {
      index = i;
      break;
    }
  }

  delete[] sortedArr;
  return index;
}

bool all_true_array(bool arr[], int size) {
  for (int i = 0; i < size; i++) {
    if (!arr[i]) {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////////
// CAN FRAME: HELPER FUNCTIONS AND DEFINITIONS
/////////////////////////////////////////////////////

// Function to initialize a CAN frame message with a given id and number of bytes
void init_can_frame(can_frame* frm, uint8_t id, uint8_t num) {
  frm->can_id = id;
  frm->can_dlc = num;

  for (int i = 0; i < num; i++) {
    frm->data[i] = '_';
  }
}

// Function to print the contents of a CAN frame
void print_can_frame_msg(can_frame* frm, uint8_t num) {
  Serial.print("ID: ");
  Serial.print(frm->can_id, HEX);
  Serial.print(" DLC: ");
  Serial.print(frm->can_dlc, HEX);
  Serial.print(" Dest: ");
  Serial.print(frm->data[0], HEX);
  Serial.print(" Data: ");
  for (int i = 1; i < num; i++) {
    Serial.print(char(frm->data[i]));
    Serial.print(" ");
  }
  Serial.println("");
}

// Function to convert a message into a CAN frame
void msg_to_can_frame(can_frame* frm, uint8_t src_id, uint8_t num, uint8_t dest_id, char* msg, int len) {
  // Set can_id to the source node ID and can_dlc to the number of bytes
  frm->can_id = src_id;
  frm->can_dlc = num;

  // Set the first byte of data to the destination node ID
  frm->data[0] = dest_id;

  // Copy the message into the data array, filling the rest with '_'
  for (int i = 1; i < num; i++) {
    if (msg[i - 1] != '\0' && i - 1 < len) {
      frm->data[i] = msg[i - 1];
    } else {
      frm->data[i] = '_';
    }
  }
}

// Function to convert a CAN frame into a message
void can_frame_to_msg(can_frame* frm, uint8_t* src_id, uint8_t* num, uint8_t* dest_id, char* msg) {
  // Set the source node ID, number of bytes, and destination node ID
  *src_id = frm->can_id;
  *num = frm->can_dlc;
  *dest_id = frm->data[0];

  // Copy the message from the data array (if array has space for null terminator)
  int i;
  for (i = 1; i < *num; i++) {
    if (frm->data[i] != '_') {
      msg[i - 1] = frm->data[i];
    } else {
      break;
    }
  }

  // Add null terminator to the message
  msg[i - 1] = '\0';
}





/////////////////////////////////////////////////////
// INTER-CORE COMMUNICATION: HELPER FUNCTIONS AND DEFINITIONS
/////////////////////////////////////////////////////

// Command IDs for intercore communication
typedef enum inter_core_cmds {
  ICC_READ_DATA = 1,   // From core1 to core0: contains data read (16 bit)
  ICC_WRITE_DATA = 2,  // From core0 to core1: contains data to write (16 bit)
  ICC_ERROR_DATA = 3   // From core1 to core0: contains regs CANINTF, EFLG
} inter_core_cmds;

// Structure for inter-core communication: CAN frame message and inter-core command
typedef struct icc_msg {
  can_frame frm;        // CAN frame message
  inter_core_cmds cmd;  // Inter-core command
} icc_msg;

/* CAN Frame struct 
typedef struct can_frame {
  uint32_t can_id;  // 32 bit CAN_ID + EFF/RTR/ERR flags 
  uint8_t can_dlc;  // frame payload length in byte (0 .. CAN_MAX_DLEN) 
  uint8_t data[8] __attribute__((aligned(8))); byte 0: destination node ID, bytes 1-7: message
} can_frame; */

/////////////////////////////////////////////////////
// Helper functions to pack and unpack bytes to 2bytes
/////////////////////////////////////////////////////

//Packs 2 bytes into an unsigned int
uint16_t bytes_to_msg(uint8_t ls_byte, uint8_t ms_byte) {
uint16_t b0 {ls_byte}, b1 {ms_byte};
return b0 + (b1 << 8);
}

//Unpacks an unsigned int into its constituting 2 bytes
void msg_to_bytes(uint16_t num, uint8_t *byte_ls, uint8_t *byte_ms) {
  *byte_ls = num; *byte_ms = (num >> 8);
} 
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