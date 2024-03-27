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

//Command IDs for intercore communication
enum inter_core_cmds {
  //From core1 to core0: contains data read (16 bit)
  ICC_READ_DATA = 1,
  // From core0 to core1: contains data to write (16 bit)
  ICC_WRITE_DATA = 2,
  // From core1 to core0: contains regs CANINTF, EFLG
  ICC_ERROR_DATA = 3
};

/////////////////////////////////////////////////////
//Inter-core message definitions and helper functions
/////////////////////////////////////////////////////

//Packs 4 bytes into an unsigned int
uint32_t bytes_to_msg(uint8_t *b) {
  uint32_t b0{ b[0] }, b1{ b[1] }, b2{ b[2] }, b3{ b[3] };
  return b0 + (b1 << 8) + (b2 << 16) + (b3 << 24);
}

//Unpacks an unsigned int into its constituting 4 bytes
void msg_to_bytes(uint32_t msg, uint8_t *bytes) {
  bytes[0] = msg;
  bytes[1] = (msg >> 8);
  bytes[2] = (msg >> 16);
  bytes[3] = (msg >> 24);
}

//Packs the CAN frame contents into an unsigned int
uint32_t can_frame_to_msg(can_frame *frm) {
  uint8_t b[4];
  b[3] = ICC_READ_DATA;
  b[2] = frm->can_id;
  b[1] = frm->data[1];
  b[0] = frm->data[0];
  return bytes_to_msg(b);
}

//Packs the CAN error flags into an unsigned int
uint32_t error_flags_to_msg(uint8_t canintf, uint8_t eflg) {
  uint8_t b[4];
  b[3] = ICC_ERROR_DATA;
  b[2] = 0;
  b[1] = canintf;
  b[0] = eflg;
  return bytes_to_msg(b);
}

//Prints the contents of a CAN message
//I updated this function for our use case
void print_message(int first_char, int second_char, int node, int id) {
  Serial.print(char(first_char));
  Serial.print(" ");
  if (char(first_char) == 'd' || char(first_char) == 'R' ){
    Serial.print(second_char);
  }
  else{
    //The case where the second char is a char
    Serial.print(char(second_char));
  }
  Serial.print(" at node ");
  Serial.print(node, HEX);
  Serial.print(" with id ");
  Serial.print(id, HEX);
  Serial.println("");
}

/////////////////////////////////////////////////////
//Helper function to print error flags
/////////////////////////////////////////////////////

char canintf_str[]{ "| MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | " };
char eflg_str[]{ "| RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | " };
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