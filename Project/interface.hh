#ifndef INTERFACE_HH
#define INTERFACE_HH

#include "macros.hh"
#include "luminaire.hh"
#include "PID.hh"

// Function to handle serial communication and user interface
void handle_serial(CLuminaire &lum, CPID &pid, float &x_ref, float &r, float &y, float &u, time_t inicial_time) {

  // Check if there is data available to read
  if (!Serial.available()) {
    return;
  }

  // Initialize variables
  char i;
  int occupancy, anti_windup, feedback;
	float duty_cycle, lux;

  // Read the command
	String command = Serial.readStringUntil('\n');

  // Process the command
	if (command.startsWith("d")) { // Set duty cycle directly
		sscanf(command.c_str(), "d %c %f", &i, &duty_cycle);
		if (i == lum.get_type()) {
      x_ref = lum.G * duty_cycle;
		}
		Serial.println("ack"); 
	} else if (command.startsWith("g d")) { // Get duty cycle
		sscanf(command.c_str(), "g d %c", &i);
		if (i == lum.get_type()) {
			Serial.print("d ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(u*100/4095);
		}
	} else if (command.startsWith("r")) { // Set reference value
		sscanf(command.c_str(), "r %c %f", &i, &lux);
		if (i == lum.get_type()) {
      x_ref = lux;
    }
		Serial.println("ack");
	} else if (command.startsWith("g r")) { // Get reference value
		sscanf(command.c_str(), "g r %c", &i);
		if (i == lum.get_type()) {
      Serial.print("r ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(x_ref);
    }
	} else if (command.startsWith("g l")) { // Measure luminance
		sscanf(command.c_str(), "g l %c", &i);
		if (i == lum.get_type()) {
      Serial.print("l ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(lum.lux_func(y));
    }
    
	} else if (command.startsWith("o")) { // Set occupancy status
		sscanf(command.c_str(), "o %c %d", &i, &occupancy);
		if (i == lum.get_type()) {
      lum.update_occupancy(occupancy);
    }
		Serial.println("ack");
	} else if (command.startsWith("g o")) { // Get occupancy status
		sscanf(command.c_str(), "g o %c", &i);
		if (i == lum.get_type()) {
      Serial.print("o ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(lum.get_occupancy());
    }
	} else if (command.startsWith("a")) { // Set anti-windup flag
		sscanf(command.c_str(), "a %c %d", &i, &anti_windup);
		if (i == lum.get_type()) {
      pid.set_anti_windup(anti_windup);
    }
		Serial.println("ack");
	} else if (command.startsWith("g a")) { // Get anti-windup flag
		sscanf(command.c_str(), "g a %c", &i);
		if (i == lum.get_type()) {
      Serial.print("a ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(pid.get_anti_windup());
    }
	} else if (command.startsWith("k")) { // Set feedback flag
		sscanf(command.c_str(), "k %c %d", &i, &feedback);
		if (i == lum.get_type()) {
      pid.set_feedback(feedback);
    }
		Serial.println("ack");
	} else if (command.startsWith("g k")) { // Get feedback flag
		sscanf(command.c_str(), "g k %c", &i);
		if (i == lum.get_type()) {
      Serial.print("k ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(pid.get_feedback());
    }
	} else if (command.startsWith("g x")) { // Get external illuminance
		sscanf(command.c_str(), "g x %c", &i);
		if (i == lum.get_type()) {
      Serial.print("x ");
      Serial.print(i);
      Serial.print(" ");
      //Serial.println(lum.get_external_illuminance());
    }
	} else if (command.startsWith("g p")) { // Get instantaneous power
		sscanf(command.c_str(), "g p %c", &i);
		if (i == lum.get_type()) {
      Serial.print("p ");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(1000*u*1/4095*PMAX);
      Serial.print("mW");
    }
	} else if (command.startsWith("g t")) { //Get time since last restart
		sscanf(command.c_str(), "g t %c", &i);
		if (i == lum.get_type()) {
      Serial.print("t ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println((millis() - inicial_time) / 1000);
    }
	} else if (command.startsWith("g e")) { // Get energy
		sscanf(command.c_str(), "g e %c", &i);
		if (i == lum.get_type()) {
      Serial.print("e ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(lum.get_total_energy_consumption());
    }
	} else if (command.startsWith("g v")) { // Get visibility error
		sscanf(command.c_str(), "g v %c", &i);
		if (i == lum.get_type()) {
      Serial.print("v ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(lum.get_total_visibility_error());
    }
	} else if (command.startsWith("g f")) { // Get flicker
		sscanf(command.c_str(), "g f %c", &i);
		if (i == lum.get_type()) {
      Serial.print("f ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(lum.get_total_flicker());
	  }
  }
}

#endif // INTERFACE_HH