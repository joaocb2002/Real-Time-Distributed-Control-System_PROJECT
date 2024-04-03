#ifndef LUMINAIRE_HH
#define LUMINAIRE_HH
#include "macros.hh"
#include "pico\unique_id.h"

class CLuminaire {

private:
  // Basic private member variables
  char type;  // Type of luminaire (A, B or C)
  double m;   // Slope of the voltage to LUX conversion function
  double b;   // Offset of the voltage to LUX conversion function

  // Specific private member variables
  bool occupied = false;  // True if the luminaire is occupied, false otherwise
  double G;               // Open loop gain of LED subsystem
  double H;               // Open loop gain of sensor subsystem (computed every iteration because depends on LUX value)
  double tau;             // Time constant of sensor subsystem (computed every iteration because depends on LUX value)

  // Metrics since last reset - these are updated every time a new sample is stored
  float total_energy_consumption = 0;  // Energy consumption since last reset (in Joules)
  float total_visibility_error = 0;    // Visibility error since last reset (in LUX)
  float total_flicker = 0;             // Flicker since last reset (in %)

  // Auxiliary variables for metrics computation
  float last_duty_cycle, last_last_duty_cycle;  // Last and last last duty cycle values
  int N = 0;                                    // Number of samples since last reset

  // Last minute buffer
  float last_minute_energy[6000] = { 0 };            // Energy consumption in the last minute (in Joules)
  float last_minute_visibility_error[6000] = { 0 };  // Visibility error in the last minute (in LUX)
  float last_minute_flicker[6000] = { 0 };           // Flicker in the last minute (in %)


public:
  float lower_bound_occ = 50;  // Lower bound of lux in occupied state
  float lower_bound_unocc = 0;  // Upper bound of lux in unoccupied state

  // Public member variables

  // Constructor
  CLuminaire() {}

  // Destructor
  ~CLuminaire() {}

  // Resetter
  void reset_lum() {
    occupied = false;
    G = 0;
    H = 0;
    tau = 0;
    total_energy_consumption = 0;
    total_visibility_error = 0;
    total_flicker = 0;
    last_duty_cycle = 0;
    last_last_duty_cycle = 0;
    N = 0;
    //lower_bound_occ = 0;
    //lower_bound_unocc = 0;
    for (int i = 0; i < 6000; i++) {
      last_minute_energy[i] = 0;
      last_minute_visibility_error[i] = 0;
      last_minute_flicker[i] = 0;
    }
  }

  // **** Initialize luminaire with right parameters (m, b, type) using its unique ID ****
  void init_lum(char* id) {

    // Set m and b according to the luminaire type
    if (strcmp(id, LUM_A_ID) == 0) {
      m = mA;
      b = bA;
      type = 'A';
    } else if (strcmp(id, LUM_B_ID) == 0) {
      m = mB;
      b = bB;
      type = 'B';
    } else if (strcmp(id, LUM_C_ID) == 0) {
      m = mC;
      b = bC;
      type = 'C';
    } else if (strcmp(id, LUM_A_ID_mike) == 0) {
      m = mA_mike;
      b = bA_mike;
      type = 'A';
    } else if (strcmp(id, LUM_B_ID_mike) == 0) {
      m = mB_mike;
      b = bB_mike;
      type = 'B';
    } else if (strcmp(id, LUM_C_ID_mike) == 0) {
      m = mC_mike;
      b = bC_mike;
      type = 'C';
    } else {  //Type A on default
      m = mA;
      b = bA;
      type = 'X';  //shows erros
    }
  }

  // **** Getters ****
  char get_type() {
    return (type);
  }  // Get luminaire type
  double get_m() {
    return (m);
  }  // Get slope of the voltage to LUX conversion function
  double get_b() {
    return (b);
  }  // Get offset of the voltage to LUX conversion function
  double get_G() {
    return (G);
  }  // Get open loop gain of LED subsystem
  double get_H() {
    return (H);
  }  // Get open loop gain of sensor subsystem
  double get_tau() {
    return (tau);
  }  // Get time constant of sensor subsystem
  bool get_occupancy() {
    return (occupied);
  }  // Get occupancy status
  float get_total_energy_consumption() {
    return (total_energy_consumption);
  }  //Get total energy consumption
  float get_total_visibility_error() {
    return (total_visibility_error / N);
  }  //Get total visibility error
  float get_total_flicker() {
    return (total_flicker / N);
  }  //Get total flicker

  // **** Setters ****
  void set_occupancy(bool occ) {
    occupied = occ;
  }  // Set occupancy status
  void set_G(double G_val) {
    G = G_val;
  }  // Set open loop gain of LED subsystem

  //**** LUX-voltage conversion functions ****
  double lux_func(int adc) {  // Voltage (from 0-4095) to LUX conversion
    double Vi = (double)adc * 3.3 / 4096;
    double res = 33000 / Vi - 10000;
    double LUX = pow(10, (log10(res) - b) / m);
    return (LUX);
  }

  double voltage_func(double LUX) {  // Voltage (from 0-4095) to LUX conversion
    double val = pow(10, (log10(LUX) * m + b));
    double Vi = 33000 / (10000 + val);
    Vi = Vi * 4096 / 3.3;
    return (Vi);
  }

  //**** System parameters ****

  void compute_open_loop_gain_H(double LUX) {  // Sensor subsystem gain H computation

    // Compute gain H, based on the reference LUX value
    double val = pow(10, (log10(LUX) * m + b));
    double gain = 33000 / (LUX * (10000 + val));

    // Update gain H
    H = gain;
  }

  void compute_tau_time_const(double LUX) {  // Sensor subsystem time constant tau computation

    // Compute time constant tau, based on the reference LUX value
    double val = pow(10, (log10(LUX) * m + b));
    val = val / (10 * (pow(10, 4) + val));
    val += 0.070;  // Add 70ms to adjust (based on experimental data)

    // Update constant tau
    tau = val;
  }


  //**** Metrics computation ****
  void update_luminaire_metrics(float duty_cycle, float lux, float reference_lux) {  // Store last sample data and update metrics

    // Compute flicker for that sample
    float flicker = 0;
    if ((duty_cycle - last_duty_cycle) * (last_duty_cycle - last_last_duty_cycle) < 0) {
      flicker = abs(duty_cycle - last_duty_cycle) + abs(last_duty_cycle - last_last_duty_cycle);
    }

    // Update last duty cycle values
    last_last_duty_cycle = last_duty_cycle;
    last_duty_cycle = duty_cycle;

    // Compute visibility error for that sample
    float visibility_error = max(0, reference_lux - lux);

    // Compute energy consumption for that sample
    float energy_consumption = PMAX * duty_cycle / 100 * TIME_SAMPLING;  // in Joules

    // Update metrics since last reset
    total_energy_consumption += energy_consumption;
    total_visibility_error += visibility_error;
    total_flicker += flicker;

    // Update last minute buffer
    last_minute_energy[N % 6000] = energy_consumption;          // in Joules
    last_minute_visibility_error[N % 6000] = visibility_error;  // in LUX
    last_minute_flicker[N % 6000] = flicker;                    // in s^-1

    // Update sample number
    N++;
  }

  void reset_metrics() {  // Reset metrics
    total_energy_consumption = 0;
    total_visibility_error = 0;
    total_flicker = 0;
    N = 0;
  }
};

#endif  // LUMINAIRE_HH
