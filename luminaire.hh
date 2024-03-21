#ifndef LUMINAIRE_HH
#define LUMINAIRE_HH
#include "macros.hh"
#include "pico\unique_id.h"

class CLuminaire {

    private:
        // Private member variables
        char type; // Type of luminaire (A, B or C)

        // Slope and offset of the voltage to LUX conversion function
        double m; // Slope
        double b; // Offset

        // Metrics since last reset
        float total_energy_consumption = 0; // Energy consumption since last reset (in Joules)
        float last_duty_cycle, last_last_duty_cycle; // Last and last last duty cycle values
        int N = 0; // Number of samples since last reset

        // Occupancy status
        bool occupied = false; // True if the luminaire is occupied, false otherwise

    
    public:
        float total_visibility_error = 0; // Visibility error since last reset (in LUX)
        float total_flicker = 0; // Flicker since last reset (in %)

        //Get unique board ID
        int len = 17;
        char id[17];

        // Public member variables
        double G; // Open loop gain of LED subsystem
        double H; // Open loop gain of sensor subsystem (computed every iteration because depends on LUX value)
        double tau; // Time constant of sensor subsystem (computed every iteration because depends on LUX value)

        // Constructor
        CLuminaire(){}

        // Destructor
        ~CLuminaire(){}

        // Initialize luminaire
        void init_lum(char* id, int len){

            // Set m and b according to the luminaire type
            if (strcmp(id, LUM_A_ID) == 0){
                m = mA;
                b = bA;
                G = GA;
                type = 'A';
            }
            else if (strcmp(id, LUM_B_ID) == 0){
                m = mB;
                b = bB;
                G = GB;
                type = 'B';
            }
            else if (strcmp(id, LUM_C_ID) == 0){
                m = mC;
                b = bC;
                G = GC;
                type = 'C';
            }
            else{ //Type A on default
                m = mA;
                b = bA;
                G = GA;
                type = 'X'; //shows erros
            }
        }

        // Get luminaire type
        char get_type(){
            return(type);
        }

        // Voltage (from 0-4095) to LUX conversion
        double lux_func(int adc){
            double Vi = (double)adc * 3.3 / 4096;
            double res = 33000 / Vi - 10000;
            double LUX = pow(10, (log10(res) - b) / m);
            return(LUX); 
        }

        // LUX to voltage (from 0-4095) conversion
        double voltage_func(double LUX){
            double val = pow(10, (log10(LUX) * m + b));
            double Vi = 33000 / (10000+val);
            Vi = Vi * 4096 / 3.3;
            return(Vi);
        }

        // Open loop gain G computation, pass true to calibrate
        double calibrate_open_loop_gain_G(){

            // Variables 
            int adc;
            double LUX0, LUX1;

            // Set LED PWM to 20% duty cycle, delay to stabilize, read ADC and compute LUX
            analogWrite(LED_PIN, 819); // 20% of 4095
            delay(5000); // 5s
            adc = analogRead(A1);
            LUX0 = lux_func(adc);

            // Set LED PWM to 100% duty cycle, delay to stabilize, read ADC and compute LUX
            analogWrite(LED_PIN, 4095); // 100% of 4095
            delay(5000);
            adc = analogRead(A1);
            LUX1 = lux_func(adc);

            // Compute slope (gain)
            double gain = (LUX1 - LUX0) / (100 - 20);
            return(gain);
        }

        // Sensor subsystem gain H computation
        double compute_open_loop_gain_H(double LUX){

            // Compute gain H, based on the reference LUX value
            double val = pow(10, (log10(LUX) * m + b));
            double gain = 33000 / (LUX * (10000+val));

            // Return gain H
            return(gain);
        }
z
        // Sensor subsystem time constant tau computation
        double compute_compute_tau_time_const(double LUX){

            // Compute time constant tau, based on the reference LUX value
            double val = pow(10, (log10(LUX) * m + b));
            double tau = val / (10*(pow(10, 4)+val));
            tau += 0.070; // Add 70ms to adjust (based on experimental data)

            // Return time constant tau
            return(tau);
        }

        // Get occupancy status
        bool get_occupancy(){
            return(occupied);
        }

        // Occupancy status update
        void update_occupancy(bool occ){
            occupied = occ;
        }

        // Store last sample data and update metrics
        void store_luminaire_data(float duty_cycle, float lux, float rlux){

            // Compute flicker for that sample
            float flicker = 0;
            if ((duty_cycle - last_duty_cycle)*(last_duty_cycle - last_last_duty_cycle) < 0){
                flicker = abs(duty_cycle - last_duty_cycle) + abs(last_duty_cycle - last_last_duty_cycle);
            }

            // Update last duty cycle values
            last_last_duty_cycle = last_duty_cycle;
            last_duty_cycle = duty_cycle;

            // Update metrics
            total_energy_consumption += PMAX*duty_cycle/100*TIME_SAMPLING; // in Joules
            total_visibility_error += max(0, rlux - lux); // in LUX
            total_flicker += flicker; // in s^-1

            // Update sample number
            N++;
        }

        //Get total energy consumption
        float get_total_energy_consumption(){
            return(total_energy_consumption);
        }

        //Get total visibility error
        float get_total_visibility_error(){
            return(total_visibility_error/N);
        }

        //Get total flicker
        float get_total_flicker(){
            return(total_flicker/N);
        }
        

};

#endif // LUMINAIRE_HH

