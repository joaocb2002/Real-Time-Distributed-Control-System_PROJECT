#ifndef PID_H
#define PID_H


// ************************************************************************************
// Classes definition
class CPID {
    private:

    bool anti_windup, feedback, feedforward; // Flags

    float I, D, P; // Controller variables
    float K, Ti, Td, Tt, b; // Controller parameters
    float bi, ad, bd, ao; // Controller coefficients
    float h; // Sampling time
    float y_old, k_old, b_old, N; // Auxiliary variables

    float u, v; // Output variables 

    const double ulow = 0, uhigh = 4095; // Control signal limits

    public:
    explicit CPID(float _h, float _K = 1, float b_ = 1,float Ti_ = 1, float Td_ = 0, float Tt_ = 1, float N_ = 10);
    ~CPID() {};

    void set_anti_windup(bool _anti_windup);
    bool get_anti_windup();
    void set_feedback(bool _feedback);
    bool get_feedback();
    void set_feedforward(bool _feedforward);
    bool get_feedforward();
    void update_parameters(double G, double H, double tau);
    void compute_coefficients();
    float compute_control(float r, float y);
    double saturate(double v, double ulow, double uhigh);
    void housekeeping(float r, float y);

};




//********************************************************************************
// METHOD DEFINITIONS

// Constructor
inline CPID::CPID(float _h, float _K, float b_, float Ti_, float Td_, float Tt_, float N_) {
    h = _h; K = _K; b = b_; Ti = Ti_; Td = Td_; Tt = Tt_; N = N_; P = 0; I = 0; D = 0; y_old = 0; k_old = 0; b_old = 0, ad = 0; bd = 0; ao = 0; anti_windup = true; feedback = true; feedforward = true;
}

// Member function to update controller parameters depending on set point LUX value
void CPID::update_parameters(double G, double H, double tau) {
    //Compute controller parameters
    Ti = tau; //Integral time constant
    K = Ti/(G*H*20); //Proportional gain - get low one to minimize sensitivity
    b = (1/(G*H*K*20)); //Set point weighting
}


void CPID::set_anti_windup(bool _anti_windup) {anti_windup = _anti_windup;} // Member function to set anti-windup flag
bool CPID::get_anti_windup() {return anti_windup;} // Member function to get anti-windup flag
void CPID::set_feedback(bool _feedback) {feedback = _feedback;} // Member function to set feedback flag
bool CPID::get_feedback() {return feedback;} // Member function to get feedback flag
void CPID::set_feedforward(bool _feedforward) {feedforward = _feedforward;} // Member function to set feedforward flag
bool CPID::get_feedforward() {return feedforward;} // Member function to get feedforward flag


// Member function to compute controller coefficients
void CPID::compute_coefficients() {
    bi = K*h/Ti; //Integral gain
    ad = Td/(Td+N*h); 
    bd = Td*K*N/(Td+N*h); //Derivative gains
    ao = anti_windup ? h / Tt : 0; // Back calculation gain (anti-windup)
}

// Member function to compute control signal
float CPID::compute_control(float r, float y) {

    //Compute control signal
    if (feedback && feedforward) { //Feedback and feedforward control
        I = I + k_old *(b_old * r-y) - K * (b * r - y); //Integral term with bumpless transfer
        P = K*(b*r-y); //Proportional term
        D = ad*D-bd*(y-y_old); //Derivative term
        v = P+I+D; //Control signal without saturation
        u = saturate(v, ulow, uhigh); 
    } else if (feedback && !feedforward) { //Feedback control
        I = I + bi*(r-y); //Integral term
        P = K*(r-y); //Proportional term
        D = ad*D-bd*(y-y_old); //Derivative term
        v = P+I+D; //Control signal without saturation
        u = saturate(v, ulow, uhigh); 
    } else if (!feedback && feedforward) { //Feedforward control
        v = K*(b*r-y); //Control signal without saturation
        u = saturate(v, ulow, uhigh); 
    } else { //No control
        u = 0;
    }

    //Return control signal
    return u;
}

// Member function to saturate a value
double CPID::saturate(double v, double ulow, double uhigh) {
    if (v < ulow) {
        return ulow;
    } else if (v > uhigh) {
        return uhigh;
    } else {
        return v;
    }
}

// Member function to 'housekeeping' tasks
void CPID::housekeeping(float r, float y){
    //Housekeeping
    y_old = y;
    k_old = K;
    b_old = b;
    if (feedback) {I += bi*(r-y) + ao*(u-v); }
}


#endif //PID_H
