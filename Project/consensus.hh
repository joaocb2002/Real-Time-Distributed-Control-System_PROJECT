#include "macros.hh"
#include "utils.hh"

#ifndef CONSENSUS_H
#define CONSENSUS_H

//Defined Constraints
#define DUB 100    //Duty cycle upper bound
#define DLB 0      //Duty cycle lower bound
#define TOL 0.001  //tolerance for rounding errors

struct consensus_out {
  float d_best[MAX_LUMINAIRES] {0};  //Float to hold the best duty cycle
  float cost_best = 100000;
};

class CONSENSUS {
private:


  float d_now[MAX_LUMINAIRES] = {0};  //Current duty cycle iterate values
  float d_avg[MAX_LUMINAIRES] = {0};  //The average duty cycle
  float K[MAX_LUMINAIRES];           //Cross coupling gains
  float o;                           //external illuminance
  float l;  //Lower bound for illuminance
  float c[MAX_LUMINAIRES] = {0};      //The cost value (maximum energy: PMAX)

  // Number of nodes in the network
  int node_num = 0;                

  //Linear Boundary variables
  float n;  
  float m;  

  //Lagrangian multipliers
  float lambda[MAX_LUMINAIRES] = {0};
  float rho = 0.2;


public:
  // Current output of the consensus algorithm
  consensus_out result;

  // Constructor
  explicit CONSENSUS(float K_[], float o_, int _node_num, float cost_coeff) {
    //Cross-coupling gains vector
    copyFloatArray(K_, MAX_LUMINAIRES,K);

    //External illuminance
    o = o_;     

    //Cost vector (by default: maximum energy for my duty cycle and 0 for the others)
    c[0] = PMAX*cost_coeff;  

    //Number of nodes
    node_num = _node_num; 

    // Compute linear boundary variables
    n = dot(K, K, MAX_LUMINAIRES);
    m = n - K[0] * K[0];
  };

  // Destructor
  ~CONSENSUS(){};

  //Methods
  bool check_feasibility();
  float evaluate_cost();
  void consensus_iterate();
  float compute_lux();
  void update_gains(float K_[], float o_);
  void update_average(float d_new_avg[]);
  void update_lagrangian();
  void update_duty(float d_[]);
  void print_consensus();
  void print_lagrangian();
  void update_lower_bound(float x_ref) { l = x_ref; };
  void update_num_luminaires(int num) { node_num = num; };
  float get_duty_avr(int i) { return d_avg[i]; };
};

bool CONSENSUS::check_feasibility() {
  if (d_now[0] < DLB - TOL) {
    return false;
  }
  if (d_now[0] > DUB + TOL) {
    return false;
  }
  if (compute_lux() < l) {
    return false;
  }
  else{
    return true;
  }
};

//Computes the illuminance reference given the cross coupling gains and the duty cycles
float CONSENSUS::compute_lux() {
  return dot(d_now, K, MAX_LUMINAIRES) + o;
};

//Computes the Augmented Lagrangian
float CONSENSUS::evaluate_cost() {
  //d is typically d_now[0]
  float cost;
  float diff[MAX_LUMINAIRES] = {};
  subtractArrays(d_now, d_avg, MAX_LUMINAIRES, diff);  //d-d_av
  cost = dot(c, d_now, MAX_LUMINAIRES) + dot(lambda, diff, MAX_LUMINAIRES) + rho / 2 * dot(diff, diff, MAX_LUMINAIRES);
  return cost;
};

void CONSENSUS::update_gains(float K_[], float o_) {
  copyFloatArray(K_, MAX_LUMINAIRES,K);
  o = o_;
};

void CONSENSUS::update_duty(float d_[]) {
  for(int i = 1;i < MAX_LUMINAIRES;i++){
    d_now[i] = d_[i];
  }
};

void CONSENSUS::update_average(float d_new_avg[]) {
  copyFloatArray(d_new_avg,MAX_LUMINAIRES,d_avg);
}

void CONSENSUS::update_lagrangian() {
  float result1[MAX_LUMINAIRES] = {};  //Usefull to avoid memory allocations
  float result2[MAX_LUMINAIRES] = {};  //Usefull to avoid memory allocations
  subtractArrays(d_now, d_avg, MAX_LUMINAIRES, result1);
  scalarProduct(result1, MAX_LUMINAIRES, rho, result2);
  sumArrays(lambda, result2, MAX_LUMINAIRES, lambda);
}

void CONSENSUS::consensus_iterate() {
  result.cost_best = 100000;  //Large number

  bool sol_unconstrained = true;
  bool sol_boundary_linear = true;
  bool sol_boundary_0 = true;
  bool sol_boundary_100 = true;
  bool sol_linear_0 = true;
  bool sol_linear_100 = true;

  float cost_temp = 0;                 //Float to hold temporary computations
  float result1[MAX_LUMINAIRES] = {0};  //Usefull to avoid memory allocations
  float result2[MAX_LUMINAIRES] = {0};  //Usefull to avoid memory allocations
  float result3[MAX_LUMINAIRES] = {0};  //Usefull to avoid memory allocations

  //Compute y
  float y[MAX_LUMINAIRES] = {0};
  scalarProduct(d_avg, MAX_LUMINAIRES, rho, result1);
  subtractArrays(result1, lambda, MAX_LUMINAIRES, result2);
  subtractArrays(result2, c, MAX_LUMINAIRES, y);

  //Unconstrained minimum
  scalarProduct(y, MAX_LUMINAIRES, 1 / rho, d_now);  //(1 / rho) * y;
  sol_unconstrained = check_feasibility();
  if (sol_unconstrained) {
    //REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
    //NO NEED TO COMPUTE THE OTHER
    cost_temp = evaluate_cost();
    if (cost_temp < result.cost_best) {
      copyFloatArray(d_now,MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;

      //If there is unconstrained sol, no need to check others
      return;
    }
  }

  //compute minimum constrained to linear boundary
  scalarProduct(y, MAX_LUMINAIRES, 1 / rho, result1);                                                  //(1 / rho) * y
  scalarProduct(K, MAX_LUMINAIRES, (1 / n) * (o - l + (1 / rho) * dot(y, K, MAX_LUMINAIRES)), result2);  //node.k/node.n*(node.o-node.L+(1/rho)*y'*node.k)
  subtractArrays(result1, result2, MAX_LUMINAIRES, d_now);                                             //(1/rho)*y - node.k/node.n*(node.o-node.L+(1/rho)*y'*node.k);
  //check feasibility of minimum constrained to linear boundary
  sol_boundary_linear = check_feasibility();
  // compute cost and if best store new optimum
  if (sol_boundary_linear) {
    cost_temp = evaluate_cost();
    if (cost_temp < result.cost_best) {
      copyFloatArray(d_now,MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;
    }
  }

  //compute minimum constrained to 0 boundary
  scalarProduct(y, MAX_LUMINAIRES, 1 / rho, d_now);  //(1 / rho) * y;
  d_now[0] = 0;
  //check feasibility of minimum constrained to 0 boundary
  sol_boundary_0 = check_feasibility();
  // compute cost and if best store new optimum
  if (sol_boundary_0) {
    cost_temp = evaluate_cost();
    if (cost_temp < result.cost_best) {
      copyFloatArray(d_now,MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;
    }
  }

  //compute minimum constrained to 100 boundary
  scalarProduct(y, MAX_LUMINAIRES, 1 / rho, d_now);  //(1 / rho) * y;
  d_now[0] = 100;
  //check feasibility of minimum constrained to 100 boundary
  sol_boundary_100 = check_feasibility();
  // compute cost and if best store new optimum
  if (sol_boundary_100) {
    cost_temp = evaluate_cost();
    if (cost_temp < result.cost_best) {
      copyFloatArray(d_now,MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;
    }
  }

  // compute minimum constrained to linear and 0 boundary
  scalarProduct(y, MAX_LUMINAIRES, 1 / rho, result1);                                                  //(1 / rho) * y
  scalarProduct(K, MAX_LUMINAIRES, 1 / m * (o - l), result2);                                          //(1 / node.m) * node.k * (node.o - node.L)
  scalarProduct(K, MAX_LUMINAIRES, (1 / rho / m) * (K[0] * y[0] - dot(y, K, MAX_LUMINAIRES)), result3);  //(1 / rho / node.m) * node.k * (node.k(node.index) * y(node.index) - y * node.k)
  subtractArrays(result1, result2, MAX_LUMINAIRES, d_now);                                             //(1 / rho) * y - (1 / node.m) * node.k * (node.o - node.L)
  sumArrays(d_now, result3, MAX_LUMINAIRES, d_now);                                                    //(1 / rho) * y - (1 / node.m) * node.k * (node.o - node.L) + (1 / rho / node.m) * node.k * (node.k(node.index) * y(node.index) - y * node.k);
  d_now[0] = 0;
  //check feasibility of minimum constrained to linear and 0 boundary
  sol_linear_0 = check_feasibility();
  //compute cost and if best store new optimum
  if (sol_linear_0) {
    cost_temp = evaluate_cost();
    if (cost_temp < result.cost_best) {
      copyFloatArray(d_now,MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;
    }
  }

  // compute minimum constrained to linear and 100 boundary
  scalarProduct(y, MAX_LUMINAIRES, 1 / rho, result1);                                                  //(1 / rho) * y
  scalarProduct(K, MAX_LUMINAIRES, 1 / m * (o - l + 100 * K[0]), result2);                             //(1/node.m)*node.k*(node.o-node.L+100*node.k(node.index))
  scalarProduct(K, MAX_LUMINAIRES, 1 / rho / m * (K[0] * y[0] - dot(y, K, MAX_LUMINAIRES)), result3);  //(1 / rho / node.m) * node.k * (node.k(node.index) * y(node.index) - y * node.k)
  subtractArrays(result1, result2, MAX_LUMINAIRES, d_now);                                             //(1 / rho) * y - (1/node.m)*node.k*(node.o-node.L+100*node.k(node.index))
  sumArrays(d_now, result3, MAX_LUMINAIRES, d_now);                                                    //(1 / rho) * y - (1/node.m)*node.k*(node.o-node.L+100*node.k(node.index)) + (1 / rho / node.m) * node.k * (node.k(node.index) * y(node.index) - y * node.k);
  d_now[0] = 100;
  //check feasibility of minimum constrained to linear and 100 boundary
  sol_linear_100 = check_feasibility();
  //compute cost and if best store new optimum
  if (sol_linear_100) {
    cost_temp = evaluate_cost();
    if (cost_temp < result.cost_best) { 
      copyFloatArray(d_now,MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;
    }
  }

  // Update d_now to the best solution
  copyFloatArray(result.d_best,MAX_LUMINAIRES,d_now);

  // Update d_now to the best solution
  copyFloatArray(result.d_best,MAX_LUMINAIRES,d_now);
};

// Method to print all members of the class to serial port
void CONSENSUS::print_consensus(){
  Serial.println("\nPrinting consensus values");
  Serial.print("d_now: ");
  printArray(d_now, MAX_LUMINAIRES);
  Serial.print("d_avg: ");
  printArray(d_avg, MAX_LUMINAIRES);
  Serial.print("K: ");
  printArray(K, MAX_LUMINAIRES);
  Serial.print("o: ");
  Serial.println(o);
  Serial.print("c: ");
  printArray(c, MAX_LUMINAIRES);
  Serial.print("n: ");
  Serial.println(n,5);
  Serial.print("m: ");
  Serial.println(m,5);
  Serial.print("lambda: ");
  printArray(lambda, MAX_LUMINAIRES);
  Serial.print("rho: ");
  Serial.println(rho);
  Serial.print("l: ");
  Serial.println(l);
  Serial.print("node_num: ");
  Serial.println(node_num);
  Serial.print("result.d_best: ");
  printArray(result.d_best, MAX_LUMINAIRES);
  Serial.print("result.cost_best: ");
  Serial.println(result.cost_best);
  Serial.println("End of consensus values\n\n\n");
}

// Method to print lagrangian multipliers (useful for debug)
void CONSENSUS::print_lagrangian(){
  Serial.print("lambda: ");
  printArray(lambda, MAX_LUMINAIRES);
  Serial.println();
}

#endif