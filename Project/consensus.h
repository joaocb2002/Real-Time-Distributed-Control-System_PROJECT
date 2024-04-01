#include "macros.hh"
#include "utils.hh"

#ifndef CONSENSUS_H
#define CONSENSUS_H

//Defined Constraints
#define DUB 100    //Duty cycle upper bound
#define DLB 0      //Duty cycle lower bound
#define TOL 0.001  //tolerance for rounding errors

struct consensus_out {
  float d_best[MAX_LUMINAIRES];  //Float to hold the best duty cycle
  float cost_best;
};

class CONSENSUS {
private:
  float d_now[MAX_LUMINAIRES] = {};  //Current duty cycle iterate values
  float d_avg[MAX_LUMINAIRES] = {};  //The average duty cycle
  float K[MAX_LUMINAIRES];           //Cross coupling gains
  float o;                           //external illuminance
  float c[MAX_LUMINAIRES] = {};      //The cost value (maximum energy??)

  //Linear Boundary variables
  float m = 1;
  float n = 5;

  //Lagrangian multipliers
  float lambda[MAX_LUMINAIRES] = {};
  float rho = 0;

  //The output of the consensus iterate
  consensus_out result;

public:
  float l;  //illuminance value

  // Constructor
  explicit CONSENSUS(float K_[], float o_) {
    //Cross-coupling gains vector
    std::copy(K_, K_ + MAX_LUMINAIRES, K);

    //Init values
    c[0] = 10;  //Maximum energy here
    o = o_;     //External Illuminance
  };

  // Destructor
  ~CONSENSUS(){};

  bool check_feasibility();
  float evaluate_cost();
  consensus_out consensus_iterate();
  float compute_lux();
  void update_gains(float K_[], float o_);
  void update_average();
  void update_lagrangian();
  void update_duty(float d_[]);
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
  return true;
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
  std::copy(K_, K_ + MAX_LUMINAIRES, K);
  o = o_;
};

void CONSENSUS::update_duty(float d_[]) {
  for(int i = 1;i < MAX_LUMINAIRES;i++){
    d_now[i] = d_[i];
  }
};

void CONSENSUS::update_average() {
  float sum = 0;
  for (int i = 0; i < MAX_LUMINAIRES; i++) {
    sum += d_now[i];
  }
  sum /= MAX_LUMINAIRES;
  for (int i = 0; i < MAX_LUMINAIRES; i++) {
    d_avg[i] = sum;
  }
}

void CONSENSUS::update_lagrangian() {
  float result1[MAX_LUMINAIRES] = {};  //Usefull to avoid memory allocations
  float result2[MAX_LUMINAIRES] = {};  //Usefull to avoid memory allocations
  subtractArrays(d_now, d_avg, MAX_LUMINAIRES, result1);
  scalarProduct(result1, MAX_LUMINAIRES, rho, result2);
  sumArrays(lambda, result2, MAX_LUMINAIRES, lambda);
}

consensus_out CONSENSUS::consensus_iterate() {
  result.cost_best = 100000;  //Large number

  bool sol_unconstrained = true;
  bool sol_boundary_linear = true;
  bool sol_boundary_0 = true;
  bool sol_boundary_100 = true;
  bool sol_linear_0 = true;
  bool sol_linear_100 = true;

  float cost_temp = 0;                 //Float to hold temporary computations
  float result1[MAX_LUMINAIRES] = {};  //Usefull to avoid memory allocations
  float result2[MAX_LUMINAIRES] = {};  //Usefull to avoid memory allocations
  float result3[MAX_LUMINAIRES] = {};  //Usefull to avoid memory allocations

  //Compute y
  float y[MAX_LUMINAIRES] = {};
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
      std::copy(d_now, d_now + MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;

      //If there is unconstrained sol, no need to check others
      return result;
    }
  }

  //compute minimum constrained to linear boundary
  scalarProduct(y, MAX_LUMINAIRES, 1 / rho, result1);                                                  //(1 / rho) * y
  scalarProduct(K, MAX_LUMINAIRES, 1 / n * (o - l + (1 / rho) * dot(y, K, MAX_LUMINAIRES)), result2);  //node.k/node.n*(node.o-node.L+(1/rho)*y'*node.k)
  subtractArrays(result1, result2, MAX_LUMINAIRES, d_now);                                             //(1/rho)*y - node.k/node.n*(node.o-node.L+(1/rho)*y'*node.k);
  //check feasibility of minimum constrained to linear boundary
  sol_boundary_linear = check_feasibility();
  // compute cost and if best store new optimum
  if (sol_boundary_linear) {
    cost_temp = evaluate_cost();
    if (cost_temp < result.cost_best) {
      std::copy(d_now, d_now + MAX_LUMINAIRES, result.d_best);
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
      std::copy(d_now, d_now + MAX_LUMINAIRES, result.d_best);
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
      std::copy(d_now, d_now + MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;
    }
  }

  // compute minimum constrained to linear and 0 boundary
  scalarProduct(y, MAX_LUMINAIRES, 1 / rho, result1);                                                  //(1 / rho) * y
  scalarProduct(K, MAX_LUMINAIRES, 1 / m * (o - l), result2);                                          //(1 / node.m) * node.k * (node.o - node.L)
  scalarProduct(K, MAX_LUMINAIRES, 1 / rho / m * (K[0] * y[0] - dot(y, K, MAX_LUMINAIRES)), result3);  //(1 / rho / node.m) * node.k * (node.k(node.index) * y(node.index) - y * node.k)
  subtractArrays(result1, result2, MAX_LUMINAIRES, d_now);                                             //(1 / rho) * y - (1 / node.m) * node.k * (node.o - node.L)
  sumArrays(d_now, result3, MAX_LUMINAIRES, d_now);                                                    //(1 / rho) * y - (1 / node.m) * node.k * (node.o - node.L) + (1 / rho / node.m) * node.k * (node.k(node.index) * y(node.index) - y * node.k);
  d_now[0] = 0;
  //check feasibility of minimum constrained to linear and 0 boundary
  sol_linear_0 = check_feasibility();
  //compute cost and if best store new optimum
  if (sol_linear_0) {
    cost_temp = evaluate_cost();
    if (cost_temp < result.cost_best) {
      std::copy(d_now, d_now + MAX_LUMINAIRES, result.d_best);
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
      std::copy(d_now, d_now + MAX_LUMINAIRES, result.d_best);
      result.cost_best = cost_temp;
    }
  }

  return result;
};

#endif