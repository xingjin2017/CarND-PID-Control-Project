#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID;

/**
 * A class that tracks Twiddle states and adjustments.
 */
class Twiddle {
 protected:
  // Twiddle parameters
  //twiddle dp 3.393e-05, 0.1, 0.000147522
  //best_average_error 0.250778 steps 1500
  //twiddle best_p 0.56357, 0, 4.12158
  double p [3] = {0.55, 0.0, 4.1};
  double dp [3] = {0.2, 0.0, 0.75};
  double best_p [3] = {0.0, 0.0, 0.0};
  int idx = 0;
  int tried = -1;
  int steps = 1500;
  enum State { NOT_STARTED, INITIAL, ROUTINE, FOLLOWUP };
  State state = NOT_STARTED;
  double best_average_error = -1.0;

  double SumDp();
  void print();
  void recordBest();

  friend PID;
};

class PID {
 public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * PID action output value.
   */
  double output;

  /*
   * Total errors.
   */
  double total_error;
  int update_counter;

  bool enable_twiddle;
  Twiddle twiddle;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Reset state on Connected.
   */
  void Reset();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
   * Calculated PID output.
   */
  double Output();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Calculate the average PID error.
   */
  double AverageError();

  /*
   * Enable Twiddle parameter tuning.
   */
  bool enabledTwiddle();

  /*
   * Whether it is time for another Twiddle try.
   */
  bool anotherTwiddleTry();

  /*
   * Reset simulator for another Twiddle try.
   */
  void resetSimulator(uWS::WebSocket<uWS::SERVER>& ws);
};

#endif /* PID_H */
