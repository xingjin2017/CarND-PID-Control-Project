#include <iostream>

#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
  Kp = Kp_init;
  Ki = Ki_init;
  Kd = Kd_init;
  cout << "PID::Init Kp " << Kp << " Ki " << Ki << " Kd " << Kd << endl;
  Reset();
}

void PID::Reset() {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  total_error = 0.0;
  update_counter = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
  output = -Kp * cte - Kd * d_error - Ki * i_error;
  total_error += cte * cte;
  update_counter++;
}

double PID::Output() {
  return output;
}

double PID::TotalError() {
  return total_error;
}

double PID::AverageError() {
  return update_counter > 0 ? total_error / update_counter : 0.0;
}

bool PID::enabledTwiddle() {
  return enable_twiddle;
}

bool PID::anotherTwiddleTry() {
  return enable_twiddle && twiddle.steps > 0
    && update_counter > twiddle.steps;
}

void PID::resetSimulator(uWS::WebSocket<uWS::SERVER>& ws) {
  twiddle.tried++;
  Twiddle::State finished_state = twiddle.state;
  Twiddle::State new_state = Twiddle::NOT_STARTED;
  bool prev_best = false;
  if (twiddle.tried >= 1) {
    // the previous run's total error.
    double average_error = AverageError();
    std::cout << "PID tried #" << twiddle.tried << " average error "
	      << AverageError() << std::endl;
    if (twiddle.best_average_error < 0.0
	|| (average_error > 0.0 && average_error < twiddle.best_average_error)) {
      twiddle.best_average_error = average_error;
      twiddle.recordBest();
      prev_best = true;
    }
  }

  // Do clean up work for the finished state.
  switch (finished_state) {
  case Twiddle::NOT_STARTED:
    new_state = Twiddle::INITIAL;
    break;
  case Twiddle::INITIAL:
    new_state = Twiddle::ROUTINE;
    twiddle.p[twiddle.idx] += twiddle.dp[twiddle.idx];
    break;
  case Twiddle::ROUTINE:
    if (prev_best) {
      twiddle.dp[twiddle.idx] *= 1.15;
      new_state = Twiddle::ROUTINE;
      //twiddle.idx = (twiddle.idx + 1) % 3;
      // Skip Pi tuning
      twiddle.idx = twiddle.idx == 0 ? 2 : 0;
      twiddle.p[twiddle.idx] += twiddle.dp[twiddle.idx];
    } else {
      new_state = Twiddle::FOLLOWUP;
      twiddle.p[twiddle.idx] -= 2 * twiddle.dp[twiddle.idx];
    }
    break;
  case Twiddle::FOLLOWUP:
    if (prev_best) {
      twiddle.dp[twiddle.idx] *= 1.1;
    } else {
      twiddle.p[twiddle.idx] += twiddle.dp[twiddle.idx];
      twiddle.dp[twiddle.idx] *= 0.90;
    }
    new_state = Twiddle::ROUTINE;
    //twiddle.idx = (twiddle.idx + 1) % 3;
    // Skip Pi tuning.
    twiddle.idx = twiddle.idx == 0 ? 2 : 0;
    twiddle.p[twiddle.idx] += twiddle.dp[twiddle.idx];
    break;
  }

  twiddle.state = new_state;
  Init(twiddle.p[0], twiddle.p[1], twiddle.p[2]);

  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

double Twiddle::SumDp() {
  return dp[0] + dp[1] + dp[2];
}

void Twiddle::print() {
  std::cout << "twiddle p " << p[0] << ", " << p[1]
	    << ", " << p[2] << std::endl;
  std::cout << "twiddle dp " << dp[0] << ", " << dp[1]
	    << ", " << dp[2] << std::endl;
  std::cout << "best_average_error " << best_average_error
	    << " steps " << steps << std::endl;
  std::cout << "twiddle best_p " << best_p[0] << ", "
	    << best_p[1] << ", " << best_p[2] << std::endl;
}

void Twiddle::recordBest() {
  for (int i = 0; i < 3; i++) {
    best_p[i] = p[i];
  }
}


