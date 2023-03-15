#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_
namespace l5player {
namespace control {
class PIDController {
 public:
  PIDController(const double kp, const double ki, const double kd);
  ~PIDController() = default;

  void Reset();

  /**
   * @brief compute control value based on the error
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  double Control(const double error, const double dt);

 protected:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  bool first_hit_ = false;
};

}  // namespace control
}  // namespace l5player
#endif