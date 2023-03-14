#ifndef PID_CONROLLER_H
#define PID_CONTROLER_H
namespace l5player
{
    namespace control
    {
        class PIDController
        {
            public:
                PIDController(const double kp, const double ki, const double kd);
                ~PIDController() = default;

                /**
                 * @brief compute control value based on the error
                 * @param error error value, the difference between
                 * a desired value and a measured value
                 * @param dt sampling time interval
                 * @return control value based on PID terms
                 */
                double Control(const double error, const double dt);

                void Reset();

            protected:
                double kp_ = 0.0;
                double ki_ = 0.0;
                double kd_ = 0.0;
                double previous_error_ = 0.0;
                double previous_output_ = 0.0;
                double integral_ = 0.0;
                bool first_hit_ = false;

                double proportional_part = 0;
                double integral_part = 0;
                double derivative_part = 0;
                double current_output = 0;
        };

    }  // namespace control
}  // namespace l5player

#endif