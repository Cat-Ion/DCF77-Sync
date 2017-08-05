#ifndef PID_HPP
#define PID_HPP
template<typename T, bool use_p = true, bool use_i = true, bool use_d = true>
class PID
{
public:
    constexpr PID(T kp, T ki, T kd, T interval,
                  T min_output, T max_output,
                  T setpoint)
        : _kp(kp)
        , _ki(ki)
        , _kd(kd)
        , _interval(interval)
        , _interval_inv(T(1)/interval)
        , _setpoint(setpoint)
        , _error(0)
        , _integral((max_output + min_output)/T(2.))
        , _derivative(0)
        , _min_output(min_output)
        , _max_output(max_output)
        , _midpoint((max_output + min_output) / T(2.))
        , _output(_integral)
    {
    }

    constexpr void set_setpoint(T sp) {
        _error += sp - _setpoint;
        _setpoint = sp;
    }

    constexpr void step(T value) {
        T new_error = _setpoint - value;
        T tmp;

        _output = _midpoint;
        if (use_i) {
            tmp = new_error; tmp *= _interval; tmp *= _ki;
            _integral += tmp;
            _output += _integral;
        }
        if (use_d) {
            tmp = new_error - _error; tmp *= _interval_inv;
            _derivative = tmp;
            _output += _kd * _derivative;
        }
        _error = new_error;
        if (use_p) {
            _output += _kp * _error;
        }
        if (_output < _min_output) {
            if (use_i) {
                _integral += _min_output - _output;
            }
            _output = _min_output;
        } else if (_output > _max_output) {
            if (use_i) {
                _integral += _max_output - _output;
            }
            _output = _max_output;
        }
    }

    constexpr T output(void) const {
        return _output;
    }

protected:
    T _kp, _ki, _kd;
    T _interval, _interval_inv;

    T _setpoint;
    T _error;
    T _integral;
    T _derivative;

    T _min_output, _max_output, _midpoint;
    T _output;
};

#endif // PID_HPP
