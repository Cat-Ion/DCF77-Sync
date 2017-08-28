#ifndef DELTASIGMA_HPP
#define DELTASIGMA_HPP
template<typename InputType, typename OutputType>
class SigmaDelta {
public:
    InputType setpoint;
    constexpr SigmaDelta()
        : setpoint(0)
        , integral(0)
    {}

    OutputType step() __attribute__ ((noinline)) {
        integral += setpoint - InputType(output());
        return output();
    }

    OutputType output() const {
        return OutputType(integral);
    }

protected:
    InputType integral;
};

#endif // DELTASIGMA_HPP
