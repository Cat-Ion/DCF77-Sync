#ifndef SMOOTHER_HPP
#define SMOOTHER_HPP

template <typename T>
class ISmoother
{
public:
    constexpr ISmoother(T const &alpha = 1, T const &v = 0) : alpha(alpha), beta(T(1)-T(alpha)), v(v) {}

    T alpha, beta;
    T v;

    constexpr T operator*() const {
        return v;
    }
};

template <typename T, int n> class Smoother: public ISmoother<T>
{
public:
    T s[n-1];

    constexpr Smoother(T const &alpha = 1, T const &v = 0)
        : ISmoother<T>(alpha, v)
    {
        for (int i = 0; i < n - 1; i++) {
            s[i] = v;
        }
    }

    constexpr ISmoother<T> &operator<<(T v) __attribute__ (( noinline )) {
        for (int i = 0; i < n - 1; i++) {
            s[i] *= this->beta;
            v    *= this->alpha;
            v    += s[i];
            s[i]  = v;
        }
        this->v *= this->beta;
        v *= this->alpha;
        this->v += v;
        return *this;
    }

    constexpr void add_all(T const &a)
    {
        for (int i = 0; i < n - 1; i++) {
            s[i] += a;
        }
        this->v += a;
    }
};
template <typename T> class Smoother<T, 0>: public ISmoother<T> {
public:
    constexpr Smoother(T const &alpha = 1, T const &v = 0)
        : ISmoother<T>(alpha, v)
    {
    }

    constexpr ISmoother<T> &operator<<(T const &v) {
        this->v = v;
        return *this;
    }

    constexpr void add_all(T const &a)
    {
        this->v += a;
    }
};
#endif // SMOOTHER_HPP
