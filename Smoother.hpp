#ifndef SMOOTHER_HPP
#define SMOOTHER_HPP

template <typename T, int n> class Smoother
{
public:
    T alpha, beta;
    T v;
    T s[n-1];

    constexpr Smoother(T const &alpha = 1, T const &v = 0)
        : alpha(alpha), beta(T(1)-T(alpha)), v(v)
    {
        for (int i = 0; i < n - 1; i++) {
            s[i] = v;
        }
    }

    constexpr Smoother &operator<<(T const &v) __attribute__ (( noinline )) {
        T a(v);
        for (int i = 0; i < n - 1; i++) {
            s[i] *= this->beta;
            a    *= this->alpha;
            a   += s[i];
            s[i] = a;
        }
        a *= this->alpha;
        this->v *= this->beta;
        this->v += a;
        return *this;
    }

    constexpr void add_all(T const &a)
    {
        for (int i = 0; i < n - 1; i++) {
            s[i] += a;
        }
        this->v += a;
    }

    constexpr T operator*() const {
        return v;
    }
};

template <typename T> class Smoother<T, 0>
{
public:
    T alpha, beta;
    T v;

    constexpr Smoother(T const &alpha = 1, T const &v = 0)
        : alpha(alpha), beta(T(1)-T(alpha)), v(v)
    {
    }

    constexpr Smoother &operator<<(T const &v) {
        this->v = v;
        return *this;
    }

    constexpr void add_all(T const &a)
    {
        this->v += a;
    }

    constexpr T operator*() const {
        return v;
    }
};

#endif // SMOOTHER_HPP
